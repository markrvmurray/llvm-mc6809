//===-- MC6809FinalLowering.cpp - MC6809 Final Lowering Pass --------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 final-lowering pass. See MC6809FinalLowering.h for the transform
// classes it runs, each with its own -O gate and STATISTIC counter.
//
//===----------------------------------------------------------------------===//

#include "MC6809FinalLowering.h"

#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mc6809-final-lowering"

using namespace llvm;

STATISTIC(NumOffsetsRelaxed,    "Number of indexed offsets relaxed to a "
                                "narrower form (_o16->_o8->_o5/_o0)");
STATISTIC(NumDupStoresElided,   "Number of dead duplicate stores to the same "
                                "slot elided");
STATISTIC(NumStoreReloadsElided,"Number of redundant reloads of unmodified "
                                "spill slots elided");
STATISTIC(NumBranchOverBranchFolded, "Number of branch-over-branch pairs "
                                     "(cond + uncond) folded into a single "
                                     "branch (bug #179)");
namespace {

// Each transform class has its own minimum -O level. Default is -O2
// (CodeGenOptLevel::Default). Set to 99 to disable a class entirely
// (useful when bisecting which class regressed).
//
// Naming convention: -mc6809-fl-<class>-min-O. The "fl" stands for
// "final lowering" and the leading "mc6809-" marks the flag as
// target-specific in `llc -help-hidden` output.
static cl::opt<int> RelaxOffsetsMinO(
    "mc6809-fl-offset-relax-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable indexed-offset relaxation "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> DupStoreMinO(
    "mc6809-fl-dup-store-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable duplicate-store elimination "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> StoreReloadMinO(
    "mc6809-fl-store-reload-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable store+reload elimination "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> BranchOverBranchMinO(
    "mc6809-fl-branch-over-branch-min-O", cl::init(0), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable branch-over-branch folding "
             "in MC6809FinalLowering (bug #179). 99 disables. Default: 0 "
             "(always on; transform is purely local and never enlarges code, "
             "and BranchFolderPass doesn't run at -O0 to do this work for "
             "us)."));
class MC6809FinalLowering : public MachineFunctionPass {
public:
  static char ID;

  MC6809FinalLowering() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FinalLoweringPass(
        *PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "MC6809 Final Lowering";
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  bool relaxOffsets(MachineFunction &MF);
  bool elideDupStores(MachineFunction &MF);
  bool elideStoreReload(MachineFunction &MF);
  bool foldBranchOverBranch(MachineFunction &MF);

  // True if the current -O level (from MF.getTarget()) >= MinO.
  // MinO == 99 means "disabled by CL flag".
  bool enabled(const MachineFunction &MF, int MinO) const {
    if (MinO >= 99)
      return false;
    int CurO = static_cast<int>(MF.getTarget().getOptLevel());
    return CurO >= MinO;
  }
};

} // namespace

/// See MC6809FrameLowering::isFrameBase.
static bool isFrameBase(const MachineFunction &MF, Register Reg) {
  return static_cast<const MC6809FrameLowering &>(
             *MF.getSubtarget().getFrameLowering())
      .isFrameBase(MF, Reg);
}

bool MC6809FinalLowering::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;

  // Class 6 runs FIRST and is NOT gated by skipFunction: at -O0 every
  // function carries `optnone`, which makes skipFunction return true, and
  // Class 6 backstops BranchFolderPass, which does not run at -O0.
  if (enabled(MF, BranchOverBranchMinO))
    Changed |= foldBranchOverBranch(MF);

  // Classes 1, 4 and 5 are size/speed optimisations and respect optnone.
  if (skipFunction(MF.getFunction()))
    return Changed;

  if (enabled(MF, RelaxOffsetsMinO))
    Changed |= relaxOffsets(MF);
  if (enabled(MF, DupStoreMinO))
    Changed |= elideDupStores(MF);
  if (enabled(MF, StoreReloadMinO))
    Changed |= elideStoreReload(MF);
  return Changed;
}

// Commit 0 — every class is a stub. Subsequent commits replace the body of
// each function with the real transform and bump the matching STATISTIC.

bool MC6809FinalLowering::relaxOffsets(MachineFunction &MF) {
  // Walk every MI; for any indexed-immediate opcode whose offset value
  // fits a narrower encoding, switch to the narrower opcode and (when
  // shrinking to the _o0 form) drop the offset operand.
  //
  // Operand layout for indexed-immediate instructions in this backend
  // (post-ExpandPostRAPseudos; see InstrInfo's expandLoadIdx for the
  // ground truth on construction):
  //
  //   _o5/_o8/_o16: <maybe-implicit data reg> + offset imm + base index reg
  //   _o0:          <maybe-implicit data reg> + base index reg (no offset)
  //
  // The data register may be implicit-def (for loads/LEA), implicit-use
  // (for stores), or absent (for the LEA destination encoded in opcode).
  // To find the offset robustly, we just scan operands for the first
  // immediate operand and treat that as the offset. The opcode-table
  // lookup in getRelaxedIdxOpcode is the source of truth for whether
  // the instruction is actually an indexed-imm form.
  const auto *TII = static_cast<const MC6809InstrInfo *>(
      MF.getSubtarget().getInstrInfo());

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      // Find the first immediate operand (= offset for indexed-imm forms).
      int OffsetIdx = -1;
      for (unsigned I = 0, E = MI.getNumOperands(); I != E; ++I) {
        if (MI.getOperand(I).isImm()) {
          OffsetIdx = I;
          break;
        }
      }
      if (OffsetIdx < 0)
        continue;

      int64_t Offset = MI.getOperand(OffsetIdx).getImm();
      auto [NewOpc, NewLen] = TII->getRelaxedIdxOpcode(MI.getOpcode(),
                                                       Offset);
      if (NewOpc == 0)
        continue;

      MI.setDesc(TII->get(NewOpc));
      if (NewLen == 0) {
        // Shrinking to _o0 — drop the (now redundant) offset operand.
        MI.removeOperand(OffsetIdx);
      }
      ++NumOffsetsRelaxed;
      Changed = true;
    }
  }
  return Changed;
}

bool MC6809FinalLowering::elideDupStores(MachineFunction &MF) {
  // Eliminate dead duplicate stores to the same spill slot:
  //
  //   stb $21,u
  //   andb #$3        ; modifies B but doesn't touch memory
  //   stb $21,u       ; overwrites the first store, no intervening read
  //
  // Per-MBB linear scan: track the most recent store to each
  // (opcode, offset, base) triple. When another store to the same
  // triple is reached without an intervening MI that could read the
  // slot, the prior store is dead — erase it.
  //
  // Conservative invalidations (clear the entire tracking map):
  //   - any MI with mayLoad (loads might read the slot we wrote)
  //   - any MI with mayStore that we don't recognize as a tracked
  //     store opcode (it might write to overlapping memory)
  //   - any call (callees may read the caller's stack)
  //   - any inline-asm or barrier
  //
  // Intra-MBB only — cross-block tracking would require a forward
  // dataflow that's well outside the scope of a final-lowering pass.

  auto isTrackedStore = [](const MachineInstr &MI) {
    // Indexed-immediate store opcodes (post-expansion forms only —
    // _o0/_o5/_o8/_o16 across STA/STB/STD/STE/STF/STW/STQ/
    // STX/STY/STU/STS).
    static const unsigned Ops[] = {
      MC6809::STAi_o0, MC6809::STAi_o5, MC6809::STAi_o8, MC6809::STAi_o16,
      MC6809::STBi_o0, MC6809::STBi_o5, MC6809::STBi_o8, MC6809::STBi_o16,
      MC6809::STDi_o0, MC6809::STDi_o5, MC6809::STDi_o8, MC6809::STDi_o16,
      MC6809::STEi_o0, MC6809::STEi_o5, MC6809::STEi_o8, MC6809::STEi_o16,
      MC6809::STFi_o0, MC6809::STFi_o5, MC6809::STFi_o8, MC6809::STFi_o16,
      MC6809::STWi_o0, MC6809::STWi_o5, MC6809::STWi_o8, MC6809::STWi_o16,
      MC6809::STQi_o0, MC6809::STQi_o5, MC6809::STQi_o8, MC6809::STQi_o16,
      MC6809::STXi_o0, MC6809::STXi_o5, MC6809::STXi_o8, MC6809::STXi_o16,
      MC6809::STYi_o0, MC6809::STYi_o5, MC6809::STYi_o8, MC6809::STYi_o16,
      MC6809::STUi_o0, MC6809::STUi_o5, MC6809::STUi_o8, MC6809::STUi_o16,
      MC6809::STSi_o0, MC6809::STSi_o5, MC6809::STSi_o8, MC6809::STSi_o16,
    };
    unsigned Opc = MI.getOpcode();
    for (unsigned O : Ops)
      if (O == Opc)
        return true;
    return false;
  };

  // Slot key = (opcode, offset_imm, base_reg). A store with _o0 form
  // has offset=0 implicitly (no explicit operand), so we treat its
  // offset key as 0.
  struct SlotKey {
    unsigned Opcode;
    int64_t Offset;
    Register Base;
    bool operator==(const SlotKey &O) const {
      return Opcode == O.Opcode && Offset == O.Offset && Base == O.Base;
    }
  };

  // A store's (opcode, offset, base) key only identifies a fixed memory
  // location while the base register holds a fixed value. The hardware
  // stack pointer $ss is adjusted by LEAS during outgoing-argument setup,
  // so two `STQi_o0 $ss` stores separated by a `LEAS N,$ss` address
  // DIFFERENT memory (e.g. a scratch slot vs. a live call-argument slot).
  // Any def of a tracked store's base register must therefore drop that
  // entry, or we would elide a still-live store as a false duplicate.
  // (The sibling elideStoreReload guards the same hazard.)
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    SmallVector<std::pair<SlotKey, MachineInstr *>, 8> RecentStores;

    auto invalidateAll = [&]() { RecentStores.clear(); };
    auto invalidateReg = [&](Register R) {
      RecentStores.erase(
          std::remove_if(RecentStores.begin(), RecentStores.end(),
                         [&, R](const std::pair<SlotKey, MachineInstr *> &P) {
                           return TRI->regsOverlap(P.first.Base, R);
                         }),
          RecentStores.end());
    };
    auto findStore = [&](const SlotKey &K) -> MachineInstr ** {
      for (auto &P : RecentStores)
        if (P.first == K)
          return &P.second;
      return nullptr;
    };
    auto recordStore = [&](const SlotKey &K, MachineInstr *MI) {
      if (auto **Slot = findStore(K)) {
        *Slot = MI;
        return;
      }
      RecentStores.push_back({K, MI});
    };

    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      // First: cheap structural classifications.
      if (MI.isCall() || MI.isInlineAsm() || MI.isReturn() ||
          MI.isBranch() || MI.isTerminator()) {
        invalidateAll();
        continue;
      }

      // A volatile store is observable: never drop it as a duplicate, and
      // never let it stand as the store that makes an earlier one dead.
      bool Tracked = isTrackedStore(MI) &&
                     !MC6809InstrInfo::hasOrderedMemOperand(MI);

      if (!Tracked) {
        // Untracked memory access invalidates everything.
        if (MI.mayLoad() || MI.mayStore()) {
          invalidateAll();
          continue;
        }
        // A def of any tracked store's base register (e.g. LEAS adjusting
        // $ss) changes which memory that store's key refers to — drop the
        // affected entries so a later same-key store can't elide it.
        for (const MachineOperand &MO : MI.operands())
          if (MO.isReg() && MO.isDef() && MO.getReg().isPhysical())
            invalidateReg(MO.getReg());
        continue;
      }

      // Build the slot key.
      SlotKey K{MI.getOpcode(), 0, Register()};
      if (!MC6809InstrInfo::getIdxBaseOffset(MI, K.Base, K.Offset)) {
        // Unexpected operand shape — be safe.
        invalidateAll();
        continue;
      }

      // If we already have a recorded store to this slot, the prior
      // one is dead.
      if (auto **Prior = findStore(K)) {
        if (*Prior) {
          (*Prior)->eraseFromParent();
          ++NumDupStoresElided;
          Changed = true;
        }
        *Prior = &MI;
      } else {
        recordStore(K, &MI);
      }
    }
  }
  return Changed;
}

bool MC6809FinalLowering::elideStoreReload(MachineFunction &MF) {
  // Eliminate redundant reloads of unmodified spill slots:
  //
  //   std 24,u      ; spill D to slot 24 (data register = $ad)
  //   ; ... no MI writes $ad and no MI writes slot 24 ...
  //   ldd 24,u      ; reload — but $ad still holds the same value.
  //
  // Erase the LDD (already-in-register).
  //
  // Per-MBB linear scan: track slotMap[(family, offset, base)] -> data
  // register currently holding the slot's value. On any clobber of a
  // data register OR a write to the slot, invalidate the corresponding
  // entries.
  //
  // Conservative invalidations:
  //   - any call: clear all entries (callees may clobber any caller-save
  //     reg AND may write through the stack)
  //   - inline-asm / barriers / branches: clear all
  //   - any unrecognised mayLoad / mayStore: clear all
  //
  // Family-based matching (STA<->LDA, STB<->LDB, STD<->LDD, etc.) means
  // we don't try to fold cross-width cases (e.g. STD followed by LDA of
  // the high byte). Strictly the safe MVP.

  // Map an opcode to (family-tag, data-register) iff it is a tracked
  // store or load. Family tag is the data-register itself (which is
  // unique per family).
  struct StoreOrLoad {
    bool IsStore;
    Register DataReg; // family tag + actual data reg
  };
  auto classify = [](unsigned Opc) -> std::optional<StoreOrLoad> {
    if (auto LS = MC6809InstrInfo::getIdxLoadStore(Opc))
      return StoreOrLoad{LS->IsStore, LS->Value};
    return std::nullopt;
  };

  // Slot key: (data-reg-family-tag, offset, base-reg). Storing 0
  // implicit for the _o0 form.
  struct SlotKey {
    Register Family;
    int64_t Offset;
    Register Base;
    bool operator==(const SlotKey &O) const {
      return Family == O.Family && Offset == O.Offset && Base == O.Base;
    }
  };

  // For sub-register aliasing: AA/AB are sub-regs of AD; AE/AF of AW;
  // AA/AB/AE/AF/AW of AQ. A write to AA leaves AD's low byte (AB)
  // intact but invalidates the "AD as a whole holds slot X" tracking.
  // TargetRegisterInfo::regsOverlap captures this correctly across all
  // 8/16/32-bit family relationships including the 6309 wider forms.
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    SmallVector<std::pair<SlotKey, Register>, 8> Slots;

    auto findSlot = [&](const SlotKey &K) -> Register * {
      for (auto &P : Slots)
        if (P.first == K)
          return &P.second;
      return nullptr;
    };
    auto recordSlot = [&](const SlotKey &K, Register DataReg) {
      if (auto *S = findSlot(K)) {
        *S = DataReg;
        return;
      }
      Slots.push_back({K, DataReg});
    };
    auto invalidateAll = [&]() { Slots.clear(); };
    auto invalidateReg = [&](Register R) {
      // Drop any slot whose tracked DATA reg OR BASE reg overlaps R.
      // The base-reg case matters for stack-pointer adjustments
      // (LEAS -2,S changes $ss, so any "slot N from $ss" tracked
      // before the LEAS now refers to a different memory location).
      // Frame-pointer ($su) base is stable within a function body
      // by construction, but we treat it uniformly for safety.
      Slots.erase(
          std::remove_if(Slots.begin(), Slots.end(),
                         [&, R](const std::pair<SlotKey, Register> &P) {
                           return TRI->regsOverlap(P.second, R) ||
                                  TRI->regsOverlap(P.first.Base, R);
                         }),
          Slots.end());
    };

    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      if (MI.isCall() || MI.isInlineAsm() || MI.isReturn() ||
          MI.isBranch() || MI.isTerminator()) {
        invalidateAll();
        continue;
      }

      auto Cls = classify(MI.getOpcode());
      // A volatile access is observable: its reload is never redundant, and
      // it records nothing a later reload could be removed against.
      if (Cls && MC6809InstrInfo::hasOrderedMemOperand(MI))
        Cls.reset();
      if (!Cls) {
        // Untracked: invalidate everything if it could touch memory or
        // any tracked data register.
        if (MI.mayLoad() || MI.mayStore()) {
          invalidateAll();
          continue;
        }
        // Otherwise, invalidate only tracked-data-reg defs.
        for (const MachineOperand &MO : MI.operands()) {
          if (MO.isReg() && MO.isDef() && MO.getReg().isPhysical())
            invalidateReg(MO.getReg());
        }
        continue;
      }

      // Build slot key.
      SlotKey K{Cls->DataReg, 0, Register()};
      if (!MC6809InstrInfo::getIdxBaseOffset(MI, K.Base, K.Offset)) {
        invalidateAll();
        continue;
      }

      // Only track loads/stores whose base is the canonical frame pointer
      // ($su) or stack pointer ($ss). Anything with a dynamic base
      // register (X/Y/U used as a generic pointer via LEA/etc.) might
      // alias an $su-relative slot — we can't prove non-aliasing without
      // pointer analysis, so a tracked store through such a base is
      // treated as an opaque mayStore (invalidateAll), and a tracked load
      // through it is treated as opaque mayLoad (invalidateAll). This
      // gives up some elision opportunities but is necessary for
      // correctness — see the qsort failure traced to STD ,Y aliasing
      // with STY $399,u in the swapfunc loop.
      if (!isFrameBase(MF, K.Base)) {
        invalidateAll();
        continue;
      }

      // Conservative invariant: AT MOST ONE slot tracks a given data
      // register at any time. On every tracked store or load (whether
      // elided or not), invalidate any prior slot tracking this data
      // reg before recording the new slot. Reasoning: keeping multiple
      // slots tracked under the same data reg is correct in principle
      // (their contents are equal at the moment of recording), but
      // creates fragile state — any subsequent partial invalidation
      // (sub-reg aliasing, base-reg moves, dynamic-base writes) has
      // multiple slots to invalidate, each with its own subtle rules,
      // and we've already burned several bug-fix cycles on edge cases.
      // Single-slot tracking gives up some elision opportunities but
      // is dramatically simpler to reason about.
      if (Cls->IsStore) {
        // The store overwrites its bytes whatever register held them: drop
        // every slot on this base whose bytes it overlaps (`std 4,s ; stx
        // 4,s` leaves no D in 4,s), then record its own.
        int64_t W = MC6809InstrInfo::getValueRegBytes(Cls->DataReg);
        Slots.erase(
            std::remove_if(Slots.begin(), Slots.end(),
                           [&](const std::pair<SlotKey, Register> &P) {
                             int64_t PW =
                                 MC6809InstrInfo::getValueRegBytes(P.second);
                             return P.first.Base == K.Base &&
                                    P.first.Offset < K.Offset + W &&
                                    K.Offset < P.first.Offset + PW;
                           }),
            Slots.end());
        invalidateReg(Cls->DataReg);
        recordSlot(K, Cls->DataReg);
      } else {
        // Load: if the slot is currently tracked AND already in DataReg,
        // the load is redundant.
        if (Register *Existing = findSlot(K)) {
          if (*Existing == Cls->DataReg) {
            // Bug #207: clear stale `killed` flags on uses of DataReg
            // (or any sub/super-register that overlaps DataReg) that
            // sit between the matching store and the about-to-be-erased
            // load. The post-RA scheduler may have set those kill
            // markers under the assumption that this LDY would re-define
            // the register; with the LDY going away, the kills become
            // load-bearing wrong — they tell the verifier that DataReg
            // is dead before any subsequent use, which would break the
            // chain.
            //
            // Walk backward from MI's predecessor, clearing kill flags
            // on uses of DataReg/aliased regs. Stop at any earlier def
            // (that's a fresh independent live range whose kills are
            // unrelated) or at the start of the BB. Skip non-real
            // instructions (DBG_VALUE/CFI/etc.).
            for (auto It = std::prev(MI.getIterator()),
                      Begin = MBB.instr_begin();;) {
              if (It->isDebugInstr() || It->isPosition()) {
                if (It == Begin) break;
                --It;
                continue;
              }
              bool DefsDataReg = false;
              for (MachineOperand &MO : It->operands()) {
                if (!MO.isReg() || !MO.getReg().isPhysical())
                  continue;
                if (!TRI->regsOverlap(MO.getReg(), Cls->DataReg))
                  continue;
                if (MO.isDef()) {
                  DefsDataReg = true;
                  // Don't clear kills on the def's own MI, but the def
                  // itself bounds the walk — kills before this point
                  // belong to a prior live range.
                } else if (MO.isKill()) {
                  MO.setIsKill(false);
                }
              }
              if (DefsDataReg) break;
              if (It == Begin) break;
              --It;
            }
            MI.eraseFromParent();
            ++NumStoreReloadsElided;
            Changed = true;
            // Even though elided, normalize tracking to single-slot
            // invariant. The slot already tracks DataReg, so this is
            // mostly a no-op, but it ensures any earlier-recorded
            // duplicate-tracking (from the time before this fix) gets
            // cleaned up.
            invalidateReg(Cls->DataReg);
            recordSlot(K, Cls->DataReg);
            continue;
          }
        }
        // Otherwise the load brings a fresh value into DataReg.
        invalidateReg(Cls->DataReg);
        recordSlot(K, Cls->DataReg);
      }
    }
  }
  return Changed;
}

// Class 6 — Branch-over-branch fold (bug #179).
//
// Pattern in real picolibc output (especially at -O0 where
// BranchFolderPass doesn't run, but also a small residual at -O2+):
//
//     LBlbc cc, L1     ; conditional branch
//     LBRAlb L2        ; unconditional branch
//   <next_block>:
//
// If `next_block == L1` (the conditional target IS the layout
// successor), invert the condition and retarget to L2, then drop the
// unconditional. Result:
//
//     LBlbc !cc, L2
//   L1:                ; (fall through)
//
// If `next_block == L2` (the unconditional's target IS the layout
// successor), the unconditional is purely redundant — drop it. The
// conditional stays unchanged. Result:
//
//     LBlbc cc, L1
//   L2:                ; (fall through; the conditional falls through
//                       ; to L2 when condition is false)
//
// If neither is the layout successor, the pair genuinely needs both
// branches and we leave it alone. (Such cases at -O2+ are
// MachineBlockPlacement-fundamental — they'd require deeper layout
// intervention to address. Out of scope here.)
//
// LLVM-standard equivalent: lib/CodeGen/BranchFolding.cpp does this
// fold at -O>=1, AND the analyzeBranch hook at MC6809InstrInfo.cpp:662
// implements the same shape inside the analysis itself when called
// with AllowModify=true. Both paths skip -O0, hence the need for this
// safety net here. Per-class gating defaults to min-O=0 so this fires
// at every -O level — duplicating BranchFolder's work at -O>=1 is
// harmless (we just no-op when nothing's left to fold) and provides
// defence-in-depth.
bool MC6809FinalLowering::foldBranchOverBranch(MachineFunction &MF) {
  auto isCondBranch = [](unsigned Opc) {
    // Bug #206 + #271 cat-1: Bbc_NoC / Bbc_OnlyC / LBlbc_NoC /
    // LBlbc_OnlyC are encoding-equivalent codegen-only variants that
    // just declare a tighter Uses set; treat identically.
    return Opc == MC6809::Bbc || Opc == MC6809::Bbc_NoC ||
           Opc == MC6809::Bbc_OnlyC || Opc == MC6809::Bbc_OnlyZ ||
           Opc == MC6809::LBlbc || Opc == MC6809::LBlbc_NoC ||
           Opc == MC6809::LBlbc_OnlyC || Opc == MC6809::LBlbc_OnlyZ ||
           Opc == MC6809::ConditionalBranchRelative ||
           Opc == MC6809::ConditionalLongBranchRelative;
  };
  auto isUncondBranch = [](unsigned Opc) {
    return Opc == MC6809::BRAb || Opc == MC6809::LBRAlb ||
           Opc == MC6809::BranchRelative ||
           Opc == MC6809::LongBranchRelative;
  };

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    // Find the last terminator (uncond branch) and the one before it
    // (must be a conditional branch). Skip blocks that don't end in
    // exactly this two-terminator shape.
    auto It = MBB.getLastNonDebugInstr();
    if (It == MBB.end() || !isUncondBranch(It->getOpcode()))
      continue;
    MachineBasicBlock::iterator UncondMI = It;

    // Walk back over any debug values to find the previous instruction.
    auto Prev = UncondMI;
    if (Prev == MBB.begin())
      continue;
    --Prev;
    while (Prev != MBB.begin() && Prev->isDebugInstr())
      --Prev;
    if (Prev->isDebugInstr() || !isCondBranch(Prev->getOpcode()))
      continue;
    MachineBasicBlock::iterator CondMI = Prev;

    // Operand layout (verified against analyzeBranch at line 602-613):
    //   Conditional: operand 0 = CC immediate, operand 1 = MBB target.
    //   Unconditional: operand 0 = MBB target.
    if (CondMI->getNumExplicitOperands() != 2 ||
        !CondMI->getOperand(0).isImm() || !CondMI->getOperand(1).isMBB() ||
        UncondMI->getNumExplicitOperands() != 1 ||
        !UncondMI->getOperand(0).isMBB())
      continue;

    MachineBasicBlock *L1 = CondMI->getOperand(1).getMBB();    // cond target
    MachineBasicBlock *L2 = UncondMI->getOperand(0).getMBB();  // uncond target
    MachineBasicBlock *LayoutNext = MBB.getNextNode();
    if (LayoutNext == nullptr)
      continue;

    if (LayoutNext == L2) {
      // Uncond goes to the next block — purely redundant. Drop it.
      UncondMI->eraseFromParent();
      ++NumBranchOverBranchFolded;
      Changed = true;
    } else if (LayoutNext == L1) {
      // Cond targets the next block — invert the CC, retarget the cond
      // to L2, drop the uncond. Use the existing getOppositeCondition
      // helper (see MC6809.h) which is already tested by analyzeBranch's
      // early-fold path.
      MC6809CC::CondCode CC =
          MC6809CC::CondCode(CondMI->getOperand(0).getImm());
      MC6809CC::CondCode Inverted = MC6809CC::getOppositeCondition(CC);
      // INVALID is the bail-out sentinel; if any branch we recognise
      // somehow has it, leave alone rather than miscompiling.
      if (Inverted == MC6809CC::INVALID)
        continue;
      CondMI->getOperand(0).setImm(Inverted);
      CondMI->getOperand(1).setMBB(L2);
      UncondMI->eraseFromParent();
      ++NumBranchOverBranchFolded;
      Changed = true;
    }
    // else: neither target is the layout successor — leave as-is.
  }
  return Changed;
}

char MC6809FinalLowering::ID = 0;

INITIALIZE_PASS(MC6809FinalLowering, DEBUG_TYPE,
                "MC6809 Final Lowering", false, false)

MachineFunctionPass *llvm::createMC6809FinalLoweringPass() {
  return new MC6809FinalLowering();
}
