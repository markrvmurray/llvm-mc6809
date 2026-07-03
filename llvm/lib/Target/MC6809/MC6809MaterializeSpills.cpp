//===-- MC6809MaterializeSpills.cpp - Preserve live regs over Q spills -----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// With the byte/word/index spill pseudo-registers retired (SPILL_A/B/D/X —
// their classes now spill through the stock frame-index path), this pass no
// longer materializes anything. What remains, until the i32 SPILL_Q tree is
// retired the same way, is the SPILL_Q support work:
//
//  * flag SPILL_Q usage so UsesSpillRegisters forces frame-pointer setup
//    at PEI time (a Q slot is addressed U-relative);
//  * preservation brackets: a SPILL_Q* operand materializes post-RA via
//    LDQ into $aq, clobbering all of $aq's sub-regs. When $aq — or one of
//    its halves — holds a live unrelated value at that point, save it to a
//    slot before the MI and reload it after (deferring byte reloads past a
//    compare's conditional branch, and into successors for terminators);
//  * BranchJumpTable's index-into-$ad materialization (the asm printer's
//    expansion hardcodes ASLB+ROLA on D).
//
//===----------------------------------------------------------------------===//

#include "MC6809MaterializeSpills.h"

#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-materialize-spills"

using namespace llvm;

/// Bug #301 (2026-05-22): return true if Reg is a 32-bit ACC spill
/// (SPILL_Q0..Q31), whose materialization uses LDQ and therefore
/// clobbers the entire $aq register (= $ad + $aw + their sub-bytes).
/// Mirrors isWideAccSpillReg but for the wider $aq class.  Half-word
/// (SPILL_Q*HI/LO) and byte (SPILL_Q*HIHI..LOLO) sub-reg forms are NOT
/// included — those materialize via narrower loads (LDD/LDA/LDB) and
/// are handled by the existing NeedSaveD/A/B paths.
static bool isQSpillReg(Register Reg) {
  return Reg >= MC6809::SPILL_Q0 && Reg <= MC6809::SPILL_Q31;
}

/// Bug #301 (2026-05-22): check if this instruction's spill operands
/// will cause $aq to be clobbered during materialization.
///
/// SPILL_Q* operands materialize via LDQ to $aq, clobbering the entire
/// 32-bit accumulator (and all its sub-regs $ad, $aw, $aa, $ab, $ae,
/// $af).  When $aq has a live value at this MI's pre-state, the LDQ
/// destroys it — Bug #301's root cause for the gmtime/timegm/ftello
/// miscompiles.
///
/// COPYs to/from physical $aq don't add an extra LDQ (the COPY itself
/// IS the materialization), so they're exempt — same logic as
/// willClobberD's COPY carve-out.
static bool willClobberQ(const MachineInstr &MI,
                         const TargetRegisterInfo &TRI) {
  // FAKE_USE generates no code — it never materialises its $spill_q*
  // operand into the physical $aq, so it clobbers nothing. (Bug #344:
  // without this, the new sub-reg save path below wraps a FAKE_USE and
  // re-breaks the Bug #256 garbage-liveness test.)
  if (MI.getOpcode() == TargetOpcode::FAKE_USE)
    return false;

  bool HasQSpill = false;
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isQSpillReg(MO.getReg()))
      HasQSpill = true;
  }
  if (!HasQSpill)
    return false;

  // COPYs: $aq↔SPILL_Q* — $aq IS the intended operand, the COPY's
  // materialization doesn't add an extra LDQ/STQ pair that wasn't
  // already needed by the COPY itself.
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (!isQSpillReg(DstReg) && TRI.regsOverlap(DstReg, MC6809::AQ))
      return false;
    if (!isQSpillReg(SrcReg) && TRI.regsOverlap(SrcReg, MC6809::AQ))
      return false;
  }

  return true;
}

namespace {

class MC6809MaterializeSpills : public MachineFunctionPass {
public:
  static char ID;

  MC6809MaterializeSpills() : MachineFunctionPass(ID) {
    initializeMC6809MaterializeSpillsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "MC6809 Materialize Spill Registers";
  }
};

bool MC6809MaterializeSpills::runOnMachineFunction(MachineFunction &MF) {
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  const auto &TII = *MF.getSubtarget().getInstrInfo();
  auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>();
  bool Changed = false;

  // Flag spill-register usage so PEI forces frame-pointer setup. Only the
  // SPILL_Q tree remains; its concrete frame slots are created by the
  // post-RA expansion of the spill itself.
  {
    bool AnySpillUsed = false;
    // Bug #316 (2026-05-21): also flag SPILL_Q*N usage.  The previous
    // version only checked SpillDRegs (i16-class) and SpillXRegs
    // (i16-pointer) and missed SPILL_Q[0..31] (i32-class spills).
    //
    // Without this check, a function that uses ONLY SPILL_Q (e.g.
    // picolibc's srandom, which does `_rand_next = seed` and needs an
    // i64-from-i16 zero-extension stash) leaves UsesSpillRegisters
    // false at determineCalleeSaves time.  hasFP then returns false
    // (no calls + no other trigger), so SavedRegs.set(SU) is NOT
    // called → PEI doesn't emit `pshs u`.  But emitPrologue later
    // sees hasFP=true (something else flips it post-PEI) and emits
    // `tfr s,u`, OVERWRITING caller's U without having saved it.
    //
    // The visible failure was Bug #315 (memcpy-1 SIGABRT at
    // -O2-hd6309-mame): main calls srand → srand calls srandom →
    // srandom corrupts U on return → cascades into main's U=0 →
    // all U-relative locals access garbage → memcpy ptr-check
    // fails → abort.
    //
    // SPILL_Q*N are 4 bytes each; the actual concrete frame slots
    // are created by the post-RA expansion of the spill itself.
    // Here we just need to flag SPILL_Q usage so UsesSpillRegisters
    // becomes true at PEI time.
    auto isSpillQUsedInFunction = [&]() -> bool {
      for (const MachineBasicBlock &MBB : MF)
        for (const MachineInstr &MI : MBB)
          for (const MachineOperand &MO : MI.operands())
            if (MO.isReg() && !MO.isImplicit() && MO.getReg().isPhysical()) {
              MCPhysReg R = MO.getReg();
              // 32-bit SPILL_Q full forms.
              if (R >= MC6809::SPILL_Q0 && R <= MC6809::SPILL_Q31)
                return true;
              // 16-bit SPILL_Q half-word sub-regs (high then low).
              if (R >= MC6809::SPILL_Q0HI && R <= MC6809::SPILL_Q31HI)
                return true;
              if (R >= MC6809::SPILL_Q0LO && R <= MC6809::SPILL_Q31LO)
                return true;
              // 8-bit SPILL_Q byte sub-regs (HI_HI .. LO_LO, 4 per slot).
              if (R >= MC6809::SPILL_Q0HIHI && R <= MC6809::SPILL_Q31LOLO)
                return true;
            }
      return false;
    };
    if (isSpillQUsedInFunction())
      AnySpillUsed = true;
    if (AnySpillUsed)
      FuncInfo->UsesSpillRegisters = true;
  }

  // Reusable save slots for preserving a live real register across a spill
  // materialisation. Each save is strictly bracketed (store before MI, reload
  // after MI, no nesting), so at most one save of each register-type is live at
  // any point — a single slot per type can be shared across the whole function.
  // Creating a fresh slot per materialisation (the old behaviour) bloated the
  // frame: these slots are minted after StackSlotColoring runs, so the coloring
  // pass never merges them (dozens of 2-byte slots for a hot function). This
  // matters especially for static-stack functions, where the frame lives in the
  // 64 KB global address space rather than on the runtime stack.
  // Only the D and Q save slots are shared. Their restores are always emitted
  // immediately after the (non-terminator) MI, so each save/restore is strictly
  // bracketed and no two are simultaneously live — one slot per type serves the
  // whole function. The A/B/IX/IY saves are NOT shared: their restores can be
  // deferred into successor blocks (a compare feeding a conditional branch, or a
  // terminator — see DeferByteRestore / insertPtr/ByteRestoreInSuccessors
  // below), so the save's live range spans BB boundaries and could overlap
  // another use of the same slot on a different path; those keep a fresh slot.
  int ReuseQSaveSlot = -1;
  int ReuseDSaveSlot = -1;
  auto getSaveSlot = [&](int &Cache, unsigned Size, bool CanReuse) -> int {
    if (!CanReuse)
      return MF.getFrameInfo().CreateStackObject(Size, Align(1), true);
    if (Cache == -1)
      Cache = MF.getFrameInfo().CreateStackObject(Size, Align(1), true);
    return Cache;
  };

  for (MachineBasicBlock &MBB : MF) {
    LivePhysRegs LPR(TRI);
    LPR.addLiveOuts(MBB);

    // Per-instruction: do we need D save/restore? IY save/restore?
    // NeedSaveA/NeedSaveB are narrower than NeedSaveD — they preserve only
    // one half of $ad around a pseudo whose expansion uses that half as a
    // scratch (bug #89). NeedSaveD saves both halves via STD/LDD.
    struct SpillInfo {
      bool NeedSaveD;
      bool NeedSaveA;
      bool NeedSaveB;
      // Bug #301 (2026-05-22): wider than NeedSaveD — saves the full
      // $aq via STQ/LDQ when SPILL_Q* materialization clobbers $aq.
      // NeedSaveD only preserves the AD sub-word (2 bytes); for SPILL_Q
      // we need all 4 bytes preserved (AD + AW).
      bool NeedSaveQ;
    };
    DenseMap<MachineInstr *, SpillInfo> NeedSave;

    for (MachineInstr &MI : llvm::reverse(MBB)) {
      LPR.stepBackward(MI);

      // Bug #91: BranchJumpTable requires its index operand (operand 0) in
      // $ad at the time the asm printer expands it (ASLB+ROLA+LEAX+LDD+JMP).
      // If regalloc placed the index in an imaginary register (RS0..RS3),
      // the Q-spill gate below won't catch it and we'd silently skip the
      // instruction. Handle it before that check: materialize the index
      // into $ad and rewrite the operand in place.
      if (MI.getOpcode() == MC6809::BranchJumpTable) {
        Register IdxReg = MI.getOperand(0).getReg();
        if (IdxReg != MC6809::AD) {
          DebugLoc DL = MI.getDebugLoc();
          if (IdxReg == MC6809::RS0 || IdxReg == MC6809::RS1 ||
              IdxReg == MC6809::RS2 || IdxReg == MC6809::RS3) {
            BuildMI(MBB, MI, DL, TII.get(MC6809::LDDd))
                .addReg(IdxReg);
          } else {
            llvm_unreachable("BranchJumpTable: unexpected index register");
          }
          MI.getOperand(0).setReg(MC6809::AD);
          Changed = true;
        }
        // Don't fall through to the spill-operand loop; we've handled
        // BranchJumpTable completely. Mark any remaining spill operands as
        // processed (there shouldn't be any — BranchJumpTable has only
        // the index + a JTI operand).
        continue;
      }

      // Only SPILL_Q* operands remain. They aren't materialized here (the
      // post-isel expansion handles SPILL_Q* via LDQ/STQ to the dedicated
      // stack slot), but the LDQ DOES clobber $aq — so the backward-pass
      // NeedSave analysis below must fire for these MIs.
      bool HasQSpill = false;
      for (const MachineOperand &MO : MI.operands()) {
        if (!MO.isReg() || !MO.getReg().isPhysical()) continue;
        if (isQSpillReg(MO.getReg())) {
          HasQSpill = true;
          break;
        }
      }
      if (!HasQSpill)
        continue;

      bool NeedD = false;
      bool NeedA = false, NeedB = false;
      bool NeedQ = false;

      // LivePhysRegs::contains(R) returns true if R or any sub-reg was added
      // to the live set, but NOT if only a super-reg was added (addReg adds
      // sub-regs only). When a byte-level value (e.g. AALSB) is the only
      // thing live, we still need to treat its parent byte (AA) as live —
      // clobbering AA destroys AALSB. Walk sub-regs explicitly.
      auto anySubRegLive = [&](MCPhysReg Reg) {
        if (LPR.contains(Reg))
          return true;
        for (MCPhysReg Sub : TRI.subregs(Reg))
          if (LPR.contains(Sub))
            return true;
        return false;
      };

      // Bug #301 (2026-05-22): SPILL_Q* materialization clobbers
      // $aq via LDQ.  If $aq (or any of its 6 sub-regs $ad/$aw/$aa/
      // $ab/$ae/$af) is live at the pre-state of this MI, the LDQ
      // destroys the value — Bug #301's gmtime/timegm/ftello shape.
      // Detect and request NeedSaveQ; the forward pass emits STQ/LDQ
      // brackets around MI.
      //
      // NeedSaveQ subsumes NeedSaveD/A/B for this MI (the STQ/LDQ
      // covers all sub-regs).  Skip the NeedD/A/B paths when NeedQ
      // fires so we don't emit redundant save/restore pairs.
      if (willClobberQ(MI, TRI)) {
        // Discriminate against false-positive sub-reg liveness: the
        // canonical Bug #301 shape has \$aq loaded as a unit (e.g.
        // `\$aq = Load_i32_Mem ...`), so LPR.contains(AQ) is true at
        // the pre-state.  Sub-reg-only liveness (e.g. \$ab live for an
        // unrelated TestBranch consumer) shouldn't fire NeedQ because
        // either the MI's own implicit-defs already account for the
        // clobber (SpillLoad_i32_Mem), or the sub-reg's value is
        // Bug #256-style garbage from an upstream SpillLoad killed by
        // FAKE_USE.  Tightening to LPR.contains(AQ) avoids both.
        if (LPR.contains(MC6809::AQ)) {
          NeedQ = true;
        } else if (anySubRegLive(MC6809::AW) &&
                   !MI.modifiesRegister(MC6809::AQ, &TRI)) {
          // Bug #344 (2026-05-25): the full $aq isn't live, but a sub-reg
          // HALF holds a live value the Q-clobber (LDQ into $aq) would
          // destroy. The $ad half is preserved cheaply by the
          // willClobberD/NeedD path below (now enabled for Q-clobbers);
          // there is no narrow NeedSaveW path for the $aw half ($ae/$af),
          // so fall back to a full NeedSaveQ when $aw is live (also covers
          // the both-halves-live case). Manifested as strtol returning a
          // NEGATED positive value: the conditional negate
          // `if (flags&FLAG_NEG) val=-val` keeps `flags&1` in $ad while
          // Sub_i32_Reg(spilled) materialises -val into $aq, and the
          // TestBranch then reads the clobbered $ad.
          NeedQ = true;
        }
      }

      // Bug #242-style exemption: tied-def of $aq is the intended
      // output.  If $aq is a tied DEF on this MI, the new $aq value
      // IS the result the consumer wants — wrapping with STQ/LDQ
      // would restore the PRE-MI value and destroy the result.
      if (NeedQ) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isDef() || MO.isImplicit() || MO.isDead())
            continue;
          if (!MO.getReg().isPhysical())
            continue;
          if (!TRI.regsOverlap(MO.getReg(), MC6809::AQ))
            continue;
          if (!MO.isTied())
            continue;
          NeedQ = false;
          break;
        }
      }

      // Bug #344 (2026-05-25): a Q-clobbering MI (Sub_i32_Reg etc. on
      // spilled $spill_q* operands) materialises into the physical $aq,
      // which clobbers the $ad half too. When only the $ad half holds a
      // live value (full $aq not live, so NeedQ stayed false above), the
      // cheap byte-granular STD/LDD save below preserves it — so run the
      // D-save analysis for Q-clobbers as well as D-clobbers.
      //
      // The `!modifiesRegister(AQ)` gate is the discriminator vs. Bug
      // #256: a MI that DECLARES it defs the $aq sub-regs (e.g.
      // SpillLoad_i32_Mem's implicit-def $aa/$ab/$ad/$aw) has its clobber
      // already accounted for by regalloc, so any sub-reg "liveness" past
      // it is killed garbage — don't save. Sub_i32_Reg & friends do NOT
      // declare the AQ scratch clobber (the root of #344), so a live
      // sub-reg there is a genuine value that must be preserved.
      bool QScratchClobber =
          willClobberQ(MI, TRI) && !MI.modifiesRegister(MC6809::AQ, &TRI);
      if (QScratchClobber && !NeedQ) {
        // Bug #313 (2026-05-22): refine NeedD to byte-granular when
        // only one half of $ad is live.  Previously: any sub-reg of
        // $ad being live → NeedD (16-bit STD/LDD save+restore).  When
        // only $ab is live (with an 8-bit value) and $aa is dead, the
        // 16-bit save preserves garbage in $aa across the
        // save/restore — latent vulnerability documented in Bug #313
        // (i1 PHI → i16 store leaks $aa garbage to memory through the
        // preservation slot).
        //
        // Discriminate: if both $aa and $ab are in the live set, save
        // 16-bit (covers either two separate 8-bit values OR a 16-bit
        // value).  If only one half is live, save just that half
        // (NeedSaveA/NeedSaveB).  The byte-level save infrastructure
        // is already in place (Bug #89 for SPILL_A/B operands); this
        // change just routes NeedD's discovery into the same byte
        // path when appropriate.
        bool ALive = LPR.contains(MC6809::AA);
        bool BLive = LPR.contains(MC6809::AB);
        bool DLive = LPR.contains(MC6809::AD);
        if (DLive || (ALive && BLive)) {
          NeedD = true;
        } else if (ALive) {
          NeedA = true;
        } else if (BLive) {
          NeedB = true;
        }
      }

      // Bug #242: when MI has an explicit, non-dead, tied DEF of $ad
      // (e.g. AND_i16_Reg / OR_i16_Reg / XOR_i16_Reg / Add_i16_Reg /
      // Sub_i16_Reg / AddSetCarry*_i16_Reg with $ad as op0), the new
      // value of $ad IS the result we want propagated. Wrapping such
      // an MI with STD-before / LDD-after would unconditionally restore
      // the PRE-MI value of $ad, destroying the def. The "needs save"
      // intuition assumes a side-effect clobber; tied-defs are NOT
      // side-effect clobbers — they ARE the operation's intended output.
      //
      // Concrete bug repro at -Os HD6309 (LTO and non-LTO both):
      //   $ad = AND_i16_Reg killed $ad(tied-def 0), killed $spill_d0
      //   $ix = COPY $ad
      // MaterializeSpills inserted STD/LDD around the AND, leaving the
      // COPY reading the pre-AND value. Test-malloc-stress at
      // Os-lto-hd6309-mame returns LSB-set "unaligned" addresses for
      // every memalign call as a result.
      if (NeedD) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isDef() || MO.isImplicit() || MO.isDead())
            continue;
          if (!MO.getReg().isPhysical())
            continue;
          if (!TRI.regsOverlap(MO.getReg(), MC6809::AD))
            continue;
          if (!MO.isTied())
            continue;
          NeedD = false;
          break;
        }
      }

      NeedSave[&MI] = {NeedD, NeedA, NeedB, NeedQ};
    }

    // Forward pass: materialize spill operands.
    MachineFrameInfo &MFI = MF.getFrameInfo();

    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      auto It = NeedSave.find(&MI);
      if (It == NeedSave.end())
        continue;

      SpillInfo &Info = It->second;
      DebugLoc DL = MI.getDebugLoc();

      int DSaveSlot = -1;
      int ASaveSlot = -1;
      int BSaveSlot = -1;
      int QSaveSlot = -1;  // Bug #301

      // Bug #301 (2026-05-22): save Q (full $aq) if needed.  Emit
      // STQ $aq → 4-byte stack slot before MI.  NeedSaveQ fires
      // when MI's SPILL_Q* operand materialization (LDQ to $aq)
      // would clobber a live $aq value.  Save is wider than NeedD
      // — must be 4 bytes to capture all of $aq's sub-regs.
      if (Info.NeedSaveQ) {
        QSaveSlot = getSaveSlot(ReuseQSaveSlot, 4, !MI.isTerminator());
        // Same Undef rationale as NeedSaveD (Bug #275): if $aq isn't
        // directly live-in at MBB.begin(), the save reads bits that
        // are part of a super-reg's value but not directly defined.
        // Mark Undef so the verifier accepts the read.
        RegState SrcFlags = RegState(0);
        bool AqDirectLiveIn = MBB.isLiveIn(MC6809::AQ);
        for (MCPhysReg Sub : TRI.subregs(MC6809::AQ))
          AqDirectLiveIn = AqDirectLiveIn || MBB.isLiveIn(Sub);
        if (MachineBasicBlock::iterator(MI) == MBB.begin() && !AqDirectLiveIn)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i32_Mem))
            .addReg(MC6809::AQ, SrcFlags)
            .addFrameIndex(QSaveSlot)
            .addImm(0);
      }

      // Save D if needed.
      if (Info.NeedSaveD) {
        DSaveSlot = getSaveSlot(ReuseDSaveSlot, 2, !MI.isTerminator());
        // Bug #275: NeedSaveD's backward LPR analysis treats $ad as
        // live whenever any super-reg of $ad (e.g. $aq) is live —
        // because LivePhysRegs::addReg(super) adds all sub-regs into
        // the set. In that case the save preserves $ad's bits as part
        // of the super-reg's value, but $ad's value AT THE SAVE POINT
        // may not be reachable from a direct $ad def upstream. The
        // verifier then flags the save's $ad use as an undefined
        // physical register. Probe true liveness via
        // computeRegisterLiveness; if $ad isn't definitely live,
        // mark the save's $ad operand Undef so the bits are still
        // round-tripped (preserving the super-reg's value) without
        // tripping the verifier.
        // Use Undef when the save is at MBB.begin() and $ad isn't a
        // direct MBB live-in. Critical-edge splitter BBs created later
        // would otherwise need to propagate $ad's liveness through
        // synthetic predecessors that don't have it, and the verifier
        // is unforgiving about that. We still emit the STD so $ad's
        // bits are preserved across the upcoming clobber (super-reg
        // value-preservation semantics); the Undef flag merely says
        // "we know the read may be undef".
        RegState SrcFlags = RegState(0);
        bool AdDirectLiveIn = MBB.isLiveIn(MC6809::AD);
        for (MCPhysReg Sub : TRI.subregs(MC6809::AD))
          AdDirectLiveIn = AdDirectLiveIn || MBB.isLiveIn(Sub);
        if (MachineBasicBlock::iterator(MI) == MBB.begin() && !AdDirectLiveIn)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i16_Mem))
            .addReg(MC6809::AD, SrcFlags)
            .addFrameIndex(DSaveSlot)
            .addImm(0);
      }

      // Bug #89: save just $aa or $ab (not both) around an 8-bit spill-
      // operand instruction whose expansion uses that half as a scratch.
      //
      // Bug #272 Phase B Scope A fallout: when the accumulator-hierarchy
      // Defs lists were cleaned up to drop the AQ/AD/AW over-claim, the
      // regalloc gained byte-granular packing freedom and surfaced
      // pre-existing cases where $aa / $ab is reachable as part of a live
      // super-reg (e.g. $aq) but is NOT a direct live-in / live-here. The
      // reverse-pass anySubRegLive(AA) check still marks NeedSaveA in such
      // cases — and rightly so, the save preserves the byte's bits as part
      // of the super-reg's value — but at the forward-pass save site the
      // verifier sees `implicit $aa` reading an undefined physical reg and
      // trips. Same shape as the AD case in Bug #275 (see above), one
      // level finer.
      //
      // Fix: probe true liveness at the save insertion point via
      // MachineBasicBlock::computeRegisterLiveness. If $aa / $ab is
      // definitely dead, mark the save's source operand Undef. The bits
      // are still round-tripped (the STA writes whatever is currently in
      // $aa and the matching LDA later reads it back), so the super-reg's
      // logical value is preserved across the upcoming clobber, and the
      // verifier is satisfied.
      if (Info.NeedSaveA) {
        ASaveSlot = MFI.CreateStackObject(1, Align(1), true);
        RegState SrcFlags = RegState(0);
        if (MBB.computeRegisterLiveness(&TRI, MC6809::AA, MI) !=
            MachineBasicBlock::LQR_Live)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AA, SrcFlags)
            .addFrameIndex(ASaveSlot)
            .addImm(0);
      }
      if (Info.NeedSaveB) {
        BSaveSlot = MFI.CreateStackObject(1, Align(1), true);
        RegState SrcFlags = RegState(0);
        if (MBB.computeRegisterLiveness(&TRI, MC6809::AB, MI) !=
            MachineBasicBlock::LQR_Live)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AB, SrcFlags)
            .addFrameIndex(BSaveSlot)
            .addImm(0);
      }

      // Nothing is materialized any more — SPILL_Q* operands are expanded
      // post-RA. Only the save/restore brackets are emitted here.
      auto After = std::next(MachineBasicBlock::iterator(MI));

      // Determine if A/B restores must be deferred to successors.
      // On the 6809, LDA/LDB set NZ flags. If we insert a byte restore
      // between a Compare (which sets CC) and its conditional branch
      // (which reads CC), the restore clobbers the comparison result.
      // This is the same CC-clobbering problem that led to the fused
      // CompareBranch pseudos (bugs #42, #59). The D and IY restores
      // (LDD, LDY) have the same NZ issue but are less common here.
      //
      // Detection: if the instruction defines CC and the NEXT instruction
      // is a conditional branch reading CC, defer A/B restores to the
      // branch's successor blocks (same path as terminators).
      bool DeferByteRestore = false;
      if (!MI.isTerminator() && (ASaveSlot >= 0 || BSaveSlot >= 0)) {
        if (MI.isCompare() ||
            MI.definesRegister(MC6809::CC, /*TRI=*/nullptr)) {
          auto NextIt = After;
          if (NextIt != MBB.end() && NextIt->isConditionalBranch())
            DeferByteRestore = true;
        }
      }

      if (!MI.isTerminator()) {
        // Restore saved registers after non-terminator instructions.
        // Bug #301 (2026-05-22): restore Q if saved.  LDQ slot → $aq
        // covers all sub-regs ($ad/$aw/$aa/$ab/$ae/$af).
        if (QSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_i32_Mem))
              .addReg(MC6809::AQ, RegState::Define)
              .addFrameIndex(QSaveSlot)
              .addImm(0);
        }
        if (DSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(DSaveSlot)
              .addImm(0);
        }
        if (!DeferByteRestore) {
          if (ASaveSlot >= 0) {
            BuildMI(MBB, After, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(MC6809::AA, RegState::Define)
                .addFrameIndex(ASaveSlot)
                .addImm(0);
          }
          if (BSaveSlot >= 0) {
            BuildMI(MBB, After, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(MC6809::AB, RegState::Define)
                .addFrameIndex(BSaveSlot)
                .addImm(0);
          }
        }
      }

      if (MI.isTerminator() || DeferByteRestore) {
        // For terminators (and deferred byte restores after compares):
        // insert the restore at the start of each successor block.
        //
        // D and IY restores go here ONLY for terminators. For deferred
        // byte restores (DeferByteRestore), only the A/B restores are
        // deferred — D and IY were already placed inline above.
        //
        // For multi-predecessor successors (critical edges), split the
        // edge by inserting a new block for the restore.
        // Bug #301 (2026-05-22): terminator-successor restore for Q.
        // Same structure as the D path above — single-predecessor
        // successors get the restore at the entry; critical-edge
        // successors get a new restore block on the edge.
        if (MI.isTerminator() && QSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i32_Mem))
                  .addReg(MC6809::AQ, RegState::Define)
                  .addFrameIndex(QSaveSlot)
                  .addImm(0);
            } else {
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i32_Mem))
                  .addReg(MC6809::AQ, RegState::Define)
                  .addFrameIndex(QSaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        }
        if (MI.isTerminator() && DSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i16_Mem))
                  .addReg(MC6809::AD, RegState::Define)
                  .addFrameIndex(DSaveSlot)
                  .addImm(0);
            } else {
              // Critical edge: split by inserting a new block.
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              // Update any branch targets in MBB that pointed to Succ.
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              // Transfer live-ins from the edge.
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              // Insert D restore and unconditional branch.
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i16_Mem))
                  .addReg(MC6809::AD, RegState::Define)
                  .addFrameIndex(DSaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        }
        // Bug #89 fix completion: A/B saves also need restoration in
        // successors when the instruction is a terminator. Without this,
        // a TestBranch with a spill operand that clobbers $ab/$aa
        // saves the live value but never restores it on the path
        // through the successor blocks. Exposed by bug #86's ACC8
        // relaxation for TestBranch_i8_Reg.
        auto insertByteRestoreInSuccessors = [&](int SaveSlot, Register ByteReg) {
          if (SaveSlot < 0) return;
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i8_Mem))
                  .addReg(ByteReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
            } else {
              // Critical edge: split by inserting a new block.
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i8_Mem))
                  .addReg(ByteReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        };
        insertByteRestoreInSuccessors(ASaveSlot, MC6809::AA);
        insertByteRestoreInSuccessors(BSaveSlot, MC6809::AB);
      }

      Changed = true;
    }

  }

  // Frame pointer ($su) liveins are added by emitPrologue (FrameLowering).
  // SU is unconditionally reserved (see MC6809RegisterInfo::getReservedRegs),
  // so the machine verifier accepts it as a livein without a def in this pass.

  return Changed;
}

char MC6809MaterializeSpills::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809MaterializeSpills, DEBUG_TYPE,
                "MC6809 Materialize Spill Registers", false, false)

MachineFunctionPass *llvm::createMC6809MaterializeSpillsPass() {
  return new MC6809MaterializeSpills();
}
