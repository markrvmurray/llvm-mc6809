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
#include "MC6809ValueTracker.h"
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

bool MC6809FinalLowering::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;

  // Class 6 runs FIRST and is NOT gated by skipFunction: at -O0 every
  // function carries `optnone`, which makes skipFunction return true, and
  // Class 6 backstops BranchFolderPass, which does not run at -O0.
  if (enabled(MF, BranchOverBranchMinO))
    Changed |= foldBranchOverBranch(MF);

  // Classes 1 and 5 are size/speed optimisations and respect optnone.
  if (skipFunction(MF.getFunction()))
    return Changed;

  if (enabled(MF, RelaxOffsetsMinO))
    Changed |= relaxOffsets(MF);
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

bool MC6809FinalLowering::elideStoreReload(MachineFunction &MF) {
  // Delete a reload of a slot whose value its register still holds, by the
  // shared MC6809ValueTracker's rules: a store drops every mirror it may
  // alias (same-base byte overlap; other bases by what the memory operands
  // say), a def of the holder or the base drops a mirror, and calls, returns,
  // branches and inline asm drop everything. Kills between the establishing
  // access and the deleted reload are cleared.
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    MC6809ValueTracker T(MF, TRI);
    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      if (MI.isCall() || MI.isInlineAsm() || MI.isReturn() ||
          MI.isBranch() || MI.isTerminator()) {
        T.clear();
        continue;
      }
      // A plain indexed load or store we can key; a volatile one is
      // observable and takes no part (it is still a store, below).
      auto LS = MC6809InstrInfo::getIdxLoadStore(MI.getOpcode());
      if (LS && MC6809InstrInfo::hasOrderedMemOperand(MI))
        LS.reset();
      Register Base;
      int64_t Off = 0;
      bool Keyed = LS && MC6809InstrInfo::getIdxBaseOffset(MI, Base, Off);

      if (MI.mayStore())
        T.storeAlias(MI, Keyed && LS->IsStore, Base);

      if (Keyed && LS->IsStore) {
        T.storeOverlap(Base, Off, MC6809InstrInfo::getValueRegBytes(LS->Value),
                       LS->Value);
        T.set({Base, static_cast<int>(Off)}, LS->Value, &MI, MI);
        continue;
      }

      if (Keyed) {
        MC6809ValueTracker::Key K{Base, static_cast<int>(Off)};
        // `ldy ,y` names a slot through the register it overwrites.
        bool SelfBase = TRI.regsOverlap(Base, LS->Value);
        if (!SelfBase)
          if (MC6809ValueTracker::Mirror *M = T.find(K);
              M && M->Reg == LS->Value) {
            MC6809ValueTracker::clearKillsToPriorDef(MI, LS->Value, TRI);
            MI.eraseFromParent();
            ++NumStoreReloadsElided;
            Changed = true;
            continue;
          }
        T.clearHolder(LS->Value);
        T.clearBase(LS->Value);
        if (!SelfBase)
          T.set(K, LS->Value, nullptr, MI);
        continue;
      }

      for (const MachineOperand &MO : MI.operands())
        if (MO.isReg() && MO.isDef() && MO.getReg().isPhysical()) {
          T.clearHolder(MO.getReg());
          T.clearBase(MO.getReg());
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
