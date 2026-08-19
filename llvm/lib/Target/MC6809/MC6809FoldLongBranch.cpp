//===-- MC6809FoldLongBranch.cpp - Long conditional branches after relaxation ==//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// BranchRelaxation widens a conditional branch that cannot reach its target
// by inverting it over an unconditional long branch:
//
//     bcc L          ->    b!cc X        (2 bytes, 3 cycles)
//                          lbra L        (3 bytes, 5 cycles)
//                    X:
//
// (with the `lbra` on a block of its own when the fall-through had to be
// made explicit). The 6809 has long conditional branches (`lbcc L`: 4
// bytes, 5 cycles, 6 taken), which the generic pass has no hook to widen
// into. This pass runs after it and folds the pair back:
//
//     lbcc L         (4 bytes; 6 cycles when taken, 5 when not)
//                    X:
//
// One byte shorter always; faster when the far target is the likely one
// (a loop's back edge), slower when it is not (an error path: 5 cycles
// against 3 to fall through). So the fold takes the branch probability:
// always when optimising for size, otherwise when the far target is taken
// at least half the time.
//
// Every fold only shrinks code, so nothing relaxed before it goes out of
// range; the offsets recomputed afterwards then also let long branches
// whose target has come within reach shrink to the short forms.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/Support/BranchProbability.h"
#include "llvm/Support/CommandLine.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-fold-long-branch"

STATISTIC(NumFolded, "Number of branch-over-long-branch pairs folded");
STATISTIC(NumShrunk, "Number of long branches shrunk to short ones");

static cl::opt<bool> EnableFoldLongBranch("mc6809-enable-fold-long-branch",
                                          cl::init(true), cl::Hidden);

namespace {

class MC6809FoldLongBranch : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldLongBranch() : MachineFunctionPass(ID) {
    initializeMC6809FoldLongBranchPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold long branches";
  }
};

} // namespace

char MC6809FoldLongBranch::ID = 0;

INITIALIZE_PASS(MC6809FoldLongBranch, DEBUG_TYPE, "MC6809 fold long branches",
                false, false)

MachineFunctionPass *llvm::createMC6809FoldLongBranchPass() {
  return new MC6809FoldLongBranch();
}

// The long sibling of a short conditional branch (or the short of a long).
static unsigned longOf(unsigned Opc) {
  switch (Opc) {
  case MC6809::Bbc: return MC6809::LBlbc;
  case MC6809::Bbc_NoC: return MC6809::LBlbc_NoC;
  case MC6809::Bbc_OnlyC: return MC6809::LBlbc_OnlyC;
  case MC6809::Bbc_OnlyZ: return MC6809::LBlbc_OnlyZ;
  default: return 0;
  }
}
static unsigned shortOf(unsigned Opc) {
  switch (Opc) {
  case MC6809::LBlbc: return MC6809::Bbc;
  case MC6809::LBlbc_NoC: return MC6809::Bbc_NoC;
  case MC6809::LBlbc_OnlyC: return MC6809::Bbc_OnlyC;
  case MC6809::LBlbc_OnlyZ: return MC6809::Bbc_OnlyZ;
  case MC6809::LBRAlb: return MC6809::BRAb;
  default: return 0;
  }
}

// `b!cc X` ending MBB, `lbra L` alone in the next block (reached from MBB
// only), X the block after that: fold into `lbcc L` in MBB and drop the
// middle block. Also the same-block form `b!cc X ; lbra L` with X the next
// block.
static bool foldPair(MachineFunction &MF, MachineBasicBlock &MBB,
                     const MC6809InstrInfo &TII, bool OptSize) {
  auto Term = MBB.getFirstTerminator();
  if (Term == MBB.end() || !longOf(Term->getOpcode()) ||
      !Term->getOperand(1).isMBB())
    return false;
  MachineInstr &Bcc = *Term;
  MachineBasicBlock *X = Bcc.getOperand(1).getMBB();
  MachineInstr *LBra = nullptr;
  MachineBasicBlock *Mid = nullptr;
  auto Next = std::next(Term);
  if (Next != MBB.end()) {
    // Same block: b!cc X ; lbra L, X the layout successor.
    if (Next->getOpcode() != MC6809::LBRAlb || std::next(Next) != MBB.end() ||
        MBB.getNextNode() != X)
      return false;
    LBra = &*Next;
  } else {
    // Two blocks: b!cc X ; [Mid: lbra L] ; X.
    Mid = MBB.getNextNode();
    if (!Mid || Mid->pred_size() != 1 || Mid->getNextNode() != X ||
        Mid->size() != 1 || Mid->front().getOpcode() != MC6809::LBRAlb ||
        !MBB.isSuccessor(Mid))
      return false;
    LBra = &Mid->front();
  }
  MachineBasicBlock *L = LBra->getOperand(0).getMBB();
  if (L == X)
    return false;
  MC6809CC::CondCode CC = MC6809CC::CondCode(Bcc.getOperand(0).getImm());
  MC6809CC::CondCode Inv = MC6809CC::getOppositeCondition(CC);
  if (Inv == MC6809CC::INVALID)
    return false;
  // Worth it? Always for size; else when the far target is the likely one.
  if (!OptSize) {
    MachineBasicBlock *Far = Mid ? Mid : L;
    auto SI = llvm::find(MBB.successors(), Far);
    if (SI == MBB.succ_end())
      return false;
    if (MBB.getSuccProbability(SI) < BranchProbability(1, 2))
      return false;
  }
  // Rewrite: lbcc L, falling through to X.
  Bcc.setDesc(TII.get(longOf(Bcc.getOpcode())));
  Bcc.getOperand(0).setImm(Inv);
  Bcc.getOperand(1).setMBB(L);
  if (Mid) {
    auto P = MBB.getSuccProbability(llvm::find(MBB.successors(), Mid));
    MBB.removeSuccessor(Mid);
    MBB.addSuccessor(L, P);
    Mid->removeSuccessor(L);
    Mid->eraseFromParent();
  } else {
    LBra->eraseFromParent();
  }
  ++NumFolded;
  return true;
}

bool MC6809FoldLongBranch::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldLongBranch)
    return false;
  const auto &TII = *MF.getSubtarget<MC6809Subtarget>().getInstrInfo();
  const bool OptSize = MF.getFunction().hasOptSize();
  bool Changed = false;
  for (auto It = MF.begin(); It != MF.end();) {
    MachineBasicBlock &MBB = *It;
    Changed |= foldPair(MF, MBB, TII, OptSize); // may erase the next block
    It = std::next(MBB.getIterator());
  }
  // Long branches whose target is now (or always was) within short range.
  // Only shrinking happens, so offsets measured once stay conservative.
  DenseMap<const MachineBasicBlock *, unsigned> Offset;
  unsigned Off = 0;
  for (const MachineBasicBlock &MBB : MF) {
    Offset[&MBB] = Off;
    for (const MachineInstr &MI : MBB)
      Off += TII.getInstSizeInBytes(MI);
  }
  Off = 0;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      unsigned Size = TII.getInstSizeInBytes(MI);
      if (unsigned Short = shortOf(MI.getOpcode())) {
        const MachineOperand &Tgt = MI.getOperand(MI.getNumExplicitOperands() - 1);
        if (Tgt.isMBB()) {
          int64_t Rel = int64_t(Offset[Tgt.getMBB()]) - int64_t(Off);
          if (TII.isBranchOffsetInRange(Short, Rel)) {
            MI.setDesc(TII.get(Short));
            ++NumShrunk;
            Changed = true;
          }
        }
      }
      Off += Size;
    }
  }
  return Changed;
}
