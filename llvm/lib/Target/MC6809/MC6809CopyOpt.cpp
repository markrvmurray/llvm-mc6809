//===-- MC6809CopyOpt.cpp - MC6809 Copy Optimization ----------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Post-RA copy optimization for MC6809.
//
// This pass runs after register allocation but before PostRAScavenging.
// It eliminates redundant COPY instructions that survive the standard
// MachineCopyPropagation pass — specifically:
//
//   1. Identity copies: $D = COPY $D → deleted
//   2. Dead copies: COPY whose destination is immediately overwritten
//      before any read → deleted
//   3. Copy-of-copy chains where the intermediate register is dead:
//      $X = COPY $D; $Y = COPY $X (X dead) → $Y = COPY $D
//
// The standard MachineCopyPropagation handles these generically, but may
// miss MC6809-specific cases involving the imaginary registers
// (RS0..RS3), which are pseudo physregs the generic pass doesn't always
// track.
//
//===----------------------------------------------------------------------===//

#include "MC6809CopyOpt.h"

#include "MC6809.h"
#include "MC6809InstrCost.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mc6809-copy-opt"

using namespace llvm;

namespace {

class MC6809CopyOpt : public MachineFunctionPass {
public:
  static char ID;

  MC6809CopyOpt() : MachineFunctionPass(ID) { llvm::initializeMC6809CopyOptPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

template <typename AcceptDefT> static bool findReachingDefs(MachineInstr &MI, SmallVectorImpl<MachineInstr *> &DefMIs, const AcceptDefT &AcceptDef) {
  assert(MI.isCopy());
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();

  Register Src = MI.getOperand(1).getReg();

  struct Entry {
    MachineBasicBlock &MBB;
    MachineBasicBlock::reverse_iterator I;
  };

  SmallVector<Entry> WorkList = {{*MI.getParent(), MachineBasicBlock::reverse_iterator(MI.getIterator())}};
  DenseSet<const MachineBasicBlock *> Seen;
  while (!WorkList.empty()) {
    Entry E = WorkList.back();
    WorkList.pop_back();
    if (Seen.contains(&E.MBB))
      continue;

    // Don't count the start MBB as seen until it's been seen as a predecessor.
    if (E.I == E.MBB.rbegin())
      Seen.insert(&E.MBB);

    bool Found = false;
    for (MachineInstr &MI : make_range(E.I, E.MBB.rend())) {
      if (!MI.modifiesRegister(Src, &TRI))
        continue;

      if (!AcceptDef(MI))
        return false;

      Found = true;
      DefMIs.push_back(&MI);
      break;
    }
    if (!Found) {
      // The register must have been live-in.
      if (E.MBB.isEntryBlock())
        return false;
      for (MachineBasicBlock *MBB : E.MBB.predecessors())
        WorkList.push_back({*MBB, MBB->rbegin()});
    }
  }
  return true;
}

bool MC6809CopyOpt::runOnMachineFunction(MachineFunction &MF) {
  const TargetRegisterInfo &TRI =
      *MF.getSubtarget().getRegisterInfo();
  bool Changed = false;

  SmallVector<MachineInstr *, 16> ToDelete;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();

      // 1. Identity copy elimination: $D = COPY $D
      if (Dst == Src) {
        LLVM_DEBUG(dbgs() << "MC6809CopyOpt: deleting identity copy: " << MI);
        ToDelete.push_back(&MI);
        continue;
      }

      // 2. Copy-of-copy chain shortening:
      //    If the source of this COPY was itself defined by a COPY,
      //    and the intermediate register has no other uses between
      //    the two COPYs, short-circuit: $Y = COPY $X; $Z = COPY $Y
      //    (Y dead) → $Z = COPY $X
      SmallVector<MachineInstr *, 4> Defs;
      bool Found = findReachingDefs(MI, Defs, [](const MachineInstr &DefMI) {
        return DefMI.isCopy();
      });

      if (Found && Defs.size() == 1) {
        MachineInstr *DefMI = Defs[0];
        Register OrigSrc = DefMI->getOperand(1).getReg();
        Register Intermediate = DefMI->getOperand(0).getReg();

        // Only attempt the chain shortening when both COPYs are in the
        // same basic block — cross-block walk is not safe with this
        // simple iterator approach.
        if (DefMI->getParent() != MI.getParent())
          continue;

        // Check that the intermediate register is dead after our COPY.
        if (MI.getOperand(1).isKill() && Intermediate == Src) {
          // Verify the original source is still live at our COPY's
          // position (it hasn't been clobbered between the two COPYs).
          bool Clobbered = false;
          MachineBasicBlock::iterator It(DefMI->getIterator());
          ++It; // skip the defining COPY
          MachineBasicBlock::iterator End(MI.getIterator());
          for (; It != End; ++It) {
            if (It->modifiesRegister(OrigSrc, &TRI)) {
              Clobbered = true;
              break;
            }
          }

          // Bug #307 round 2 (2026-05-18): the chain shortening
          // transfers the `kill` flag from $Intermediate onto
          // $OrigSrc.  This is only safe if $OrigSrc has no other
          // uses after MI — otherwise we'd prematurely kill a
          // value that's still live (e.g. the InstructionSelector's
          // `MaterializeCC_C_to_byte` bridge MI in imaxabs/llabs at
          // -Og-hd6309-mame has an implicit use of $phantom_carry_1
          // a few MIs after this COPY chain).
          //
          // Scan forward from MI to the end of the MBB looking for
          // any use of OrigSrc.  Abort the chain shortening if we
          // find one.  (Cross-MBB liveness conservatively assumed
          // alive — the cross-block walk is gated above anyway.)
          bool OrigSrcUsedLater = false;
          MachineBasicBlock::iterator ScanIt = std::next(MI.getIterator());
          MachineBasicBlock::iterator ScanEnd = MI.getParent()->end();
          for (; ScanIt != ScanEnd; ++ScanIt) {
            if (ScanIt->readsRegister(OrigSrc, &TRI)) {
              OrigSrcUsedLater = true;
              break;
            }
            // If $OrigSrc gets re-defined between MI and ScanIt,
            // any further uses read the new value, not the one we'd
            // be killing — so the kill flag transfer is safe.
            if (ScanIt->modifiesRegister(OrigSrc, &TRI))
              break;
          }

          if (!Clobbered && !OrigSrcUsedLater) {
            LLVM_DEBUG(dbgs() << "MC6809CopyOpt: shortening chain: " << MI
                              << "  original src: " << printReg(OrigSrc, &TRI)
                              << "\n");
            MI.getOperand(1).setReg(OrigSrc);
            Changed = true;
          }
        }
      }
    }
  }

  for (MachineInstr *MI : ToDelete) {
    MI->eraseFromParent();
    Changed = true;
  }

  return Changed;
}

char MC6809CopyOpt::ID = 0;

INITIALIZE_PASS(MC6809CopyOpt, DEBUG_TYPE, "Optimize copies for MC6809", false, false)

MachineFunctionPass *llvm::createMC6809CopyOptPass() { return new MC6809CopyOpt(); }
