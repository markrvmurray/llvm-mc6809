//===-- MC6809CopyOpt.cpp - MC6809 Copy Optimization ----------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 pass to fully optimize COPY operations before
// lowering.
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
  return true;
}

char MC6809CopyOpt::ID = 0;

INITIALIZE_PASS(MC6809CopyOpt, DEBUG_TYPE, "Optimize copies for MC6809", false, false)

MachineFunctionPass *llvm::createMC6809CopyOptPass() { return new MC6809CopyOpt(); }
