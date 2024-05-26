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

static Register findForwardedCopy(MachineInstr &MI, SmallVectorImpl<MachineInstr *> &NewSrcMIs) {
  Register Src = MI.getOperand(1).getReg();
  Register NewSrc = 0;
  if (!findReachingDefs(MI, NewSrcMIs, [&](MachineInstr &Def) {
        if (!Def.isCopy())
          return false;
        Register Dst = Def.getOperand(0).getReg();
        if (Dst != Src)
          return false;
        Register NewSrcCand = Def.getOperand(1).getReg();
        if (NewSrc && NewSrc != NewSrcCand)
          return false;
        NewSrc = NewSrcCand;
        return true;
      })) {
    return 0;
  }
  return NewSrc;
}

static bool findLdImm(MachineInstr &MI, SmallVectorImpl<MachineInstr *> &LdImms) {
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MI.getMF()->getSubtarget().getInstrInfo();
  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();
  return findReachingDefs(MI, LdImms, [&](MachineInstr &Def) {
    if (!Def.isMoveImmediate())
      return false;
    if (Def.getOperand(0).getReg() != Src)
      return false;
    const TargetRegisterClass *RC = TII.getRegClass(Def.getDesc(), 0, &TRI, *MI.getMF());
    if (!RC->contains(Dst))
      return false;
    if (LdImms.empty())
      return true;
    return LdImms.front()->isIdenticalTo(Def);
  });
}

static bool isClobbered(MachineInstr &MI, Register NewSrc, const SmallVectorImpl<MachineInstr *> &NewSrcMIs) {
  const TargetRegisterInfo &TRI = *MI.getMF()->getSubtarget().getRegisterInfo();

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
      if (is_contained(NewSrcMIs, &MI)) {
        Found = true;
        break;
      }
      if (MI.modifiesRegister(NewSrc, &TRI))
        return true;
    }
    if (!Found)
      for (MachineBasicBlock *MBB : E.MBB.predecessors())
        WorkList.push_back({*MBB, MBB->rbegin()});
  }
  return false;
}

bool MC6809CopyOpt::runOnMachineFunction(MachineFunction &MF) {
#if 0
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const MC6809RegisterInfo &TRI = *STI.getRegisterInfo();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  auto CostMode = MC6809InstrCost::getModeFor(MF);

  LLVM_DEBUG(dbgs() << MF.getName() << "\n");

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (!MI.isCopy())
        continue;

      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();
      SmallVector<MachineInstr *> NewSrcMIs;
      Register NewSrc = findForwardedCopy(MI, NewSrcMIs);
      if (!NewSrc)
        continue;

      LLVM_DEBUG(dbgs() << MI);
      LLVM_DEBUG(dbgs() << "Found candidate: " << printReg(NewSrc, &TRI) << '\n');

      if (TRI.copyCost(Dst, NewSrc, STI).value(CostMode) > TRI.copyCost(Dst, Src, STI).value(CostMode)) {
        LLVM_DEBUG(dbgs() << "New copy is more expensive.\n");
        continue;
      }

      if (isClobbered(MI, NewSrc, NewSrcMIs)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

      LLVM_DEBUG(dbgs() << "Rewriting copy: " << MI);
      for (MachineInstr *NewSrcMI : NewSrcMIs)
        NewSrcMI->clearRegisterKills(NewSrc, &TRI);
      if (Dst == NewSrc) {
        LLVM_DEBUG(dbgs() << "Erased.\n");
        MI.eraseFromParent();
      } else {
        MI.getOperand(1).setReg(NewSrc);
        MI.getOperand(1).setIsKill(false);
        LLVM_DEBUG(dbgs() << "Rewrote to: " << MI);
      }
    }

    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (!MI.isCopy())
        continue;

      auto [Dst, Src] = MI.getFirst2Regs();
      auto LdImmCostVal = MC6809InstrCost(2, 2).value(CostMode);

      if (!MC6809::Imag16RegClass.contains(Dst) && Dst != MC6809::C && Dst != MC6809::V && TRI.copyCost(Dst, Src, STI).value(CostMode) <= LdImmCostVal)
        continue;

      SmallVector<MachineInstr *> LdImms;
      if (!findLdImm(MI, LdImms))
        continue;

      LLVM_DEBUG(dbgs() << MI);
      LLVM_DEBUG(dbgs() << "Found remat candidate: " << *LdImms.front());

      if (isClobbered(MI, LdImms.front()->getOperand(0).getReg(), LdImms)) {
        LLVM_DEBUG(dbgs() << "Clobbered.\n");
        continue;
      }

      for (MachineInstr *LdImm : LdImms)
        LdImm->clearRegisterKills(Src, &TRI);
      LdImms.front()->clearRegisterKills(Src, &TRI);
      TII.reMaterialize(MBB, MI, Dst, 0, *LdImms.front(), TRI);
      MI.eraseFromParent();
    }
  }

  for (MachineBasicBlock *MBB : post_order(&MF)) {
    LivePhysRegs LPR(TRI);

    recomputeLivenessFlags(*MBB);
    for (MachineInstr &MI : make_early_inc_range(*MBB)) {
      if (MI.isCopy() && MI.getOperand(0).isDead()) {
        LLVM_DEBUG(dbgs() << "Erasing dead copy: " << MI);
        MI.eraseFromParent();
      }
    }

    if (!MBB->isEntryBlock()) {
      recomputeLivenessFlags(*MBB);
      MBB->clearLiveIns();
      computeAndAddLiveIns(LPR, *MBB);
    }
  }
#endif
  return true;
}

char MC6809CopyOpt::ID = 0;

INITIALIZE_PASS(MC6809CopyOpt, DEBUG_TYPE, "Optimize copies for MC6809", false, false)

MachineFunctionPass *llvm::createMC6809CopyOptPass() { return new MC6809CopyOpt(); }
