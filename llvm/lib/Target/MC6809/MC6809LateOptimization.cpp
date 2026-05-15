//===-- MC6809LateOptimization.cpp - MC6809 Late Optimization -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 late optimization pass.
//
// This pass performs simple optimizations once pseudo-instructions have been
// fully lowered. These optimizations might otherwise increase register pressure
// and cause spills, so they're done opportunistically at the very end.
//
//===----------------------------------------------------------------------===//

#include "MC6809LateOptimization.h"

#include "MC6809.h"
#include "MC6809InstrBuilder.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mc6809-late-opt"

using namespace llvm;

namespace {

class MC6809LateOptimization : public MachineFunctionPass {
public:
  static char ID;

  MC6809LateOptimization() : MachineFunctionPass(ID) { llvm::initializeMC6809LateOptimizationPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
  bool tailJMP(MachineBasicBlock &MBB) const;
  bool simplifyAndZero(MachineBasicBlock &MBB) const;
};

bool MC6809LateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= tailJMP(MBB);
    Changed |= simplifyAndZero(MBB);
  }
  return Changed;
}

// Bug #272 Phase B Scope A followup: convert `ANDA #0` / `ANDB #0` to
// `CLRA` / `CLRB`.  AND with immediate 0 always produces 0 regardless of
// the input, so the implicit USE of the accumulator is unnecessary —
// CLR is strictly better: 1 byte instead of 2, identical NZ/V/C flags,
// and no read of the (possibly undef) accumulator value.  Surfaced as
// a Og verifier hit at test-double-free.c:133 where the codegen emits
// `ANDA #0` to zero the high byte of a returned i16 / i8-zext value;
// post-Scope-A the implicit $aa read was no longer covered by the AQ
// over-claim and the verifier tripped on the undef read.
bool MC6809LateOptimization::simplifyAndZero(MachineBasicBlock &MBB) const {
  bool Changed = false;
  const TargetInstrInfo &TII =
      *MBB.getParent()->getSubtarget().getInstrInfo();
  for (auto It = MBB.begin(); It != MBB.end(); ) {
    auto NextIt = std::next(It);
    unsigned Opc = It->getOpcode();
    unsigned NewOpc = 0;
    if (Opc == MC6809::ANDAi8 && It->getOperand(0).isImm() &&
        It->getOperand(0).getImm() == 0)
      NewOpc = MC6809::CLRAa;
    else if (Opc == MC6809::ANDBi8 && It->getOperand(0).isImm() &&
             It->getOperand(0).getImm() == 0)
      NewOpc = MC6809::CLRBa;
    if (NewOpc != 0) {
      MachineInstr &MI = *It;
      DebugLoc DL = MI.getDebugLoc();
      BuildMI(MBB, MI, DL, TII.get(NewOpc));
      MI.eraseFromParent();
      Changed = true;
    }
    It = NextIt;
  }
  return Changed;
}

bool MC6809LateOptimization::tailJMP(MachineBasicBlock &MBB) const {
  if (MBB.size() < 2)
    return false;
  auto It = std::prev(MBB.end());
  if (It->getOpcode() != MC6809::RTSr)
    return false;
  MachineInstr &RTS = *It;
  --It;
  if (!(It->getOpcode() == MC6809::LBSRlb || It->getOpcode() == MC6809::BSRb))
    return false;
  MachineInstr &JSR = *It;
  RTS.eraseFromParent();
  JSR.setDesc(JSR.getMF()->getSubtarget().getInstrInfo()->get(MC6809::TailJump));
  return true;
}

} // namespace

char MC6809LateOptimization::ID = 0;

INITIALIZE_PASS(MC6809LateOptimization, DEBUG_TYPE, "MC6809 Late Optimizations", false, false)

MachineFunctionPass *llvm::createMC6809LateOptimizationPass() { return new MC6809LateOptimization; }
