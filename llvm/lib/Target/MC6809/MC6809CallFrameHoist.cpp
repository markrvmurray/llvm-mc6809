//===-- MC6809CallFrameHoist.cpp - Hoist out of call frames -----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 pass that moves computation out of a call's
// argument setup before legalization.
//
// Between ADJCALLSTACKDOWN and its call the arguments are stored into the
// outgoing-argument area at the bottom of the frame, addressed from S. Any
// operation there that the legalizer turns into a library call opens a second
// call sequence inside the first, and that call's arguments go into the same
// area -- over the ones the outer call has already stored. The machine
// verifier reports it as a FrameSetup after another FrameSetup.
//
// The call lowering itself only places argument moves there, but the
// combiners rebuild a value where it is used: the split of a 64-bit
// zero-extended argument became a 32-bit extend inside the frame, which the
// artifact combiner made an AND with 255, a library call on the 6309. So,
// just before legalization, every pure generic instruction in a call's
// argument setup whose operands are all available before the frame is moved
// to just before it. What stays is the argument placement proper: the copies
// of S, the addresses built from them, the stores and anything depending on
// them.
//
//===----------------------------------------------------------------------===//

#include "MC6809CallFrameHoist.h"

#include "MC6809.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/InitializePasses.h"

#define DEBUG_TYPE "mc6809-call-frame-hoist"

using namespace llvm;

STATISTIC(NumHoisted, "Instructions moved out of a call's argument setup");

namespace {

class MC6809CallFrameHoist : public MachineFunctionPass {
public:
  static char ID;

  MC6809CallFrameHoist() : MachineFunctionPass(ID) {
    initializeMC6809CallFrameHoistPass(*PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "MC6809 Call Frame Hoist";
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

// A constant or an address: never a library call, so it stays where it is
// unless an instruction being moved needs it.
static bool isLeaf(const MachineInstr &MI) {
  switch (MI.getOpcode()) {
  case TargetOpcode::G_CONSTANT:
  case TargetOpcode::G_FCONSTANT:
  case TargetOpcode::G_FRAME_INDEX:
  case TargetOpcode::G_GLOBAL_VALUE:
  case TargetOpcode::G_IMPLICIT_DEF:
    return true;
  default:
    return false;
  }
}

// A generic instruction that only computes a value from virtual registers and
// immediates: no memory access, no side effect, no physical register.
static bool isPureGeneric(const MachineInstr &MI) {
  if (!isPreISelGenericOpcode(MI.getOpcode()) || MI.isPHI() ||
      MI.mayLoadOrStore() || MI.hasUnmodeledSideEffects() || MI.isCall() ||
      MI.isTerminator())
    return false;
  for (const MachineOperand &MO : MI.operands())
    if (MO.isReg() && !MO.getReg().isVirtual())
      return false;
  return true;
}

bool MC6809CallFrameHoist::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  const unsigned SetupOpc = TII.getCallFrameSetupOpcode();
  bool Changed = false;

  const MachineRegisterInfo &MRI = MF.getRegInfo();
  for (MachineBasicBlock &MBB : MF) {
    MachineInstr *Setup = nullptr;
    // Virtual registers defined inside the open frame and left there, and
    // the constants and addresses defined there, which can follow an
    // instruction that needs them.
    SmallSet<Register, 16> StayingDefs;
    SmallSet<Register, 16> Leaves;
    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      if (MI.getOpcode() == SetupOpc) {
        Setup = &MI;
        StayingDefs.clear();
        Leaves.clear();
        continue;
      }
      if (!Setup)
        continue;
      if (MI.isCall()) {
        Setup = nullptr;
        continue;
      }
      if (MI.isDebugInstr())
        continue;
      if (isLeaf(MI)) {
        Leaves.insert(MI.getOperand(0).getReg());
        continue;
      }
      bool Hoist = isPureGeneric(MI) &&
                   none_of(MI.uses(), [&](const MachineOperand &MO) {
                     return MO.isReg() && StayingDefs.count(MO.getReg());
                   });
      if (Hoist) {
        for (const MachineOperand &MO : MI.uses()) {
          if (!MO.isReg() || !Leaves.erase(MO.getReg()))
            continue;
          MachineInstr *Leaf = MRI.getVRegDef(MO.getReg());
          MBB.splice(Setup->getIterator(), &MBB, Leaf->getIterator());
          ++NumHoisted;
        }
        MBB.splice(Setup->getIterator(), &MBB, MI.getIterator());
        ++NumHoisted;
        Changed = true;
        continue;
      }
      for (const MachineOperand &MO : MI.defs())
        if (MO.isReg() && MO.getReg().isVirtual())
          StayingDefs.insert(MO.getReg());
    }
  }
  return Changed;
}

char MC6809CallFrameHoist::ID = 0;

INITIALIZE_PASS(MC6809CallFrameHoist, DEBUG_TYPE,
                "Move computation out of MC6809 call argument setup", false,
                false)

MachineFunctionPass *llvm::createMC6809CallFrameHoistPass() {
  return new MC6809CallFrameHoist();
}
