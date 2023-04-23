//===-- MC6809PostRAScavenging.cpp - MC6809 Post RA Scavenging ------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 post-register-allocation register scavenging pass.
//
// This pass runs immediately after post-RA pseudo expansion. These pseudos
// (including COPY) often require temporary registers on MC6809; moreso than on
// other platforms. Accordingly, they emit virtual registers instead, and this
// pass performs register scavenging to assign them to physical registers,
// freeing them up via save and restore if neccesary. A very similar process is
// performed in prologue/epilogue insertion.
//
//===----------------------------------------------------------------------===//

#include "MC6809PostRAScavenging.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/RegisterScavenging.h"

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"

#define DEBUG_TYPE "mc6809-scavenging"

using namespace llvm;

namespace {

class MC6809PostRAScavenging : public MachineFunctionPass {
public:
  static char ID;

  MC6809PostRAScavenging() : MachineFunctionPass(ID) {
    llvm::initializeMC6809PostRAScavengingPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809PostRAScavenging::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(MachineFunctionProperties::Property::NoVRegs))
    return false;

  RegScavenger RS;
  scavengeFrameVirtualRegs(MF, RS);

  return true;
}

} // namespace

char MC6809PostRAScavenging::ID = 0;

INITIALIZE_PASS(MC6809PostRAScavenging, DEBUG_TYPE, "Scavenge virtual registers emitted by post-RA pseudos", false, false)

MachineFunctionPass *llvm::createMC6809PostRAScavengingPass() {
  return new MC6809PostRAScavenging();
}
