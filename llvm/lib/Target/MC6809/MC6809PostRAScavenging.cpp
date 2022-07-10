//===-- MC6809PostRAScavenging.cpp - MC6809 Post RA Scavenging ------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 post-register-allocation register scavenging
// pass.
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

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#define DEBUG_TYPE "mc6809-scavenging"

using namespace llvm;

namespace {

class MC6809PostRAScavenging : public MachineFunctionPass {
public:
  static char ID;

  MC6809PostRAScavenging() : MachineFunctionPass(ID) {
    llvm::initializeMC6809PostRAScavengingPass(
        *PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809PostRAScavenging::runOnMachineFunction(MachineFunction &MF) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MC6809PostRAScavenging : MF = "; MF.dump(););
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  if (MF.getProperties().hasProperty(MachineFunctionProperties::Property::NoVRegs))
    return false;

  // Protect NZ from the scavenger by bundling.
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB)
      if (MI.definesRegister(MC6809::NZVC)) {
        // Branch folding may have eliminated the use of N, Z, V, C
        auto Succ = ++MachineBasicBlock::iterator(MI);
        if (Succ != MBB.end() && Succ->readsRegister(MC6809::NZVC, &TRI)) {
          MI.bundleWithSucc();
          for (MachineOperand &MO : Succ->operands())
            if (MO.isReg() &&
                (MO.getReg() == MC6809::N || MO.getReg() == MC6809::Z))
              MO.setIsInternalRead();
        }
      }

  RegScavenger RS;
  scavengeFrameVirtualRegs(MF, RS);

  // Once all virtual registers are scavenged, nothing else in the pipeline can
  // be inserted between NZ defs and uses.
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB.instrs())
      if (MI.isBundledWithPred()) {
        MI.unbundleFromPred();
        for (MachineOperand &MO : MI.operands())
          if (MO.isReg() && MO.isInternalRead())
            MO.setIsInternalRead(false);
      }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MC6809PostRAScavenging : MF = "; MF.dump(););
  return true;
}

} // namespace

char MC6809PostRAScavenging::ID = 0;

INITIALIZE_PASS(MC6809PostRAScavenging, DEBUG_TYPE,
                "Scavenge virtual registers emitted by post-RA pseudos", false,
                false)

MachineFunctionPass *llvm::createMC6809PostRAScavengingPass() {
  return new MC6809PostRAScavenging();
}
