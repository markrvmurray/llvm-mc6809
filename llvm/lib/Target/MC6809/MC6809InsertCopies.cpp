//===-- MC6809InsertCopies.cpp - MC6809 Post-ISel Cleanup -----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines a post-instruction-selection cleanup pass.
//
//===----------------------------------------------------------------------===//

#include "MC6809InsertCopies.h"

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mc6809-insert-copies"

using namespace llvm;

namespace {

class MC6809InsertCopies : public MachineFunctionPass {
public:
  static char ID;

  MC6809InsertCopies() : MachineFunctionPass(ID) { llvm::initializeMC6809InsertCopiesPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809InsertCopies::runOnMachineFunction(MachineFunction &MF) {
  // Do not gate on skipFunction: at -O0 clang stamps optnone on every
  // function, which would skip the physreg pinning below.
  bool Changed = false;

  // i32 test pseudos (SignTest_i32 / EqZero_i32 / EqConst_i32) produce their
  // byte result in $ab while clobbering all of $aq (Defs=[NZ,V,C,AQ] — the
  // expansion's OR-chain / SUBW+SBCD genuinely destroys it). Every byte
  // accumulator is a sub-register of AQ, so a *virtual* ACC8 dst has no
  // register that can coexist with the declared clobber: regalloc sees the
  // dead AQ def interfering with every candidate and fails with "ran out of
  // registers" (surfaced when the byte classes lost their SPILL_* escape
  // registers). Model the hardware truth instead, x86-MUL style: pin the
  // dst operand to physical $ab (the byte the expansion leaves the result
  // in) and copy out to the original vreg. The coalescer collapses the COPY
  // whenever the consumer can take $ab directly.
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      switch (MI.getOpcode()) {
      case MC6809::SignTest_i32:
      case MC6809::EqZero_i32:
      case MC6809::EqConst_i32:
      case MC6809::CompareSet_i8_i32_Imm: {
        MachineOperand &DstMO = MI.getOperand(0);
        if (!DstMO.isReg() || !DstMO.getReg().isVirtual())
          break;
        Register DstVReg = DstMO.getReg();
        DstMO.setReg(MC6809::AB);
        BuildMI(MBB, std::next(MachineBasicBlock::iterator(MI)),
                MI.getDebugLoc(), TII.get(TargetOpcode::COPY), DstVReg)
            .addReg(MC6809::AB);
        Changed = true;
        break;
      }
      default:
        break;
      }
    }

  return Changed;
}

} // namespace

char MC6809InsertCopies::ID = 0;

INITIALIZE_PASS(MC6809InsertCopies, DEBUG_TYPE, "MC6809 Copy Insertion", false, false)

MachineFunctionPass *llvm::createMC6809InsertCopiesPass() { return new MC6809InsertCopies; }
