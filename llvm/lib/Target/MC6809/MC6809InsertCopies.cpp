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
// STACK16 vreg → physical $ss replacement (was bug #55):
//   The TableGen-generated selector emits Push_i8/i16/Ptr instructions with
//   a STACK16 def operand (modeling the stack pointer side effect). STACK16
//   contains only $su and $ss, both of which are reserved (SS is the live
//   stack pointer, SU is the frame pointer — see MC6809RegisterInfo). The
//   register allocator therefore has no allocatable register for STACK16
//   vregs. Push/Pull always operate on S anyway, so we replace the vreg
//   reference with the physical $ss before RA runs.
//
//===----------------------------------------------------------------------===//

#include "MC6809InsertCopies.h"

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

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
  if (skipFunction(MF.getFunction()))
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;

  // Replace STACK16 virtual registers with physical $ss. Push/Pull pseudos
  // always operate on S, and STACK16 has no allocatable register (both $su
  // and $ss are reserved).
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB)
      for (MachineOperand &MO : MI.operands()) {
        if (!MO.isReg() || !MO.getReg().isVirtual())
          continue;
        if (MRI.getRegClassOrNull(MO.getReg()) == &MC6809::STACK16RegClass) {
          MO.setReg(MC6809::SS);
          Changed = true;
        }
      }

  return Changed;
}

} // namespace

char MC6809InsertCopies::ID = 0;

INITIALIZE_PASS(MC6809InsertCopies, DEBUG_TYPE, "MC6809 Copy Insertion", false, false)

MachineFunctionPass *llvm::createMC6809InsertCopiesPass() { return new MC6809InsertCopies; }
