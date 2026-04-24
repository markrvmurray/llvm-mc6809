//===-- MC6809SanitiseDebugInfo.cpp - MC6809 debug-info sanitiser ---------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Bug #156: clang's debug-info widening of small-typed locals (e.g. rsize_t
// = size_t = i16 widened to s32 via DW_OP_LLVM_convert) can leave behind
// DBG_VALUE references to vregs that have no producer in the function. At
// -O1+ RemoveRedundantDebugValues prunes them; at -Og they survive into the
// register allocation pipeline and trip LiveVariables's
//   assert(MRI->getVRegDef(Reg) && "Register use before def!")
// at LiveVariables.cpp:159. While LiveVariables's runOnBlock is supposed
// to skip isDebugOrPseudoInstr instructions, the dangling-vreg case can
// still surface through downstream passes (FAKE_USE has isMeta but
// isDebugOrPseudoInstr returns false, etc.).
//
// This pass walks every DBG_VALUE / DBG_VALUE_LIST in the function and,
// for each register operand that's a virtual register with no defining
// instruction in MRI, replaces that operand with $noreg (the standard
// "undefined location" idiom). The variable becomes "optimised out" at
// the affected scope in the resulting DWARF — same outcome as -O1+
// already produces. The runtime value of the source-level variable lives
// on the narrow producer that does have a def; only the widened debugger
// view is sacrificed.
//
//===----------------------------------------------------------------------===//

#include "MC6809SanitiseDebugInfo.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mc6809-sanitise-debug-info"

using namespace llvm;

namespace {

class MC6809SanitiseDebugInfo : public MachineFunctionPass {
public:
  static char ID;

  MC6809SanitiseDebugInfo() : MachineFunctionPass(ID) {
    llvm::initializeMC6809SanitiseDebugInfoPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809SanitiseDebugInfo::runOnMachineFunction(MachineFunction &MF) {
  // Do not gate on skipFunction: -O0 stamps optnone everywhere and we
  // still need to run because LiveVariables runs at every opt level.
  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (!MI.isDebugValue() && !MI.isDebugValueList())
        continue;
      for (MachineOperand &MO : MI.debug_operands()) {
        if (!MO.isReg())
          continue;
        Register R = MO.getReg();
        if (!R.isVirtual())
          continue;
        if (MRI.def_empty(R)) {
          MO.ChangeToRegister(/*Reg=*/0, /*isDef=*/false);
          Changed = true;
        }
      }
    }
  }

  return Changed;
}

} // namespace

char MC6809SanitiseDebugInfo::ID = 0;

INITIALIZE_PASS(MC6809SanitiseDebugInfo, DEBUG_TYPE,
                "MC6809 Debug-Info Sanitiser", false, false)

MachineFunctionPass *llvm::createMC6809SanitiseDebugInfoPass() {
  return new MC6809SanitiseDebugInfo;
}
