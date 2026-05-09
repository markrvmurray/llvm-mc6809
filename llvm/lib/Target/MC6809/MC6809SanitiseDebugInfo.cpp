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
// Bug #243 (2026-05-09): the pass also erases DBG_VALUEs whose
// DIExpression contains DW_OP_LLVM_fragment. Bug #239's G_PHI s16 clamp
// on HD6309 causes the legalizer's narrowScalar to split a wide producer
// (G_ABS s64, G_MERGE_VALUES s64) that had a surviving DBG_VALUE,
// emitting fragment-bearing DBG_VALUEs that are valid at legalize time
// but later trip LiveDebugVariables::UserValue::insertDebugValue (called
// from VirtRegRewriter): the recombined expression fails the upstream
// `cast<DIExpression>(Expr)->isValid()` assert at MachineInstr.cpp:2414.
// Affects -Og -g HD6309 builds — eight picolibc functions (llabs,
// lldiv, imaxabs, imaxdiv, flsll, difftime, ubsan_val_to_imax,
// ubsan_val_to_umax) regain the ability to build at all; debug-info for
// the wide local is "optimised out" at the affected scope. Plain MC6809
// is unaffected because s64 already narrows long before #239's clamp.
//
//===----------------------------------------------------------------------===//

#include "MC6809SanitiseDebugInfo.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/IR/DebugInfoMetadata.h"
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

  SmallVector<MachineInstr *, 8> ToErase;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (!MI.isDebugValue() && !MI.isDebugValueList())
        continue;
      // Bug #243: erase DBG_VALUEs whose DIExpression contains
      // DW_OP_LLVM_fragment. These are emitted by the legalizer's
      // narrowScalar when it splits a wide producer (G_ABS s64,
      // G_MERGE_VALUES s64) that had a surviving DBG_VALUE. The fragments
      // themselves are valid at legalize time, but LiveDebugVariables's
      // UserValue::insertDebugValue (called from VirtRegRewriter) later
      // builds a recombined expression that fails the upstream
      // `cast<DIExpression>(Expr)->isValid()` assert at MachineInstr.cpp.
      // Affects -Og -g HD6309 builds (Bug #239's G_PHI s16 clamp surfaced
      // it). The narrowed runtime values still live in their narrow
      // producer registers; only the fragmented DWARF view is sacrificed.
      if (const DIExpression *Expr = MI.getDebugExpression()) {
        if (Expr->isFragment()) {
          ToErase.push_back(&MI);
          continue;
        }
      }
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
  for (MachineInstr *MI : ToErase) {
    MI->eraseFromParent();
    Changed = true;
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
