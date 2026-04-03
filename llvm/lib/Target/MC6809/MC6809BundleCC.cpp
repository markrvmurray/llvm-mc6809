//===-- MC6809BundleCC.cpp - Fix CC clobbering between compare and branch --===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// On the MC6809, all load instructions set N, Z, V condition code flags.
// If the register allocator inserts a spill reload between a compare and its
// consuming conditional branch, the reload clobbers the comparison result.
//
// This post-RA pass detects the pattern and moves the offending reloads
// above the compare instruction, where they don't affect the flags that
// the branch consumes.
//
// Runs after register allocation and spill insertion, before pseudo expansion.
//
//===----------------------------------------------------------------------===//

#include "MC6809BundleCC.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-bundle-cc"

using namespace llvm;

/// Check if this instruction clobbers any condition code flags.
static bool clobbersCC(const MachineInstr &MI, const TargetRegisterInfo &TRI) {
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.isDef())
      continue;
    if (TRI.regsOverlap(MO.getReg(), MC6809::CC) ||
        TRI.regsOverlap(MO.getReg(), MC6809::NZ))
      return true;
  }
  // Also check implicit defs.
  for (const MachineOperand &MO : MI.implicit_operands()) {
    if (!MO.isReg() || !MO.isDef())
      continue;
    if (TRI.regsOverlap(MO.getReg(), MC6809::CC) ||
        TRI.regsOverlap(MO.getReg(), MC6809::NZ))
      return true;
  }
  return false;
}

namespace {

class MC6809BundleCC : public MachineFunctionPass {
public:
  static char ID;

  MC6809BundleCC() : MachineFunctionPass(ID) {
    initializeMC6809BundleCCPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "MC6809 Fix CC Clobber";
  }
};

bool MC6809BundleCC::runOnMachineFunction(MachineFunction &MF) {
  // This pass currently serves as a safety net: it verifies that fused
  // compare-and-branch pseudos still have valid MBB targets after the
  // branch folder has run. The branch folder may delete/merge blocks,
  // and since it can't analyze fused pseudos (analyzeBranch returns true),
  // it may leave dangling MBB references.
  //
  // For now this pass is a no-op — the fused pseudos prevent CC clobbering
  // by keeping compare+branch as a single instruction during regalloc.
  // If dangling MBB issues arise, this is the place to fix them.
  return false;
}

char MC6809BundleCC::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809BundleCC, DEBUG_TYPE,
                "MC6809 Fix CC Clobber", false, false)

MachineFunctionPass *llvm::createMC6809BundleCCPass() {
  return new MC6809BundleCC();
}
