//===-- MC6809NoShortBranches.cpp - Reject short branches -----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass strenuously objects to any short-branch instruction reaching the
// late pipeline. The MC6809 backend's policy is to emit only LONG branches
// until a future relaxation pass is written that can safely promote them to
// short branches when the offset is provably in range.
//
// Why: short branches use an 8-bit signed offset (-128..+127). When the
// branch target is further away than that, the offset wraps and the CPU
// silently jumps to a wrong address. This caused bug #58 (test_strstr stack
// corruption at -O0): a `bra .LBB_2` was emitted with a -142 offset, the
// short-branch encoding truncated it to +114, and execution wandered into
// unrelated code.
//
// To prevent regressions, this pass runs in addPreEmitPass right after the
// LLVM BranchRelaxation pass. If any short-branch opcode reaches it, the
// pass aborts with a fatal error naming the offending instruction. When the
// future relaxation pass lands, this guard can be loosened to only check
// after that pass has run.
//
//===----------------------------------------------------------------------===//

#include "MC6809NoShortBranches.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

#define DEBUG_TYPE "mc6809-no-short-branches"

using namespace llvm;

namespace {

class MC6809NoShortBranches : public MachineFunctionPass {
public:
  static char ID;

  MC6809NoShortBranches() : MachineFunctionPass(ID) {
    llvm::initializeMC6809NoShortBranchesPass(*PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override { return "MC6809 No Short Branches"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  static bool isShortBranch(unsigned Opcode);
};

bool MC6809NoShortBranches::isShortBranch(unsigned Opcode) {
  switch (Opcode) {
  // Short unconditional branch and never-branch (NOP-like).
  case MC6809::BRAb:
  case MC6809::BRNb:
  // Short conditional branch.
  case MC6809::Bbc:
  // Short subroutine call.
  case MC6809::BSRb:
  // The pseudo forms — we never expect to see these in late pipeline either,
  // because expandPostRAPseudo has already converted them to LBRA/LBcc.
  // List them anyway so we catch any pseudo that escapes expansion.
  case MC6809::BranchRelative:
  case MC6809::ConditionalBranchRelative:
  case MC6809::JumpRelative:
    return true;
  default:
    return false;
  }
}

bool MC6809NoShortBranches::runOnMachineFunction(MachineFunction &MF) {
  for (const MachineBasicBlock &MBB : MF) {
    for (const MachineInstr &MI : MBB) {
      if (isShortBranch(MI.getOpcode())) {
        std::string Msg;
        raw_string_ostream OS(Msg);
        OS << "MC6809: short branch reached late pipeline in function '"
           << MF.getName() << "' (BB " << MBB.getNumber() << "): ";
        MI.print(OS);
        OS << "MC6809 emits only long branches until a relaxation pass is "
              "written. Short branches risk silent offset truncation (was "
              "bug #58).";
        report_fatal_error(StringRef(Msg));
      }
    }
  }
  return false;
}

} // namespace

char MC6809NoShortBranches::ID = 0;

INITIALIZE_PASS(MC6809NoShortBranches, DEBUG_TYPE,
                "MC6809 No Short Branches", false, false)

MachineFunctionPass *llvm::createMC6809NoShortBranchesPass() {
  return new MC6809NoShortBranches();
}
