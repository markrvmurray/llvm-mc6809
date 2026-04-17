//===-- MC6809NoShortBranches.cpp - Reject short branches -----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass strenuously objects to two classes of encoding error reaching the
// late pipeline:
//
// 1. SHORT BRANCHES (bug #58): short branches use an 8-bit signed offset
//    (-128..+127). When the branch target is further away, the offset wraps
//    and the CPU silently jumps to a wrong address.
//
// 2. INDEXED OFFSET OVERFLOW (bug #122): _o8 indexed instructions use an
//    8-bit signed offset from U or S. When the frame is larger than 127
//    bytes and an expansion emits _o8 for a slot beyond that range, the
//    offset wraps to negative and the instruction reads/writes from below
//    the frame, silently corrupting data. Similarly, _o5 instructions use
//    a 5-bit signed offset (-16..+15).
//
// Both checks run in addPreEmitPass. They are safety nets — the correct
// fix for each case is to emit the right encoding in the first place. But
// silent wrong-code from offset truncation is catastrophic, so these
// guards catch any regressions immediately.
//
//===----------------------------------------------------------------------===//

#include "MC6809NoShortBranches.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
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
  static void checkIndexedOffsetRange(const MachineInstr &MI,
                                      const MachineFunction &MF);
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

/// Check that any _o8 or _o5 indexed instruction's immediate offset fits
/// the encoding range. An out-of-range offset wraps via signed truncation,
/// silently accessing the wrong memory address (was bug #122).
void MC6809NoShortBranches::checkIndexedOffsetRange(
    const MachineInstr &MI, const MachineFunction &MF) {
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  StringRef Name = TII.getName(MI.getOpcode());

  int Lo, Hi;
  const char *Form;
  if (Name.ends_with("_o8")) {
    Lo = -128; Hi = 127; Form = "_o8 (8-bit signed)";
  } else if (Name.ends_with("_o5")) {
    Lo = -16; Hi = 15; Form = "_o5 (5-bit signed)";
  } else {
    return; // Not a short-offset indexed instruction.
  }

  // The immediate offset is the first operand that is an immediate.
  // _o8/_o5 instructions have layout: (implicit def), imm, reg.
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isImm())
      continue;
    int64_t Offset = MO.getImm();
    if (Offset < Lo || Offset > Hi) {
      std::string Msg;
      raw_string_ostream OS(Msg);
      OS << "MC6809: indexed offset " << Offset << " out of range ["
         << Lo << ".." << Hi << "] for " << Form
         << " instruction in function '" << MF.getName()
         << "': ";
      MI.print(OS);
      OS << "The expansion that produced this instruction should have "
            "used the _o16 variant for large offsets (was bug #122).";
      report_fatal_error(StringRef(Msg));
    }
    break; // Only check the first immediate (the offset).
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
      checkIndexedOffsetRange(MI, MF);
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
