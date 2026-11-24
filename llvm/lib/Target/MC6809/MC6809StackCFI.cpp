//===-- MC6809StackCFI.cpp - CFA notes for pushes in a function body ------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// In a function without a frame pointer the CFA is S plus a known offset, and
// the prologue and epilogue say how that offset moves.  The body moves S too:
// a temporary pushed and taken back (`pshs u ; addd ,s++`), a `leas`, a pull.
// Between such a push and its pop the offset the CFI gives is short by the
// pushed bytes, and a debugger stopped there reads the pushed value as the
// return address.
//
// This pass follows S through each block of such a function and notes every
// change with `.cfi_adjust_cfa_offset`.  It runs last, after every pass that
// creates, moves or merges instructions, so the notes describe the final code
// and cannot get in the way of a peephole.  Where blocks meet in a layout that
// does not follow the control flow (a block placed after a return, say), LLVM's
// CFIInstrInserter, which runs next, restates the CFA.
//
// A frame-setup or frame-destroy instruction is the prologue's or epilogue's,
// which describe it themselves.  A function with a frame pointer keeps its CFA
// on U and needs nothing.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/MC/MCDwarf.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-stack-cfi"

STATISTIC(NumAdjusted, "Number of CFA adjustments noted for stack moves in a function body");
STATISTIC(NumUntracked, "Number of body instructions that move S by an amount not known here");

namespace {

class MC6809StackCFI : public MachineFunctionPass {
public:
  static char ID;
  MC6809StackCFI() : MachineFunctionPass(ID) { initializeMC6809StackCFIPass(*PassRegistry::getPassRegistry()); }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override { return "MC6809 CFA notes for body stack moves"; }
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

} // namespace

char MC6809StackCFI::ID = 0;

INITIALIZE_PASS(MC6809StackCFI, DEBUG_TYPE, "MC6809 CFA notes for body stack moves", false, false)

MachineFunctionPass *llvm::createMC6809StackCFIPass() { return new MC6809StackCFI(); }

// Bytes a push or pull of Reg moves S by.
static int64_t pushedBytes(Register Reg, const TargetRegisterInfo &TRI) { return TRI.getSpillSize(*TRI.getMinimalPhysRegClass(Reg)); }

// How far MI moves S: negative for a push.  Known is cleared when MI moves S
// by an amount this cannot tell (S loaded or transferred, or offset by a
// register or from another one).
static int64_t stackDelta(const MachineInstr &MI, const TargetInstrInfo &TII, const TargetRegisterInfo &TRI, bool &Known) {
  Known = true;
  switch (MI.getOpcode()) {
  case MC6809::PSHSs:
  case MC6809::PULSs: {
    int64_t Bytes = 0;
    for (const MachineOperand &MO : MI.explicit_operands())
      if (MO.isReg() && MO.getReg() != MC6809::SS)
        Bytes += pushedBytes(MO.getReg(), TRI);
    return MI.getOpcode() == MC6809::PSHSs ? -Bytes : Bytes;
  }
  case MC6809::PSHSWx:
    return -2;
  case MC6809::PULSWx:
    return 2;
  default:
    break;
  }

  // `leas n,s` moves S by n; `leas ,s` not at all.  Any other `leas` -- from
  // another register, by a register, PC-relative -- sets S to something this
  // cannot follow.
  if (TII.getName(MI.getOpcode()).starts_with("LEAS")) {
    StringRef Name = TII.getName(MI.getOpcode());
    bool BaseS = false;
    const MachineOperand *Imm = nullptr;
    for (const MachineOperand &MO : MI.explicit_uses()) {
      if (MO.isReg())
        BaseS = MO.getReg() == MC6809::SS;
      else if (MO.isImm())
        Imm = &MO;
    }
    if (BaseS && (Name == "LEASi_o0" || (Imm && (Name == "LEASi_o5" || Name == "LEASi_o8" || Name == "LEASi_o16"))))
      return Imm ? Imm->getImm() : 0;
    Known = false;
    return 0;
  }

  // An indexed operand that steps S: `,s+` `,s++` `,-s` `,--s`.
  uint64_t TSFlags = MI.getDesc().TSFlags;
  if (MC6809::stepsIndexRegister(TSFlags))
    for (const MachineOperand &MO : MI.explicit_operands())
      if (MO.isReg() && MO.getReg() == MC6809::SS)
        return int64_t(MC6809::getPostIncrementBytes(TSFlags)) - int64_t(MC6809::getPreDecrementBytes(TSFlags));

  // A call pushes its return address and the callee takes it back.
  if (MI.isCall() || !MI.modifiesRegister(MC6809::SS, &TRI))
    return 0;
  Known = false;
  return 0;
}

bool MC6809StackCFI::runOnMachineFunction(MachineFunction &MF) {
  if (!MF.needsFrameMoves())
    return false;
  if (MF.getSubtarget().getFrameLowering()->hasFP(MF))
    return false;

  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (auto I = MBB.begin(), E = MBB.end(); I != E; ++I) {
      MachineInstr &MI = *I;
      if (MI.isMetaInstruction() || MI.getFlag(MachineInstr::FrameSetup) || MI.getFlag(MachineInstr::FrameDestroy))
        continue;
      bool Known;
      int64_t Delta = stackDelta(MI, TII, TRI, Known);
      if (!Known) {
        ++NumUntracked;
        LLVM_DEBUG(dbgs() << "mc6809-stack-cfi: S moved by an unknown amount in " << MF.getName() << ": " << MI);
        continue;
      }
      if (!Delta || MI.isTerminator())
        continue;
      // S fell by -Delta, so the CFA is that much further above it.
      unsigned CFIIndex = MF.addFrameInst(MCCFIInstruction::createAdjustCfaOffset(nullptr, -Delta));
      BuildMI(MBB, std::next(I), MI.getDebugLoc(), TII.get(TargetOpcode::CFI_INSTRUCTION)).addCFIIndex(CFIIndex);
      ++NumAdjusted;
      ++I; // past the note just inserted
      Changed = true;
    }
  }
  return Changed;
}
