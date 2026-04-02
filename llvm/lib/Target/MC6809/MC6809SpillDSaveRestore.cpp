//===-- MC6809SpillDSaveRestore.cpp - Save/restore D around spill ops -----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Spill pseudo-register expansion routes through D (LDD/STD). If D holds a
// live value at the point of a spill operation, the expansion silently
// clobbers it. This pass inserts Store_i16_Mem/Load_i16_Mem pairs to save
// and restore D around such instructions.
//
// Runs in addPrePEI — after register allocation (spill regs assigned) but
// before PEI and pseudo expansion. Uses LivePhysRegs for liveness analysis.
// Save/restore uses U-relative frame slots (unaffected by S changes).
//
//===----------------------------------------------------------------------===//

#include "MC6809SpillDSaveRestore.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-spill-d-save"

using namespace llvm;

static bool isSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3:
  case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3:
  case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3:
  case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
    return true;
  default:
    return false;
  }
}

/// Check if a register overlaps with D (AD, AA, AB).
static bool overlapsDReg(Register Reg, const TargetRegisterInfo &TRI) {
  return TRI.regsOverlap(Reg, MC6809::AD);
}

/// Check if a register is an INDEX spill (SPILL_X0..X3).
static bool isIndexSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_X0: case MC6809::SPILL_X1:
  case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return true;
  default:
    return false;
  }
}

/// Check if this instruction will clobber IX due to INDEX spill operands.
static bool willClobberX(const MachineInstr &MI, const TargetRegisterInfo &TRI) {
  bool HasIndexSpill = false;
  for (const MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg().isPhysical() && isIndexSpillReg(MO.getReg()))
      HasIndexSpill = true;
  }
  if (!HasIndexSpill)
    return false;

  // COPY IX↔SPILL_X: IX is the intended operand (no extra clobber).
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (!isIndexSpillReg(DstReg) && TRI.regsOverlap(DstReg, MC6809::IX))
      return false; // COPY defines IX from spill — IX is the output
    if (!isIndexSpillReg(SrcReg) && TRI.regsOverlap(SrcReg, MC6809::IX))
      return false; // COPY stores IX to spill — IX is being read
  }

  return true;
}

/// Check if this instruction's expansion will clobber D due to spill operands.
/// Returns false for INDEX→SPILL copies (use STX/LDX) and D→SPILL copies
/// (D is the intended operand).
static bool willClobberD(const MachineInstr &MI, const TargetRegisterInfo &TRI) {
  bool HasSpillOp = false;
  bool DIsDirectOperand = false;

  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isSpillReg(MO.getReg()))
      HasSpillOp = true;
    if (overlapsDReg(MO.getReg(), TRI))
      DIsDirectOperand = true;
  }

  if (!HasSpillOp)
    return false;

  // For COPYs, check specific safe cases.
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    // INDEX↔SPILL copies use STX/LDX directly — no D clobber.
    if (isSpillReg(DstReg) && (SrcReg == MC6809::IX || SrcReg == MC6809::IY))
      return false;
    if (isSpillReg(SrcReg) && (DstReg == MC6809::IX || DstReg == MC6809::IY))
      return false;
    // D↔SPILL: D is the intended operand.
    if (!isSpillReg(DstReg) && overlapsDReg(DstReg, TRI))
      return false;
    if (!isSpillReg(SrcReg) && overlapsDReg(SrcReg, TRI))
      return false;
  }

  // For non-COPY pseudos where D is a direct operand and there's no spill
  // that would clobber it: D is the intended operand, no save needed.
  // (If a spill is defined alongside D, the D clobber is from the spill
  // expansion going through D — the save IS needed.)
  if (!MI.isCopy() && DIsDirectOperand && !HasSpillOp)
    return false;

  return true;
}

namespace {

class MC6809SpillDSaveRestore : public MachineFunctionPass {
public:
  static char ID;

  MC6809SpillDSaveRestore() : MachineFunctionPass(ID) {
    initializeMC6809SpillDSaveRestorePass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "MC6809 Spill D Save/Restore";
  }
};

bool MC6809SpillDSaveRestore::runOnMachineFunction(MachineFunction &MF) {
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  const auto &TII = *MF.getSubtarget().getInstrInfo();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    LivePhysRegs LPR(TRI);
    LPR.addLiveOuts(MBB);

    // Walk backwards, collecting instructions that need D or IX save/restore.
    SmallVector<MachineInstr *, 4> NeedSaveD;
    SmallVector<MachineInstr *, 4> NeedSaveX;

    for (MachineInstr &MI : llvm::reverse(MBB)) {
      LPR.stepBackward(MI);

      // Check D clobber from ACC16 spill operations.
      if (willClobberD(MI, TRI)) {
        bool DLive = LPR.contains(MC6809::AD) ||
                     LPR.contains(MC6809::AA) ||
                     LPR.contains(MC6809::AB);
        if (DLive) {
          LLVM_DEBUG(dbgs() << "Spill-Save: D live at " << MI);
          NeedSaveD.push_back(&MI);
        }
      }

      // Check IX clobber from INDEX spill operations.
      if (willClobberX(MI, TRI)) {
        bool XLive = LPR.contains(MC6809::IX);
        if (XLive) {
          LLVM_DEBUG(dbgs() << "Spill-Save: IX live at " << MI);
          NeedSaveX.push_back(&MI);
        }
      }
    }

    MachineFrameInfo &MFI = MF.getFrameInfo();

    // Save/restore D around ACC16 spill operations.
    if (!NeedSaveD.empty()) {
      int DSaveSlot = MFI.CreateStackObject(2, Align(1), /*isSpillSlot=*/true);
      for (MachineInstr *MI : NeedSaveD) {
        DebugLoc DL = MI->getDebugLoc();
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i16_Mem))
            .addReg(MC6809::AD)
            .addFrameIndex(DSaveSlot)
            .addImm(0);
        auto After = std::next(MachineBasicBlock::iterator(MI));
        BuildMI(MBB, After, DL, TII.get(MC6809::Load_i16_Mem))
            .addReg(MC6809::AD, RegState::Define)
            .addFrameIndex(DSaveSlot)
            .addImm(0);
        Changed = true;
      }
    }

    // Save/restore IX around INDEX spill operations.
    // Uses Store_iPtr_Mem / Load_iPtr_Mem (STX/LDX, no D clobber).
    if (!NeedSaveX.empty()) {
      int XSaveSlot = MFI.CreateStackObject(2, Align(1), /*isSpillSlot=*/true);
      for (MachineInstr *MI : NeedSaveX) {
        DebugLoc DL = MI->getDebugLoc();
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_iPtr_Mem))
            .addReg(MC6809::IX)
            .addFrameIndex(XSaveSlot)
            .addImm(0);
        auto After = std::next(MachineBasicBlock::iterator(MI));
        BuildMI(MBB, After, DL, TII.get(MC6809::Load_iPtr_Mem))
            .addReg(MC6809::IX, RegState::Define)
            .addFrameIndex(XSaveSlot)
            .addImm(0);
        Changed = true;
      }
    }
  }

  return Changed;
}

char MC6809SpillDSaveRestore::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809SpillDSaveRestore, DEBUG_TYPE,
                "MC6809 Spill D Save/Restore", false, false)

MachineFunctionPass *llvm::createMC6809SpillDSaveRestorePass() {
  return new MC6809SpillDSaveRestore();
}
