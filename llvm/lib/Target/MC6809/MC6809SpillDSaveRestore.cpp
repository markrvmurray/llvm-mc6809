//===-- MC6809SpillDSaveRestore.cpp - Save/restore D around spill ops -----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Spill pseudo-register expansion routes through the D register. If a
// sub-register of D (typically B for i8 args) is live across spill operations,
// the expansion clobbers it. This pass saves live D sub-registers to a frame
// slot at function entry and restores them before each use point (typically
// a call with implicit $ab).
//
// Uses U-relative (frame pointer) addressing, which is unaffected by S changes.
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
    // Check if $ab (B register) is a live-in to this block.
    bool BLiveIn = MBB.isLiveIn(MC6809::AB) || MBB.isLiveIn(MC6809::AD);
    if (!BLiveIn)
      continue;

    // Scan for: (a) spill register operands, (b) calls with implicit $ab.
    bool HasSpillOps = false;
    SmallVector<MachineInstr *, 2> CallsUsingAB;

    for (MachineInstr &MI : MBB) {
      // Check for spill register operands.
      for (const MachineOperand &MO : MI.operands()) {
        if (MO.isReg() && MO.getReg().isPhysical() && isSpillReg(MO.getReg())) {
          HasSpillOps = true;
          break;
        }
      }
      // Check for calls with implicit $ab.
      if (MI.isCall()) {
        for (const MachineOperand &MO : MI.implicit_operands()) {
          if (MO.isReg() && MO.isUse() &&
              (MO.getReg() == MC6809::AB || MO.getReg() == MC6809::AD)) {
            CallsUsingAB.push_back(&MI);
            break;
          }
        }
      }
    }

    if (!HasSpillOps || CallsUsingAB.empty())
      continue;

    LLVM_DEBUG(dbgs() << "Spill-D-Save: B live-in with spill ops and call in "
                      << MBB.getName() << "\n");

    // Allocate a frame slot for saving D (2 bytes, U-relative).
    MachineFrameInfo &MFI = MF.getFrameInfo();
    int SaveSlot = MFI.CreateStackObject(2, Align(1), /*isSpillSlot=*/true);

    // Insert STD [SaveSlot] at the beginning of the block (after any frame
    // setup marked with FrameSetup flag).
    auto InsertPt = MBB.begin();
    while (InsertPt != MBB.end() && InsertPt->getFlag(MachineInstr::FrameSetup))
      ++InsertPt;
    // Also skip past TFR S,U (frame pointer setup) which isn't marked FrameSetup.
    while (InsertPt != MBB.end() && InsertPt->getOpcode() == MC6809::TFRp)
      ++InsertPt;

    DebugLoc DL;
    BuildMI(MBB, InsertPt, DL, TII.get(MC6809::Store_i16_Mem))
        .addReg(MC6809::AD)
        .addFrameIndex(SaveSlot)
        .addImm(0);

    // Insert LDD [SaveSlot] before each call that uses $ab.
    for (MachineInstr *Call : CallsUsingAB) {
      BuildMI(MBB, Call, Call->getDebugLoc(), TII.get(MC6809::Load_i16_Mem))
          .addReg(MC6809::AD, RegState::Define)
          .addFrameIndex(SaveSlot)
          .addImm(0);
    }

    Changed = true;
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
