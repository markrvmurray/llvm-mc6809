//===-- MC6809RegisterInfo.cpp - MC6809 Register Information --------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MC6809RegisterInfo.h"
#include "MC6809FrameLowering.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MC6809GenRegisterInfo.inc"

using namespace llvm;

MC6809RegisterInfo::MC6809RegisterInfo()
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0, /*PC=*/0, /*HwMode=*/0) {}

MC6809RegisterInfo::MC6809RegisterInfo(const Triple &TT)
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0, /*PC=*/0, /*HwMode=*/0) {}

BitVector MC6809RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  const MC6809FrameLowering *TFI = getFrameLowering(MF);

  // Mark special registers as reserved.
  Reserved.set(MC6809::PC);
  Reserved.set(MC6809::SS);
  Reserved.set(MC6809::DP);
  Reserved.set(MC6809::CC);
  Reserved.set(MC6809::A0);
  Reserved.set(MC6809::AV);
  Reserved.set(MC6809::MD);

  // Mark frame pointer as reserved if needed.
  if (TFI->hasFP(MF))
    Reserved.set(MC6809::SU);

  return Reserved;
}

const MCPhysReg *
MC6809RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const MC6809FrameLowering &TFI = *getFrameLowering(*MF);
  return MC6809_CSR_SaveList;
}

const uint32_t *
MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF, CallingConv::ID CallingConv) const {
  return MC6809_CSR_RegMask;
}

const TargetRegisterClass *
MC6809RegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MC6809::INDEX16RegClass)
    return &MC6809::ACC16RegClass;
  else if (RC == &MC6809::CCFlagRegClass)
    return &MC6809::ACC8RegClass;
  return RC;
}

bool
MC6809RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj, unsigned FIOperandNum, RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj");

  MachineInstr &MI = *II;
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  const MC6809FrameLowering *TFI = getFrameLowering(MF);
  DebugLoc dl = MI.getDebugLoc();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned BasePtr = (TFI->hasFP(MF) ? MC6809::SU : MC6809::SS);
  int Offset = MF.getFrameInfo().getObjectOffset(FrameIndex);

  Offset += 2; // Skip the saved PC

  if (!TFI->hasFP(MF))
    Offset += MF.getFrameInfo().getStackSize();
  else
    Offset += 2; // Skip the saved FP

  // Fold imm into offset
  Offset += MI.getOperand(FIOperandNum + 1).getImm();
  MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);
  return false;
}

Register
MC6809RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MC6809::SU : MC6809::SS;
}

int
copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) {
  const auto &TRI = *STI.getRegisterInfo();
  if (DestReg == SrcReg)
    return 0;

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  if (AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    if (AreClasses(MC6809::ADcRegClass, MC6809::ABcRegClass))
      return 0;
    if (AreClasses(MC6809::AWcRegClass, MC6809::AFcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    if (AreClasses(MC6809::ABcRegClass, MC6809::ADcRegClass))
      return 0;
    if (AreClasses(MC6809::AFcRegClass, MC6809::AWcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::ACC32RegClass, MC6809::ACC8RegClass)) {
    if (AreClasses(MC6809::AQcRegClass, MC6809::AFcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC32RegClass)) {
    if (AreClasses(MC6809::AFcRegClass, MC6809::AQcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::ACC32RegClass, MC6809::ACC16RegClass)) {
    if (AreClasses(MC6809::AQcRegClass, MC6809::AWcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC32RegClass)) {
    if (AreClasses(MC6809::AWcRegClass, MC6809::AQcRegClass))
      return 0;
    return 1;
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::ACC8RegClass)) {
    return 0;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::BIT1RegClass)) {
    return 0;
  } else if (AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)) {
    return 1;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass)) {
    return 1;
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::CCFlagRegClass) || AreClasses(MC6809::BIT1RegClass, MC6809::CARRYRegClass)) {
    return 16;
  } else if (AreClasses(MC6809::CCFlagRegClass, MC6809::BIT1RegClass) || AreClasses(MC6809::CARRYRegClass, MC6809::BIT1RegClass)) {
    return 16;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::AllArithFlagRegClass)) {
    return 4;
  } else if (AreClasses(MC6809::AllArithFlagRegClass, MC6809::ACC8RegClass)) {
    return 4;
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ArithFlagRegClass)) {
    return 8;
  } else if (AreClasses(MC6809::ArithFlagRegClass, MC6809::ACC8RegClass)) {
    return 8;
  }
  llvm_unreachable("Unexpected physical register copy cost.");
}

bool MC6809RegisterInfo::getRegAllocationHints(Register VirtReg, ArrayRef<MCPhysReg> Order, SmallVectorImpl<MCPhysReg> &Hints, const MachineFunction &MF, const VirtRegMap *VRM, const LiveRegMatrix *Matrix) const {
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  DenseMap<Register, int> RegScores;

  DenseMap<Register, int> OriginalIndex;
  for (const auto &R : enumerate(Order))
    OriginalIndex[R.value()] = R.index();

  SmallSet<const MachineInstr *, 32> Visited;
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(VirtReg)) {
    if (!Visited.insert(&MI).second)
      continue;
    switch (MI.getOpcode()) {
    default:
      continue;
    case MC6809::COPY: {
      const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg ? MI.getOperand(0) : MI.getOperand(1);
      const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg ? MI.getOperand(1) : MI.getOperand(0);
      Register OtherReg = Other.getReg();
      if (OtherReg.isVirtual()) {
        if (!VRM->hasPhys(OtherReg))
          break;
        OtherReg = VRM->getPhys(OtherReg);
      }
      if (Other.getSubReg())
        OtherReg = TRI.getSubReg(OtherReg, Other.getSubReg());
      int WorstCost = 0;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        WorstCost = std::max(WorstCost, copyCost(SelfReg, OtherReg, STI));
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        int Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost < WorstCost)
          RegScores[R] += WorstCost - Cost;
      }
      break;
    }
    }
  }

  SmallVector<std::pair<Register, int>> RegsAndScores(RegScores.begin(), RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, int> &A, const std::pair<Register, int> &B) {
    if (A.second > B.second)
      return true;
    if (A.second < B.second)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}
