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
#include "llvm/ADT/SmallSet.h"
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

MC6809RegisterInfo::MC6809RegisterInfo() : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0, /*PC=*/0, /*HwMode=*/0) {}

MC6809RegisterInfo::MC6809RegisterInfo(const Triple &TT) : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0, /*PC=*/0, /*HwMode=*/0) {}

BitVector MC6809RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  const MC6809FrameLowering *TFI = getFrameLowering(MF);

  // Mark special registers as reserved.
  Reserved.set(MC6809::PC);
  Reserved.set(MC6809::SS);
  Reserved.set(MC6809::DP);
  // Reserved.set(MC6809::CC);
  Reserved.set(MC6809::A0);
  Reserved.set(MC6809::AV);
  Reserved.set(MC6809::MD);

  // Mark frame pointer as reserved if needed.
  if (TFI->hasFP(MF))
    Reserved.set(MC6809::SU);

  return Reserved;
}

const MCPhysReg *MC6809RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const { return MC6809_CSR_SaveList; }

const uint32_t *MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF, CallingConv::ID CallingConv) const { return MC6809_CSR_RegMask; }

const TargetRegisterClass *MC6809RegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MC6809::INDEX16RegClass)
    return &MC6809::ACC16RegClass;
  else if (RC == &MC6809::CCFlagRegClass)
    return &MC6809::ACC8RegClass;
  return RC;
}

bool MC6809RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj, unsigned FIOperandNum, RegScavenger *RS) const {
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

Register MC6809RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MC6809::SU : MC6809::SS;
}

#if 0
int copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) {
  if (DestReg == SrcReg)
    return 0;

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) { return Dest.contains(DestReg) && Src.contains(SrcReg); };

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
#endif

bool MC6809RegisterInfo::getRegAllocationHints(Register VirtReg, ArrayRef<MCPhysReg> Order, SmallVectorImpl<MCPhysReg> &Hints, const MachineFunction &MF, const VirtRegMap *VRM, const LiveRegMatrix *Matrix) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  DenseMap<Register, MC6809InstrCost> RegScores;
  auto CostMode = MC6809InstrCost::getModeFor(MF);

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
      MC6809InstrCost WorstCost;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MC6809InstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) > WorstCost.value(CostMode))
          WorstCost = Cost;
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MC6809InstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) < WorstCost.value(CostMode))
          RegScores[R] += (WorstCost - Cost);
      }
      break;
    }
    }
  }

  SmallVector<std::pair<Register, MC6809InstrCost>> RegsAndScores(RegScores.begin(), RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, MC6809InstrCost> &A, const std::pair<Register, MC6809InstrCost> &B) {
    auto AVal = A.second.value(CostMode);
    auto BVal = B.second.value(CostMode);
    if (AVal > BVal)
      return true;
    if (AVal < BVal)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}

MC6809InstrCost MC6809RegisterInfo::copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) const {
  if (DestReg == SrcReg)
    return MC6809InstrCost();

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) { return Dest.contains(DestReg) && Src.contains(SrcReg); };

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : DestReg = "; dumpReg(DestReg););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest BIT1RegClass.contains(" << DestReg << ") = " << MC6809::BIT1RegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest CCFlagRegClass.contains(" << DestReg << ") = " << MC6809::CCFlagRegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest ACC8RegClass.contains(" << DestReg << ") = " << MC6809::ACC8RegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest ACC16RegClass.contains(" << DestReg << ") = " << MC6809::ACC16RegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest ACC32RegClass.contains(" << DestReg << ") = " << MC6809::ACC32RegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Dest INDEX16RegClass.contains(" << DestReg << ") = " << MC6809::INDEX16RegClass.contains(DestReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : SrcReg = "; dumpReg(SrcReg););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src BIT1RegClass.contains(" << SrcReg << ") = " << MC6809::BIT1RegClass.contains(SrcReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src CCFlagRegClass.contains(" << SrcReg << ") = " << MC6809::CCFlagRegClass.contains(SrcReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src ACC8RegClass.contains(" << SrcReg << ") = " << MC6809::ACC8RegClass.contains(SrcReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src ACC16RegClass.contains(" << SrcReg << ") = " << MC6809::ACC16RegClass.contains(SrcReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src ACC32RegClass.contains(" << SrcReg << ") = " << MC6809::ACC32RegClass.contains(SrcReg) << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src INDEX16RegClass.contains(" << SrcReg << ") = " << MC6809::INDEX16RegClass.contains(SrcReg) << "\n";);

  auto TransferCost = MC6809InstrCost(1, 2);
  auto Push8Cost = MC6809InstrCost(1, 3);
  auto Push16Cost = MC6809InstrCost(1, 4);
  auto Pop8Cost = MC6809InstrCost(1, 3);
  auto Pop16Cost = MC6809InstrCost(1, 4);
  auto ClVCost = MC6809InstrCost(1, 2);
  auto JumpCost = MC6809InstrCost(3, 4);
  auto BranchCost = MC6809InstrCost(2, 4);
  auto LoadImm8Cost = MC6809InstrCost(2, 2);
  auto LoadImm16Cost = MC6809InstrCost(2, 3);
  auto AluImm8Cost = MC6809InstrCost(2, 2);
  auto AluImm16Cost = MC6809InstrCost(2, 3);
  auto ImpossibleCost = MC6809InstrCost(32768, 32768);

  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    return TransferCost + TransferCost;
  }
  if (AreClasses(MC6809::ACC32RegClass, MC6809::ACC8RegClass)) {
    return TransferCost + TransferCost + TransferCost + TransferCost;
  }
  if (AreClasses(MC6809::Imag8RegClass, MC6809::ACC8RegClass)) {
    return MC6809InstrCost(2, 3);
  }
  if (AreClasses(MC6809::Imag16RegClass, MC6809::ACC16RegClass)) {
    return MC6809InstrCost(2, 4);
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::Imag8RegClass)) {
    return MC6809InstrCost(2, 3);
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::Imag16RegClass)) {
    return MC6809InstrCost(2, 4);
  }
  if (AreClasses(MC6809::Imag8RegClass, MC6809::Imag8RegClass)) {
    return Push8Cost + Pop8Cost + copyCost(DestReg, MC6809::AA, STI) + copyCost(MC6809::AA, SrcReg, STI);
  }
  if (AreClasses(MC6809::Imag16RegClass, MC6809::Imag16RegClass)) {
    return Push16Cost + Pop16Cost + copyCost(DestReg, MC6809::AD, STI) + copyCost(MC6809::AD, SrcReg, STI);
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::INDEX16RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC32RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::CCFlagRegClass) || AreClasses(MC6809::CCFlagRegClass, MC6809::ACC8RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    Register SrcReg8 = getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 = getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    auto BitCost = MC6809InstrCost(3, 4);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MC6809::C) {
        MC6809InstrCost Cost = AluImm8Cost;
        if (!MC6809::ACC8RegClass.contains(SrcReg))
          Cost += copyCost(MC6809::AA, SrcReg, STI);
        return Cost;
      }

      if (MC6809::ACC8RegClass.contains(SrcReg)) {
        return Push8Cost + Pop8Cost + BranchCost + BitCost + JumpCost + ClVCost;
      }
      return copyCost(MC6809::AA, SrcReg, STI) + BranchCost + BitCost + JumpCost + ClVCost;
    }
    if (DestReg8) {
      DestReg = DestReg8;

      Register Tmp = DestReg;
      if (!MC6809::ACC8RegClass.contains(Tmp))
        Tmp = MC6809::AA;
      MC6809InstrCost Cost = LoadImm8Cost * 2 + BranchCost;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
      return Cost;
    }
    return (Push8Cost + Pop8Cost) * 3 + AluImm8Cost + BranchCost + ClVCost;
  }

  llvm_unreachable("Unexpected physical register copy.");
}
