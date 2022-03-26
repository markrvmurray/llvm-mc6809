//===-- MC6809RegisterInfo.cpp - MC6809 Register Information
//--------------------===//
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
#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
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
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                            /*PC=*/0, /*HwMode=*/0) {}

BitVector MC6809RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  const MC6809FrameLowering *TFI = getFrameLowering(MF);

  // Mark special registers as reserved.
  Reserved.set(MC6809::PC);
  Reserved.set(MC6809::SS);
  Reserved.set(MC6809::DP);
  Reserved.set(MC6809::CC);
  Reserved.set(MC6809::A0);

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
MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                         CallingConv::ID CallingConv) const {
  return MC6809_CSR_RegMask;
}

const TargetRegisterClass *
MC6809RegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                              const MachineFunction &) const {
  if (RC->hasSuperClass(&MC6809::BIT1RegClass))
    return &MC6809::ACC16RegClass;
  if (RC->hasSuperClass(&MC6809::BIT8RegClass))
    return &MC6809::ACC16RegClass;
  if (RC->hasSuperClass(&MC6809::ACC8RegClass))
    return &MC6809::ACC16RegClass;
  return RC;
}

const TargetRegisterClass *
MC6809RegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MC6809::INDEX16RegClass)
    return &MC6809::ACC16RegClass;
  return RC;
}

// These values were chosen empirically based on the desired behavior of llc
// test cases. These values will likely need to be retuned as more examples come
// up.  Unfortunately, the way the register allocator actually uses this is very
// heuristic, and if tuning these params doesn't suffice, we'll need to build a
// more sophisticated analysis into the register allocator.
unsigned
MC6809RegisterInfo::getCSRFirstUseCost(const MachineFunction &MF) const {
  if (MF.getFunction().doesNotRecurse()) {
    return 15 * 16384 / 10;
  }
  return 5 * 16384 / 10;
}

bool MC6809RegisterInfo::saveScavengerRegister(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
    MachineBasicBlock::iterator &UseMI, const TargetRegisterClass *RC,
    Register Reg) const {
  // Note: NZVC cannot be live at this point, since it's only live in
  // terminators, and virtual registers are never inserted into terminators.

  // Consider the regions in a basic block where a physical register is live.
  // The register scavenger will select one of these regions to spill and mark
  // the physical register as available within that region. Such a region cannot
  // contain any calls, since the physical registers are clobbered by calls.
  // This means that a save/restore pair for that physical register cannot
  // overlap with any other save/restore pair for the same physical register.

  MachineIRBuilder Builder(MBB, I);
  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  switch (Reg) {
  default:
    errs() << "Register: " << getName(Reg) << "\n";
    report_fatal_error("Scavenger spill for register not yet implemented.");
  case MC6809::CC:
  case MC6809::AA:
  case MC6809::AB:
  case MC6809::AD:
  case MC6809::IX:
  case MC6809::IY:
  case MC6809::SU: {
    Builder.buildInstr(MC6809::PH, {}, {Reg});
    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MC6809::PL, {Reg}, {});
    break;
  }
  }

  return true;
}

bool MC6809RegisterInfo::canSaveScavengerRegister(Register Reg) const {
  return true;
}

void MC6809RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                             int SPAdj, unsigned FIOperandNum,
                                             RegScavenger *RS) const {
  MachineFunction &MF = *MI->getMF();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  assert(!SPAdj);

  int Idx = MI->getOperand(FIOperandNum).getIndex();
  int64_t Offset = MFI.getObjectOffset(Idx);
  if (FIOperandNum + 1 < MI->getNumOperands() &&
      MI->getOperand(FIOperandNum + 1).isImm())
    Offset += MI->getOperand(FIOperandNum + 1).getImm();
  else
    Offset += MI->getOperand(FIOperandNum).getOffset();

  if (MFI.getStackID(Idx) == TargetStackID::Default) {
    // All offsets are relative to the incoming SP
    // 1) Addr = Offset_SP + SP
    //
    // However, the incoming SP isn't available throughout the function; only
    // the frame pointer is. So we need to obtain the FP relative offset such
    // that:
    // 2) Addr = Offset_FP + FP
    //
    // Susbtituting (2) into (1) gives:
    // 3) Offset_FP = Offset_SP + SP - FP
    //
    // The frame pointer is:
    // 4) FP = SP - Stack_Size
    //
    // Substituting (4) into (3) gives:
    // 5) Offset_FP = Offset_SP + Stack_Size
    Offset += MFI.getStackSize();
  }

  switch (MI->getOpcode()) {
  default:
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MC6809::TI_STATIC_STACK, Offset,
                             MI->getOperand(FIOperandNum).getTargetFlags());
    break;
  }
}

Register MC6809RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MC6809::SU : MC6809::SS;
}

int copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) {
  const auto &TRI = *STI.getRegisterInfo();
  if (DestReg == SrcReg)
    return 0;

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
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
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 = TRI.getMatchingSuperReg(DestReg, MC6809::sub_lsb,
                                                &MC6809::ACC8RegClass);
    int Cost;

    // XXXX: FIXME: MarkM - handle all the CC bits; the below is broken
    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MC6809::C) {
        // Cmp #1
        Cost = 4;
        if (!MC6809::ACC8RegClass.contains(SrcReg))
          Cost += copyCost(MC6809::AA, SrcReg, STI);
      }
    }
    if (DestReg8) {
      DestReg = DestReg8;
      Register Tmp = DestReg;
      if (!MC6809::ACC8RegClass.contains(Tmp))
        Tmp = MC6809::AA;
      // TFR, AND,
      Cost = 13;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
    }
    return Cost;
  }

  llvm_unreachable("Unexpected physical register copy.");
}

bool MC6809RegisterInfo::getRegAllocationHints(
    Register VirtReg, ArrayRef<MCPhysReg> Order,
    SmallVectorImpl<MCPhysReg> &Hints, const MachineFunction &MF,
    const VirtRegMap *VRM, const LiveRegMatrix *Matrix) const {
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
      const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg
                                       ? MI.getOperand(0)
                                       : MI.getOperand(1);
      const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg
                                        ? MI.getOperand(1)
                                        : MI.getOperand(0);
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

  SmallVector<std::pair<Register, int>> RegsAndScores(RegScores.begin(),
                                                      RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, int> &A,
                          const std::pair<Register, int> &B) {
    if (A.second > B.second)
      return true;
    if (A.second < B.second)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}
