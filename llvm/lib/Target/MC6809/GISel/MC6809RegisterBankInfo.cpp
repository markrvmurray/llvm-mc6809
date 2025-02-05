//===- MC6809RegisterBankInfo.cpp -----------------------------------*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file implements the targeting of the RegisterBankInfo class for MC6809.
/// \todo This should be generated by TableGen.
//===----------------------------------------------------------------------===//

#include "MC6809RegisterBankInfo.h"
#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/RegisterBank.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-registerbank"

#define GET_TARGET_REGBANK_IMPL
#include "MC6809GenRegisterBank.inc"

using namespace llvm;
// This file will be TableGen'ed at some point.
#define GET_TARGET_REGBANK_INFO_IMPL
#include "MC6809GenRegisterBankInfo.def"

MC6809RegisterBankInfo::MC6809RegisterBankInfo(const TargetRegisterInfo &TRI) {

  // validate RegBank initialization.
  const RegisterBank &RBACCUM = getRegBank(MC6809::ACCUMRegBankID);
  (void)RBACCUM;
  assert(&MC6809::ACCUMRegBank == &RBACCUM && "Incorrect ACCUM RegBank initialization.");

  const RegisterBank &RBINDEX = getRegBank(MC6809::INDEXRegBankID);
  (void)RBINDEX;
  assert(&MC6809::INDEXRegBank == &RBINDEX && "Incorrect INDEX RegBank initialization.");

  const RegisterBank &RBCC = getRegBank(MC6809::CCRegBankID);
  (void)RBCC;
  assert(&MC6809::CCRegBank == &RBCC && "Incorrect CC RegBank initialization.");

  // The ACCUM register bank is fully defined by all the registers in
  // AQ + its subclasses.
  assert(RBACCUM.covers(*TRI.getRegClass(MC6809::ACC32RegClassID)) && "Subclass not added?");
  assert(RBACCUM.getSize() == 32 && "ACCs should hold up to 32 bits");
}

const RegisterBank &
MC6809RegisterBankInfo::getRegBankFromRegClass(const TargetRegisterClass &RC, LLT) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  if (MC6809::ACC8RegClass.hasSubClassEq(&RC) ||
      MC6809::ACC16RegClass.hasSubClassEq(&RC) ||
      MC6809::ACC32RegClass.hasSubClassEq(&RC)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : ACCUM\n";);
    return getRegBank(MC6809::ACCUMRegBankID);
  } else if (MC6809::INDEX16RegClass.hasSubClassEq(&RC)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : INDEX\n";);
    return getRegBank(MC6809::INDEXRegBankID);
  } else if (MC6809::CCondRegClass.hasSubClassEq(&RC)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : CC\n";);
    return getRegBank(MC6809::CCRegBankID);
  }
  llvm_unreachable("Unsupported register kind.");
}

MC6809GenRegisterBankInfo::PartialMappingIdx
MC6809GenRegisterBankInfo::getPartialMappingIdx(const LLT &Ty) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  if (Ty.isVector())
    llvm_unreachable("Vector is unsupported.");
  if (Ty.isPointer())
    return PMI_INDEX;

  switch (Ty.getSizeInBits()) {
  case 1: return PMI_COND;
  case 8: return PMI_ACC8;
  case 16: return PMI_ACC16;
  case 32: return PMI_ACC32;
  default:
    llvm_unreachable("Unsupported register size.");
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

void
MC6809RegisterBankInfo::getInstrPartialMappingIdxs(const MachineInstr &MI, const MachineRegisterInfo &MRI, SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned NumOperands = MI.getNumOperands();
  for (unsigned Idx = 0; Idx < NumOperands; ++Idx) {
    auto &MO = MI.getOperand(Idx);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : MO = "; MO.dump(););
    if (!MO.isReg())
      OpRegBankIdx[Idx] = PMI_None;
    else
      OpRegBankIdx[Idx] = getPartialMappingIdx(MRI.getType(MO.getReg()));
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : OpRegBankIdx[" << Idx << "] = " << OpRegBankIdx[Idx] << "\n";);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

bool
MC6809RegisterBankInfo::getInstrValueMapping(const MachineInstr &MI, const SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx, SmallVectorImpl<const ValueMapping *> &OpdsMapping) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned NumOperands = MI.getNumOperands();
  for (unsigned Idx = 0; Idx < NumOperands; ++Idx) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : MI = "; MI.dump(););
    if (!MI.getOperand(Idx).isReg())
      continue;

    auto Mapping = getValueMapping(OpRegBankIdx[Idx], 1);
    if (!Mapping->isValid()) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit false : MI = "; MI.dump(););
      return false;
    }

    OpdsMapping[Idx] = Mapping;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit true : MI = "; MI.dump(););
  return true;
}

const RegisterBankInfo::InstructionMapping &
MC6809RegisterBankInfo::getSameOperandsMapping(const MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineRegisterInfo &MRI = MF.getRegInfo();

  unsigned NumOperands = MI.getNumOperands();
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());

  if (NumOperands != 3 || (Ty != MRI.getType(MI.getOperand(1).getReg())) || (Ty != MRI.getType(MI.getOperand(2).getReg())))
    llvm_unreachable("Unsupported operand mapping.");

  // XXXX: FixMe: MarkM - this will often be incorrect on 6809 - assumes all 3 operands are the same types
  auto Mapping = getValueMapping(getPartialMappingIdx(Ty), 3);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Calling final getInstructionMapping() : MI = "; MI.dump(););
  return getInstructionMapping(DefaultMappingID, 1, Mapping, NumOperands);
}

const RegisterBankInfo::InstructionMapping &
MC6809RegisterBankInfo::getInstrMapping(const MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  const MachineFunction &MF = *MI.getParent()->getParent();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  unsigned Opc = MI.getOpcode();
  unsigned NumOperands = MI.getNumOperands();

  // Try the default logic for non-generic instructions that are either copies
  // or already have some operands assigned to banks.
  if (!isPreISelGenericOpcode(Opc) || Opc == TargetOpcode::G_PHI) {
    const InstructionMapping &Mapping = getInstrMappingImpl(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 10 : Mapping = "; Mapping.dump(););
    if (Mapping.isValid())
      return Mapping;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Done default logic\n";);

  switch (Opc) {
#if 0
  case TargetOpcode::G_FRAME_INDEX:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Not yet doing G_FRAME_INDEX\n";);
    llvm_unreachable("OINQUE DEBUG Does this have to be here?");
#endif /* 0 */
  case TargetOpcode::G_ADD:
  case TargetOpcode::G_SUB:
  case TargetOpcode::G_MUL:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Doing getSameOperandsMapping()\n";);
    return getSameOperandsMapping(MI);
  case TargetOpcode::G_SHL:
  case TargetOpcode::G_LSHR:
  case TargetOpcode::G_ASHR: {
    LLT Ty = MRI.getType(MI.getOperand(0).getReg());
    auto Mapping = getValueMapping(getPartialMappingIdx(Ty), 3);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : Mapping = "; Mapping->dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : Doing getInstructionMapping() 3 operands\n";);
    return getInstructionMapping(DefaultMappingID, 1, Mapping, NumOperands);
  }
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : default (nothing)\n";);
    break;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 30 : MI = "; MI.dump(););
  // Track the bank of each register.
  SmallVector<PartialMappingIdx, 4> OpRegBankIdx(NumOperands);
  getInstrPartialMappingIdxs(MI, MRI, OpRegBankIdx);

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 40 : MI = "; MI.dump(););
  // Finally construct the computed mapping.
  SmallVector<const ValueMapping *, 8> OpdsMapping(NumOperands);
  if (!getInstrValueMapping(MI, OpRegBankIdx, OpdsMapping))
    return getInvalidInstructionMapping();

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Calling final getInstructionMapping()\n";);
  return getInstructionMapping(DefaultMappingID, /* Cost */ 1, getOperandsMapping(OpdsMapping), NumOperands);
}

void MC6809RegisterBankInfo::applyMappingImpl(const OperandsMapper &OpdMapper) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  return applyDefaultMapping(OpdMapper);
}
