//===- MC6809RegisterBankInfo.cpp --------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the targeting of the RegisterBankInfo class for MC6809.
//
// The 6502 doesn't really have register banks. A distinction could be made
// between the real and imaginary registers, but the Register Bank Selector
// doesn't take register pressure into account when allocating banks. Since the
// hardware registers are extremely tight, we have the bank selector allocate
// everything to the same "Any" register bank. The register allocator proper
// will later select real registers for each value, taking register pressure
// fully into account.
//
//===----------------------------------------------------------------------===//

#include "MC6809RegisterBankInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/GlobalISel/RegisterBank.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

#define DEBUG_TYPE "mc6809-register-bank"

#define GET_TARGET_REGBANK_IMPL
#include "MC6809GenRegisterBank.inc"

using namespace llvm;

const RegisterBankInfo::InstructionMapping &
MC6809RegisterBankInfo::getInstrMapping(const MachineInstr &MI) const {
  const auto &Mapping = getInstrMappingImpl(MI);
  if (Mapping.isValid())
    return Mapping;

  const auto &MRI = MI.getMF()->getRegInfo();
  unsigned NumOperands = MI.getNumOperands();

  SmallVector<const ValueMapping *, 8> ValMappings(NumOperands);
  for (unsigned Idx = 0; Idx < NumOperands; ++Idx) {
    const auto &Operand = MI.getOperand(Idx);
    if (!Operand.isReg())
      continue;
    // Only the destination is expected for PHIs.
    if (MI.isPHI() && Idx == 1) {
      NumOperands = 1;
      break;
    }
    LLT Ty = MRI.getType(Operand.getReg());
    ValMappings[Idx] = &getValueMapping(0, Ty.getSizeInBits(), MC6809::ACCRegBank);
  }
  return getInstructionMapping(/*ID=*/1, /*Cost=*/1,
                               getOperandsMapping(ValMappings), NumOperands);
}

void MC6809RegisterBankInfo::applyMappingImpl(
    const OperandsMapper &OpdMapper) const {
  applyDefaultMapping(OpdMapper);
}

const RegisterBank &
MC6809RegisterBankInfo::getRegBankFromRegClass(const TargetRegisterClass &RC,
                                            LLT) const {
  switch (RC.getID()) {
  case MC6809::CcRegClassID:
  case MC6809::NcRegClassID:
  case MC6809::VcRegClassID:
  case MC6809::ZcRegClassID:
  case MC6809::ANY1RegClassID:
  case MC6809::BIT1RegClassID:
  case MC6809::MDCondRegClassID:
  case MC6809::NZVCcRegClassID:
  case MC6809::CCFlagRegClassID:
  case MC6809::AllocatableFlagsRegClassID:
  case MC6809::CCcRegClassID:
  case MC6809::CCondRegClassID:
    return getRegBank(MC6809::CCRegBankID);

  case MC6809::BIT8RegClassID:
  case MC6809::AAcRegClassID:
  case MC6809::ABcRegClassID:
  case MC6809::ADcRegClassID:
  case MC6809::AEcRegClassID:
  case MC6809::AFcRegClassID:
  case MC6809::AWcRegClassID:
  case MC6809::ACCRegClassID:
  case MC6809::ACC8RegClassID:
  case MC6809::ACC16RegClassID:
  case MC6809::ACC32RegClassID:
  case MC6809::ACC_and_AAcRegClassID:
  case MC6809::ACC_and_ABcRegClassID:
  case MC6809::ACC_and_ADcRegClassID:
  case MC6809::ACC_and_AEcRegClassID:
  case MC6809::ACC_and_AFcRegClassID:
  case MC6809::ACC_and_AWcRegClassID:
  case MC6809::ACC8_and_BIT8RegClassID:
  case MC6809::ACC_and_ACC8RegClassID:
  case MC6809::ACC_and_BIT8RegClassID:
    return getRegBank(MC6809::ACCRegBankID);

  case MC6809::IXcRegClassID:
  case MC6809::IYcRegClassID:
  case MC6809::SScRegClassID:
  case MC6809::SUcRegClassID:
  case MC6809::INDEX16RegClassID:
  case MC6809::STACK16RegClassID:
  case MC6809::PTR16RegClassID:
  case MC6809::INDEX16_and_IXcRegClassID:
  case MC6809::INDEX16_and_IYcRegClassID:
  case MC6809::INDEX16_and_SScRegClassID:
  case MC6809::INDEX16_and_SUcRegClassID:
    return getRegBank(MC6809::INDEXRegBankID);

  case MC6809::DtPageRegClassID:
    return getRegBank(MC6809::DPRegBankID);

  default:
    LLVM_DEBUG(dbgs() << "Not handling register class ID : " << RC.getID() << "\n";);
    llvm_unreachable("Register class not supported");
  }
}
