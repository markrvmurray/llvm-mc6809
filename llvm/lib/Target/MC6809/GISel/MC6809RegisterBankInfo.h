//===- MC6809RegisterBankInfo ----------------------------------*- C++ -*--===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file declares the targeting of the RegisterBankInfo class for MC6809.
/// \todo This should be generated by TableGen.
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H

#include "llvm/CodeGen/RegisterBankInfo.h"

#define GET_REGBANK_DECLARATIONS
#include "MC6809GenRegisterBank.inc"

namespace llvm {

class LLT;

class MC6809GenRegisterBankInfo : public RegisterBankInfo {
protected:
#define GET_TARGET_REGBANK_CLASS
#include "MC6809GenRegisterBank.inc"
#define GET_TARGET_REGBANK_INFO_CLASS
#include "MC6809GenRegisterBankInfo.def"

  static RegisterBankInfo::PartialMapping PartMappings[];
  static RegisterBankInfo::ValueMapping ValMappings[];

  static PartialMappingIdx getPartialMappingIdx(const LLT &Ty);
  static const RegisterBankInfo::ValueMapping *
  getValueMapping(PartialMappingIdx Idx, unsigned NumOperands);
};

class TargetRegisterInfo;

/// This class provides the information for the target register banks.
class MC6809RegisterBankInfo final : public MC6809GenRegisterBankInfo {
private:
  /// Get an instruction mapping.
  /// \return An InstructionMappings with a statically allocated
  /// OperandsMapping.
  const InstructionMapping &
  getSameOperandsMapping(const MachineInstr &MI) const;

  /// Track the bank of each instruction operand(register)
  static void
  getInstrPartialMappingIdxs(const MachineInstr &MI,
                             const MachineRegisterInfo &MRI,
                             SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx);

  /// Construct the instruction ValueMapping from PartialMappingIdxs
  /// \return true if mapping succeeded.
  static bool
  getInstrValueMapping(const MachineInstr &MI,
                       const SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx,
                       SmallVectorImpl<const ValueMapping *> &OpdsMapping);

public:
  MC6809RegisterBankInfo(const TargetRegisterInfo &TRI);

  const RegisterBank &getRegBankFromRegClass(const TargetRegisterClass &RC,
                                             LLT Ty) const override;

  /// See RegisterBankInfo::applyMapping.
  void applyMappingImpl(const OperandsMapper &OpdMapper) const override;

  const InstructionMapping &
  getInstrMapping(const MachineInstr &MI) const override;
};

} // End namespace llvm

#endif
