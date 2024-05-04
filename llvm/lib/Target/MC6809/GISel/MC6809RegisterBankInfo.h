//===- MC6809RegisterBankInfo.h ---------------------------------*- C++ -*----===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the targeting of the RegisterBankInfo class for MC6809.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H

#include "llvm/CodeGen/RegisterBankInfo.h"

#define GET_REGBANK_DECLARATIONS
#include "MC6809GenRegisterBank.inc"

namespace llvm {

class MC6809GenRegisterBankInfo : public RegisterBankInfo {
protected:
#define GET_TARGET_REGBANK_CLASS
#include "MC6809GenRegisterBank.inc"
#define GET_TARGET_REGBANK_INFO_CLASS
#include "MC6809GenRegisterBankInfo.def"

  static RegisterBankInfo::PartialMapping PartMappings[];
  static RegisterBankInfo::ValueMapping ValMappings[];

  static PartialMappingIdx getPartialMappingIdx(const LLT &Ty);
  static const RegisterBankInfo::ValueMapping *getValueMapping(PartialMappingIdx Idx, unsigned NumOperands);
};

class MC6809RegisterBankInfo final : public MC6809GenRegisterBankInfo {
public:
  // MC6809RegisterBankInfo(const TargetRegisterInfo &TRI);
  MC6809RegisterBankInfo();
  const InstructionMapping &getInstrMapping(const MachineInstr &MI) const override;

  void applyMappingImpl(MachineIRBuilder &Builder, const OperandsMapper &OpdMapper) const override;

  const RegisterBank &getRegBankFromRegClass(const TargetRegisterClass &RC, LLT Ty) const override;

private:
  /// Track the bank of each instruction operand(register)
  static void getInstrPartialMappingIdxs(const MachineInstr &MI, const MachineRegisterInfo &MRI, SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx);

  /// Construct the instruction ValueMapping from PartialMappingIdxs
  /// \return true if mapping succeeded.
  static bool getInstrValueMapping(const MachineInstr &MI, const SmallVectorImpl<PartialMappingIdx> &OpRegBankIdx, SmallVectorImpl<const ValueMapping *> &OpdsMapping);

  /// Get an instruction mapping.
  /// \return An InstructionMappings with a statically allocated
  /// OperandsMapping.
  const InstructionMapping &getSameOperandsMapping(const MachineInstr &MI) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H
