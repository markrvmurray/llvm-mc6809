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

#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"

#define GET_REGBANK_DECLARATIONS
#include "MC6809GenRegisterBank.inc"

namespace llvm {

class MC6809GenRegisterBankInfo : public RegisterBankInfo {
protected:
#define GET_TARGET_REGBANK_CLASS
#include "MC6809GenRegisterBank.inc"
};

class MC6809RegisterBankInfo final : public MC6809GenRegisterBankInfo {
public:
  const InstructionMapping &
  getInstrMapping(const MachineInstr &MI) const override;

  void applyMappingImpl(const OperandsMapper &OpdMapper) const override;

  const RegisterBank &getRegBankFromRegClass(const TargetRegisterClass &RC,
                                             LLT Ty) const override;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809REGISTERBANKINFO_H
