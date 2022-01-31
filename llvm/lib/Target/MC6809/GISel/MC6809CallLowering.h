//===-- MC6809CallLowering.h - Call lowering -----------------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes how to lower LLVM calls to machine code calls.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H

#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"

namespace llvm {

// class MC6809TargetLowering;
// class MachineInstrBuilder;

class MC6809CallLowering : public CallLowering {

public:
  MC6809CallLowering(const llvm::TargetLowering *TL) : CallLowering(TL) {}

  bool lowerReturn(MachineIRBuilder &MIRBuilder, const Value *Val,
                   ArrayRef<Register> VRegs, FunctionLoweringInfo &FLI)
                     const override;

  bool lowerFormalArguments(MachineIRBuilder &MIRBuilder, const Function &F,
                            ArrayRef<ArrayRef<Register>> VRegs,
                            FunctionLoweringInfo &FLI) const override;

  bool lowerCall(MachineIRBuilder &MIRBuilder,
                 CallLoweringInfo &Info) const override;

private:
  bool lowerReturnVal(MachineIRBuilder &MIRBuilder, const Value *Val,
                      ArrayRef<Register> VRegs,
                      MachineInstrBuilder &Ret) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H
