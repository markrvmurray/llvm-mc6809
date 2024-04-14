//===- MC6809InlineAsmLowering.h --------------------------------*- C++ -*-===//
//
// Part of the LLVM-MC6809 Project, under the Apache License v2.0 with LLVM
// Exceptions. See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file describes how to lower LLVM inline asm to machine code INLINEASM.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INLINEASMLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809INLINEASMLOWERING_H

#include "MC6809ISelLowering.h"
#include "llvm/CodeGen/GlobalISel/InlineAsmLowering.h"

namespace llvm {

class MC6809TargetLowering;

class MC6809InlineAsmLowering : public InlineAsmLowering {
public:
  MC6809InlineAsmLowering(MC6809TargetLowering *TLI);

  bool lowerAsmOperandForConstraint(Value *Val, StringRef Constraint, std::vector<MachineOperand> &Ops, MachineIRBuilder &MIRBuilder) const override;
};

} // namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MC6809INLINEASMLOWERING_H
