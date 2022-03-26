//===-- MC6809InstructionSelector.h - MC6809 Instruction Selector -----*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 instruction selector.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INSTRUCTIONSELECTOR_H
#define LLVM_LIB_TARGET_MC6809_MC6809INSTRUCTIONSELECTOR_H

#include "MC6809RegisterBankInfo.h"
#include "MC6809Subtarget.h"
#include "MC6809TargetMachine.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"

namespace llvm {

InstructionSelector *
createMC6809InstructionSelector(const MC6809TargetMachine &TM,
                                MC6809Subtarget &STI,
                                MC6809RegisterBankInfo &RBI);

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809INSTRUCTIONSELECTOR_H
