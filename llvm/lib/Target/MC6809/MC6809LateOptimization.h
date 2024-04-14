//===-- MC6809LateOptimization.h - MC6809 Late Optimizationn ----*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 late optimization pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809LATEOPTIMIZATION_H
#define LLVM_LIB_TARGET_MC6809_MC6809LATEOPTIMIZATION_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809LateOptimizationPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809LATEOPTIMIZATION_H
