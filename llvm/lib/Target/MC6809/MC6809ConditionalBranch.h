//===-- MC6809ConditionalBranch.h - MC6809 Conditional Branch - C++ -----*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 pass to correctly associate the condition in
// a GISel instruction with the machine's conditional branch instruction.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809CONDITIONALBRANCH_H
#define LLVM_LIB_TARGET_MC6809_MC6809CONDITIONALBRANCH_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809ConditionalBranchPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809CONDITIONALBRANCH_H
