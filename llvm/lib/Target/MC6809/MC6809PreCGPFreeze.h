//===-- MC6809PreCGPFreeze.h - MC6809 pre-CGP freeze canonicalization -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 Pre-CodeGenPrepare Freeze pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809PRECGPFREEZE_H
#define LLVM_LIB_TARGET_MC6809_MC6809PRECGPFREEZE_H

#include "llvm/Pass.h"

namespace llvm {

FunctionPass *createMC6809PreCGPFreezePass();

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809PRECGPFREEZE_H
