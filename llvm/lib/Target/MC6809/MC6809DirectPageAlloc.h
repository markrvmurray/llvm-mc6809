//===-- MC6809DirectPageAlloc.h - MC6809 Direct Page Allocation -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file This file declares the MC6809 direct page allocation pass.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809DIRECTPAGEALLOC_H
#define LLVM_LIB_TARGET_MC6809_MC6809DIRECTPAGEALLOC_H

#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMC6809DirectPageAllocPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809DIRECTPAGEALLOC_H
