//===-- MC6809StaticStackAlloc.h - MC6809 Static Stack Allocation -----*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 static stack allocation pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809STATICSTACKALLOC_H
#define LLVM_LIB_TARGET_MC6809_MC6809STATICSTACKALLOC_H

#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMC6809StaticStackAllocPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809STATICSTACKALLOC_H
