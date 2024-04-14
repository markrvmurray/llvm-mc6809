//===-- MC6809Internalize.h - MC6809 Libcall Internalization ----*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.n with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 library call internalization pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INTERNALIZE_H
#define LLVM_LIB_TARGET_MC6809_MC6809INTERNALIZE_H

namespace llvm {

class ModulePass;

ModulePass *createMC6809InternalizePass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809INTERNALIZE_H
