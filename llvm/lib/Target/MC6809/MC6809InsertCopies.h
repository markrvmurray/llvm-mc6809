//===-- MC6809InsertCopies.h - MC6809 Copy Insertion --------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 copy insertion pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INSERTCOPIES_H
#define LLVM_LIB_TARGET_MC6809_MC6809INSERTCOPIES_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809InsertCopiesPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809INSERTCOPIES_H
