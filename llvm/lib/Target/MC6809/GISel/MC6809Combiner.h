//===-- MC6809Combiner.h - MC6809 GlobalIsel Combiner -----------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 global machine instruction combiner.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809COMBINER_H
#define LLVM_LIB_TARGET_MC6809_MC6809COMBINER_H

#include "llvm/Pass.h"

namespace llvm {

FunctionPass *createMC6809Combiner();

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809COMBINER_H
