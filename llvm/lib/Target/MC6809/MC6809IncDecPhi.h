//===-- MC6809IncDecPhi.h - MC6809 Increment Decrement PHI ------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 pass to separate an increment/decrement from an
// ADC of a PHI of -1 or 1.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INCDECPHI_H
#define LLVM_LIB_TARGET_MC6809_MC6809INCDECPHI_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809IncDecPhiPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809INCDECPHI_H
