//===-- MC6809ExpandPseudos.h - Bottom-up post-RA pseudo expansion -*- C++ -*-//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809EXPANDPSEUDOS_H
#define LLVM_LIB_TARGET_MC6809_MC6809EXPANDPSEUDOS_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809ExpandPseudosPass();

} // namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MC6809EXPANDPSEUDOS_H
