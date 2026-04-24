//===-- MC6809SanitiseDebugInfo.h - MC6809 debug-info sanitiser -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 debug-info sanitiser pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809SANITISEDEBUGINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809SANITISEDEBUGINFO_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809SanitiseDebugInfoPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809SANITISEDEBUGINFO_H
