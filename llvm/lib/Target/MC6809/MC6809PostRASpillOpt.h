//===-- MC6809PostRASpillOpt.h - MC6809 Post RA Spill Optimization --------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809POSTRASPILLOPT_H
#define LLVM_LIB_TARGET_MC6809_MC6809POSTRASPILLOPT_H

#include "llvm/InitializePasses.h"

namespace llvm {
class MachineFunctionPass;
MachineFunctionPass *createMC6809PostRASpillOptPass();
void initializeMC6809PostRASpillOptPass(PassRegistry &);
} // namespace llvm

#endif
