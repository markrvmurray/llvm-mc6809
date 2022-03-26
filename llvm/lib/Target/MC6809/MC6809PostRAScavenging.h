//===-- MC6809PostRAScavenging.h - MC6809 Post RA Register Scavenging -*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 post-register-allocation register scavenging
// pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809POSTRASCAVENGING_H
#define LLVM_LIB_TARGET_MC6809_MC6809POSTRASCAVENGING_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809PostRAScavengingPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809POSTRASCAVENGING_H
