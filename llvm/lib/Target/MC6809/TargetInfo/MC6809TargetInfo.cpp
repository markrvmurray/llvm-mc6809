//===-- MC6809TargetInfo.cpp - MC6809 Target Implementation
//---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "TargetInfo/MC6809TargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

namespace llvm {
Target &getTheMC6809Target() {
  static Target TheMC6809Target;
  return TheMC6809Target;
}
} // namespace llvm

extern "C" LLVM_EXTERNAL_VISIBILITY void
LLVMInitializeMC6809TargetInfo() { // NOLINT
  llvm::RegisterTarget<llvm::Triple::mc6809> X(
      llvm::getTheMC6809Target(), "mc6809", "Motorola MC6809 and variants",
      "MC6809");
}
