//===-- MC6809MCAsmInfo.h - MC6809 asm properties ---------------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MC6809MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_ASM_INFO_H
#define LLVM_MC6809_ASM_INFO_H

#include "llvm/MC/MCAsmInfoELF.h"

namespace llvm {

class Triple;

/// Specifies the format of MC6809 assembly files.
class MC6809MCAsmInfo : public MCAsmInfoELF {
public:
  explicit MC6809MCAsmInfo(const Triple &TT, const MCTargetOptions &Options);
};

} // end namespace llvm

#endif // LLVM_MC6809_ASM_INFO_H
