//===-- MC6809TargetObjectFile.h - MC6809 Object Info -----------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_TARGET_OBJECT_FILE_H
#define LLVM_MC6809_TARGET_OBJECT_FILE_H

#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"

namespace llvm {

/// Lowering for an MC6809 ELF32 object file.
class MC6809TargetObjectFile : public TargetLoweringObjectFileELF {
  typedef TargetLoweringObjectFileELF Base;
};

} // end namespace llvm

#endif // LLVM_MC6809_TARGET_OBJECT_FILE_H
