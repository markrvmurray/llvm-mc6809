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

public:
  /// Emit a `.dp.bss*` (direct-page zero-init) section as @nobits even though
  /// it carries an explicit section name. LLVM only auto-classifies a
  /// `.bss`-prefixed name as BSS; a `.dp.bss` name would otherwise fall through
  /// to @progbits and pin real zero bytes into the image.
  MCSection *getExplicitSectionGlobal(const GlobalObject *GO, SectionKind Kind,
                                      const TargetMachine &TM) const override;
};

} // end namespace llvm

#endif // LLVM_MC6809_TARGET_OBJECT_FILE_H
