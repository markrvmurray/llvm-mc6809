//===- MC6809Flags.h - MC6809 ELF e_flags tools -----------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This header contains shared EF_MC6809_* information and verification tools for
// targeting MC6809.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_BINARYFORMAT_MC6809FLAGS_H
#define LLVM_BINARYFORMAT_MC6809FLAGS_H

#include "llvm/Support/ScopedPrinter.h"

namespace llvm {

class FeatureBitset;

namespace MC6809 {

/// EnumEntries for general EF_MC6809_* printing.
extern const ArrayRef<EnumEntry<unsigned>> ElfHeaderMC6809Flags;

/// Makes a string describing all set EF_MC6809_* bits.
std::string makeEFlagsString(unsigned EFlags);

} // namespace MC6809
} // namespace llvm

#endif // LLVM_BINARYFORMAT_MC6809FLAGS_H
