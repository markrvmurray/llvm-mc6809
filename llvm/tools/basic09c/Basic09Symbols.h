//===- Basic09Symbols.h - BASIC09 symbol collection ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09SYMBOLS_H
#define LLVM_TOOLS_BASIC09C_BASIC09SYMBOLS_H

#include "Basic09AST.h"
#include "llvm/Support/raw_ostream.h"
#include <string>

namespace llvm {
namespace basic09 {

bool dumpSymbols(const ASTNode &Root, raw_ostream &OS, raw_ostream &ErrOS);

} // namespace basic09
} // namespace llvm

#endif
