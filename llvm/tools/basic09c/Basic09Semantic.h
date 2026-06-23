//===- Basic09Semantic.h - BASIC09 semantic checks ------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09SEMANTIC_H
#define LLVM_TOOLS_BASIC09C_BASIC09SEMANTIC_H

#include "Basic09AST.h"
#include "llvm/Support/raw_ostream.h"

namespace llvm {
namespace basic09 {

bool analyzeSemantics(const ASTNode &Root, raw_ostream &ErrOS);

} // namespace basic09
} // namespace llvm

#endif
