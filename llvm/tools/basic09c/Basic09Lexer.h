//===- Basic09Lexer.h - BASIC09 lexer --------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09LEXER_H
#define LLVM_TOOLS_BASIC09C_BASIC09LEXER_H

#include "Basic09Token.h"
#include "llvm/ADT/StringRef.h"
#include <string>
#include <vector>

namespace llvm {
namespace basic09 {

bool normalizeSource(StringRef Source, std::string &Out, std::string &Error);
std::vector<Token> lex(StringRef Source);

} // namespace basic09
} // namespace llvm

#endif
