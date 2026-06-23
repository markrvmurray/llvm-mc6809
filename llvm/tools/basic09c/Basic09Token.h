//===- Basic09Token.h - BASIC09 tokens -------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09TOKEN_H
#define LLVM_TOOLS_BASIC09C_BASIC09TOKEN_H

#include "llvm/ADT/StringRef.h"
#include <string>

namespace llvm {
namespace basic09 {

enum class TokenKind {
  Identifier,
  Integer,
  HexInteger,
  Real,
  String,
  Newline,
  Punctuation,
  EndOfFile,
};

struct Token {
  TokenKind Kind;
  std::string Text;
  unsigned Line = 1;
  unsigned Column = 1;
};

StringRef getTokenKindName(TokenKind Kind);

} // namespace basic09
} // namespace llvm

#endif
