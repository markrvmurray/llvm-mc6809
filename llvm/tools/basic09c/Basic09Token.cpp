//===- Basic09Token.cpp - BASIC09 tokens ---------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09Token.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::basic09;

StringRef llvm::basic09::getTokenKindName(TokenKind Kind) {
  switch (Kind) {
  case TokenKind::Identifier:
    return "identifier";
  case TokenKind::Integer:
    return "integer";
  case TokenKind::HexInteger:
    return "hex-integer";
  case TokenKind::Real:
    return "real";
  case TokenKind::String:
    return "string";
  case TokenKind::Newline:
    return "newline";
  case TokenKind::Punctuation:
    return "punctuation";
  case TokenKind::EndOfFile:
    return "eof";
  }
  llvm_unreachable("unknown token kind");
}
