//===- Basic09Parser.h - BASIC09 parser ------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09PARSER_H
#define LLVM_TOOLS_BASIC09C_BASIC09PARSER_H

#include "Basic09AST.h"
#include "Basic09Token.h"
#include "llvm/ADT/StringRef.h"
#include <memory>
#include <string>
#include <vector>

namespace llvm {
namespace basic09 {

class Parser {
public:
  explicit Parser(ArrayRef<Token> Tokens);

  std::unique_ptr<ASTNode> parseProgram();
  StringRef getError() const { return Error; }

private:
  using Line = std::vector<Token>;

  std::vector<Line> Lines;
  size_t Index = 0;
  std::string Error;

  std::unique_ptr<ASTNode> parseStatement();
  std::unique_ptr<ASTNode> parseIf();
  std::unique_ptr<ASTNode> parseElse();
  std::unique_ptr<ASTNode> parseProcedure();
  std::unique_ptr<ASTNode> parseBlockUntil(ArrayRef<StringRef> StopKeywords);
  std::unique_ptr<ASTNode> parseDelimitedBlock(StringRef Kind,
                                               StringRef EndKeyword);
  std::unique_ptr<ASTNode> makeLeafFromLine(StringRef Kind, const Line &L);
  std::unique_ptr<ASTNode> makeLeaf(StringRef Kind);
  void appendStatementDetails(ASTNode &Node, StringRef Kind, const Line &L);
  void appendExpression(ASTNode &Node, StringRef Role, const Line &L);

  bool atEnd() const;
  bool currentStartsWith(StringRef Keyword) const;
  bool currentContains(StringRef Keyword) const;
  bool consumeIf(StringRef Keyword);
  Line consumeLine();
  std::string currentText() const;
  static StringRef classifyLine(const Line &L);
  static StringRef classifyOnLine(const Line &L);
  static std::string lineText(const Line &L);
  static StringRef firstTokenText(const Line &L);
  static bool lineContains(const Line &L, StringRef Keyword);
  static size_t findToken(const Line &L, StringRef Text, size_t Start = 0);
  static Line sliceLine(const Line &L, size_t Begin, size_t End);
};

} // namespace basic09
} // namespace llvm

#endif
