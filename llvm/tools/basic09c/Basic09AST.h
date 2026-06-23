//===- Basic09AST.h - BASIC09 AST ------------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_TOOLS_BASIC09C_BASIC09AST_H
#define LLVM_TOOLS_BASIC09C_BASIC09AST_H

#include "llvm/Support/raw_ostream.h"
#include <memory>
#include <string>
#include <vector>

namespace llvm {
namespace basic09 {

struct ASTNode {
  std::string Kind;
  std::string Text;
  unsigned Line = 1;
  std::vector<std::unique_ptr<ASTNode>> Children;

  ASTNode(std::string Kind, std::string Text = "", unsigned Line = 1)
      : Kind(std::move(Kind)), Text(std::move(Text)), Line(Line) {}
};

void dumpAST(const ASTNode &Node, raw_ostream &OS, unsigned Indent = 0);

} // namespace basic09
} // namespace llvm

#endif
