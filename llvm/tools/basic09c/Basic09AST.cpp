//===- Basic09AST.cpp - BASIC09 AST --------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09AST.h"

using namespace llvm;
using namespace llvm::basic09;

void llvm::basic09::dumpAST(const ASTNode &Node, raw_ostream &OS,
                            unsigned Indent) {
  OS.indent(Indent) << Node.Kind;
  if (!Node.Text.empty())
    OS << " \"" << Node.Text << '"';
  OS << " @" << Node.Line << '\n';
  for (const std::unique_ptr<ASTNode> &Child : Node.Children)
    dumpAST(*Child, OS, Indent + 2);
}
