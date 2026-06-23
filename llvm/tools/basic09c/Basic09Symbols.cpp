//===- Basic09Symbols.cpp - BASIC09 symbol collection --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09Symbols.h"
#include "Basic09Semantic.h"
#include "llvm/ADT/StringRef.h"
#include <vector>

using namespace llvm;
using namespace llvm::basic09;

namespace {

const ASTNode *findChild(const ASTNode &Node, StringRef Kind) {
  for (const std::unique_ptr<ASTNode> &Child : Node.Children)
    if (Child->Kind == Kind)
      return Child.get();
  return nullptr;
}

std::string typeText(const ASTNode &Decl) {
  const ASTNode *Type = findChild(Decl, "TypeName");
  if (!Type)
    return "";
  std::string Result = Type->Text;
  if (const ASTNode *Length = findChild(*Type, "StringLength"))
    Result += "[" + Length->Text + "]";
  return Result;
}

std::string boundsText(const ASTNode &Decl) {
  std::string Result;
  for (const std::unique_ptr<ASTNode> &Child : Decl.Children) {
    if (Child->Kind != "Bound")
      continue;
    if (!Result.empty())
      Result += ",";
    Result += Child->Text;
  }
  if (!Result.empty())
    Result = "[" + Result + "]";
  return Result;
}

std::string procNameFromHeader(StringRef Header) {
  Header = Header.trim();
  if (Header.consume_front("PROCEDURE"))
    return Header.trim().split(' ').first.split('(').first.str();
  return Header.str();
}

void collectNodes(const ASTNode &Node, StringRef Kind,
                  std::vector<const ASTNode *> &Out) {
  if (Node.Kind == Kind)
    Out.push_back(&Node);
  for (const std::unique_ptr<ASTNode> &Child : Node.Children)
    collectNodes(*Child, Kind, Out);
}

void dumpDecl(raw_ostream &OS, StringRef Prefix, const ASTNode &Decl) {
  OS << "  " << Prefix << ' ' << Decl.Text;
  std::string Type = typeText(Decl);
  if (!Type.empty())
    OS << " : " << Type;
  OS << boundsText(Decl) << '\n';
}

StringRef runTargetName(const ASTNode &Run) {
  const ASTNode *CallExpr = findChild(Run, "Call");
  if (!CallExpr)
    return "";
  const ASTNode *Designator = findChild(*CallExpr, "Call");
  if (!Designator)
    return "";
  return Designator->Text;
}

class SymbolDumper {
public:
  explicit SymbolDumper(raw_ostream &OS) : OS(OS) {}

  bool run(const ASTNode &Root) {
    dumpTypes(Root);
    dumpProcedures(Root);
    return true;
  }

private:
  raw_ostream &OS;

  void dumpTypes(const ASTNode &Root) {
    std::vector<const ASTNode *> Types;
    collectNodes(Root, "Type", Types);

    for (const ASTNode *Type : Types) {
      const ASTNode *Name = findChild(*Type, "TypeName");
      if (!Name)
        continue;
      OS << "Type " << Name->Text << '\n';

      for (const std::unique_ptr<ASTNode> &Child : Type->Children) {
        if (Child->Kind != "Field")
          continue;
        dumpDecl(OS, "Field", *Child);
      }
    }
  }

  void dumpProcedures(const ASTNode &Root) {
    std::vector<const ASTNode *> Procedures;
    collectNodes(Root, "Procedure", Procedures);

    for (const ASTNode *Proc : Procedures) {
      std::string Name = procNameFromHeader(Proc->Text);
      OS << "Procedure " << Name << '\n';
      dumpProcedureChildren(*Proc);
    }
  }

  void dumpProcedureChildren(const ASTNode &Node) {
    if (Node.Kind == "Param" || Node.Kind == "Dim") {
      StringRef Prefix = Node.Kind == "Param" ? "Param" : "Local";
      for (const std::unique_ptr<ASTNode> &Child : Node.Children) {
        if (Child->Kind != "Decl")
          continue;
        dumpDecl(OS, Prefix, *Child);
      }
      return;
    }

    if (Node.Kind == "Label") {
      StringRef Label = StringRef(Node.Text).rtrim(":");
      OS << "  Label " << Label << '\n';
      return;
    }

    if (Node.Kind == "Data") {
      for (const std::unique_ptr<ASTNode> &Child : Node.Children)
        if (Child->Kind == "Value")
          OS << "  Data " << Child->Text << '\n';
      return;
    }

    if (Node.Kind == "Run") {
      StringRef Target = runTargetName(Node);
      if (!Target.empty())
        OS << "  Run " << Target << '\n';
      return;
    }

    for (const std::unique_ptr<ASTNode> &Child : Node.Children)
      dumpProcedureChildren(*Child);
  }
};

} // namespace

bool llvm::basic09::dumpSymbols(const ASTNode &Root, raw_ostream &OS,
                                raw_ostream &ErrOS) {
  if (!analyzeSemantics(Root, ErrOS))
    return false;
  SymbolDumper(OS).run(Root);
  return true;
}
