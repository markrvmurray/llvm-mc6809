//===- Basic09Semantic.cpp - BASIC09 semantic checks ---------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09Semantic.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/ADT/StringSet.h"
#include "llvm/Support/WithColor.h"
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

bool isBuiltinType(StringRef Name) {
  return Name == "INTEGER" || Name == "REAL" || Name == "STRING" ||
         Name == "BOOLEAN" || Name == "BYTE";
}

class SemanticAnalyzer {
public:
  explicit SemanticAnalyzer(raw_ostream &ErrOS) : ErrOS(ErrOS) {}

  bool run(const ASTNode &Root) {
    collectGlobalTypes(Root);
    checkGlobalTypes(Root);
    checkProcedures(Root);
    return !HadError;
  }

private:
  raw_ostream &ErrOS;
  bool HadError = false;
  StringSet<> GlobalTypes;

  void collectGlobalTypes(const ASTNode &Root) {
    for (const std::unique_ptr<ASTNode> &Child : Root.Children)
      if (Child->Kind == "Type")
        if (const ASTNode *Name = findChild(*Child, "TypeName"))
          GlobalTypes.insert(Name->Text);
  }

  void checkGlobalTypes(const ASTNode &Root) {
    StringSet<> SeenTypes;
    for (const std::unique_ptr<ASTNode> &Child : Root.Children) {
      if (Child->Kind != "Type")
        continue;
      const ASTNode *Name = findChild(*Child, "TypeName");
      if (!Name)
        continue;
      noteDuplicate(SeenTypes, Name->Text, "type", Name->Line);
      checkTypeFields(*Child, GlobalTypes);
    }
  }

  void checkTypeFields(const ASTNode &Type, const StringSet<> &KnownTypes) {
    StringSet<> SeenFields;
    for (const std::unique_ptr<ASTNode> &Child : Type.Children) {
      if (Child->Kind != "Field")
        continue;
      noteDuplicate(SeenFields, Child->Text, "field", Child->Line);
      validateType(*Child, KnownTypes);
    }
  }

  void collectProcedureTypes(const ASTNode &Proc,
                             std::vector<const ASTNode *> &Types,
                             StringSet<> &KnownTypes) {
    collectNodes(Proc, "Type", Types);
    for (const ASTNode *Type : Types)
      if (const ASTNode *Name = findChild(*Type, "TypeName"))
        KnownTypes.insert(Name->Text);
  }

  void addGlobalTypes(StringSet<> &KnownTypes) {
    for (const auto &Type : GlobalTypes)
      KnownTypes.insert(Type.getKey());
  }

  void checkProcedureTypes(ArrayRef<const ASTNode *> Types,
                           const StringSet<> &KnownTypes) {
    StringSet<> SeenTypes;
    for (const ASTNode *Type : Types) {
      const ASTNode *Name = findChild(*Type, "TypeName");
      if (!Name)
        continue;
      noteDuplicate(SeenTypes, Name->Text, "type", Name->Line);
      checkTypeFields(*Type, KnownTypes);
    }
  }

  void checkProcedures(const ASTNode &Root) {
    std::vector<const ASTNode *> Procedures;
    collectNodes(Root, "Procedure", Procedures);

    StringSet<> SeenProcedures;
    for (const ASTNode *Proc : Procedures) {
      noteDuplicate(SeenProcedures, procNameFromHeader(Proc->Text), "procedure",
                    Proc->Line);

      StringSet<> KnownTypes;
      addGlobalTypes(KnownTypes);
      std::vector<const ASTNode *> ProcedureTypes;
      collectProcedureTypes(*Proc, ProcedureTypes, KnownTypes);
      checkProcedureTypes(ProcedureTypes, KnownTypes);

      StringSet<> Locals;
      StringSet<> Labels;
      collectLabels(*Proc, Labels);
      checkProcedureChildren(*Proc, KnownTypes, Locals, Labels);
    }
  }

  void collectLabels(const ASTNode &Node, StringSet<> &Labels) {
    if (Node.Kind == "Label") {
      noteDuplicate(Labels, StringRef(Node.Text).rtrim(":"), "label",
                    Node.Line);
      return;
    }
    for (const std::unique_ptr<ASTNode> &Child : Node.Children)
      collectLabels(*Child, Labels);
  }

  void checkProcedureChildren(const ASTNode &Node,
                              const StringSet<> &KnownTypes,
                              StringSet<> &Locals, StringSet<> &Labels) {
    if (Node.Kind == "Type" || Node.Kind == "Label")
      return;

    if (Node.Kind == "Param" || Node.Kind == "Dim") {
      for (const std::unique_ptr<ASTNode> &Child : Node.Children) {
        if (Child->Kind != "Decl")
          continue;
        noteDuplicate(Locals, Child->Text, "local", Child->Line);
        validateType(*Child, KnownTypes);
      }
      return;
    }

    if (Node.Kind == "BranchTarget") {
      validateBranchTarget(Node, Labels);
      return;
    }

    if (Node.Kind == "Unsupported") {
      reportUnsupported(Node);
      return;
    }

    if (Node.Kind == "For")
      validateForNext(Node);

    for (const std::unique_ptr<ASTNode> &Child : Node.Children)
      checkProcedureChildren(*Child, KnownTypes, Locals, Labels);
  }

  raw_ostream &errorAt(unsigned Line) {
    return WithColor::error(ErrOS, "basic09c") << "line " << Line << ": ";
  }

  void noteDuplicate(StringSet<> &Seen, StringRef Name, StringRef What,
                     unsigned Line) {
    if (Seen.insert(Name).second)
      return;
    errorAt(Line) << "duplicate " << What << " '" << Name << "'\n";
    HadError = true;
  }

  void validateBranchTarget(const ASTNode &Target, const StringSet<> &Labels) {
    if (Labels.contains(Target.Text))
      return;
    errorAt(Target.Line) << "unknown label '" << Target.Text << "'\n";
    HadError = true;
  }

  void validateForNext(const ASTNode &For) {
    const ASTNode *Variable = findChild(For, "Variable");
    const ASTNode *NextVariable = findChild(For, "NextVariable");
    if (!Variable || !NextVariable || Variable->Text == NextVariable->Text)
      return;
    errorAt(NextVariable->Line)
        << "NEXT variable '" << NextVariable->Text
        << "' does not match FOR variable '" << Variable->Text << "'\n";
    HadError = true;
  }

  void reportUnsupported(const ASTNode &Node) {
    errorAt(Node.Line) << "unsupported statement '" << Node.Text << "'\n";
    HadError = true;
  }

  void validateType(const ASTNode &Decl, const StringSet<> &KnownTypes) {
    const ASTNode *Type = findChild(Decl, "TypeName");
    if (!Type || isBuiltinType(Type->Text) || KnownTypes.contains(Type->Text))
      return;
    errorAt(Type->Line) << "unknown type '" << Type->Text << "' for '"
                        << Decl.Text << "'\n";
    HadError = true;
  }
};

} // namespace

bool llvm::basic09::analyzeSemantics(const ASTNode &Root, raw_ostream &ErrOS) {
  return SemanticAnalyzer(ErrOS).run(Root);
}
