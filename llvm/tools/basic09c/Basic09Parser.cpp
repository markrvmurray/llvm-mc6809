//===- Basic09Parser.cpp - BASIC09 parser --------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09Parser.h"
#include "llvm/ADT/StringSwitch.h"

using namespace llvm;
using namespace llvm::basic09;

namespace {

static std::string tokensText(ArrayRef<Token> Tokens) {
  std::string Result;
  for (const Token &Tok : Tokens) {
    if (!Result.empty() && Tok.Text != "," && Tok.Text != ";" &&
        Tok.Text != ")" && Tok.Text != "]" && Tok.Text != ":" &&
        Tok.Text != "." && Tok.Text != "(" && Tok.Text != "[" &&
        Result.back() != '(' && Result.back() != '[' && Result.back() != '.' &&
        Result.back() != '#')
      Result.push_back(' ');
    Result += Tok.Text;
  }
  return Result;
}

class ExpressionParser {
public:
  explicit ExpressionParser(ArrayRef<Token> Tokens) : Tokens(Tokens) {}

  std::unique_ptr<ASTNode> parse() {
    if (Tokens.empty())
      return nullptr;
    return parseOr();
  }

private:
  ArrayRef<Token> Tokens;
  size_t Index = 0;

  bool atEnd() const { return Index >= Tokens.size(); }

  const Token &peek() const { return Tokens[Index]; }

  bool consume(StringRef Text) {
    if (!atEnd() && peek().Text == Text) {
      ++Index;
      return true;
    }
    return false;
  }

  bool consumeKeyword(StringRef Text) {
    if (!atEnd() && peek().Kind == TokenKind::Identifier &&
        peek().Text == Text) {
      ++Index;
      return true;
    }
    return false;
  }

  std::unique_ptr<ASTNode> parseOr() {
    std::unique_ptr<ASTNode> LHS = parseAnd();
    while (!atEnd() && (peek().Text == "OR" || peek().Text == "XOR")) {
      std::string Op = peek().Text;
      unsigned Line = peek().Line;
      ++Index;
      LHS = makeBinary(Op, Line, std::move(LHS), parseAnd());
    }
    return LHS;
  }

  std::unique_ptr<ASTNode> parseAnd() {
    std::unique_ptr<ASTNode> LHS = parseNot();
    while (consumeKeyword("AND"))
      LHS =
          makeBinary("AND", Tokens[Index - 1].Line, std::move(LHS), parseNot());
    return LHS;
  }

  std::unique_ptr<ASTNode> parseNot() {
    if (consumeKeyword("NOT")) {
      auto Node =
          std::make_unique<ASTNode>("Unary", "NOT", Tokens[Index - 1].Line);
      Node->Children.push_back(parseNot());
      return Node;
    }
    return parseCompare();
  }

  std::unique_ptr<ASTNode> parseCompare() {
    std::unique_ptr<ASTNode> LHS = parseAdd();
    while (!atEnd() &&
           (peek().Text == "=" || peek().Text == "<>" || peek().Text == "<" ||
            peek().Text == "<=" || peek().Text == ">" || peek().Text == ">=")) {
      std::string Op = peek().Text;
      unsigned Line = peek().Line;
      ++Index;
      LHS = makeBinary(Op, Line, std::move(LHS), parseAdd());
    }
    return LHS;
  }

  std::unique_ptr<ASTNode> parseAdd() {
    std::unique_ptr<ASTNode> LHS = parseMul();
    while (!atEnd() && (peek().Text == "+" || peek().Text == "-")) {
      std::string Op = peek().Text;
      unsigned Line = peek().Line;
      ++Index;
      LHS = makeBinary(Op, Line, std::move(LHS), parseMul());
    }
    return LHS;
  }

  std::unique_ptr<ASTNode> parseMul() {
    std::unique_ptr<ASTNode> LHS = parsePower();
    while (!atEnd() && (peek().Text == "*" || peek().Text == "/")) {
      std::string Op = peek().Text;
      unsigned Line = peek().Line;
      ++Index;
      LHS = makeBinary(Op, Line, std::move(LHS), parsePower());
    }
    return LHS;
  }

  std::unique_ptr<ASTNode> parsePower() {
    std::unique_ptr<ASTNode> LHS = parseUnary();
    if (consume("^"))
      LHS =
          makeBinary("^", Tokens[Index - 1].Line, std::move(LHS), parsePower());
    return LHS;
  }

  std::unique_ptr<ASTNode> parseUnary() {
    if (!atEnd() && (peek().Text == "+" || peek().Text == "-")) {
      std::string Op = peek().Text;
      unsigned Line = peek().Line;
      ++Index;
      auto Node = std::make_unique<ASTNode>("Unary", Op, Line);
      Node->Children.push_back(parseUnary());
      return Node;
    }
    return parsePrimary();
  }

  std::unique_ptr<ASTNode> parsePrimary() {
    if (atEnd())
      return std::make_unique<ASTNode>("MissingExpr");

    Token Tok = peek();
    if (consume("(")) {
      std::unique_ptr<ASTNode> Inner = parseOr();
      consume(")");
      return Inner;
    }

    ++Index;
    switch (Tok.Kind) {
    case TokenKind::Integer:
      return std::make_unique<ASTNode>("Integer", Tok.Text, Tok.Line);
    case TokenKind::HexInteger:
      return std::make_unique<ASTNode>("HexInteger", Tok.Text, Tok.Line);
    case TokenKind::Real:
      return std::make_unique<ASTNode>("Real", Tok.Text, Tok.Line);
    case TokenKind::String:
      return std::make_unique<ASTNode>("String", Tok.Text, Tok.Line);
    case TokenKind::Identifier:
      return parseDesignator(Tok);
    default:
      return std::make_unique<ASTNode>("Token", Tok.Text, Tok.Line);
    }
  }

  std::unique_ptr<ASTNode> parseDesignator(const Token &Name) {
    std::unique_ptr<ASTNode> Node;
    if (consume("(")) {
      Node = std::make_unique<ASTNode>("Call", Name.Text, Name.Line);
      parseArgumentList(*Node, ")");
    } else {
      Node = std::make_unique<ASTNode>("Var", Name.Text, Name.Line);
    }

    while (!atEnd()) {
      if (consume(".")) {
        if (atEnd())
          break;
        Token Field = peek();
        ++Index;
        auto FieldNode =
            std::make_unique<ASTNode>("Field", Field.Text, Field.Line);
        if (consume("("))
          parseArgumentList(*FieldNode, ")");
        Node->Children.push_back(std::move(FieldNode));
        continue;
      }
      if (consume("(")) {
        auto IndexNode = std::make_unique<ASTNode>("Index", "", Name.Line);
        parseArgumentList(*IndexNode, ")");
        Node->Children.push_back(std::move(IndexNode));
        continue;
      }
      break;
    }
    return Node;
  }

  void parseArgumentList(ASTNode &Node, StringRef EndToken) {
    if (consume(EndToken))
      return;
    while (!atEnd()) {
      Node.Children.push_back(parseOr());
      if (consume(EndToken))
        return;
      if (!consume(","))
        return;
    }
  }

  static std::unique_ptr<ASTNode> makeBinary(StringRef Op, unsigned Line,
                                             std::unique_ptr<ASTNode> LHS,
                                             std::unique_ptr<ASTNode> RHS) {
    auto Node = std::make_unique<ASTNode>("Binary", Op.str(), Line);
    Node->Children.push_back(std::move(LHS));
    Node->Children.push_back(std::move(RHS));
    return Node;
  }
};

static std::unique_ptr<ASTNode> parseExpression(ArrayRef<Token> Tokens) {
  return ExpressionParser(Tokens).parse();
}

static bool isOpenToken(StringRef Text) { return Text == "(" || Text == "["; }

static bool isCloseToken(StringRef Text) { return Text == ")" || Text == "]"; }

static size_t findTopLevelToken(ArrayRef<Token> Tokens, StringRef Text,
                                size_t Start = 0) {
  unsigned Depth = 0;
  for (size_t I = Start; I < Tokens.size(); ++I) {
    if (Depth == 0 && Tokens[I].Text == Text)
      return I;
    if (isOpenToken(Tokens[I].Text)) {
      ++Depth;
      continue;
    }
    if (isCloseToken(Tokens[I].Text)) {
      if (Depth)
        --Depth;
      continue;
    }
  }
  return Tokens.size();
}

static std::vector<ArrayRef<Token>> splitTopLevel(ArrayRef<Token> Tokens,
                                                  StringRef Separator) {
  std::vector<ArrayRef<Token>> Parts;
  size_t Begin = 0;
  while (Begin <= Tokens.size()) {
    size_t End = findTopLevelToken(Tokens, Separator, Begin);
    Parts.push_back(Tokens.slice(Begin, End - Begin));
    if (End == Tokens.size())
      break;
    Begin = End + 1;
  }
  return Parts;
}

static std::unique_ptr<ASTNode> buildTypeNode(ArrayRef<Token> Tokens) {
  if (Tokens.empty())
    return nullptr;
  auto Type = std::make_unique<ASTNode>("TypeName", tokensText(Tokens),
                                        Tokens.front().Line);
  size_t Open = findTopLevelToken(Tokens, "[");
  if (Open != Tokens.size() && !Tokens.empty() &&
      Tokens.front().Text == "STRING") {
    size_t Close = findTopLevelToken(Tokens, "]", Open + 1);
    Type->Text = "STRING";
    if (Open + 1 < Close) {
      auto Size = std::make_unique<ASTNode>(
          "StringLength", tokensText(Tokens.slice(Open + 1, Close - Open - 1)),
          Tokens[Open + 1].Line);
      if (std::unique_ptr<ASTNode> Expr =
              parseExpression(Tokens.slice(Open + 1, Close - Open - 1)))
        Size->Children.push_back(std::move(Expr));
      Type->Children.push_back(std::move(Size));
    }
  }
  return Type;
}

static std::unique_ptr<ASTNode> buildDeclarator(ArrayRef<Token> Tokens,
                                                ArrayRef<Token> TypeTokens,
                                                StringRef Kind) {
  if (Tokens.empty())
    return nullptr;
  auto Decl = std::make_unique<ASTNode>(Kind.str(), Tokens.front().Text,
                                        Tokens.front().Line);
  size_t Open = findTopLevelToken(Tokens, "(");
  if (Open != Tokens.size()) {
    Decl->Text = tokensText(Tokens.slice(0, Open));
    size_t Close = findTopLevelToken(Tokens, ")", Open + 1);
    if (Close != Tokens.size()) {
      for (ArrayRef<Token> BoundTokens :
           splitTopLevel(Tokens.slice(Open + 1, Close - Open - 1), ",")) {
        if (BoundTokens.empty())
          continue;
        auto Bound = std::make_unique<ASTNode>("Bound", tokensText(BoundTokens),
                                               BoundTokens.front().Line);
        if (std::unique_ptr<ASTNode> Expr = parseExpression(BoundTokens))
          Bound->Children.push_back(std::move(Expr));
        Decl->Children.push_back(std::move(Bound));
      }
    }
  }
  if (!TypeTokens.empty()) {
    if (std::unique_ptr<ASTNode> Type = buildTypeNode(TypeTokens))
      Decl->Children.push_back(std::move(Type));
  }
  return Decl;
}

static void appendDeclGroups(ASTNode &Node, ArrayRef<Token> Body,
                             StringRef DeclKind) {
  for (ArrayRef<Token> Group : splitTopLevel(Body, ";")) {
    if (Group.empty())
      continue;
    size_t Colon = findTopLevelToken(Group, ":");
    ArrayRef<Token> Declarators = Group.slice(0, Colon);
    ArrayRef<Token> TypeTokens =
        Colon == Group.size() ? ArrayRef<Token>() : Group.slice(Colon + 1);
    for (ArrayRef<Token> Declarator : splitTopLevel(Declarators, ",")) {
      if (std::unique_ptr<ASTNode> Decl =
              buildDeclarator(Declarator, TypeTokens, DeclKind))
        Node.Children.push_back(std::move(Decl));
    }
  }
}

static void appendTypeFields(ASTNode &Node, ArrayRef<Token> Body) {
  size_t Equals = findTopLevelToken(Body, "=");
  if (Equals == Body.size())
    return;
  if (Equals > 0)
    Node.Children.push_back(std::make_unique<ASTNode>(
        "TypeName", tokensText(Body.slice(0, Equals)), Body.front().Line));

  for (ArrayRef<Token> Group : splitTopLevel(Body.slice(Equals + 1), ";")) {
    if (Group.empty())
      continue;
    size_t Colon = findTopLevelToken(Group, ":");
    if (Colon == Group.size())
      continue;
    ArrayRef<Token> FieldTokens = Group.slice(0, Colon);
    ArrayRef<Token> TypeTokens = Group.slice(Colon + 1);
    for (ArrayRef<Token> Field : splitTopLevel(FieldTokens, ",")) {
      if (std::unique_ptr<ASTNode> Decl =
              buildDeclarator(Field, TypeTokens, "Field"))
        Node.Children.push_back(std::move(Decl));
    }
  }
}

static void appendParsedExpression(ASTNode &Node, StringRef Role,
                                   ArrayRef<Token> Tokens) {
  if (Tokens.empty())
    return;
  auto Expr = std::make_unique<ASTNode>(Role.str(), tokensText(Tokens),
                                        Tokens.empty() ? Node.Line
                                                       : Tokens.front().Line);
  if (std::unique_ptr<ASTNode> Parsed = parseExpression(Tokens))
    Expr->Children.push_back(std::move(Parsed));
  Node.Children.push_back(std::move(Expr));
}

static void appendExpressionList(ASTNode &Node, StringRef Role,
                                 ArrayRef<Token> Tokens,
                                 StringRef Separator = ",") {
  for (ArrayRef<Token> Part : splitTopLevel(Tokens, Separator))
    appendParsedExpression(Node, Role, Part);
}

static void appendPrintItems(ASTNode &Node, ArrayRef<Token> Tokens) {
  unsigned Depth = 0;
  size_t Begin = 0;
  for (size_t I = 0; I <= Tokens.size(); ++I) {
    bool AtEnd = I == Tokens.size();
    bool IsSeparator = false;
    if (!AtEnd) {
      if (isOpenToken(Tokens[I].Text))
        ++Depth;
      else if (isCloseToken(Tokens[I].Text) && Depth)
        --Depth;
      IsSeparator =
          Depth == 0 && (Tokens[I].Text == "," || Tokens[I].Text == ";");
    }
    if (!AtEnd && !IsSeparator)
      continue;

    appendParsedExpression(Node, "Item", Tokens.slice(Begin, I - Begin));
    if (!AtEnd) {
      Node.Children.push_back(std::make_unique<ASTNode>(
          "Separator", Tokens[I].Text, Tokens[I].Line));
      Begin = I + 1;
    }
  }
}

static void appendTargets(ASTNode &Node, StringRef Role,
                          ArrayRef<Token> Tokens) {
  for (ArrayRef<Token> Part : splitTopLevel(Tokens, ",")) {
    if (Part.empty())
      continue;
    Node.Children.push_back(std::make_unique<ASTNode>(
        Role.str(), tokensText(Part), Part.front().Line));
  }
}

static std::string branchTargetText(ArrayRef<Token> Tokens) {
  std::string Text = tokensText(Tokens);
  StringRef Trimmed = StringRef(Text).trim().rtrim(":");
  if (Tokens.size() == 1 && Tokens.front().Kind == TokenKind::Integer)
    return "LN" + Tokens.front().Text;
  return Trimmed.str();
}

static void appendBranchTarget(ASTNode &Node, ArrayRef<Token> Tokens) {
  if (Tokens.empty())
    return;
  Node.Children.push_back(std::make_unique<ASTNode>(
      "BranchTarget", branchTargetText(Tokens), Tokens.front().Line));
}

static void appendBranchTargets(ASTNode &Node, ArrayRef<Token> Tokens) {
  for (ArrayRef<Token> Part : splitTopLevel(Tokens, ","))
    appendBranchTarget(Node, Part);
}

static void appendPath(ASTNode &Node, ArrayRef<Token> Tokens) {
  if (!Tokens.empty() && Tokens.front().Text == "#")
    Tokens = Tokens.drop_front();
  appendParsedExpression(Node, "Path", Tokens);
}

static void appendMode(ASTNode &Node, ArrayRef<Token> Tokens) {
  if (!Tokens.empty())
    Node.Children.push_back(std::make_unique<ASTNode>(
        "Mode", tokensText(Tokens), Tokens.front().Line));
}

static void appendNextVariable(ASTNode &Node, ArrayRef<Token> Tokens) {
  if (Tokens.size() > 1)
    Node.Children.push_back(std::make_unique<ASTNode>(
        "NextVariable", Tokens[1].Text, Tokens[1].Line));
}

} // namespace

Parser::Parser(ArrayRef<Token> Tokens) {
  Line Current;
  for (const Token &Tok : Tokens) {
    if (Tok.Kind == TokenKind::EndOfFile)
      break;
    if (Tok.Kind == TokenKind::Newline) {
      if (!Current.empty()) {
        Lines.push_back(std::move(Current));
        Current = Line();
      }
      continue;
    }
    Current.push_back(Tok);
  }
  if (!Current.empty())
    Lines.push_back(std::move(Current));
}

std::unique_ptr<ASTNode> Parser::parseProgram() {
  auto Root = std::make_unique<ASTNode>("Program");
  while (!atEnd()) {
    std::unique_ptr<ASTNode> Stmt = parseStatement();
    if (!Stmt)
      break;
    Root->Children.push_back(std::move(Stmt));
  }
  return Root;
}

std::unique_ptr<ASTNode> Parser::parseStatement() {
  if (atEnd())
    return nullptr;

  if (currentStartsWith("PROCEDURE"))
    return parseProcedure();
  if (currentStartsWith("IF"))
    return parseIf();
  if (currentStartsWith("FOR"))
    return parseDelimitedBlock("For", "NEXT");
  if (currentStartsWith("WHILE"))
    return parseDelimitedBlock("While", "ENDWHILE");
  if (currentStartsWith("REPEAT"))
    return parseDelimitedBlock("Repeat", "UNTIL");
  if (currentStartsWith("LOOP"))
    return parseDelimitedBlock("Loop", "ENDLOOP");
  if (currentStartsWith("EXITIF"))
    return parseDelimitedBlock("ExitIf", "ENDEXIT");

  return makeLeaf(classifyLine(Lines[Index]));
}

std::unique_ptr<ASTNode> Parser::parseIf() {
  Line Header = consumeLine();
  auto Node = std::make_unique<ASTNode>("If", lineText(Header), Header[0].Line);

  size_t Then = findToken(Header, "THEN");
  if (Then != Header.size())
    appendExpression(*Node, "Condition", sliceLine(Header, 1, Then));
  if (Then != Header.size() && Then + 1 < Header.size()) {
    size_t ElsePos = findToken(Header, "ELSE", Then + 1);
    size_t EndIfPos = findToken(Header, "ENDIF", Then + 1);
    size_t ThenEnd = std::min(ElsePos, EndIfPos);
    Line ThenLine = sliceLine(Header, Then + 1, ThenEnd);
    if (!ThenLine.empty()) {
      auto ThenBlock = std::make_unique<ASTNode>("Block");
      if (ThenLine.size() == 1 && ThenLine.front().Kind == TokenKind::Integer) {
        ThenBlock->Children.push_back(std::make_unique<ASTNode>(
            "BranchTarget", branchTargetText(ThenLine), ThenLine.front().Line));
      } else {
        ThenBlock->Children.push_back(
            makeLeafFromLine(classifyLine(ThenLine), ThenLine));
      }
      Node->Children.push_back(std::move(ThenBlock));
    }
    if (ElsePos != Header.size()) {
      size_t ElseEnd = EndIfPos == Header.size() ? Header.size() : EndIfPos;
      Line ElseLine = sliceLine(Header, ElsePos + 1, ElseEnd);
      auto Else = std::make_unique<ASTNode>("Else");
      auto ElseBlock = std::make_unique<ASTNode>("Block");
      if (!ElseLine.empty())
        ElseBlock->Children.push_back(
            makeLeafFromLine(classifyLine(ElseLine), ElseLine));
      Else->Children.push_back(std::move(ElseBlock));
      Node->Children.push_back(std::move(Else));
    } else if (!atEnd() && currentStartsWith("ELSE")) {
      Node->Children.push_back(parseElse());
    }
    return Node;
  }

  if (Then != Header.size() && !atEnd() && Lines[Index].size() == 1 &&
      Lines[Index].front().Kind == TokenKind::Integer) {
    auto ThenBlock = std::make_unique<ASTNode>("Block");
    Line TargetLine = consumeLine();
    ThenBlock->Children.push_back(std::make_unique<ASTNode>(
        "BranchTarget", branchTargetText(TargetLine), TargetLine.front().Line));
    Node->Children.push_back(std::move(ThenBlock));
    return Node;
  }

  Node->Children.push_back(parseBlockUntil({"ELSE", "ENDIF"}));
  if (currentStartsWith("ELSE"))
    Node->Children.push_back(parseElse());
  if (!consumeIf("ENDIF")) {
    Error = "expected ENDIF";
    return nullptr;
  }
  return Node;
}

std::unique_ptr<ASTNode> Parser::parseElse() {
  Line ElseLine = consumeLine();
  auto Else = std::make_unique<ASTNode>("Else");
  auto ElseBlock = std::make_unique<ASTNode>("Block");

  Line Inline = sliceLine(ElseLine, 1, ElseLine.size());
  if (!Inline.empty())
    ElseBlock->Children.push_back(
        makeLeafFromLine(classifyLine(Inline), Inline));
  else
    ElseBlock = parseBlockUntil({"ENDIF"});

  Else->Children.push_back(std::move(ElseBlock));
  return Else;
}

std::unique_ptr<ASTNode> Parser::parseProcedure() {
  Line Header = consumeLine();
  auto Node =
      std::make_unique<ASTNode>("Procedure", lineText(Header), Header[0].Line);
  Node->Children.push_back(parseBlockUntil({"PROCEDURE"}));
  return Node;
}

std::unique_ptr<ASTNode> Parser::parseDelimitedBlock(StringRef Kind,
                                                     StringRef EndKeyword) {
  Line Header = consumeLine();
  auto Node =
      std::make_unique<ASTNode>(Kind.str(), lineText(Header), Header[0].Line);
  appendStatementDetails(*Node, Kind, Header);
  if (lineContains(Header, EndKeyword)) {
    if (Kind == "For") {
      size_t End = findToken(Header, EndKeyword);
      appendNextVariable(*Node, ArrayRef<Token>(Header).slice(End));
    }
    return Node;
  }
  if (!currentContains(EndKeyword))
    Node->Children.push_back(parseBlockUntil({EndKeyword}));
  if (currentStartsWith(EndKeyword)) {
    Line End = consumeLine();
    if (Kind == "For")
      appendNextVariable(*Node, End);
    return Node;
  }
  Error = ("expected " + EndKeyword).str();
  return nullptr;
}

std::unique_ptr<ASTNode> Parser::parseBlockUntil(ArrayRef<StringRef> Stops) {
  auto Block = std::make_unique<ASTNode>("Block");
  while (!atEnd()) {
    for (StringRef Stop : Stops)
      if (currentStartsWith(Stop))
        return Block;
    std::unique_ptr<ASTNode> Stmt = parseStatement();
    if (!Stmt)
      return nullptr;
    Block->Children.push_back(std::move(Stmt));
  }
  return Block;
}

std::unique_ptr<ASTNode> Parser::makeLeafFromLine(StringRef Kind,
                                                  const Line &L) {
  auto Node = std::make_unique<ASTNode>(Kind.str(), lineText(L), L[0].Line);
  appendStatementDetails(*Node, Kind, L);
  return Node;
}

std::unique_ptr<ASTNode> Parser::makeLeaf(StringRef Kind) {
  Line L = consumeLine();
  return makeLeafFromLine(Kind, L);
}

void Parser::appendStatementDetails(ASTNode &Node, StringRef Kind,
                                    const Line &L) {
  if (Kind == "Dim" || Kind == "Param") {
    appendDeclGroups(Node, ArrayRef<Token>(L).slice(1), "Decl");
    return;
  }

  if (Kind == "Type") {
    appendTypeFields(Node, ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "Assign" || Kind == "Let") {
    size_t Op = findToken(L, ":=");
    if (Op == L.size())
      Op = findToken(L, "=");
    if (Op != L.size()) {
      size_t LHSBegin = firstTokenText(L) == "LET" ? 1 : 0;
      Node.Children.push_back(std::make_unique<ASTNode>(
          "LHS", lineText(sliceLine(L, LHSBegin, Op)), L[LHSBegin].Line));
      appendExpression(Node, "RHS", sliceLine(L, Op + 1, L.size()));
    }
    return;
  }

  if (Kind == "Print") {
    size_t Begin = firstTokenText(L) == "?" ? 1 : 1;
    if (Begin < L.size())
      appendPrintItems(Node, ArrayRef<Token>(L).slice(Begin));
    return;
  }

  if (Kind == "PrintUsing") {
    size_t Using = findToken(L, "USING");
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", Using + 1);
    if (Using + 1 < L.size())
      appendParsedExpression(
          Node, "Format",
          ArrayRef<Token>(L).slice(Using + 1, Comma - Using - 1));
    if (Comma != L.size())
      appendExpressionList(Node, "Item", ArrayRef<Token>(L).slice(Comma + 1));
    return;
  }

  if (Kind == "Input") {
    size_t Begin = 1;
    if (Begin < L.size() && L[Begin].Kind == TokenKind::String) {
      appendParsedExpression(Node, "Prompt",
                             ArrayRef<Token>(L).slice(Begin, 1));
      if (Begin + 1 < L.size() &&
          (L[Begin + 1].Text == ";" || L[Begin + 1].Text == ","))
        Begin += 2;
      else
        Begin += 1;
    }
    if (Begin < L.size())
      appendTargets(Node, "Target", ArrayRef<Token>(L).slice(Begin));
    return;
  }

  if (Kind == "Run") {
    if (L.size() > 1)
      appendExpression(Node, "Call", sliceLine(L, 1, L.size()));
    return;
  }

  if (Kind == "Poke") {
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
    appendParsedExpression(Node, "Address",
                           ArrayRef<Token>(L).slice(1, Comma - 1));
    if (Comma != L.size())
      appendParsedExpression(Node, "Value",
                             ArrayRef<Token>(L).slice(Comma + 1));
    return;
  }

  if (Kind == "Goto" || Kind == "Gosub" || Kind == "Restore") {
    if (L.size() > 1)
      appendBranchTarget(Node, ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "Read") {
    if (L.size() > 1)
      appendTargets(Node, "Target", ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "Data") {
    if (L.size() > 1)
      appendExpressionList(Node, "Value", ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "OnErrorGoto") {
    size_t Transfer = findToken(L, "GOTO");
    if (Transfer != L.size())
      appendBranchTarget(Node, ArrayRef<Token>(L).slice(Transfer + 1));
    return;
  }

  if (Kind == "OnGoto" || Kind == "OnGosub") {
    size_t Transfer =
        Kind == "OnGosub" ? findToken(L, "GOSUB") : findToken(L, "GOTO");
    if (Transfer != L.size()) {
      appendParsedExpression(Node, "Selector",
                             ArrayRef<Token>(L).slice(1, Transfer - 1));
      appendBranchTargets(Node, ArrayRef<Token>(L).slice(Transfer + 1));
    }
    return;
  }

  if (Kind == "Open") {
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
    size_t Colon = findTopLevelToken(ArrayRef<Token>(L), ":", Comma + 1);
    appendPath(Node, ArrayRef<Token>(L).slice(1, Comma - 1));
    if (Comma != L.size())
      appendParsedExpression(
          Node, "FileName",
          ArrayRef<Token>(L).slice(Comma + 1, Colon - Comma - 1));
    if (Colon != L.size())
      appendMode(Node, ArrayRef<Token>(L).slice(Colon + 1));
    return;
  }

  if (Kind == "Create") {
    if (L.size() > 1 && L[1].Text == "#") {
      size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
      size_t Colon = findTopLevelToken(ArrayRef<Token>(L), ":", Comma + 1);
      appendPath(Node, ArrayRef<Token>(L).slice(1, Comma - 1));
      if (Comma != L.size())
        appendParsedExpression(
            Node, "FileName",
            ArrayRef<Token>(L).slice(Comma + 1, Colon - Comma - 1));
      if (Colon != L.size())
        appendMode(Node, ArrayRef<Token>(L).slice(Colon + 1));
    } else {
      size_t Colon = findTopLevelToken(ArrayRef<Token>(L), ":", 1);
      appendParsedExpression(Node, "FileName",
                             ArrayRef<Token>(L).slice(1, Colon - 1));
      if (Colon != L.size())
        appendMode(Node, ArrayRef<Token>(L).slice(Colon + 1));
    }
    return;
  }

  if (Kind == "Close") {
    if (L.size() > 1)
      appendPath(Node, ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "Delete" || Kind == "Chd" || Kind == "Shell") {
    if (L.size() > 1)
      appendParsedExpression(Node, "Argument", ArrayRef<Token>(L).slice(1));
    return;
  }

  if (Kind == "ReadFile" || Kind == "GetFile") {
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
    appendPath(Node, ArrayRef<Token>(L).slice(1, Comma - 1));
    if (Comma != L.size())
      appendTargets(Node, "Target", ArrayRef<Token>(L).slice(Comma + 1));
    return;
  }

  if (Kind == "WriteFile" || Kind == "PutFile") {
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
    appendPath(Node, ArrayRef<Token>(L).slice(1, Comma - 1));
    if (Comma != L.size())
      appendExpressionList(Node, "Value", ArrayRef<Token>(L).slice(Comma + 1));
    return;
  }

  if (Kind == "Seek") {
    size_t Comma = findTopLevelToken(ArrayRef<Token>(L), ",", 1);
    appendPath(Node, ArrayRef<Token>(L).slice(1, Comma - 1));
    if (Comma != L.size())
      appendParsedExpression(Node, "Position",
                             ArrayRef<Token>(L).slice(Comma + 1));
    return;
  }

  if (Kind == "While") {
    size_t Do = findToken(L, "DO");
    appendExpression(Node, "Condition", sliceLine(L, 1, Do));
    return;
  }

  if (Kind == "Repeat") {
    size_t Until = findToken(L, "UNTIL");
    if (Until != L.size())
      appendExpression(Node, "Condition", sliceLine(L, Until + 1, L.size()));
    return;
  }

  if (Kind == "ExitIf") {
    size_t Then = findToken(L, "THEN");
    appendExpression(Node, "Condition", sliceLine(L, 1, Then));
    return;
  }

  if (Kind == "For") {
    if (L.size() > 1)
      Node.Children.push_back(
          std::make_unique<ASTNode>("Variable", L[1].Text, L[1].Line));
    size_t Assign = findToken(L, ":=");
    if (Assign == L.size())
      Assign = findToken(L, "=");
    size_t To = findToken(L, "TO");
    size_t Step = findToken(L, "STEP");
    if (Assign != L.size() && To != L.size()) {
      appendExpression(Node, "Start", sliceLine(L, Assign + 1, To));
      appendExpression(Node, "End", sliceLine(L, To + 1, Step));
      if (Step != L.size())
        appendExpression(Node, "Step", sliceLine(L, Step + 1, L.size()));
    }
    return;
  }
}

StringRef Parser::classifyOnLine(const Line &L) {
  if (L.size() > 1 && L[1].Text == "ERROR")
    return lineContains(L, "GOTO") ? "OnErrorGoto" : "OnError";
  return lineContains(L, "GOSUB") ? "OnGosub" : "OnGoto";
}

void Parser::appendExpression(ASTNode &Node, StringRef Role, const Line &L) {
  auto Expr = std::make_unique<ASTNode>(Role.str(), lineText(L),
                                        L.empty() ? Node.Line : L[0].Line);
  if (std::unique_ptr<ASTNode> Parsed = parseExpression(L))
    Expr->Children.push_back(std::move(Parsed));
  Node.Children.push_back(std::move(Expr));
}

bool Parser::atEnd() const { return Index >= Lines.size(); }

bool Parser::currentStartsWith(StringRef Keyword) const {
  return !atEnd() && firstTokenText(Lines[Index]) == Keyword;
}

bool Parser::currentContains(StringRef Keyword) const {
  if (atEnd())
    return false;
  for (const Token &Tok : Lines[Index])
    if (Tok.Text == Keyword)
      return true;
  return false;
}

bool Parser::consumeIf(StringRef Keyword) {
  if (!currentStartsWith(Keyword))
    return false;
  ++Index;
  return true;
}

Parser::Line Parser::consumeLine() { return Lines[Index++]; }

std::string Parser::currentText() const {
  return atEnd() ? std::string() : lineText(Lines[Index]);
}

StringRef Parser::classifyLine(const Line &L) {
  StringRef First = firstTokenText(L);
  if (First == "?")
    return lineContains(L, "USING") ? "PrintUsing" : "Print";
  StringRef Kind =
      StringSwitch<StringRef>(First)
          .Case("DIM", "Dim")
          .Case("TYPE", "Type")
          .Case("PARAM", "Param")
          .Case("LET", "Let")
          .Case("PRINT", lineContains(L, "USING") ? "PrintUsing" : "Print")
          .Case("INPUT", "Input")
          .Case("RUN", "Run")
          .Case("POKE", "Poke")
          .Case("RETURN", "Return")
          .Case("END", "End")
          .Case("ENDIF", "EndIf")
          .Case("STOP", "Stop")
          .Case("BYE", "Bye")
          .Case("GOTO", "Goto")
          .Case("GOSUB", "Gosub")
          .Case("ON", classifyOnLine(L))
          .Case("READ", lineContains(L, "#") ? "ReadFile" : "Read")
          .Case("DATA", "Data")
          .Case("RESTORE", "Restore")
          .Case("DEG", "Deg")
          .Case("RAD", "Rad")
          .Case("BASE", "Base")
          .Case("OPEN", "Open")
          .Case("CLOSE", "Close")
          .Case("CREATE", "Create")
          .Case("DELETE", "Delete")
          .Case("GET", "GetFile")
          .Case("PUT", "PutFile")
          .Case("WRITE", "WriteFile")
          .Case("SEEK", "Seek")
          .Case("SHELL", "Shell")
          .Case("CHD", "Chd")
          .Case("CHX", "Chx")
          .Case("EXIT", "Exit")
          .Default("");

  if (!Kind.empty())
    return Kind;
  if (L.size() >= 2 && L[1].Text == ":")
    return "Label";
  if (lineContains(L, ":=") || lineContains(L, "="))
    return "Assign";
  return "Unsupported";
}

std::string Parser::lineText(const Line &L) { return tokensText(L); }

StringRef Parser::firstTokenText(const Line &L) {
  return L.empty() ? StringRef() : StringRef(L.front().Text);
}

bool Parser::lineContains(const Line &L, StringRef Keyword) {
  return findToken(L, Keyword) != L.size();
}

size_t Parser::findToken(const Line &L, StringRef Text, size_t Start) {
  for (size_t I = Start; I < L.size(); ++I)
    if (L[I].Text == Text)
      return I;
  return L.size();
}

Parser::Line Parser::sliceLine(const Line &L, size_t Begin, size_t End) {
  End = std::min(End, L.size());
  if (Begin >= End)
    return Line();
  return Line(L.begin() + Begin, L.begin() + End);
}
