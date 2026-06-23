//===- Basic09Lexer.cpp - BASIC09 lexer ----------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "Basic09Lexer.h"
#include "llvm/ADT/SmallVector.h"
#include <cctype>

using namespace llvm;
using namespace llvm::basic09;

static bool isIdentStart(char C) {
  return std::isalpha(static_cast<unsigned char>(C));
}

static bool isIdentBody(char C) {
  return std::isalnum(static_cast<unsigned char>(C)) || C == '_' || C == '$';
}

static char toBasic09Upper(char C) {
  return static_cast<char>(std::toupper(static_cast<unsigned char>(C)));
}

static bool startsLineComment(StringRef Source, size_t I) {
  if (Source.substr(I).starts_with("//"))
    return true;
  if (I + 3 <= Source.size() && Source.substr(I, 3).equals_insensitive("REM")) {
    if (I + 3 == Source.size())
      return true;
    char Next = Source[I + 3];
    return !isIdentBody(Next);
  }
  return false;
}

static bool appendNormalizedLineNumber(StringRef Line, std::string &Out) {
  size_t I = 0;
  while (I < Line.size() && (Line[I] == ' ' || Line[I] == '\t')) {
    Out.push_back(Line[I]);
    ++I;
  }

  size_t NumberStart = I;
  while (I < Line.size() && std::isdigit(static_cast<unsigned char>(Line[I])))
    ++I;
  if (I == NumberStart)
    return false;

  size_t AfterNumber = I;
  while (AfterNumber < Line.size() &&
         (Line[AfterNumber] == ' ' || Line[AfterNumber] == '\t'))
    ++AfterNumber;
  if (AfterNumber == Line.size())
    return false;

  if (I == Line.size() || Line[I] == '\r' || Line[I] == '\n' ||
      Line[I] == ' ' || Line[I] == '\t') {
    Out += "LN";
    Out.append(Line.slice(NumberStart, I).str());
    Out += ":\n";
    while (I < Line.size() && (Line[I] == ' ' || Line[I] == '\t'))
      ++I;
    Out.append(Line.substr(I).str());
    return true;
  }

  return false;
}

bool llvm::basic09::normalizeSource(StringRef Source, std::string &Out,
                                    std::string &Error) {
  std::string CanonicalSource;
  for (size_t I = 0; I < Source.size(); ++I) {
    if (Source[I] == '\r') {
      CanonicalSource.push_back('\n');
      if (I + 1 < Source.size() && Source[I + 1] == '\n')
        ++I;
      continue;
    }
    CanonicalSource.push_back(Source[I]);
  }

  SmallVector<StringRef, 0> Lines;
  StringRef(CanonicalSource).split(Lines, '\n', -1, false);

  for (StringRef Line : Lines) {
    std::string ExpandedLine;
    if (!appendNormalizedLineNumber(Line, ExpandedLine))
      ExpandedLine = Line.str();

    bool InString = false;
    for (size_t I = 0; I < ExpandedLine.size();) {
      char C = ExpandedLine[I];
      if (C == '"') {
        InString = !InString;
        Out.push_back(C);
        ++I;
        continue;
      }
      if (!InString && startsLineComment(ExpandedLine, I))
        break;
      if (!InString && I + 1 < ExpandedLine.size() && C == '(' &&
          ExpandedLine[I + 1] == '*') {
        size_t End = ExpandedLine.find("*)", I + 2);
        if (End == std::string::npos)
          break;
        I = End + 2;
        continue;
      }
      if (!InString && C == '\\') {
        Out.push_back('\n');
        ++I;
        continue;
      }
      Out.push_back(InString ? C : toBasic09Upper(C));
      ++I;
    }
    if (InString) {
      Error = "unterminated string literal";
      return false;
    }
    Out.push_back('\n');
  }

  return true;
}

namespace {

class Lexer {
public:
  explicit Lexer(StringRef Source) : Source(Source) {}

  std::vector<Token> run() {
    std::vector<Token> Tokens;
    while (!atEnd()) {
      char C = peek();
      if (C == ' ' || C == '\t' || C == '\r') {
        advance();
        continue;
      }
      if (C == '\n') {
        Tokens.push_back(makeToken(TokenKind::Newline, "\\n"));
        advance();
        continue;
      }
      if (C == '"') {
        Tokens.push_back(lexString());
        continue;
      }
      if (C == '$' && Offset + 1 < Source.size() &&
          std::isxdigit(static_cast<unsigned char>(Source[Offset + 1]))) {
        Tokens.push_back(lexHexInteger());
        continue;
      }
      if (std::isdigit(static_cast<unsigned char>(C)) ||
          (C == '.' && Offset + 1 < Source.size() &&
           std::isdigit(static_cast<unsigned char>(Source[Offset + 1])))) {
        Tokens.push_back(lexNumber());
        continue;
      }
      if (isIdentStart(C)) {
        Tokens.push_back(lexIdentifier());
        continue;
      }
      Tokens.push_back(lexPunctuation());
    }
    Tokens.push_back(makeToken(TokenKind::EndOfFile, ""));
    return Tokens;
  }

private:
  StringRef Source;
  size_t Offset = 0;
  unsigned Line = 1;
  unsigned Column = 1;

  bool atEnd() const { return Offset >= Source.size(); }
  char peek() const { return Source[Offset]; }

  char advance() {
    char C = Source[Offset++];
    if (C == '\n') {
      ++Line;
      Column = 1;
    } else {
      ++Column;
    }
    return C;
  }

  Token makeToken(TokenKind Kind, StringRef Text) const {
    return Token{Kind, Text.str(), Line, Column};
  }

  Token lexString() {
    unsigned StartLine = Line;
    unsigned StartColumn = Column;
    size_t Start = Offset;
    advance();
    while (!atEnd() && peek() != '"' && peek() != '\n')
      advance();
    if (!atEnd() && peek() == '"')
      advance();
    return Token{TokenKind::String, Source.slice(Start, Offset).str(),
                 StartLine, StartColumn};
  }

  Token lexHexInteger() {
    unsigned StartLine = Line;
    unsigned StartColumn = Column;
    size_t Start = Offset;
    advance();
    while (!atEnd() && std::isxdigit(static_cast<unsigned char>(peek())))
      advance();
    return Token{TokenKind::HexInteger, Source.slice(Start, Offset).str(),
                 StartLine, StartColumn};
  }

  Token lexNumber() {
    unsigned StartLine = Line;
    unsigned StartColumn = Column;
    size_t Start = Offset;
    bool SawDot = false;
    bool SawExponent = false;

    if (peek() == '.') {
      SawDot = true;
      advance();
    }

    while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
      advance();

    if (!atEnd() && peek() == '.') {
      SawDot = true;
      advance();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        advance();
    }

    if (!atEnd() && (peek() == 'E' || peek() == 'e')) {
      SawExponent = true;
      advance();
      if (!atEnd() && (peek() == '+' || peek() == '-'))
        advance();
      while (!atEnd() && std::isdigit(static_cast<unsigned char>(peek())))
        advance();
    }

    return Token{SawDot || SawExponent ? TokenKind::Real : TokenKind::Integer,
                 Source.slice(Start, Offset).str(), StartLine, StartColumn};
  }

  Token lexIdentifier() {
    unsigned StartLine = Line;
    unsigned StartColumn = Column;
    size_t Start = Offset;
    advance();
    while (!atEnd() && isIdentBody(peek()))
      advance();
    return Token{TokenKind::Identifier, Source.slice(Start, Offset).str(),
                 StartLine, StartColumn};
  }

  Token lexPunctuation() {
    unsigned StartLine = Line;
    unsigned StartColumn = Column;
    size_t Start = Offset;
    char C = advance();
    if (!atEnd()) {
      char N = peek();
      if ((C == ':' && N == '=') || (C == '<' && (N == '>' || N == '=')) ||
          (C == '>' && N == '='))
        advance();
    }
    return Token{TokenKind::Punctuation, Source.slice(Start, Offset).str(),
                 StartLine, StartColumn};
  }
};

} // namespace

std::vector<Token> llvm::basic09::lex(StringRef Source) {
  return Lexer(Source).run();
}
