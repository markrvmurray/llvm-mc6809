//===- basic09c.cpp - BASIC09 frontend prototype -------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This is an intentionally small BASIC09 frontend scaffold.  The first
// supported operation is a token dump that makes the source normalization
// boundary explicit before parser, semantic, and LLVM IR lowering work is
// added.
//
//===----------------------------------------------------------------------===//

#include "Basic09AST.h"
#include "Basic09Lexer.h"
#include "Basic09Parser.h"
#include "Basic09Semantic.h"
#include "Basic09Symbols.h"
#include "Basic09Token.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorOr.h"
#include "llvm/Support/MemoryBuffer.h"
#include "llvm/Support/WithColor.h"
#include "llvm/Support/raw_ostream.h"
#include <memory>
#include <string>
#include <system_error>

using namespace llvm;
using namespace llvm::basic09;

namespace {

static cl::OptionCategory Basic09CCategory("basic09c options");

static cl::opt<bool> DumpTokens("dump-tokens",
                                cl::desc("Dump normalized BASIC09 tokens"),
                                cl::cat(Basic09CCategory));

static cl::opt<bool> DumpAST("dump-ast",
                             cl::desc("Parse BASIC09 source and dump the AST"),
                             cl::cat(Basic09CCategory));

static cl::opt<bool> SyntaxOnly("syntax-only",
                                cl::desc("Parse BASIC09 source and exit"),
                                cl::cat(Basic09CCategory));

static cl::opt<bool>
    AnalyzeOnly("analyze-only",
                cl::desc("Parse and semantically analyze BASIC09 source"),
                cl::cat(Basic09CCategory));

static cl::opt<bool>
    DumpSymbols("dump-symbols",
                cl::desc("Parse BASIC09 source and dump symbols"),
                cl::cat(Basic09CCategory));

static cl::opt<std::string> InputFilename(cl::Positional,
                                          cl::desc("<input BASIC09 source>"),
                                          cl::Required,
                                          cl::cat(Basic09CCategory));

static bool normalizeAndLex(StringRef Source, std::vector<Token> &Tokens) {
  std::string Normalized;
  std::string Error;
  if (!normalizeSource(Source, Normalized, Error)) {
    WithColor::error(errs(), "basic09c") << Error << '\n';
    return false;
  }

  Tokens = lex(Normalized);
  return true;
}

static int dumpTokens(StringRef Source) {
  std::vector<Token> Tokens;
  if (!normalizeAndLex(Source, Tokens))
    return 1;

  for (const Token &Tok : Tokens) {
    outs() << Tok.Line << ':' << Tok.Column << ": "
           << getTokenKindName(Tok.Kind);
    if (Tok.Kind != TokenKind::EndOfFile)
      outs() << " \"" << Tok.Text << '"';
    outs() << '\n';
  }
  return 0;
}

static int dumpParsedAST(StringRef Source) {
  std::vector<Token> Tokens;
  if (!normalizeAndLex(Source, Tokens))
    return 1;

  Parser P(Tokens);
  std::unique_ptr<ASTNode> Root = P.parseProgram();
  if (!P.getError().empty()) {
    WithColor::error(errs(), "basic09c") << P.getError() << '\n';
    return 1;
  }

  dumpAST(*Root, outs());
  return 0;
}

static int parseSyntaxOnly(StringRef Source) {
  std::vector<Token> Tokens;
  if (!normalizeAndLex(Source, Tokens))
    return 1;

  Parser P(Tokens);
  (void)P.parseProgram();
  if (!P.getError().empty()) {
    WithColor::error(errs(), "basic09c") << P.getError() << '\n';
    return 1;
  }
  return 0;
}

static int analyzeOnly(StringRef Source) {
  std::vector<Token> Tokens;
  if (!normalizeAndLex(Source, Tokens))
    return 1;

  Parser P(Tokens);
  std::unique_ptr<ASTNode> Root = P.parseProgram();
  if (!P.getError().empty()) {
    WithColor::error(errs(), "basic09c") << P.getError() << '\n';
    return 1;
  }
  return analyzeSemantics(*Root, errs()) ? 0 : 1;
}

static int dumpParsedSymbols(StringRef Source) {
  std::vector<Token> Tokens;
  if (!normalizeAndLex(Source, Tokens))
    return 1;

  Parser P(Tokens);
  std::unique_ptr<ASTNode> Root = P.parseProgram();
  if (!P.getError().empty()) {
    WithColor::error(errs(), "basic09c") << P.getError() << '\n';
    return 1;
  }
  return dumpSymbols(*Root, outs(), errs()) ? 0 : 1;
}

} // namespace

int main(int argc, char **argv) {
  cl::HideUnrelatedOptions(Basic09CCategory);
  cl::ParseCommandLineOptions(argc, argv, "BASIC09 frontend prototype\n");

  ErrorOr<std::unique_ptr<MemoryBuffer>> InputOrErr =
      MemoryBuffer::getFileOrSTDIN(InputFilename);
  if (!InputOrErr) {
    WithColor::error(errs(), "basic09c")
        << "cannot read '" << InputFilename
        << "': " << InputOrErr.getError().message() << '\n';
    return 1;
  }

  if (DumpTokens)
    return dumpTokens((*InputOrErr)->getBuffer());
  if (DumpAST)
    return dumpParsedAST((*InputOrErr)->getBuffer());
  if (SyntaxOnly)
    return parseSyntaxOnly((*InputOrErr)->getBuffer());
  if (AnalyzeOnly)
    return analyzeOnly((*InputOrErr)->getBuffer());
  if (DumpSymbols)
    return dumpParsedSymbols((*InputOrErr)->getBuffer());

  WithColor::error(errs(), "basic09c")
      << "no action specified; use --dump-tokens, --dump-ast, "
         "--dump-symbols, --analyze-only, or --syntax-only\n";
  return 1;
}
