//===---- MC6809AsmParser.cpp - Parse MC6809 assembly to MCInst instructions ----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MC6809FixupKinds.h"
#include "MCTargetDesc/MC6809MCELFStreamer.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/MCAsmLexer.h"
#include "llvm/MC/MCParser/MCParsedAsmOperand.h"
#include "llvm/MC/MCParser/MCTargetAsmParser.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCValue.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/MathExtras.h"

#include <sstream>

#define DEBUG_TYPE "mc6809-asm-parser"

namespace llvm {

#define GET_REGISTER_MATCHER
#include "MC6809GenAsmMatcher.inc"

class MC6809Operand : public MCParsedAsmOperand {
public:
  MC6809Operand() = delete;

  MC6809Operand(const MCExpr *Val, SMLoc S, SMLoc E)
      : Kind(k_Immediate), Imm(Val), Start(S), End(E){};

  MC6809Operand(unsigned RegNum, SMLoc S, SMLoc E)
      : Kind(k_Register), Reg(RegNum), Start(S), End(E){};

  MC6809Operand(unsigned short RegList, SMLoc S, SMLoc E)
      : Kind(k_RegList), RegList(RegList), Start(S), End(E){};

  MC6809Operand(unsigned char CondCode, SMLoc S, SMLoc E)
      : Kind(k_CondCode), CondCode(CondCode), Start(S), End(E){};

  MC6809Operand(StringRef Str, SMLoc S) : Kind(k_Token), Tok(Str), Start(S){};

private:
  enum KindTy {
    k_None,
    k_Immediate,
    k_Register,
    k_RegList,
    k_CondCode,
    k_Token,
  } Kind{k_None};

  MCExpr const *Imm{};
  unsigned int Reg{};
  unsigned short RegList{};
  unsigned char CondCode{};
  StringRef Tok;
  SMLoc Start, End;

public:
  template <int64_t Low, int64_t High> bool isImmediate() const {
    if (!isImm()) {
      return false;
    }

    // if it's a MC6809-specific modifier, we need to figure out the size based
    // on the type of expression.  If the largest value that the modifier
    // can produce is larger than the expression that this immediate can hold,
    // we have to refuse to match it, so that we don't inadvertently match
    // direct-page addresses against 16-bit modifiers.
    const auto *MME = dyn_cast<MC6809MCExpr>(getImm());
    if (MME) {
      const MC6809::Fixups Kind = MME->getFixupKind();
      const MCFixupKindInfo &Info =
          MC6809FixupKinds::getFixupKindInfo(Kind, nullptr);
      int64_t MaxValue = (1 << Info.TargetSize) - 1;
      bool Evaluated = false;
      int64_t Constant = 0;
      Evaluated = MME->evaluateAsConstant(Constant);
      // if the constant is non-zero, evaluate for size now
      if (Evaluated && Constant > 0) {
        return (Constant <= MaxValue);
      }
      return (MaxValue <= High);
    }

    // if it's a symbol ref, we'll replace it later
    const auto *SRE = dyn_cast<MCSymbolRefExpr>(getImm());
    if (SRE) {
      return true;
    }
    // if it's an immediate but not castable to one, it must be a label
    const auto *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE) {
      return true;
    }
    int64_t Value = CE->getValue();
    return Value >= Low && Value <= High;
  }

  virtual bool is(KindTy K) const { return (Kind == K); }
  bool isToken() const override { return is(k_Token); }
  bool isImm() const override { return is(k_Immediate); }
  bool isReg() const override { return is(k_Register); }
  bool isMem() const override {
    assert(false);
    return false;
  }
  bool isCondCode() const { return is(k_CondCode); }
  bool isRegList() const { return is(k_RegList); }
  SMLoc getStartLoc() const override { return Start; }
  SMLoc getEndLoc() const override { return End; }
  const StringRef &getToken() const {
    assert(isToken());
    return Tok;
  }
  const MCExpr *getImm() const {
    assert(isImm());
    return Imm;
  };
  unsigned getReg() const override {
    assert(isReg());
    return Reg;
  }
  unsigned short getRegList() const {
    assert(isRegList());
    return RegList;
  }
  unsigned char getCondCode() const {
    assert(isRegList());
    return RegList;
  }

  virtual bool isImm3() const { return isImmediate<0, 7>(); }
  virtual bool isImm8() const { return isImmediate<0, 0x100 - 1>(); }
  virtual bool isImm16() const { return isImmediate<0, 0x10000 - 1>(); }
  virtual bool isImm8To16() const { return (!isImm8() && isImm16()); }
  virtual bool isRel5() const { return isImmediate<-16, 15>(); }
  virtual bool isRel8() const { return isImmediate<-128, 127>(); }
  virtual bool isRel16() const { return isImmediate<-32768, 32767>(); }
  virtual bool isPCRel8() const { return isImmediate<-128, 127>(); }
  virtual bool isPCRel16() const { return isImmediate<-32768, 32767>(); }
  virtual bool isAddr8() const { return isImm8(); }
  virtual bool isAddr16() const { return isImm16(); }
  virtual bool isRegAny() const { return isReg(); }

  static void addExpr(MCInst &Inst, const MCExpr *Expr) {
    if (const auto *CE = dyn_cast<MCConstantExpr>(Expr)) {
      Inst.addOperand(MCOperand::createImm(CE->getValue()));
    } else {
      Inst.addOperand(MCOperand::createExpr(Expr));
    }
  }

  void addImmOperands(MCInst &Inst, unsigned N) const {
    assert(Kind == k_Immediate && "Unexpected operand kind");
    assert(N == 1 && "Invalid number of operands!");

    const MCExpr *Expr = getImm();
    addExpr(Inst, Expr);
  }

  void addRegOperands(MCInst &Inst, unsigned /*N*/) const {
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addRegAnyOperands(MCInst &Inst, unsigned /*N*/) const {
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addRegListOperands(MCInst &Inst, unsigned /*N*/) const {
    Inst.addOperand(MCOperand::createReg(getReg()));
  }

  void addPCRel8Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addPCRel16Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addRel5Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addRel8Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addRel16Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr8Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addAddr16Operands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  void addCondCodeOperands(MCInst &Inst, unsigned N) const {
    addImmOperands(Inst, N);
  }

  static std::unique_ptr<MC6809Operand> createImm(const MCExpr *Val, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<MC6809Operand>(Val, S, E);
  }

  static std::unique_ptr<MC6809Operand> createReg(unsigned RegNum, SMLoc S,
                                               SMLoc E) {
    return std::make_unique<MC6809Operand>(RegNum, S, E);
  }

  static std::unique_ptr<MC6809Operand> createToken(StringRef Str, SMLoc S) {
    return std::make_unique<MC6809Operand>(Str, S);
  }

  void print(raw_ostream &O) const override {
    switch (Kind) {
    case k_None:
      O << "None";
      break;
    case k_Token:
      O << "Token: \"" << getToken() << "\"";
      break;
    case k_Register:
      O << "Register: " << getReg();
      break;
    case k_RegList:
      O << "RegList: " << getRegList();
      break;
    case k_Immediate:
      O << "Immediate: \"" << *getImm() << "\"";
      break;
    case k_CondCode:
      O << "Immediate: \"" << getCondCode() << "\"";
      break;
    }
    O << "\n";
  };
};

/// Parses MC6809 assembly from a stream.
class MC6809AsmParser : public MCTargetAsmParser {
  const MCSubtargetInfo &STI;
  MCAsmParser &Parser;
  const MCRegisterInfo *MRI;
  const std::string GenerateStubs = "gs";

#define GET_ASSEMBLER_HEADER
#include "MC6809GenAsmMatcher.inc"

public:
  enum MC6809MatchResultTy {
    Match_UnknownError = FIRST_TARGET_MATCH_RESULT_TY,
#define GET_OPERAND_DIAGNOSTIC_TYPES
#include "MC6809GenAsmMatcher.inc"

  };

  MC6809AsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser,
               const MCInstrInfo &MII, const MCTargetOptions &Options)
      : MCTargetAsmParser(Options, STI, MII), STI(STI), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));

    if (MCAssembler *Asm = Parser.getStreamer().getAssemblerPtr())
      Asm->setELFHeaderEFlags(MC6809_MC::makeEFlags(STI.getFeatureBits()));
  }
  MCAsmLexer &getLexer() const { return Parser.getLexer(); }
  MCAsmParser &getParser() const { return Parser; }

  bool parsePrimaryExpr(const MCExpr *&Res, SMLoc &EndLoc) override {
    return MCTargetAsmParser::parsePrimaryExpr(Res, EndLoc);
  }

  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands,
                      uint64_t const &ErrorInfo) {
    SMLoc ErrorLoc = Loc;
    char const *Diag = nullptr;

    if (ErrorInfo != ~0U) {
      if (ErrorInfo >= Operands.size()) {
        Diag = "too few operands for instruction";
      } else {
        ErrorLoc = Operands[ErrorInfo]->getStartLoc();
      }
    }

    if (Diag == nullptr) {
      Diag = "invalid operand for instruction";
    }

    return Error(ErrorLoc, Diag);
  }

  bool missingFeature(llvm::SMLoc const &Loc, uint64_t const & /*ErrorInfo*/) {
    return Error(Loc,
                 "instruction requires a CPU feature not currently enabled");
  }

  bool emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
    Inst.setLoc(Loc);
    Out.emitInstruction(Inst, STI);

    return false;
  }

  /// MatchAndEmitInstruction - Recognize a series of operands of a parsed
  /// instruction as an actual MCInst and emit it to the specified MCStreamer.
  /// This returns false on success and returns true on failure to match.
  ///
  /// On failure, the target parser is responsible for emitting a diagnostic
  /// explaining the match failure.
  bool MatchAndEmitInstruction(SMLoc Loc, unsigned & /*Opcode*/,
                               OperandVector &Operands, MCStreamer &Out,
                               uint64_t &ErrorInfo,
                               bool MatchingInlineAsm) override {
    MCInst Inst;
    unsigned MatchResult =
        // we always want ConvertToMapAndConstraints to be called
        MatchInstructionImpl(Operands, Inst, ErrorInfo, MatchingInlineAsm);
    switch (MatchResult) {
    case Match_Success:
      return emit(Inst, Loc, Out);
    case Match_MissingFeature:
      return missingFeature(Loc, ErrorInfo);
    case Match_InvalidOperand:
      return invalidOperand(Loc, Operands, ErrorInfo);
    case Match_MnemonicFail:
      return Error(Loc, "invalid instruction");
    case Match_InvalidAddr8:
      return Error(Loc, "operand must be an 8-bit address");
    case Match_InvalidAddr16:
      return Error(Loc, "operand must be an 16-bit address");
    case Match_InvalidRel5:
      return Error(Loc, "operand must be an 5-bit index offset");
    case Match_InvalidRel8:
      return Error(Loc, "operand must be an 8-bit index offset");
    case Match_InvalidRel16:
      return Error(Loc, "operand must be an 16-bit index offset");
    case Match_InvalidPCRel8:
      return Error(Loc, "operand must be an 8-bit PC relative address");
    case Match_InvalidPCRel16:
      return Error(Loc, "operand must be an 16-bit PC relative address");
    case Match_NearMisses:
      return Error(Loc, "found some near misses");
    default:
      return true;
    }
  }

  /// ParseDirective - Parse a target specific assembler directive
  ///
  /// The parser is positioned following the directive name.  The target
  /// specific directive parser should parse the entire directive doing or
  /// recording any target specific work, or return true and do nothing if the
  /// directive is not target specific. If the directive is specific for
  /// the target, the entire line is parsed up to and including the
  /// end-of-statement token and false is returned.
  ///
  /// \param DirectiveID - the identifier token of the directive.
  bool ParseDirective(AsmToken DirectiveID) override {
    // todo
    return true;
  }

  virtual signed char hexToChar(const signed char Letter) {
    if (Letter >= '0' && Letter <= '9') {
      return static_cast<signed char>(Letter - '0');
    }
    if (Letter >= 'a' && Letter <= 'f') {
      return static_cast<signed char>(Letter - 'a' + 10);
    }
    return -1;
  }

  // Converts what could be a hex string to an integer value.
  // Result must fit into 32 bits.  More than that is an error.
  // Like everything else in this particular API, it returns false on success.
  virtual bool tokenToHex(uint64_t &Res, const AsmToken &Tok) {
    Res = 0;
    std::string Text = Tok.getString().str();
    if (Text.size() > 8) {
      return true;
    }
    std::transform(Text.begin(), Text.end(), Text.begin(),
                   [](unsigned char C) { return std::tolower(C); });
    for (char Letter : Text) {
      signed char Converted = hexToChar(Letter);
      if (Converted == -1) {
        return true;
      }
      Res = (Res * 16) + Converted;
    }
    return false;
  }

  void eatThatToken(OperandVector &Operands) {
    Operands.push_back(MC6809Operand::createToken(getLexer().getTok().getString(),
                                               getLexer().getLoc()));
    Lex();
  }

  bool tryParseRelocExpression(OperandVector &Operands) {
    bool IsNegated = false;
    MC6809MCExpr::VariantKind ModifierKind = MC6809MCExpr::VK_MC6809_NONE;

    SMLoc S = Parser.getTok().getLoc();

    // Check for sign
    AsmToken tokens[2];
    size_t ReadCount = Parser.getLexer().peekTokens(tokens);

    if (ReadCount == 2) {
      if ((tokens[0].getKind() == AsmToken::Identifier &&
           tokens[1].getKind() == AsmToken::LParen) ||
          (tokens[0].getKind() == AsmToken::LParen &&
           tokens[1].getKind() == AsmToken::Minus)) {

        AsmToken::TokenKind CurTok = Parser.getLexer().getKind();
        if (CurTok == AsmToken::Minus ||
            tokens[1].getKind() == AsmToken::Minus) {
          IsNegated = true;
        } else {
          assert(CurTok == AsmToken::Plus);
          IsNegated = false;
        }

        // Eat the sign
        if (CurTok == AsmToken::Minus || CurTok == AsmToken::Plus)
          Parser.Lex();
      }
    }

    MCExpr const *InnerExpression;
    if (Parser.getTok().getKind() == AsmToken::Less ||
        Parser.getTok().getKind() == AsmToken::Greater) {

      switch (Parser.getTok().getKind()) {
      case AsmToken::Less:
        ModifierKind = MC6809MCExpr::VK_MC6809_ADDR_8;
        break;
      case AsmToken::Greater:
        ModifierKind = MC6809MCExpr::VK_MC6809_ADDR_16;
        break;
      default:
        assert(false);
      }

      Parser.Lex();

      if (getParser().parseExpression(InnerExpression))
        return true;

    } else {
      // Check if we have a target specific modifier (lo8, hi8, &c)
      if (Parser.getTok().getKind() != AsmToken::Identifier ||
          Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
        // Not a reloc expr
        return true;
      }
      StringRef ModifierName = Parser.getTok().getString();
      ModifierKind = MC6809MCExpr::getKindByName(ModifierName.str());

      if (ModifierKind != MC6809MCExpr::VK_MC6809_NONE) {
        Parser.Lex();
        Parser.Lex(); // Eat modifier name and parenthesis
        if (Parser.getTok().getString() == GenerateStubs &&
            Parser.getTok().getKind() == AsmToken::Identifier) {
          std::string GSModName = ModifierName.str() + "_" + GenerateStubs;
          ModifierKind = MC6809MCExpr::getKindByName(GSModName);
          if (ModifierKind != MC6809MCExpr::VK_MC6809_NONE) {
            Parser.Lex(); // Eat gs modifier name
          }
        }
      } else {
        return Error(Parser.getTok().getLoc(), "unknown modifier");
      }

      if (tokens[1].getKind() == AsmToken::Minus ||
          tokens[1].getKind() == AsmToken::Plus) {
        Parser.Lex();
        assert(Parser.getTok().getKind() == AsmToken::LParen);
        Parser.Lex(); // Eat the sign and parenthesis
      }

      if (getParser().parseExpression(InnerExpression))
        return true;

      if (tokens[1].getKind() == AsmToken::Minus ||
          tokens[1].getKind() == AsmToken::Plus) {
        assert(Parser.getTok().getKind() == AsmToken::RParen);
        Parser.Lex(); // Eat closing parenthesis
      }

      // If we have a modifier wrap the inner expression
      assert(Parser.getTok().getKind() == AsmToken::RParen);
      Parser.Lex(); // Eat closing parenthesis
    }

    MCExpr const *Expression = MC6809MCExpr::create(ModifierKind, InnerExpression,
                                                 IsNegated, getContext());

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    Operands.push_back(MC6809Operand::createImm(Expression, S, E));

    return false;
  }

  bool tryParseExpr(OperandVector &Operands, StringRef ErrorMsg) {
    if (!tryParseRelocExpression(Operands)) {
      return false;
    }

    MCExpr const *Expression;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (getParser().parseExpression(Expression)) {
      Parser.eatToEndOfStatement();
      return Error(getLexer().getLoc(), ErrorMsg);
    }
    Operands.push_back(MC6809Operand::createImm(Expression, S, E));
    return false;
  }

  OperandMatchResultTy tryParseRegister(unsigned &RegNo, SMLoc &StartLoc,
                                        SMLoc &EndLoc) override {
    std::string AnyCase(StartLoc.getPointer(),
                        EndLoc.getPointer() - StartLoc.getPointer());
    std::transform(AnyCase.begin(), AnyCase.end(), AnyCase.begin(),
                   [](unsigned char C) { return std::tolower(C); });
    StringRef RegisterName(AnyCase.c_str(), AnyCase.size());
    RegNo = MatchRegisterName(RegisterName);
    if (RegNo == 0) {
      // If the user has requested to ignore short register names, then ignore
      // them
      if (getSTI().getFeatureBits()[MC6809::FeatureAltRegisterNamesOnly] ==
          false) {
        RegNo = MatchRegisterAltName(RegisterName);
      }
    }
    return (RegNo != 0) ? MatchOperand_Success : MatchOperand_NoMatch;
  }

  OperandMatchResultTy tryParseRegister(OperandVector &Operands) {
    unsigned RegNo = 0;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (tryParseRegister(RegNo, S, E) == MatchOperand_Success) {
      Operands.push_back(MC6809Operand::createReg(RegNo, S, E));
      return MatchOperand_Success;
    }
    return MatchOperand_NoMatch;
  }

  // This depends on some stuff in MC6809GenAsmMatcher.inc, so we can't define it
  // inline like everything else in this file.  See below.
  OperandMatchResultTy tryParseAsmParamRegClass(OperandVector &Operands);

  bool ParseInstruction(ParseInstructionInfo & /*Info*/, StringRef Mnemonic,
                        SMLoc NameLoc, OperandVector &Operands) override {
    /*
    On 65xx family instructions, mnemonics and addressing modes take the form:

    mnemonic (#)expr
    mnemonic [(]expr[),xy]*
    mnemonic a

    65816 only:
    mnemonic [(]expr[),sxy]*
    mnemonic \[ expr \]

    Any constant may be prefixed by a $, indicating that it is a hex constant.
    Such onstants can appear anywhere an integer appears in an expr, so expr
    parsing needs to take that into account.

    Handle all these cases, fairly loosely, and let tablegen sort out what's
    what.

    */
    // First, the mnemonic goes on the stack.
    Operands.push_back(MC6809Operand::createToken(Mnemonic, NameLoc));
    bool FirstTime = true;
    while (getLexer().isNot(AsmToken::EndOfStatement)) {
      if (getLexer().is(AsmToken::Hash)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands,
                          "immediate operand must be an expression evaluating "
                          "to a value between 0 and 255 inclusive")) {
          FirstTime = false;
          continue;
        }
      }
      if (getLexer().is(AsmToken::LParen)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands,
                          "expression expected after left parenthesis")) {
          FirstTime = false;
          continue;
        }
      }
      // I don't know what llvm has against commas, but for some reason
      // TableGen makes an effort to ignore them during parsing.  So,
      // strangely enough, we have to throw out commas too, even though
      // they have semantic meaning on MC6809 platforms.
      if (getLexer().is(AsmToken::Comma)) {
        Lex();
        continue;
      }

      StringRef TokName;
      SMLoc TokLoc;
      TokName = getLexer().getTok().getString();
      TokLoc = getLexer().getTok().getLoc();

      // We'll only ever see this on rol a or asl a commands, so handle it
      // as a special case
      if (FirstTime && (TokName == "a" || TokName == "A")) {
        eatThatToken(Operands);
        FirstTime = false;
        continue;
      }

      // We'll only ever see a register name on the third or later parameter.
      if (tryParseAsmParamRegClass(Operands) == MatchOperand_Success) {
        Parser.Lex();
        continue;
      }

      if (FirstTime && !tryParseExpr(Operands, "expression expected")) {
        FirstTime = false;
        continue;
      }
      FirstTime = false;

      // Okay then, you're a token.  Hope you're happy.
      eatThatToken(Operands);
    }
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  }

  bool ParseRegister(unsigned &RegNo, SMLoc &StartLoc, SMLoc &EndLoc) override {

    auto Result = tryParseRegister(RegNo, StartLoc, EndLoc);
    return (Result != MatchOperand_Success);
  }

}; // class MC6809AsmParser

#define GET_MATCHER_IMPLEMENTATION
#include "MC6809GenAsmMatcher.inc"

// Parse only registers that can be considered parameters to real MC6809
// instructions.  The instruction parser considers x, y, and s to be
// strings, not registers, so make a point of filtering those cases out
// of what's acceptable.
OperandMatchResultTy
MC6809AsmParser::tryParseAsmParamRegClass(OperandVector &Operands) {
  unsigned RegNo = 0;
  SMLoc S = getLexer().getLoc();
  SMLoc E = getLexer().getTok().getEndLoc();
  if (tryParseRegister(RegNo, S, E) == MatchOperand_Success) {
    auto AsmParamReg = MC6809Operand::createReg(RegNo, S, E);
    // If it's x, y, or s, then drop it
    if (validateOperandClass(*AsmParamReg, MCK_MC6809AsmParamRegClass) ==
        MCTargetAsmParser::Match_Success)
      return MatchOperand_NoMatch;
    Operands.push_back(MC6809Operand::createReg(RegNo, S, E));
    return MatchOperand_Success;
  }
  return MatchOperand_NoMatch;
}

extern "C" void LLVM_EXTERNAL_VISIBILITY
LLVMInitializeMC6809AsmParser() { // NOLINT
  RegisterMCAsmParser<MC6809AsmParser> X(getTheMC6809Target());
}

} // namespace llvm
