//===--- MC6809AsmParser.cpp - Parse MC6809 assembly to MCInst instructions --===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809FixupKinds.h"
#include "MCTargetDesc/MC6809MCELFStreamer.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MCTargetDesc/MC6809TargetStreamer.h"
#include "llvm/ADT/APInt.h"
#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringSwitch.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCAsmMacro.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstBuilder.h"
#include "llvm/MC/MCParser/AsmLexer.h"
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
  /// Create an immediate MC6809Operand.
  MC6809Operand(const MC6809Subtarget &STI, const MCExpr *Val, SMLoc S, SMLoc E) : Kind(k_Immediate), Imm(Val), Start(S), End(E){};
  /// Create a register MC6809Operand.
  MC6809Operand(const MC6809Subtarget &STI, unsigned RegNum, SMLoc S, SMLoc E) : Kind(k_Register), Reg(RegNum), Start(S), End(E){};
  /// Create a token MC6809Operand.
  MC6809Operand(const MC6809Subtarget &STI, StringRef Str, SMLoc S) : Kind(k_Token), Tok(Str), Start(S){};
  /// Create a register list MC6809Operand.
  MC6809Operand(const MC6809Subtarget &STI, SmallVector<unsigned, 8> Regs, SMLoc S, SMLoc E) : Kind(k_RegList), RegListRegs(std::move(Regs)), Start(S), End(E){};

private:
  enum KindTy {
    k_None,
    k_Immediate,
    k_Register,
    k_Token,
    k_RegList,
  } Kind{k_None};

  MCExpr const *Imm{};
  unsigned int Reg{};
  StringRef Tok;
  SmallVector<unsigned, 8> RegListRegs;
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
      // Imm16 modifier enforces a lower bound which rejects Imm8.
      if (Kind == MC6809::Imm16 && High < 0x10000 - 1)
        return false;
      const MCFixupKindInfo &Info = MC6809FixupKinds::getFixupKindInfo(Kind, nullptr);
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

  bool is(KindTy K) const { return (Kind == K); }
  bool isToken() const override { return is(k_Token); }
  bool isImm() const override { return is(k_Immediate); }
  bool isReg() const override { return is(k_Register); }
  bool isMem() const override {
    assert(false);
    return false;
  }
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
  MCRegister getReg() const override {
    assert(isReg());
    return Reg;
  }

  bool isImm3() const { return isImmediate<0, 0x8 - 1>(); }
  bool isImm5() const { return isImmediate<0, 0x20 - 1>(); }
  bool isImm8() const { return isImmediate<-0x80, 0xFF>(); }
  bool isImm16() const { return isImmediate<-0x8000, 0xFFFF>(); }
  bool isImm8To16() const { return isImm16() && !isImm8(); }

  bool isRel5() const { return isImmediate<-16, 15>(); }
  bool isRel8() const { return isImmediate<-128, 127>(); }
  bool isRel16() const { return isImmediate<-32768, 32767>(); }

  // PC-relative for short branches (BRA/BEQ/...). Symbol refs accepted —
  // there's no relaxation rule, but in practice every short-branch use
  // either has a numeric offset or a *nearby* label, and codegen avoids
  // emitting short branches at all (bug #58). Hand-written runtime asm
  // relies on this lenient form.
  bool isPCRel8() const { return isImmediate<-128, 255>(); }
  bool isPCRel16() const { return isImmediate<-32768, 65535>(); }

  // PC-relative for indexed addressing (LEAX/LDD/STX/etc with `,PCR`).
  // Strict: symbol refs are rejected so that the matcher always picks
  // the 16-bit form for symbolic operands. Why: the assembler has no
  // relaxation rule to promote o8PC to o16PC, and indexed PC-rel
  // operands routinely target labels that are *outside* the ±127 byte
  // range (jump tables sit after the function body, etc.). Accepting
  // a symbol ref at the 8-bit indexed form silently truncates when the
  // offset overflows. This was bug #68: a `LEAX JT,PCR` for a jump
  // table 919 bytes away got encoded with offset -105 and jumped into
  // the middle of the function. Restricting only the indexed-PC-rel
  // path keeps short branches working unchanged.
  bool isPCRel8Idx() const {
    if (isImm() && isa<MCSymbolRefExpr>(getImm()))
      return false;
    return isImmediate<-128, 255>();
  }

  bool isAddr8() const {
    // For constants, use the offseted direct page.
    auto DirectPageOffset = 0;
    if (DirectPageOffset != 0 && isImm()) {
      const auto *CE = dyn_cast<MCConstantExpr>(getImm());
      if (CE) {
        int64_t Value = CE->getValue();
        return Value >= DirectPageOffset && Value <= DirectPageOffset + 0xFF;
      }
    }
    return isImm8();
  }
  bool isAddr16() const { return isImm16(); }

  bool isRegAny() const { return isReg(); }

  bool isRegList() const { return is(k_RegList); }

  bool isCondCode() const {
    if (!isImm())
      return false;
    const auto *CE = dyn_cast<MCConstantExpr>(getImm());
    if (!CE)
      return false;
    int64_t Value = CE->getValue();
    return Value >= 0 && Value <= 15;
  }

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

  void addRegOperands(MCInst &Inst, unsigned /*N*/) const { Inst.addOperand(MCOperand::createReg(getReg())); }

  void addRegAnyOperands(MCInst &Inst, unsigned /*N*/) const { Inst.addOperand(MCOperand::createReg(getReg())); }

  void addRegListOperands(MCInst &Inst, unsigned /*N*/) const {
    assert(Kind == k_RegList && "Unexpected operand kind");
    for (unsigned R : RegListRegs)
      Inst.addOperand(MCOperand::createReg(R));
  }

  void addRel5Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addRel8Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addRel16Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addPCRel8Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addPCRel8IdxOperands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addPCRel16Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addAddr8Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addAddr16Operands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  void addCondCodeOperands(MCInst &Inst, unsigned N) const { addImmOperands(Inst, N); }

  static std::unique_ptr<MC6809Operand> createImm(const MC6809Subtarget &STI, const MCExpr *Val, SMLoc S, SMLoc E) { return std::make_unique<MC6809Operand>(STI, Val, S, E); }

  static std::unique_ptr<MC6809Operand> createReg(const MC6809Subtarget &STI, unsigned RegNum, SMLoc S, SMLoc E) { return std::make_unique<MC6809Operand>(STI, RegNum, S, E); }

  static std::unique_ptr<MC6809Operand> createToken(const MC6809Subtarget &STI, StringRef Str, SMLoc S) { return std::make_unique<MC6809Operand>(STI, Str, S); }

  static std::unique_ptr<MC6809Operand> createRegList(const MC6809Subtarget &STI, SmallVector<unsigned, 8> Regs, SMLoc S, SMLoc E) { return std::make_unique<MC6809Operand>(STI, std::move(Regs), S, E); }

  void print(raw_ostream &O, const MCAsmInfo &MAI) const override {
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
    case k_Immediate:
      O << "Immediate: \"";
      MAI.printExpr(O, *getImm());
      O << "\"";
      break;
    case k_RegList:
      O << "RegList: {";
      for (unsigned I = 0; I < RegListRegs.size(); I++) {
        if (I > 0) O << ",";
        O << RegListRegs[I];
      }
      O << "}";
      break;
    }
    O << "\n";
  };
};

/// Parses MC6809 assembly from a stream.
class MC6809AsmParser : public MCTargetAsmParser {
  const MC6809Subtarget &MCSTI;
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

  MC6809AsmParser(const MCSubtargetInfo &STI, MCAsmParser &Parser, const MCInstrInfo &MII, const MCTargetOptions &Options) : MCTargetAsmParser(Options, STI, MII), MCSTI(static_cast<const MC6809Subtarget &>(STI)), Parser(Parser) {
    MCAsmParserExtension::Initialize(Parser);
    MRI = getContext().getRegisterInfo();

    Parser.addAliasForDirective(".hword", ".byte");
    Parser.addAliasForDirective(".word", ".2byte");
    Parser.addAliasForDirective(".dword", ".4byte");
    Parser.addAliasForDirective(".xword", ".8byte");

    setAvailableFeatures(ComputeAvailableFeatures(STI.getFeatureBits()));
  }
  AsmLexer &getLexer() const { return Parser.getLexer(); }
  MCAsmParser &getParser() const { return Parser; }

  bool parsePrimaryExpr(const MCExpr *&Res, SMLoc &EndLoc) override { return MCTargetAsmParser::parsePrimaryExpr(Res, EndLoc); }

  bool invalidOperand(SMLoc const &Loc, OperandVector const &Operands, uint64_t const &ErrorInfo) {
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

  bool missingFeature(llvm::SMLoc const &Loc, uint64_t const & /*ErrorInfo*/) { return Error(Loc, "instruction requires a CPU feature not currently enabled"); }

  bool emit(MCInst &Inst, SMLoc const &Loc, MCStreamer &Out) const {
    Inst.setLoc(Loc);
    Out.emitInstruction(Inst, MCSTI);

    return false;
  }

  /// MatchAndEmitInstruction - Recognize a series of operands of a parsed
  /// instruction as an actual MCInst and emit it to the specified MCStreamer.
  /// This returns false on success and returns true on failure to match.
  ///
  /// On failure, the target parser is responsible for emitting a diagnostic
  /// explaining the match failure.
  bool matchAndEmitInstruction(SMLoc Loc, unsigned & /*Opcode*/, OperandVector &Operands, MCStreamer &Out, uint64_t &ErrorInfo, bool MatchingInlineAsm) override {
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
      return Error(Loc, "operand must be a 16-bit address");
    case Match_InvalidPCRel8:
      return Error(Loc, "operand must be an 8-bit PC relative address");
    case Match_InvalidPCRel8Idx:
      return Error(Loc, "operand must be an 8-bit PC relative literal "
                        "(symbol references must use the 16-bit form)");
    case Match_immediate:
      return Error(Loc, "operand must be an immediate value");
    case Match_NearMisses:
      return Error(Loc, "found some near misses");
    default:
      return true;
    }
  }

  MC6809TargetStreamer &getTargetStreamer() {
    MCTargetStreamer &TS = *getParser().getStreamer().getTargetStreamer();
    return static_cast<MC6809TargetStreamer &>(TS);
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
    StringRef IDVal = DirectiveID.getIdentifier();
    if (IDVal.starts_with(".directpage"))
      return parseDirectpage(DirectiveID.getLoc());
    return true;
  }

  bool parseDirectpage(SMLoc DirectiveLoc) {
    auto parseOp = [&]() -> bool {
      StringRef Name;
      SMLoc Loc = getTok().getLoc();
      if (Parser.parseIdentifier(Name))
        return Error(Loc, "expected identifier");

      if (Parser.discardLTOSymbol(Name))
        return false;

      MCSymbol *Sym = getContext().getOrCreateSymbol(Name);

      if (!getTargetStreamer().emitDirectiveDirectPage(Sym))
        return Error(Loc, "unable to mark symbol as direct page");
      return false;
    };

    return parseMany(parseOp);
  }

  signed char hexToChar(const signed char Letter) {
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
  bool tokenToHex(uint64_t &Res, const AsmToken &Tok) {
    Res = 0;
    std::string Text = Tok.getString().str();
    if (Text.size() > 8) {
      return true;
    }
    std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char C) { return std::tolower(C); });
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
    Operands.push_back(MC6809Operand::createToken(MCSTI, getLexer().getTok().getString(), getLexer().getLoc()));
    Lex();
  }

  bool tryPushSPC700AdditionExpr(OperandVector &Operands, const MCExpr *LHS, const MCBinaryExpr *BE, SMLoc S, SMLoc E) {
    // On SPC700, the expression can end in +X or +Y, which should be parsed
    // as an addressing mode, not as part of the expression.
    if (const auto *SE = dyn_cast<MCSymbolRefExpr>(BE->getRHS())) {
      if ((SE->getSymbol().getName().equals_insensitive("x") || SE->getSymbol().getName().equals_insensitive("y")) && BE->getOpcode() == MCBinaryExpr::Add) {
        Operands.push_back(MC6809Operand::createImm(MCSTI, LHS, S, E));
        Operands.push_back(MC6809Operand::createToken(MCSTI, "+", BE->getLoc()));
        Operands.push_back(MC6809Operand::createToken(MCSTI, SE->getSymbol().getName(), SE->getLoc()));
        return false;
      }
    }
    return true;
  }

  void pushExpr(OperandVector &Operands, const MCExpr *Val, SMLoc S, SMLoc E) { Operands.push_back(MC6809Operand::createImm(MCSTI, Val, S, E)); }

  enum ExpressionType { ExprTypeOther, ExprTypeImmediate, ExprTypeAddress };

  bool tryParseRelocExpression(OperandVector &Operands, ExpressionType EType) {
    bool IsNegated = false;
    MC6809MCExpr::VariantKind ModifierKind = MC6809MCExpr::VK_NONE;

    SMLoc S = Parser.getTok().getLoc();

    // Check for sign
    AsmToken tokens[2];
    size_t ReadCount = Parser.getLexer().peekTokens(tokens);

    if (ReadCount == 2) {
      if ((tokens[0].getKind() == AsmToken::Identifier && tokens[1].getKind() == AsmToken::LParen) || (tokens[0].getKind() == AsmToken::LParen && tokens[1].getKind() == AsmToken::Minus)) {

        AsmToken::TokenKind CurTok = Parser.getLexer().getKind();
        if (CurTok == AsmToken::Minus || tokens[1].getKind() == AsmToken::Minus) {
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

    /*
    Shorthands for addressing modes conform to WDC's 65816 standard:

    #<addr == mos16lo(addr)
    #>addr == mos16hi(addr)
    #^addr == mos24bank(addr)

     <addr == mos8(addr)
     |addr == mos16(addr)
     !addr == mos16(addr)
     >addr == mos24(addr)
    */
    MCExpr const *InnerExpression;
    if (EType == ExprTypeImmediate && (Parser.getTok().getKind() == AsmToken::Less || Parser.getTok().getKind() == AsmToken::Greater || Parser.getTok().getKind() == AsmToken::Caret)) {

      bool IsImm16 = false;
      switch (Parser.getTok().getKind()) {
      case AsmToken::Less:
        ModifierKind = IsImm16 ? MC6809MCExpr::VK_ADDR16 : MC6809MCExpr::VK_ADDR8;
        break;
      case AsmToken::Greater:
        ModifierKind = MC6809MCExpr::VK_ADDR16;
        break;
      default:
        assert(false);
      }

      Parser.Lex();

      if (getParser().parseExpression(InnerExpression))
        return true;
    } else if (EType == ExprTypeAddress && (Parser.getTok().getKind() == AsmToken::Less || Parser.getTok().getKind() == AsmToken::Greater || Parser.getTok().getKind() == AsmToken::Pipe || Parser.getTok().getKind() == AsmToken::Exclaim)) {

      switch (Parser.getTok().getKind()) {
      case AsmToken::Less:
        ModifierKind = MC6809MCExpr::VK_ADDR8;
        break;
      case AsmToken::Greater:
        ModifierKind = MC6809MCExpr::VK_ADDR16;
        break;
      default:
        assert(false);
      }

      Parser.Lex();

      if (getParser().parseExpression(InnerExpression))
        return true;

    } else {
      // Check if we have a target specific modifier (lo8, hi8, &c)
      if (Parser.getTok().getKind() != AsmToken::Identifier || Parser.getLexer().peekTok().getKind() != AsmToken::LParen) {
        // Not a reloc expr
        return true;
      }
      StringRef ModifierName = Parser.getTok().getString();
      ModifierKind = MC6809MCExpr::getKindByName(ModifierName.str(), EType != ExprTypeAddress);

      if (ModifierKind != MC6809MCExpr::VK_NONE) {
        Parser.Lex();
        Parser.Lex(); // Eat modifier name and parenthesis
        if (Parser.getTok().getString() == GenerateStubs && Parser.getTok().getKind() == AsmToken::Identifier) {
          std::string GSModName = ModifierName.str() + "_" + GenerateStubs;
          ModifierKind = MC6809MCExpr::getKindByName(GSModName, EType != ExprTypeAddress);
          if (ModifierKind != MC6809MCExpr::VK_NONE) {
            Parser.Lex(); // Eat gs modifier name
          }
        }
      } else {
        return Error(Parser.getTok().getLoc(), "unknown modifier");
      }

      if (tokens[1].getKind() == AsmToken::Minus || tokens[1].getKind() == AsmToken::Plus) {
        Parser.Lex();
        assert(Parser.getTok().getKind() == AsmToken::LParen);
        Parser.Lex(); // Eat the sign and parenthesis
      }

      if (getParser().parseExpression(InnerExpression))
        return true;

      if (tokens[1].getKind() == AsmToken::Minus || tokens[1].getKind() == AsmToken::Plus) {
        assert(Parser.getTok().getKind() == AsmToken::RParen);
        Parser.Lex(); // Eat closing parenthesis
      }

      // If we have a modifier wrap the inner expression
      assert(Parser.getTok().getKind() == AsmToken::RParen);
      Parser.Lex(); // Eat closing parenthesis
    }

    MCExpr const *Expression = MC6809MCExpr::create(ModifierKind, InnerExpression, IsNegated, getContext());

    SMLoc E = SMLoc::getFromPointer(Parser.getTok().getLoc().getPointer() - 1);
    pushExpr(Operands, Expression, S, E);

    return false;
  }

  bool tryParseExpr(OperandVector &Operands, ExpressionType EType, StringRef ErrorMsg) {
    if (!tryParseRelocExpression(Operands, EType)) {
      return false;
    }

    MCExpr const *Expression;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (getParser().parseExpression(Expression)) {
      Parser.eatToEndOfStatement();
      return Error(getLexer().getLoc(), ErrorMsg);
    }
    pushExpr(Operands, Expression, S, E);
    return false;
  }

  ParseStatus tryParseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override {
    std::string AnyCase(StartLoc.getPointer(), EndLoc.getPointer() - StartLoc.getPointer());
    std::transform(AnyCase.begin(), AnyCase.end(), AnyCase.begin(), [](unsigned char C) { return std::tolower(C); });
    StringRef RegisterName(AnyCase.c_str(), AnyCase.size());
    Reg = MatchRegisterName(RegisterName);
    if (Reg == 0) {
      // If the user has requested to ignore short register names, then ignore
      // them
      if (getSTI().getFeatureBits()[MC6809::FeatureAltRegisterNamesOnly] == false) {
        Reg = MatchRegisterAltName(RegisterName);
      }
    }
    return (Reg != 0) ? ParseStatus::Success : ParseStatus::NoMatch;
  }

  ParseStatus tryParseRegister(OperandVector &Operands) {
    MCRegister Reg = 0;
    SMLoc S = getLexer().getLoc();
    SMLoc E = getLexer().getTok().getEndLoc();
    if (tryParseRegister(Reg, S, E).isSuccess()) {
      Operands.push_back(MC6809Operand::createReg(MCSTI, Reg, S, E));
      return ParseStatus::Success;
    }
    return ParseStatus::NoMatch;
  }

  // Parse a comma-separated list of register names for push/pull instructions.
  // Called by the AsmMatcher via the ParserMethod on RegListAsmOperand.
  // Produces a single k_RegList operand containing all the registers.
  ParseStatus parseRegList(OperandVector &Operands) {
    SMLoc S = getLexer().getLoc();
    SMLoc E;
    SmallVector<unsigned, 8> Regs;

    while (true) {
      MCRegister Reg = 0;
      SMLoc RS = getLexer().getLoc();
      E = getLexer().getTok().getEndLoc();
      if (tryParseRegister(Reg, RS, E).isFailure())
        return ParseStatus::Failure;
      if (Reg == 0)
        break;
      Regs.push_back(Reg);
      Parser.Lex(); // Consume the register token
      if (getLexer().isNot(AsmToken::Comma))
        break;
      Parser.Lex(); // Consume the comma
    }

    if (Regs.empty())
      return ParseStatus::NoMatch;

    Operands.push_back(MC6809Operand::createRegList(MCSTI, std::move(Regs), S, E));
    return ParseStatus::Success;
  }

  // Parse only registers that can be considered parameters to real MC6809
  // instructions.
  //
  // For MOS targets, a, x, y, z, and s are treated as literal token strings
  // (not register operands) because MOS AsmString patterns use them as tokens.
  //
  // For MC6809 targets, INDEX16 registers (x, y, u, s) are parsed as register
  // operands so the AsmMatcher can match them against INDEX16:$ireg in AsmString
  // patterns. Other register names (a, b, d, pc) are left as tokens because
  // they appear as literal tokens in accumulator-offset and PC-relative patterns.
  ParseStatus tryParseAsmParamRegClass(OperandVector &Operands) {
    SMLoc S = getLexer().getLoc();

    // MC6809: parse all register names as register operands. The AsmMatcher
    // treats register names in AsmString patterns (e.g., "a" in "a , $ireg")
    // as register class constraints, so they must be register operands.
    // Only try register matching for identifier tokens — integer tokens like
    // "0" can match numeric register names (e.g., A0 is named "0") but should
    // be parsed as expressions.
    if (getSTI().getFeatureBits()[MC6809::Feature6809]) {
      if (getLexer().isNot(AsmToken::Identifier))
        return ParseStatus::NoMatch;
      MCRegister Reg = 0;
      SMLoc E = getLexer().getTok().getEndLoc();
      if (tryParseRegister(Reg, S, E).isSuccess()) {
        Operands.push_back(MC6809Operand::createReg(MCSTI, Reg, S, E));
        return ParseStatus::Success;
      }
      return ParseStatus::NoMatch;
    }

    // MOS: some register names are treated as token strings.
    const char *LowerStr = StringSwitch<const char *>(getLexer().getTok().getString())
                               .CaseLower("a", "a")
                               .CaseLower("x", "x")
                               .CaseLower("y", "y")
                               .CaseLower("z", "z")
                               .CaseLower("s", "s")
                               .CaseLower("sp", "s")
                               .CaseLower("r", "r")     // 65EL02
                               .CaseLower("rp", "r")    // 65EL02
                               .CaseLower("ya", "ya")   // SPC700
                               .CaseLower("c", "c")     // SPC700
                               .CaseLower("psw", "psw") // SPC700
                               .Default(nullptr);
    if (LowerStr != nullptr) {
      Operands.push_back(MC6809Operand::createToken(MCSTI, LowerStr, S));
      return ParseStatus::Success;
    }

    MCRegister Reg = 0;
    SMLoc E = getLexer().getTok().getEndLoc();
    if (tryParseRegister(Reg, S, E).isSuccess()) {
      Operands.push_back(MC6809Operand::createReg(MCSTI, Reg, S, E));
      return ParseStatus::Success;
    }
    return ParseStatus::NoMatch;
  }

  bool parseInstruction(ParseInstructionInfo & /*Info*/, StringRef Mnemonic, SMLoc NameLoc, OperandVector &Operands) override {
    /*
    On 65xx family instructions, mnemonics and addressing modes take the form:

    mnemonic (#)expr
    mnemonic [(]expr[),xy]*
    mnemonic a

    65816 and 45GS02 only:
    mnemonic [(]expr[),sxyz]*
    mnemonic \[ expr \]

    SPC700:
    mnemonic expr.bit
    mnemonic [(]expr[)]+[xy]*
    mnemonic \[ expr \]

    Any constant may be prefixed by a $, indicating that it is a hex constant.
    Such onstants can appear anywhere an integer appears in an expr, so expr
    parsing needs to take that into account.

    Handle all these cases, fairly loosely, and let tablegen sort out what's
    what.

    */
    // MC6809: conditional branches use "b$cond" / "lb$cond" mnemonics.
    // The AsmMatcher expects mnemonic "b" or "lb" with a separate CondCode
    // operand. Split the mnemonic and push the condition code as an immediate.
    if (getSTI().getFeatureBits()[MC6809::Feature6809]) {
      std::string MnLower = Mnemonic.lower();
      StringRef Prefix;
      StringRef CCStr;
      if (MnLower.size() > 2 && MnLower.substr(0, 2) == "lb") {
        Prefix = "lb";
        CCStr = StringRef(MnLower).substr(2);
      } else if (MnLower.size() > 1 && MnLower[0] == 'b') {
        Prefix = "b";
        CCStr = StringRef(MnLower).substr(1);
      }
      if (!Prefix.empty()) {
        int CCVal = StringSwitch<int>(CCStr)
                        .Case("hi", MC6809CC::HI)
                        .Case("ls", MC6809CC::LS)
                        .Case("hs", MC6809CC::HS)
                        .Case("cs", MC6809CC::CS)
                        .Case("lo", MC6809CC::LO)
                        .Case("cc", MC6809CC::CC)
                        .Case("ne", MC6809CC::NE)
                        .Case("eq", MC6809CC::EQ)
                        .Case("vc", MC6809CC::VC)
                        .Case("vs", MC6809CC::VS)
                        .Case("pl", MC6809CC::PL)
                        .Case("mi", MC6809CC::MI)
                        .Case("ge", MC6809CC::GE)
                        .Case("lt", MC6809CC::LT)
                        .Case("gt", MC6809CC::GT)
                        .Case("le", MC6809CC::LE)
                        .Default(-1);
        if (CCVal >= 0) {
          // Push the shortened mnemonic ("b" or "lb") and the CC operand.
          Operands.push_back(MC6809Operand::createToken(MCSTI, Prefix, NameLoc));
          const MCExpr *CCExpr = MCConstantExpr::create(CCVal, getContext());
          SMLoc E = SMLoc::getFromPointer(NameLoc.getPointer() + Mnemonic.size());
          Operands.push_back(MC6809Operand::createImm(MCSTI, CCExpr, NameLoc, E));
          // Fall through to parse the target operand normally.
          goto parse_operands;
        }
      }
    }

    // First, the mnemonic goes on the stack.
    Operands.push_back(MC6809Operand::createToken(MCSTI, Mnemonic, NameLoc));

parse_operands:
    // MC6809: push/pull instructions take a comma-separated register list.
    // Parse them specially before the main loop consumes registers individually.
    if (getSTI().getFeatureBits()[MC6809::Feature6809]) {
      std::string MnLower = Mnemonic.lower();
      if (MnLower == "pshs" || MnLower == "puls" ||
          MnLower == "pshu" || MnLower == "pulu") {
        if (parseRegList(Operands).isSuccess()) {
          Parser.Lex(); // Consume the EndOfStatement
          return false;
        }
        return Error(getLexer().getLoc(), "expected register list");
      }
    }

    AsmToken::TokenKind RightHandSide = AsmToken::Eof;
    while (getLexer().isNot(AsmToken::EndOfStatement) && getLexer().isNot(AsmToken::Eof)) {
      // Handle special characters.
      if (getLexer().is(AsmToken::Hash)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands, ExprTypeImmediate, "immediate operand must be an expression evaluating to a value between 0 and 65535 inclusive")) {
          continue;
        }
      }
      // Handle parentheses and brackets.
      if (getLexer().is(AsmToken::LParen)) {
        eatThatToken(Operands);
        if (!tryParseExpr(Operands, ExprTypeAddress, "expression expected after left parenthesis")) {
          RightHandSide = AsmToken::RParen;
          continue;
        }
      }
      // MC6809: push [ and ] as tokens for the AsmMatcher. The AsmString
      // patterns include literal [ and ] (e.g., "[ , $ireg ]", "[ $offset , $ireg ]").
      // MOS: parse [ expr ] as a bracketed expression.
      if (getLexer().is(AsmToken::LBrac)) {
        if (getSTI().getFeatureBits()[MC6809::Feature6809]) {
          eatThatToken(Operands);
          continue;
        }
        eatThatToken(Operands);
        if (!tryParseExpr(Operands, ExprTypeAddress, "expression expected after left bracket")) {
          RightHandSide = AsmToken::RBrac;
          continue;
        }
      }
      if (getLexer().is(AsmToken::RBrac)) {
        eatThatToken(Operands);
        continue;
      }
      if (RightHandSide != AsmToken::Eof && getLexer().is(RightHandSide)) {
        eatThatToken(Operands);
        RightHandSide = AsmToken::Eof;
        continue;
      }
      // The AsmMatcher treats commas in AsmString patterns as operand separators,
      // not matchable tokens. Consume them so the operand stream contains only
      // the actual operands (offsets, registers, etc.).
      if (getLexer().is(AsmToken::Comma)) {
        Lex();
        continue;
      }

      // MC6809: handle < and > address mode prefixes as tokens.
      // The Direct AsmString pattern uses a literal "<" token ("lda < $addr"),
      // so we push it as a token for the AsmMatcher. The ">" prefix is consumed
      // silently since Extended is the default (no ">" in the AsmString).
      if (getSTI().getFeatureBits()[MC6809::Feature6809]) {
        if (getLexer().is(AsmToken::Less)) {
          eatThatToken(Operands);
          continue;
        }
        if (getLexer().is(AsmToken::Greater)) {
          Lex(); // Consume > silently; extended is the default
          continue;
        }
      }

      // We'll only ever see a register name on the third or later parameter.
      if (tryParseAsmParamRegClass(Operands).isSuccess()) {
        Parser.Lex();
        continue;
      }

      // MC6809: handle +/- as auto-increment/decrement tokens when they
      // appear after a register (post-inc: ,x+ ,x++) or after another +/-
      // (second char of ++ or --). Also handle pre-decrement (,-x ,--x)
      // where - appears before a register. Must be checked before tryParseExpr
      // which would destructively fail on bare +/- tokens.
      if (getSTI().getFeatureBits()[MC6809::Feature6809] &&
          (getLexer().is(AsmToken::Plus) || getLexer().is(AsmToken::Minus))) {
        bool PushAsToken = false;
        if (!Operands.empty()) {
          auto *LastOp = static_cast<MC6809Operand *>(Operands.back().get());
          // After a register: post-increment/decrement (,x+ ,x++)
          // After a register: post-increment/decrement (,x+ ,x++)
          PushAsToken = LastOp->isReg();
          // After a + or - token: merge into ++ or --
          if (!PushAsToken && LastOp->isToken()) {
            StringRef LastTok = LastOp->getToken();
            if ((LastTok == "+" && getLexer().is(AsmToken::Plus)) ||
                (LastTok == "-" && getLexer().is(AsmToken::Minus))) {
              // Replace the last "+" with "++" (or "-" with "--")
              Operands.pop_back();
              SMLoc TokLoc = getLexer().getLoc();
              Operands.push_back(MC6809Operand::createToken(
                  MCSTI, getLexer().is(AsmToken::Plus) ? "++" : "--", TokLoc));
              Lex();
              continue;
            }
          }
        }
        // Pre-decrement: - before a register (,-x ,--x)
        if (!PushAsToken && getLexer().is(AsmToken::Minus)) {
          AsmToken NextTok = getLexer().peekTok();
          if (NextTok.getKind() == AsmToken::Minus) {
            PushAsToken = true; // First - of --
          } else if (NextTok.getKind() == AsmToken::Identifier) {
            // Check if the identifier is a register name
            MCRegister Reg = 0;
            SMLoc NS = NextTok.getLoc();
            SMLoc NE = NextTok.getEndLoc();
            tryParseRegister(Reg, NS, NE);
            if (Reg != 0)
              PushAsToken = true; // - before a register
          }
        }
        if (PushAsToken) {
          eatThatToken(Operands);
          continue;
        }
      }

      if (!tryParseExpr(Operands, ExprTypeAddress, "expression expected")) {
        continue;
      }

      // Okay then, you're a token.  Hope you're happy.
      eatThatToken(Operands);
    }
    Parser.Lex(); // Consume the EndOfStatement
    return false;
  }

  bool parseRegister(MCRegister &Reg, SMLoc &StartLoc, SMLoc &EndLoc) override { return !tryParseRegister(Reg, StartLoc, EndLoc).isSuccess(); }

}; // class MC6809AsmParser

#define GET_MATCHER_IMPLEMENTATION
#include "MC6809GenAsmMatcher.inc"

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMC6809AsmParser() { // NOLINT
  RegisterMCAsmParser<MC6809AsmParser> X(getTheMC6809Target());
}

} // namespace llvm
