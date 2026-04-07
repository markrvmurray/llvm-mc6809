//===-- MC6809MCCodeEmitter.cpp - Convert MC6809 Code to Machine Code -----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MC6809MCCodeEmitter class.
//
//===----------------------------------------------------------------------===//

#include "MC6809MCCodeEmitter.h"

#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mccodeemitter"

#define GET_INSTRMAP_INFO
#include "MC6809GenInstrInfo.inc"
#undef GET_INSTRMAP_INFO

namespace llvm {

void MC6809MCCodeEmitter::emitInstruction(uint64_t Val, unsigned Size, const MCSubtargetInfo &STI, raw_ostream &OS) const {
  for (int64_t i = 0; i < Size; ++i) {
    OS << (char)(Val & 0xff);
    Val = Val >> 8;
  }
}

static void emitLittleEndian(uint64_t Val, unsigned Size, SmallVectorImpl<char> &CB) {
  for (int64_t I = 0; I < Size; ++I) {
    CB.push_back((char)(Val & 0xff));
    Val = Val >> 8;
  }
}

void MC6809MCCodeEmitter::encodeInstruction(const MCInst &MI, SmallVectorImpl<char> &CB, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction
  unsigned Size = Desc.getSize();

  assert(Size > 0 && "Instruction size cannot be zero");

  unsigned FixupsBefore = Fixups.size();
  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);

  // Page-2 (0x10) and page-3 (0x11) instructions have a prefix byte that
  // the TableGen-generated encoder doesn't account for in fixup offsets.
  // Detect the prefix and adjust all new fixup offsets by +1.
  if (Size >= 3) {
    uint8_t FirstByte = BinaryOpCode & 0xFF;
    if (FirstByte == 0x10 || FirstByte == 0x11) {
      for (unsigned I = FixupsBefore; I < Fixups.size(); ++I)
        Fixups[I] = MCFixup::create(Fixups[I].getOffset() + 1,
                                    Fixups[I].getValue(),
                                    Fixups[I].getKind(),
                                    Fixups[I].isPCRel());
    }
  }

  emitLittleEndian(BinaryOpCode, Size, CB);
}

template <MC6809::Fixups Fixup, unsigned Offset> unsigned MC6809MCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  auto MO = MI.getOperand(OpNo);

  if (MO.isExpr()) {
    if (isa<MC6809MCExpr>(MO.getExpr())) {
      // If the expression is already a MC6809MCExpr,
      // we shouldn't perform any more fixups. Without this check, we would
      // instead create a fixup to the symbol named 'lo8(symbol)' which
      // is not correct.
      return getExprOpValue(MO.getExpr(), Fixups, STI, Offset);
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(Fixup);
    bool IsPCRel = (Fixup == MC6809::PCRel8 || Fixup == MC6809::PCRel16);
    Fixups.push_back(MCFixup::create(Offset, MO.getExpr(), FixupKind, IsPCRel));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned MC6809MCCodeEmitter::getExprOpValue(const MCExpr *Expr, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI, unsigned int Offset) const {

  MCExpr::ExprKind Kind = Expr->getKind();

  if (Kind == MCExpr::Binary) {
    Expr = static_cast<const MCBinaryExpr *>(Expr)->getLHS();
    Kind = Expr->getKind();
  }

  if (Kind == MC6809MCExpr::Target) {
    MC6809MCExpr const *MC6809Expr = cast<MC6809MCExpr>(Expr);
    int64_t Result;
    if (MC6809Expr->evaluateAsConstant(Result)) {
      return Result;
    }

    MCFixupKind FixupKind = static_cast<MCFixupKind>(MC6809Expr->getFixupKind());
    Fixups.push_back(MCFixup::create(Offset, MC6809Expr, FixupKind));
    return 0;
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}

unsigned MC6809MCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  if (MO.isImm())
    return MO.getImm();
  if (MO.isReg()) {
    unsigned Reg = MO.getReg();
    // Map index registers to 2-bit postbyte encoding: X=0, Y=1, U=2, S=3.
    // TFR/EXG instructions use encodeRegOpValue for 4-bit encoding.
    switch (Reg) {
    case MC6809::IX: return 0;
    case MC6809::IY: return 1;
    case MC6809::SU: return 2;
    case MC6809::SS: return 3;
    default: return Reg;
    }
  }

  assert(MO.isExpr());

  const MCExpr *Expr = MO.getExpr();
  if (isa<MCSymbolRefExpr>(Expr)) {
    // MC6809 has a 16-bit address bus; default to 16-bit fixups for symbol
    // references that reach this fallback path (e.g. i16imm operands).
    Fixups.push_back(MCFixup::create(0, Expr, FK_Data_2));
    return 0;
  }

  int64_t Res;
  if (Expr->evaluateAsAbsolute(Res)) {
    return Res;
  }

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned MC6809MCCodeEmitter::encodeImm3(const MCInst &MI, unsigned Op, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const { return MI.getOperand(Op).getImm(); }

unsigned MC6809MCCodeEmitter::encodeRegOpValue(const MCInst &MI, unsigned Op, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  unsigned Reg = MI.getOperand(Op).getReg();
  // Map registers to 4-bit TFR/EXG/TFM postbyte encoding.
  switch (Reg) {
  case MC6809::AD: return 0;   // D
  case MC6809::IX: return 1;   // X
  case MC6809::IY: return 2;   // Y
  case MC6809::SU: return 3;   // U
  case MC6809::SS: return 4;   // S
  case MC6809::PC: return 5;   // PC
  case MC6809::AW: return 6;   // W (6309)
  case MC6809::AV: return 7;   // V (6309)
  case MC6809::AA: return 8;   // A
  case MC6809::AB: return 9;   // B
  case MC6809::CC: return 10;  // CC
  case MC6809::DP: return 11;  // DP
  case MC6809::AE: return 14;  // E (6309)
  case MC6809::AF: return 15;  // F (6309)
  default: return Reg;
  }
}

unsigned MC6809MCCodeEmitter::encodeBIT8RegOpValue(const MCInst &MI, unsigned Op, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  // BIT8 register encoding: CC=0, A=1, B=2
  unsigned Reg = MI.getOperand(Op).getReg();
  switch (Reg) {
  case MC6809::CC: return 0;
  case MC6809::AA: return 1;
  case MC6809::AB: return 2;
  default: llvm_unreachable("Invalid BIT8 register");
  }
}

unsigned MC6809MCCodeEmitter::encodeRegListOpValue(const MCInst &MI, unsigned Op, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const {
  unsigned res = 0;
  for (unsigned I = Op, E = MI.getNumOperands(); I < E; ++I) {
    unsigned Reg = MI.getOperand(I).getReg();
    switch (Reg) {
    default:
      llvm_unreachable("Register not allowed for this instruction");
    case MC6809::CC:
      res |= 0b00000001;
      break;
    case MC6809::AA:
      res |= 0b00000010;
      break;
    case MC6809::AB:
      res |= 0b00000100;
      break;
    case MC6809::AD:
      res |= 0b00000110;  // D = A + B
      break;
    case MC6809::DP:
      res |= 0b00001000;
      break;
    case MC6809::IX:
      res |= 0b00010000;
      break;
    case MC6809::IY:
      res |= 0b00100000;
      break;
    case MC6809::SU:
    case MC6809::SS:
      res |= 0b01000000;
      break;
    case MC6809::PC:
      res |= 0b10000000;
      break;
    }
  }
  return res;
}

unsigned MC6809MCCodeEmitter::encodeCondCodeOpValue(const MCInst &MI, unsigned Op, SmallVectorImpl<MCFixup> &Fixups, const MCSubtargetInfo &STI) const { return MI.getOperand(Op).getImm(); }

MCCodeEmitter *createMC6809MCCodeEmitter(const MCInstrInfo &MCII, MCContext &Ctx) { return new MC6809MCCodeEmitter(MCII, Ctx); }

} // end of namespace llvm

#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "MC6809GenMCCodeEmitter.inc"
