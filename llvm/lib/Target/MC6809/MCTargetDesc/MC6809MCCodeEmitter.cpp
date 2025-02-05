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

void MC6809MCCodeEmitter::emitInstruction(uint64_t Val, unsigned Size,
                                          const MCSubtargetInfo &STI,
                                          raw_ostream &OS) const {
  for (int64_t i = 0; i < Size; ++i) {
    OS << (char)(Val & 0xff);
    Val = Val >> 8;
  }
}

void MC6809MCCodeEmitter::encodeInstruction(const MCInst &MI, raw_ostream &OS,
                                            SmallVectorImpl<MCFixup> &Fixups,
                                            const MCSubtargetInfo &STI) const {

  const MCInstrDesc &Desc = MCII.get(MI.getOpcode());
  // Get byte count of instruction
  unsigned Size = Desc.getSize();

  assert(Size > 0 && "Instruction size cannot be zero");

  uint64_t BinaryOpCode = getBinaryCodeForInstr(MI, Fixups, STI);
  emitInstruction(BinaryOpCode, Size, STI, OS);
}

template <MC6809::Fixups Fixup, unsigned Offset>
unsigned MC6809MCCodeEmitter::encodeImm(const MCInst &MI, unsigned OpNo,
                                        SmallVectorImpl<MCFixup> &Fixups,
                                        const MCSubtargetInfo &STI) const {
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
    Fixups.push_back(
        MCFixup::create(Offset, MO.getExpr(), FixupKind, MI.getLoc()));

    return 0;
  }

  assert(MO.isImm());
  return MO.getImm();
}

unsigned MC6809MCCodeEmitter::getExprOpValue(const MCExpr *Expr,
                                             SmallVectorImpl<MCFixup> &Fixups,
                                             const MCSubtargetInfo &STI,
                                             unsigned int Offset) const {

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

    MCFixupKind FixupKind =
        static_cast<MCFixupKind>(MC6809Expr->getFixupKind());
    Fixups.push_back(MCFixup::create(Offset, MC6809Expr, FixupKind));
    return 0;
  }

  assert(Kind == MCExpr::SymbolRef);
  return 0;
}

unsigned
MC6809MCCodeEmitter::getMachineOpValue(const MCInst &MI, const MCOperand &MO,
                                       SmallVectorImpl<MCFixup> &Fixups,
                                       const MCSubtargetInfo &STI) const {
  if (MO.isImm())
    return MO.getImm();

  assert(MO.isExpr());

  const MCExpr *Expr = MO.getExpr();
  if (isa<MCSymbolRefExpr>(Expr)) {
    Fixups.push_back(MCFixup::create(0, Expr, MCFixupKind::FK_Data_1));
    return 0;
  }

  int64_t Res;
  if (Expr->evaluateAsAbsolute(Res)) {
    return Res;
  }

  llvm_unreachable("Unhandled expression!");
  return 0;
}

unsigned MC6809MCCodeEmitter::encodeImm3(const MCInst &MI, unsigned Op,
                                         SmallVectorImpl<MCFixup> &Fixups,
                                         const MCSubtargetInfo &STI) const {
  return MI.getOperand(Op).getImm();
}

unsigned
MC6809MCCodeEmitter::encodeRegOpValue(const MCInst &MI, unsigned Op,
                                      SmallVectorImpl<MCFixup> &Fixups,
                                      const MCSubtargetInfo &STI) const {
  return MI.getOperand(Op).getReg();
}

unsigned
MC6809MCCodeEmitter::encodeRegListOpValue(const MCInst &MI, unsigned Op,
                                          SmallVectorImpl<MCFixup> &Fixups,
                                          const MCSubtargetInfo &STI) const {
  return MI.getOperand(Op).getImm();
}

unsigned
MC6809MCCodeEmitter::encodeCondCodeOpValue(const MCInst &MI, unsigned Op,
                                           SmallVectorImpl<MCFixup> &Fixups,
                                           const MCSubtargetInfo &STI) const {
  return MI.getOperand(Op).getImm();
}

MCCodeEmitter *createMC6809MCCodeEmitter(const MCInstrInfo &MCII,
                                         MCContext &Ctx) {
  return new MC6809MCCodeEmitter(MCII, Ctx);
}

} // end of namespace llvm

#define ENABLE_INSTR_PREDICATE_VERIFIER
#include "MC6809GenMCCodeEmitter.inc"
