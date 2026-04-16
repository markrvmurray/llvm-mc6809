//===-- MC6809InstPrinter.cpp - Convert MC6809 MCInst to assembly syntax
//--------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This class prints an MC6809 MCInst to a .s file.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstPrinter.h"
#include "MC6809.h"

#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/raw_ostream.h"

#include <cstring>
#include <sstream>

#define DEBUG_TYPE "asm-printer"

namespace llvm {

void MC6809InstPrinter::printInst(const MCInst *MI, uint64_t Address, StringRef Annot, const MCSubtargetInfo &STI, raw_ostream &OS) {
  std::string AiryOperands;
  raw_string_ostream AiryOperandStream(AiryOperands);
  auto MnemonicInfo = getMnemonic(*MI);
  assert(MnemonicInfo.second && "Missing opcode for instruction.");
  printInstruction(MI, Address, AiryOperandStream);
  AiryOperands = AiryOperandStream.str();
  size_t SpacesSeen = 0;
  std::string CorrectOperands;
  for (const auto &Letter : AiryOperands) {
    if (isspace(Letter) != 0) {
      if (++SpacesSeen <= 2) {
        CorrectOperands += '\t';
      }
      continue;
    }
    CorrectOperands += Letter;
  }
  OS << CorrectOperands;
}

void MC6809InstPrinter::printOperand(const MCInst *MI, unsigned OpNo, raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);

  if (Op.isReg()) {
    printRegName(O, Op.getReg());
  } else if (Op.isImm()) {
    O << formatImm(Op.getImm());
  } else {
    assert(Op.isExpr() && "Unknown operand kind in printOperand");
    MAI.printExpr(O, *Op.getExpr());
  }
}

void MC6809InstPrinter::printRegisterList(const MCInst *MI, unsigned OpNo, raw_ostream &O) {
  bool DoneOne = false;

  for (unsigned i = 0; i < MI->getNumOperands(); i++) {
      if (DoneOne)
        O << ",";
      printRegName(O, MI->getOperand(i).getReg());
      DoneOne = true;
  }
}

void MC6809InstPrinter::printRegName(raw_ostream &O, MCRegister Reg) {
  O << getRegisterName(Reg);
}

void MC6809InstPrinter::printCondCode(const MCInst *MI, unsigned OpNo, raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  enum MC6809CC::CondCode CC = (enum MC6809CC::CondCode)(Op.getImm() & 0xF);
  O << MC6809CC::getCCString(CC);
}

// Print a PC-relative branch target whose displacement is signed 8-bit.
// Used by short-branch operands (pcrel8) and the 8-bit variants of the
// indexed PC-relative addressing modes (pcrel8_idx, pcrel8_imm_idx).
// The absolute target shown in stream-disassembler mode is computed
// against PC-after-the-whole-instruction, so we consult MCInstrDesc for
// the real instruction size rather than hardcoding it — the old `+ 2`
// was correct only for the 2-byte short-branch encoding and produced
// off-by-(Size-2) numeric targets for everything else (bug #120).
void MC6809InstPrinter::printBranchOperand8(const MCInst *MI, uint64_t Address, unsigned OpNo, raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (!Op.isImm())
    return printOperand(MI, OpNo, O);
  int64_t Displacement = (int8_t)Op.getImm();
  if (PrintBranchImmAsAddress) {
    uint64_t InstSize = MII.get(MI->getOpcode()).getSize();
    O << formatImm(Displacement + Address + InstSize);
  } else {
    O << formatImm(Displacement);
  }
}

// Print a PC-relative branch target whose displacement is signed 16-bit.
// Used by long-branch operands (pcrel16) and the 16-bit variants of the
// indexed PC-relative addressing modes. The previous shared
// `printBranchOperand` truncated 16-bit displacements to `(int8_t)`,
// which silently mis-printed any long-branch target whose offset didn't
// happen to fit in a signed byte (bug #120). Instruction size is taken
// from MCInstrDesc so the arithmetic is correct for every encoding
// (LBRA: 3 bytes, LBcc: 4 bytes, indexed long-branch variants: 4-5).
void MC6809InstPrinter::printBranchOperand16(const MCInst *MI, uint64_t Address, unsigned OpNo, raw_ostream &O) {
  const MCOperand &Op = MI->getOperand(OpNo);
  if (!Op.isImm())
    return printOperand(MI, OpNo, O);
  int64_t Displacement = (int16_t)Op.getImm();
  if (PrintBranchImmAsAddress) {
    uint64_t InstSize = MII.get(MI->getOpcode()).getSize();
    O << formatImm(Displacement + Address + InstSize);
  } else {
    O << formatImm(Displacement);
  }
}

format_object<int64_t> MC6809InstPrinter::formatHex(int64_t Value) const {
  switch (PrintHexStyle) {
  case HexStyle::C:
    if (Value < 0) {
      return format("-$%" PRIx64, -Value);
    } else {
      return format("$%" PRIx64, Value);
    }
  case HexStyle::Asm:
    if (Value < 0) {
      return format("-$%" PRIx64, -Value);
    } else {
      return format("$%" PRIx64, Value);
    }
  }
  llvm_unreachable("unsupported print style");
}

format_object<uint64_t> MC6809InstPrinter::formatHex(uint64_t Value) const {
  switch (PrintHexStyle) {
  case HexStyle::C:
    return format("$%" PRIx64, Value);
  case HexStyle::Asm:
    return format("$%" PRIx64, Value);
  }
  llvm_unreachable("unsupported print style");
}

// Include the auto-generated portion of the assembly writer.
#define PRINT_ALIAS_INSTR
#include "MC6809GenAsmWriter.inc"

} // end of namespace llvm
