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

#include "MCTargetDesc/MC6809MCTargetDesc.h"

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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI->dump(););
  std::string AiryOperands;
  raw_string_ostream AiryOperandStream(AiryOperands);
  auto MnemonicInfo = getMnemonic(MI);
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
    O << *Op.getExpr();
  }
}

static const char *RegNameForSU[8] = {"cc", "a", "b", "dp", "x", "y", "s", "pc"};
static const char *RegNameForSS[8] = {"cc", "a", "b", "dp", "x", "y", "u", "pc"};
void MC6809InstPrinter::printRegisterList(const MCInst *MI, unsigned OpNo, raw_ostream &O) {
  unsigned Opcode = MI->getOpcode();
  const MCOperand &Op = MI->getOperand(OpNo);
  unsigned RegList = Op.getImm() & 0xFF;
  const char **RegName;
  bool DoneOne = false;

  if (Opcode == MC6809::PSHSs || Opcode == MC6809::PULSs)
    RegName = RegNameForSS;
  else if (Opcode == MC6809::PSHUs || Opcode == MC6809::PULUs)
    RegName = RegNameForSU;
  else
    llvm_unreachable("Unknown opcode for reglist operand");
  do {
    if (RegList & 1) {
      if (DoneOne)
        O << ",";
      O << *RegName;
      DoneOne = true;
    }
    RegName++;
    RegList >>= 1;
  } while (RegList);
}

void MC6809InstPrinter::printRegName(raw_ostream &O, unsigned RegNo) const {
  O << getRegisterName(RegNo);
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
