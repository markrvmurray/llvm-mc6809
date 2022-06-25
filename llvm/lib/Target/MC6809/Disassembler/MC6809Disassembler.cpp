//===- MC6809Disassembler.cpp - Disassembler for MC6809 ---------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is part of the MC6809 Disassembler.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-disassembler"

using DecodeStatus = MCDisassembler::DecodeStatus;

static DecodeStatus DecodeCondCodeOperand(MCInst &MI, uint64_t Bits,
                                          uint64_t Address,
                                          const void *Decoder);

static DecodeStatus DecodeRegListOperand(MCInst &MI, uint64_t Bits,
                                         uint64_t Address, const void *Decoder);

static DecodeStatus DecodeRegOperand(MCInst &MI, uint64_t Bits,
                                     uint64_t Address, const void *Decoder);

static DecodeStatus DecodeINDEX16RegisterClass(MCInst &MI, uint64_t RegNo,
                                               uint64_t Address,
                                               const void *Decoder);

static DecodeStatus DecodeBIT8RegisterClass(MCInst &MI, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder);

#include "MC6809GenDisassemblerTables.inc"

namespace {
/// A disassembler class for MC6809.
class MC6809Disassembler : public MCDisassembler {
public:
  MC6809Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx)
      : MCDisassembler(STI, Ctx) {}
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size,
                              ArrayRef<uint8_t> Bytes, uint64_t Address,
                              raw_ostream &CStream) const override;

  // clang-format off
// MRVM START MARKER
  unsigned DecoderTableSize = 13;
  struct DecoderTableList {
    const uint8_t *Table;
    unsigned Size;
  } DecoderTable[13] = {
    { DecoderTable_Page_1_Size_16, 2 },
    { DecoderTable_Page_1_Size_24, 3 },
    { DecoderTable_Page_1_Size_32, 4 },
    { DecoderTable_Page_1_Size_40, 5 },
    { DecoderTable_Page_1_Size_8, 1 },
    { DecoderTable_Page_2_Size_16, 2 },
    { DecoderTable_Page_2_Size_24, 3 },
    { DecoderTable_Page_2_Size_32, 4 },
    { DecoderTable_Page_2_Size_40, 5 },
    { DecoderTable_Page_3_Size_16, 2 },
    { DecoderTable_Page_3_Size_24, 3 },
    { DecoderTable_Page_3_Size_32, 4 },
    { DecoderTable_Page_3_Size_40, 5 },
  };
// MRVM END MARKER
  // clang-format on
};
} // namespace

MCDisassembler *createMC6809Disassembler(const Target &T,
                                         const MCSubtargetInfo &STI,
                                         MCContext &Ctx) {
  return new MC6809Disassembler(STI, Ctx);
}

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMC6809Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMC6809Target(),
                                         createMC6809Disassembler);
}

static DecodeStatus DecodeCondCodeOperand(MCInst &MI, uint64_t Bits,
                                          uint64_t Address,
                                          const void *Decoder) {
  MI.addOperand(MCOperand::createImm(Bits));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeRegListOperand(MCInst &MI, uint64_t Bits,
                                         uint64_t Address,
                                         const void *Decoder) {
  unsigned Regs[] = {MC6809::CC, MC6809::AA, MC6809::AB, MC6809::DP,
                     MC6809::IX, MC6809::IY, MC6809::SU, MC6809::PC};

  // Sanity check the postbyte. It must not be zero, and must fit into 8 bits.
  if (Bits == 0 or Bits >= 256)
    return MCDisassembler::Fail;

  for (unsigned i = 0; i < 8; i++)
    if (Bits & (1 << i))
      MI.addOperand(MCOperand::createReg(Regs[i]));

  return MCDisassembler::Success;
}

static const unsigned RegDecoderTable[] = {
    MC6809::AD, MC6809::IX, MC6809::IY, MC6809::SU, MC6809::SS, MC6809::PC,
    MC6809::AV, MC6809::AW, MC6809::AA, MC6809::AB, MC6809::CC, MC6809::DP,
    MC6809::A0, MC6809::A0, MC6809::AE, MC6809::AF};

static DecodeStatus DecodeRegOperand(MCInst &MI, uint64_t RegNo,
                                     uint64_t Address, const void *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeINDEX16RegisterClass(MCInst &MI, uint64_t RegNo,
                                               uint64_t Address,
                                               const void *Decoder) {
  if (RegNo > 3)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo + 1];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeBIT8RegisterClass(MCInst &MI, uint64_t RegNo,
                                            uint64_t Address,
                                            const void *Decoder) {
  if (RegNo > 2)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

DecodeStatus MC6809Disassembler::getInstruction(MCInst &Instr, uint64_t &Size,
                                                ArrayRef<uint8_t> Bytes,
                                                uint64_t Address,
                                                raw_ostream &CStream) const {
  uint64_t Insn;
  DecodeStatus retVal, attempt;

  if (Bytes.size() == 0) {
    Size = 0;
    return MCDisassembler::Fail;
  }

  retVal = MCDisassembler::Fail;
  for (size_t InsnSize = 1; InsnSize <= (unsigned)std::min(5ul, Bytes.size());
       InsnSize++) {
    Insn = 0;
    for (unsigned i = 0; i < InsnSize; i++)
      Insn |= Bytes[i] << (8 * i);
    for (unsigned i = 0; i < DecoderTableSize; i++) {
      if (DecoderTable[i].Size == (InsnSize)) {
        if ((attempt = decodeInstruction(DecoderTable[i].Table, Instr, Insn,
                                         Address, this, STI)) !=
            MCDisassembler::Fail) {
          retVal = attempt;
          Size = DecoderTable[i].Size;
          break;
        }
      }
    }
    if (retVal != MCDisassembler::Fail)
      break;
  }
  return retVal;
}

using DecodeFunc = DecodeStatus (*)(MCInst &, unsigned int, uint64_t,
                                    const void *);
