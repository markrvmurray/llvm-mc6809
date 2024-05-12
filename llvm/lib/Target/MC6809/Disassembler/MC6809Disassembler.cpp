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

template <int Bits>
static DecodeStatus DecodeSImm(MCInst &Inst, uint64_t Imm, uint64_t Address, const MCDisassembler *Decoder);

static DecodeStatus DecodeCondCodeOperand(MCInst &MI, uint64_t Bits, uint64_t Address, const MCDisassembler *Decoder);

static DecodeStatus DecodeRegListOperand(MCInst &MI, uint64_t Bits, uint64_t Address, const MCDisassembler *Decoder);

static DecodeStatus DecodeRegOperand(MCInst &MI, uint64_t Regno, uint64_t Address, const MCDisassembler *Decoder);

static DecodeStatus DecodeINDEX16RegisterClass(MCInst &MI, uint64_t RegNo, uint64_t Address, const MCDisassembler *Decoder);

static DecodeStatus DecodeBIT8RegisterClass(MCInst &MI, uint64_t RegNo, uint64_t Address, const MCDisassembler *Decoder);

#include "MC6809GenDisassemblerTables.inc"

namespace {
/// A disassembler class for MC6809.
class MC6809Disassembler : public MCDisassembler {
public:
  MC6809Disassembler(const MCSubtargetInfo &STI, MCContext &Ctx) : MCDisassembler(STI, Ctx) {}
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size, ArrayRef<uint8_t> Bytes, uint64_t Address, raw_ostream &CStream) const override;

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

MCDisassembler *createMC6809Disassembler(const Target &T, const MCSubtargetInfo &STI, MCContext &Ctx) { return new MC6809Disassembler(STI, Ctx); }

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMC6809Disassembler() {
  // Register the disassembler.
  TargetRegistry::RegisterMCDisassembler(getTheMC6809Target(), createMC6809Disassembler);
}

template <int Bits>
static DecodeStatus DecodeSImm(MCInst &Inst, uint64_t Imm, uint64_t Address, const MCDisassembler *Decoder) {
  if (Imm & ~((1LL << Bits) - 1))
    return MCDisassembler::Fail;

  // Imm is a signed immediate, so sign extend it.
  if (Imm & (1 << (Bits - 1)))
    Imm |= ~((1LL << Bits) - 1);

  Inst.addOperand(MCOperand::createImm(Imm));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeCondCodeOperand(MCInst &MI, uint64_t Bits, uint64_t Address, const MCDisassembler *Decoder) {
  MI.addOperand(MCOperand::createImm(Bits));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeRegListOperand(MCInst &MI, uint64_t Bits, uint64_t Address, const MCDisassembler *Decoder) {
  unsigned RegsS[] = {MC6809::CC, MC6809::AA, MC6809::AB, MC6809::DP, MC6809::IX, MC6809::IY, MC6809::SU, MC6809::PC};
  unsigned RegsU[] = {MC6809::CC, MC6809::AA, MC6809::AB, MC6809::DP, MC6809::IX, MC6809::IY, MC6809::SS, MC6809::PC};

  // Sanity check the postbyte. It must not be zero, and must fit into 8 bits.
  if (Bits == 0 or Bits >= 256)
    return MCDisassembler::Fail;

  for (unsigned i = 0; i < 8; i++)
    if (Bits & (1 << i)) {
      if (MI.getOpcode() == MC6809::PSHSs || MI.getOpcode() == MC6809::PULSs)
        MI.addOperand(MCOperand::createReg(RegsS[i]));
      else
        MI.addOperand(MCOperand::createReg(RegsU[i]));
    }

  return MCDisassembler::Success;
}

static const unsigned RegDecoderTable[] = {MC6809::AD, MC6809::IX, MC6809::IY, MC6809::SU, MC6809::SS, MC6809::PC, MC6809::AV, MC6809::AW, MC6809::AA, MC6809::AB, MC6809::CC, MC6809::DP, MC6809::A0, MC6809::A0, MC6809::AE, MC6809::AF};

static DecodeStatus DecodeRegOperand(MCInst &MI, uint64_t RegNo, uint64_t Address, const MCDisassembler *Decoder) {
  if (RegNo > 15)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeINDEX16RegisterClass(MCInst &MI, uint64_t RegNo, uint64_t Address, const MCDisassembler *Decoder) {
  if (RegNo > 3)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo + 1];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

static DecodeStatus DecodeBIT8RegisterClass(MCInst &MI, uint64_t RegNo, uint64_t Address, const MCDisassembler *Decoder) {
  if (RegNo > 2)
    return MCDisassembler::Fail;

  unsigned Reg = RegDecoderTable[RegNo];
  MI.addOperand(MCOperand::createReg(Reg));
  return MCDisassembler::Success;
}

DecodeStatus MC6809Disassembler::getInstruction(MCInst &Instr, uint64_t &Size, ArrayRef<uint8_t> Bytes, uint64_t Address, raw_ostream &CStream) const {
  for (size_t InsnSize : seq_inclusive(1, 5)) {
    if (Bytes.size() < InsnSize)
      return MCDisassembler::Fail;
    uint64_t Insn = 0;
    for (size_t Byte : seq((size_t)0, InsnSize))
      Insn |= ((uint64_t)Bytes[Byte]) << (8 * Byte);
    for (unsigned i = 0; i < DecoderTableSize; i++) {
      if (DecoderTable[i].Size == InsnSize) {
        DecodeStatus Result = decodeInstruction(DecoderTable[i].Table, Instr, Insn, Address, this, STI);
        if (Result != MCDisassembler::Fail) {
          Size = InsnSize;
          return Result;
        }
      }
    }
  }
  return MCDisassembler::Fail;
}

using DecodeFunc = DecodeStatus (*)(MCInst &, unsigned int, uint64_t, const void *);
