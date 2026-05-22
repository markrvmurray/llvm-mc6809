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
#include "llvm/MC/MCDecoder.h"
#include "llvm/MC/MCDecoderOps.h"
#include "llvm/MC/MCDisassembler/MCDisassembler.h"
#include "llvm/MC/MCInst.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace llvm::MCD;

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
///
/// Bug #323 (2026-05-22): owns its `MCSubtargetInfo` via unique_ptr.
/// The Bug #314 fix originally created an `OverriddenSTI` local in the
/// factory and passed it by reference to `MCDisassembler(STI, Ctx)` —
/// but `MCDisassembler` stores `const MCSubtargetInfo&` as a member
/// (see `MCDisassembler.h:213`), so the reference dangled past factory
/// return.  Reads from `STI.getFeatureBits()` then returned undefined
/// data, which silently broke predicate-gated HD6309 instruction
/// decoding (LDQ, STQ, SEXW, OIM/AIM/EIM/TIM, W-relative indexing,
/// etc.) whenever the factory's `OverriddenSTI` stack frame was
/// reused.  Hold the override in a unique_ptr so its lifetime matches
/// the disassembler.
class MC6809Disassembler : public MCDisassembler {
public:
  MC6809Disassembler(std::unique_ptr<const MCSubtargetInfo> OwnedSTI,
                     MCContext &Ctx)
      : MCDisassembler(*OwnedSTI, Ctx), OwnedSTI(std::move(OwnedSTI)) {}
  DecodeStatus getInstruction(MCInst &Instr, uint64_t &Size, ArrayRef<uint8_t> Bytes, uint64_t Address, raw_ostream &CStream) const override;

private:
  std::unique_ptr<const MCSubtargetInfo> OwnedSTI;

public:

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
};
} // namespace

MCDisassembler *createMC6809Disassembler(const Target &T, const MCSubtargetInfo &STI, MCContext &Ctx) {
  // Bug #314 (2026-05-21): the disassembler is asked to decode bytes
  // whose origin we generally can't determine (`llvm-objdump -d` is
  // the typical caller, and ELF flags don't currently identify the
  // target subtarget for MC6809 ELF — Flags is always 0x0).  Without
  // any hint, llvm-objdump defaults the subtarget to the base
  // generic MC6809, which lacks the HD6309 page-2/page-3 instruction
  // set — and so all HD6309 opcodes (LDQ, STQ, ADCD, etc.) decode as
  // `<unknown>`.  This is hostile for our actual workflow where
  // ~every production binary is HD6309.
  //
  // The fix: always force the HD6309 (`mc6809-insns-6309`) feature
  // on for the disassembler-side STI.  HD6309 is a strict superset
  // of MC6809 in encoding space, so this can NEVER cause a legitimate
  // MC6809 binary to mis-decode — any byte sequence that's valid
  // as MC6809 is also valid (and identically decoded) as HD6309.
  // The only difference is that bytes that would have been `<unknown>`
  // on plain MC6809 now decode as their HD6309 op.
  //
  // If the user wants to assert "this binary MUST be plain 6809 — flag
  // any 6309 op as illegal," they can pass `--mcpu=mc6809` to
  // llvm-objdump, which the disassembler honors via the existing
  // STI.checkFeatures() path in the generated decode table.  This
  // override only sets the default when the user doesn't specify.
  auto OverriddenSTI = std::make_unique<MCSubtargetInfo>(STI);
  if (!OverriddenSTI->checkFeatures("+mc6809-insns-6309"))
    OverriddenSTI->ApplyFeatureFlag("+mc6809-insns-6309");
  return new MC6809Disassembler(std::move(OverriddenSTI), Ctx);
}

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

// Bug #212 (disassembler half): the HD6309 TFR/EXG postbyte spec
// assigns code 6 to W (16-bit accumulator E:F) and code 7 to V
// (16-bit user register). The encoder at MC6809MCCodeEmitter.cpp:176
// gets this right (AW→6, AV→7), and the AsmParser does too. Until
// this fix the disassembler had AV/AW swapped at indices 6/7 in
// RegDecoderTable, so `1f 06` (codegen-emitted `tfr d,w`) rendered
// as `tfr d,v` in llvm-objdump. Cosmetic only — the binary running
// on real HD6309 / MAME was always correct — but the misrendering
// made bug #212's earlier asm-level diagnosis look more alarming
// than it was (the apparent `tfr d,v` followed by `stw 6,y`
// suggested a value-routing miscompile when it's actually a
// preserve-D-into-W ahead of an unrelated B reload).
static const unsigned RegDecoderTable[] = {MC6809::AD, MC6809::IX, MC6809::IY, MC6809::SU, MC6809::SS, MC6809::PC, MC6809::AW, MC6809::AV, MC6809::AA, MC6809::AB, MC6809::CC, MC6809::DP, MC6809::A0, MC6809::A0, MC6809::AE, MC6809::AF};

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
  // BIT8 register encoding: 0=CC, 1=A, 2=B
  static const unsigned BIT8DecoderTable[] = {MC6809::CC, MC6809::AA, MC6809::AB};
  if (RegNo > 2)
    return MCDisassembler::Fail;

  unsigned Reg = BIT8DecoderTable[RegNo];
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
