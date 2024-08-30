//===-- MC6809AsmBackend.cpp - MC6809 Asm Backend  ------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MC6809AsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MC6809AsmBackend.h"
#include "MCTargetDesc/MC6809ELFObjectWriter.h"
#include "MCTargetDesc/MC6809FixupKinds.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809Subtarget.h"

#include "llvm/ADT/StringExtras.h"
#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCFragment.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <climits>
#include <cstdint>
#include <memory>
#include <string>

namespace llvm {
namespace MC6809 {
struct InstructionRelaxationEntry {
  unsigned From;
  unsigned To;
};

#include "MC6809GenSearchableTables.inc"
} // namespace MC6809

MCAsmBackend *createMC6809AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MC6809AsmBackend(STI.getTargetTriple().getOS());
}

void MC6809AsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  unsigned int Kind = Fixup.getKind();
  uint32_t Offset = Fixup.getOffset();

  if (Kind == MC6809::AddrAsciz) {
    std::string ValueStr = utostr(Value);
    assert(((ValueStr.size() + 1 + Offset) <= Data.size()) &&
           "Invalid offset within MC6809 instruction for modifier!");
    std::copy(ValueStr.begin(), ValueStr.end(), Data.begin() + Offset);
    Data[Offset + ValueStr.size()] = '\0';
    return;
  }

  unsigned int Bytes = 0;
  switch (Kind) {
  case MC6809::Imm8:
  case MC6809::Addr8:
  case MC6809::PCRel8:
  case FK_Data_1:
    Bytes = 1;
    break;
  case FK_Data_2:
  case MC6809::Imm16:
  case MC6809::Addr16:
  case MC6809::PCRel16:
    Bytes = 2;
    break;
  case FK_Data_4:
    Bytes = 4;
    break;
  case FK_Data_8:
    Bytes = 8;
    break;
  default:
    llvm_unreachable("unknown fixup kind");
    return;
  }

  assert(((Bytes + Offset) <= Data.size()) &&
         "Invalid offset within MC6809 instruction for modifier!");
  char *RangeStart = Data.begin() + Offset;

  for (char &Out : make_range(RangeStart, RangeStart + Bytes)) {
    Out = Value & 0xff;
    Value = Value >> 8;
  }
}

bool MC6809AsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup,
                                         uint64_t Value) const {
  return true;
}

static cl::opt<bool> ForcePCRelReloc(
    "mos-force-pcrel-reloc",
    cl::desc("Force relocation entries to be emitted for PCREL fixups."),
    cl::init(false), cl::Hidden);

static int getRelativeMC6809PCCorrection(const bool IsPCRel16) {
  // MC6809's PC relative addressing is off by one or two from the standard LLVM
  // PC relative convention.
  return IsPCRel16 ? -2 : -1;
}

static bool fitsIntoFixup(const int64_t SignedValue, const bool IsPCRel16) {
  return SignedValue >= (IsPCRel16 ? INT16_MIN : INT8_MIN) &&
         SignedValue <= (IsPCRel16 ? INT16_MAX : INT8_MAX);
}

bool MC6809AsmBackend::evaluateTargetFixup(const MCAssembler &Asm,
                                        const MCFixup &Fixup,
                                        const MCFragment *DF,
                                        const MCValue &Target,
                                        const MCSubtargetInfo *STI,
                                        uint64_t &Value, bool &WasForced) {
  // ForcePCRelReloc is a CLI option to force relocation emit, primarily for
  // testing R_MC6809_PCREL_*.
  WasForced = ForcePCRelReloc;

  const bool IsPCRel16 = Fixup.getKind() == (MCFixupKind)MC6809::PCRel16;
  assert((IsPCRel16 || Fixup.getKind() == (MCFixupKind)MC6809::PCRel8) &&
         "unexpected target fixup kind");

  // Logic taken from MCAssembler::evaluateFixup.
  bool IsResolved = false;
  if (Target.getSymB()) {
    IsResolved = false;
  } else if (!Target.getSymA()) {
    IsResolved = false;
  } else {
    const MCSymbolRefExpr *A = Target.getSymA();
    const MCSymbol &SA = A->getSymbol();
    if (A->getKind() != MCSymbolRefExpr::VK_None || SA.isUndefined()) {
      IsResolved = false;
    } else {
      IsResolved = Asm.getWriter().isSymbolRefDifferenceFullyResolvedImpl(
          Asm, SA, *DF, false, true);
    }
  }

  Value = Target.getConstant();

  if (const MCSymbolRefExpr *A = Target.getSymA()) {
    const MCSymbol &Sym = A->getSymbol();
    if (Sym.isDefined())
      Value += Asm.getSymbolOffset(Sym);
  }
  if (const MCSymbolRefExpr *B = Target.getSymB()) {
    const MCSymbol &Sym = B->getSymbol();
    if (Sym.isDefined())
      Value -= Asm.getSymbolOffset(Sym);
  }

  Value -= Asm.getFragmentOffset(*DF) + Fixup.getOffset();
  Value += getRelativeMC6809PCCorrection(IsPCRel16);

  return IsResolved && !WasForced && fitsIntoFixup(Value, IsPCRel16);
}

// Derived from findAssociatedFragment.
bool isBasedOnDirectPageSymbol(const MCExpr *E) {
  switch (E->getKind()) {
  case MCExpr::Target:
    return isBasedOnDirectPageSymbol(cast<MC6809MCExpr>(E)->getSubExpr());

  case MCExpr::Constant:
    return false;

  case MCExpr::SymbolRef:
    return cast<MCSymbolELF>(cast<MCSymbolRefExpr>(E)->getSymbol()).getOther() &
           ELF::STO_MC6809_DIRECTPAGE;

  case MCExpr::Unary:
    return cast<MCUnaryExpr>(E)->getSubExpr()->findAssociatedFragment();

  case MCExpr::Binary: {
    const MCBinaryExpr *BE = cast<MCBinaryExpr>(E);
    return isBasedOnDirectPageSymbol(BE->getLHS()) ||
           isBasedOnDirectPageSymbol(BE->getRHS());
  }
  }

  llvm_unreachable("Invalid assembly expression kind!");
}

bool MC6809AsmBackend::fixupNeedsRelaxationAdvanced(const MCAssembler &Asm,
                                                 const MCFixup &Fixup,
                                                 bool Resolved, uint64_t Value,
                                                 const MCRelaxableFragment *DF,
                                                 const bool WasForced) const {
  // On 65816, it is possible to zero-bank relax from Addr16 to Addr24. The
  // assembler relaxes in a loop until instructions cannot be relaxed further,
  // so this is able to follow zero-page relaxation.
  bool BankRelax = false;
  MC6809AsmBackend::relaxInstructionTo(DF->getInst(), *DF->getSubtargetInfo(),
                                    BankRelax);

  auto Info = getFixupKindInfo(Fixup.getKind());
  const auto *MME = dyn_cast<MC6809MCExpr>(Fixup.getValue());
  // If this is a target-specific relaxation, e.g. a modifier, then the Info
  // field already knows the exact width of the answer, so decide now.
  if (MME != nullptr)
    return (Info.TargetSize > (BankRelax ? 16 : 8));

  // Now the fixup kind is not target-specific.  Yet, if it requires more than
  // 8 (or 16) bits, then relaxation is needed.
  if (Info.TargetSize > (BankRelax ? 16 : 8))
    return true;

  if (Info.Flags & MCFixupKindInfo::FKF_IsPCRel) {
    // This fixup concerns a relative branch.
    // If the fixup is unresolved, we can't know if relaxation is needed.
    return !Resolved || !fitsIntoFixup(Value, false);
  }

  // See if the expression is derived from a zero page symbol.
  if (isBasedOnDirectPageSymbol(Fixup.getValue()))
    return false;

  // In order to resolve an eight to sixteen bit possible relaxation, we need
  // to figure out whether the symbol in question is in zero page or not.  If
  // it is in zero page, then we don't need to do anything.  If not, we need
  // to relax the instruction to 16 bits.
  MCFragment *Frag = Fixup.getValue()->findAssociatedFragment();
  if (!Frag)
    return true;

  // If we're not writing to ELF, punt on this whole idea, just do the
  // relaxation for safety's sake
  const auto *Sec = dyn_cast_if_present<MCSectionELF>(Frag->getParent());
  if (!Sec)
    return true;

  // If the section of the symbol is marked with special zero-page flag
  // then this is an 8 bit instruction and it doesn't need
  // relaxation.
  if (Sec->getFlags() & ELF::SHF_MC6809_DIRECTPAGE)
    return false;

  return !MC6809::isDirectPageSectionName(Sec->getName());
}

MCFixupKindInfo const &MC6809AsmBackend::getFixupKindInfo(MCFixupKind Kind) const {
  if (Kind < FirstTargetFixupKind) {
    return MCAsmBackend::getFixupKindInfo(Kind);
  }

  return MC6809FixupKinds::getFixupKindInfo(static_cast<MC6809::Fixups>(Kind), this);
}

unsigned MC6809AsmBackend::getNumFixupKinds() const {
  return MC6809::Fixups::NumTargetFixupKinds;
}

unsigned MC6809AsmBackend::relaxInstructionTo(const MCInst &Inst,
                                           const MCSubtargetInfo &STI,
                                           bool &BankRelax) {
  // Attempt branch relaxation.

  // Attempt direct page/bank relaxation.

  return 0;
}

template <typename Fn>
static bool visitRelaxableOperand(const MCInst &Inst,
                                  const MCSubtargetInfo &STI, Fn Visit) {
  bool BankRelax = false;
  unsigned RelaxTo = MC6809AsmBackend::relaxInstructionTo(Inst, STI, BankRelax);

  return RelaxTo && Inst.getNumOperands() <= 2 &&
         Visit(Inst.getOperand(Inst.getNumOperands() - 1), RelaxTo, BankRelax);
}

static bool isImmediateBankRelaxable(const MCSubtargetInfo &STI, int64_t Imm,
                                     bool BankRelax) {
  if (BankRelax)
    return Imm >= 0 && Imm <= UINT16_MAX;

  uint32_t ZpAddrOffset =
      static_cast<const MC6809Subtarget &>(STI).getDirectPageOffset();
  return Imm >= ZpAddrOffset && Imm <= ZpAddrOffset + 0xFF;
}

void MC6809AsmBackend::relaxForImmediate(MCInst &Inst,
                                      const MCSubtargetInfo &STI) {
  // Two steps are required for zero-bank relaxation on 65816.
  while (visitRelaxableOperand(Inst, STI,
                               [&Inst, &STI](const MCOperand &Operand,
                                             unsigned RelaxTo, bool BankRelax) {
                                 int64_t Imm;
                                 if (!Operand.evaluateAsConstantImm(Imm) ||
                                     isImmediateBankRelaxable(STI, Imm,
                                                              BankRelax)) {
                                   // If the expression evaluates cleanly to an
                                   // 8-bit value (or 16-bit for bank
                                   // relaxation), then it doesn't need
                                   // relaxation.
                                   return false;
                                 }
                                 // This instruction can be relaxed, do it now.
                                 Inst.setOpcode(RelaxTo);
                                 return true;
                               }))
    ;
}

bool MC6809AsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) const {
  return visitRelaxableOperand(Inst, STI,
                               [](const MCOperand &Operand, unsigned RelaxTo,
                                  bool BankRelax) { return Operand.isExpr(); });
}

void MC6809AsmBackend::relaxInstruction(MCInst &Inst,
                                     const MCSubtargetInfo &STI) const {
  unsigned Opcode = relaxInstructionTo(Inst, STI);
  if (Opcode != 0) {
    Inst.setOpcode(Opcode);
  }
}

bool MC6809AsmBackend::writeNopData(raw_ostream &OS, uint64_t Count,
                                 const MCSubtargetInfo *STI) const {
  while (Count--) {
    // Sports. It's in the game. Knowing the 6502 hexadecimal representation of
    // a NOP on 6502 used to be an interview question at Electronic Arts.
    OS << '\x12'; // MC6809 NOP
  }
  return true;
}

void MC6809AsmBackend::translateOpcodeToSubtarget(MCInst &Inst,
                                               const MCSubtargetInfo &STI) {
}

std::unique_ptr<llvm::MCObjectTargetWriter>
MC6809AsmBackend::createObjectTargetWriter() const {
  return std::make_unique<MC6809ELFObjectWriter>(OSType);
}

} // namespace llvm
