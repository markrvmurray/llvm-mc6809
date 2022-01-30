//===-- MC6809AsmBackend.cpp - MC6809 Asm Backend  ------------------------------===//
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

#include "llvm/ADT/StringRef.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDirectives.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCFixupKindInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSectionELF.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/Casting.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/raw_ostream.h"

#include <climits>
#include <cstdint>
#include <memory>
#include <string>

namespace llvm {

namespace MC6809 {
struct BranchInstructionRelaxationEntry {
  unsigned From;
  unsigned To;
};

#define GET_MC6809BranchInstructionRelaxationTable_DECL
#define GET_MC6809BranchInstructionRelaxationTable_IMPL
#define GET_MC6809BranchSectionTable_DECL
#define GET_MC6809BranchSectionTable_IMPL
#include "MC6809GenSearchableTables.inc"
} // namespace MC6809

MCAsmBackend *createMC6809AsmBackend(const Target &T, const MCSubtargetInfo &STI,
                                  const MCRegisterInfo &MRI,
                                  const llvm::MCTargetOptions &TO) {
  return new MC6809AsmBackend(STI.getTargetTriple().getOS());
}

void MC6809AsmBackend::adjustFixupValue(const MCFixup &Fixup,
                                     const MCValue &Target, uint64_t &Value,
                                     MCContext *Ctx) const {
  unsigned Kind = Fixup.getKind();

  // Parsed LLVM-generated temporary labels are already
  // adjusted for instruction size, but normal labels aren't.
  //
  // To handle both cases, we simply un-adjust the temporary label
  // case so it acts like all other labels.
  if (const MCSymbolRefExpr *A = Target.getSymA()) {
    if (A->getSymbol().isTemporary()) {
      switch (Kind) {
      case FK_Data_1:
      case FK_Data_2:
      case FK_Data_4:
      case FK_Data_8:
        // Don't shift value for absolute addresses.
        break;
      default:
        Value += 2;
      }
    }
  }

  switch (Kind) {
  case MC6809::PCRel8:
    /* MC6809 pc-relative instructions are counted from the end of the instruction,
     * not the middle of it.
     */
    Value = (Value - 1) & 0xff;
    break;

  case MC6809::PCRel16:
    /* MC6809 pc-relative instructions are counted from the end of the instruction,
     * not the middle of it.
     */
    Value = (Value - 1) & 0xffff;
    break;

  // Fixups which do not require adjustments.
  case FK_Data_1:
  case FK_Data_2:
  case FK_Data_4:
  case FK_Data_8:
    break;

  default:
    llvm_unreachable("don't know how to adjust this fixup");
    break;
  }
}

void MC6809AsmBackend::applyFixup(const MCAssembler &Asm, const MCFixup &Fixup,
                               const MCValue &Target,
                               MutableArrayRef<char> Data, uint64_t Value,
                               bool IsResolved,
                               const MCSubtargetInfo *STI) const {
  unsigned int Kind = Fixup.getKind();
  uint32_t Offset = Fixup.getOffset();

  unsigned int Bytes = 0;
  switch (Kind) {
  case MC6809::Imm8:
  case MC6809::Addr8:
  case MC6809::Rel5:
  case MC6809::Rel8:
  case MC6809::PCRel8:
  case FK_Data_1:
    Bytes = 1;
    break;
  case FK_Data_2:
  case MC6809::Rel16:
  case MC6809::PCRel16:
  case MC6809::Addr16:
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
  for (char &Out :
       make_range(Data.begin() + Offset, Data.begin() + Bytes + Offset)) {
    Out = Value & 0xff;
    Value = Value >> 8;
  }
}

bool MC6809AsmBackend::fixupNeedsRelaxation(const MCFixup &Fixup, uint64_t Value,
                                         const MCRelaxableFragment *DF,
                                         const MCAsmLayout &Layout) const {
  return false;
}

bool MC6809AsmBackend::evaluateTargetFixup(const MCAssembler &Asm,
                                        const MCAsmLayout &Layout,
                                        const MCFixup &Fixup,
                                        const MCFragment *DF,
                                        const MCValue &Target, uint64_t &Value,
                                        bool &WasForced) {
  assert(Fixup.getKind() == (MCFixupKind)MC6809::PCRel8 &&
         "unexpected target fixup kind");
  Value = Target.getConstant();
  if (const MCSymbolRefExpr *A = Target.getSymA()) {
    const MCSymbol &Sym = A->getSymbol();
    if (Sym.isDefined())
      Value += Layout.getSymbolOffset(Sym);
  }
  if (const MCSymbolRefExpr *B = Target.getSymB()) {
    const MCSymbol &Sym = B->getSymbol();
    if (Sym.isDefined())
      Value -= Layout.getSymbolOffset(Sym);
  }
  uint32_t Offset = Layout.getFragmentOffset(DF) + Fixup.getOffset();
  Value -= Offset;
  // MC6809's PC relative addressing is off by one from the standard LLVM
  // PC relative convention.
  --Value;
  // If this result fits safely into 8 bits, we're done
  int64_t SignedValue = Value;
  return (INT8_MIN <= SignedValue && SignedValue <= INT8_MAX);
}

bool MC6809AsmBackend::fixupNeedsRelaxationAdvanced(const MCFixup &Fixup,
                                                 bool Resolved, uint64_t Value,
                                                 const MCRelaxableFragment *DF,
                                                 const MCAsmLayout &Layout,
                                                 const bool WasForced) const {
#if 0
  auto Info = getFixupKindInfo(Fixup.getKind());
  const auto *MME = dyn_cast<MC6809MCExpr>(Fixup.getValue());
  // If this is a target-specific relaxation, e.g. a modifier, then the Info
  // field already knows the exact width of the answer, so decide now.
  if (MME != nullptr) {
    return (Info.TargetSize > 8);
  }
  // Now the fixup kind is not target-specific.  Yet, if it requires more than
  // 8 bits, then relaxation is needed.
  if (Info.TargetSize > 8) {
    return true;
  }
  // In order to resolve an eight to sixteen bit possible relaxation, we need to
  // figure out whether the symbol in question is in direct page or not.  If it is
  // in direct page, then we don't need to do anything.  If not, we need to relax
  // the instruction to 16 bits.
  const char *FixupNameStart = Fixup.getValue()->getLoc().getPointer();
  // If there's no symbol name, and if the fixup does not have a known size,
  // then  we can't assume it lives in direct page.
  if (FixupNameStart == nullptr) {
    return true;
  }
  size_t FixupLength = 0;
  bool Finished = false;
  do {
    const char C = FixupNameStart[FixupLength];
    if ((C >= 'A' && C <= 'Z') || (C >= 'a' && C <= 'z') ||
        (C >= '0' && C <= '9') || (C == '$') || (C == '_')) {
      ++FixupLength;
      continue;
    }
    Finished = true;
  } while (Finished == false);
  StringRef FixupName(FixupNameStart, FixupLength);
  // The list of symbols is maintained by the assembler, and since that list
  // is not maintained in alpha order, it seems that we need to iterate across
  // it to find the symbol in question... is there a non-O(n) way to do this?
  for (const auto &Symbol : Layout.getAssembler().symbols()) {
    const auto SymbolName = Symbol.getName();
    if (FixupName == SymbolName) {
      // If this symbol has not been assigned to a section, then it can't
      // be in direct page
      if (!Symbol.isInSection()) {
        return true;
      }
      const auto &Section = Symbol.getSection();
      const auto *ELFSection = dyn_cast_or_null<MCSectionELF>(&Section);
      /// If we're not writing to ELF, punt on this whole idea, just do the
      /// relaxation for safety's sake
      if (ELFSection == nullptr) {
        return true;
      }
      /// If the section of the symbol is marked with special direct-page flag
      /// then this is an 8 bit instruction and it doesn't need
      /// relaxation.
      if ((ELFSection->getFlags() & ELF::SHF_MC6809_DIRECTPAGE) != 0) {
        return false;
      }
      const auto &ELFSectionName = ELFSection->getName();
      /// If the section of the symbol is one of the prenamed direct page
      /// sections, then this is an 8 bit instruction and it doesn't need
      /// relaxation.
      if (isBranchSectionName(ELFSectionName))
        return false;
    }
  }
  // we have no convincing reason not to do the relaxation
  return true;
#else
  return false;
#endif
}

bool MC6809AsmBackend::isBranchSectionName(StringRef Name) {
  return is_contained(MC6809::MC6809BranchSectionTable, Name);
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

unsigned MC6809AsmBackend::relaxInstructionTo(const MCInst &Inst) {
  const auto *BRIRE =
      MC6809::getBranchInstructionRelaxationEntry(Inst.getOpcode());
  if (BRIRE == nullptr) {
    return 0;
  }
  return BRIRE->To;
}

template <typename Fn>
static bool visitRelaxableOperand(const MCInst &Inst, Fn Visit) {
  unsigned RelaxTo = MC6809AsmBackend::relaxInstructionTo(Inst);
  if (RelaxTo == 0) {
    // If the instruction can't be relaxed, then it doesn't need relaxation.
    return false;
  }
  if (Inst.getNumOperands() != 1) {
    // If the instruction doesn't have exactly one operand, then it doesn't
    // need relaxation.
    return false;
  }
  return Visit(Inst.getOperand(0), RelaxTo);
}

void MC6809AsmBackend::relaxForImmediate(MCInst &Inst) {
  visitRelaxableOperand(Inst, [&Inst](const MCOperand &Operand,
                                      unsigned RelaxTo) {
    int64_t Imm;
    if (!Operand.evaluateAsConstantImm(Imm) || (Imm >= 0 && Imm <= UCHAR_MAX)) {
      // If the expression evaluates cleanly to an 8-bit value, then it doesn't
      // need relaxation.
      return false;
    }
    // This instruction can be relaxed, do it now.
    Inst.setOpcode(RelaxTo);
    return true;
  });
}

bool MC6809AsmBackend::mayNeedRelaxation(const MCInst &Inst,
                                      const MCSubtargetInfo &STI) const {
  return visitRelaxableOperand(Inst,
                               [](const MCOperand &Operand, unsigned RelaxTo) {
                                 if (!Operand.isExpr()) {
                                   // If the instruction isn't an expression,
                                   // then it doesn't need relaxation.
                                   return false;
                                 }
                                 // okay you got us, it MAY need relaxation, if
                                 // the instruction CAN be relaxed.
                                 return true;
                               });
}

void MC6809AsmBackend::relaxInstruction(MCInst &Inst,
                                     const MCSubtargetInfo &STI) const {
  unsigned Opcode = relaxInstructionTo(Inst);
  if (Opcode != 0) {
    Inst.setOpcode(Opcode);
  }
}

bool MC6809AsmBackend::writeNopData(raw_ostream &OS, uint64_t Count,
                                 const MCSubtargetInfo *STI) const {
  // todo: fix for virtual targets
  while ((Count--) > 0) {
    OS << 0xEA; // Sports. It's in the game.  Knowing the 6502 hexadecimal
                // representation of a NOP on 6502, used to be an interview
                // question at Electronic Arts.
  }
  return true;
}

std::unique_ptr<llvm::MCObjectTargetWriter>
MC6809AsmBackend::createObjectTargetWriter() const {
  return std::make_unique<MC6809ELFObjectWriter>(OSType);
}

} // namespace llvm
