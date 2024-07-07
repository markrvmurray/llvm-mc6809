//===--- MC6809MCELFStreamer.cpp - MC6809 subclass of MCELFStreamer -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file is a stub that parses a MCInst bundle and passes the
// instructions on to the real streamer.
//
//===----------------------------------------------------------------------===//
#include "MC6809MCTargetDesc.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFStreamer.h"
#define DEBUG_TYPE "mc6809mcelfstreamer"

#include "MCTargetDesc/MC6809MCELFStreamer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

using namespace llvm;

namespace llvm {

void MC6809MCELFStreamer::initSections(bool NoExecStack, const MCSubtargetInfo &STI) {
  Has6309Instructions = STI.hasFeature(MC6809::Feature6309);

  MCContext &Ctx = getContext();
  switchSection(Ctx.getObjectFileInfo()->getTextSection());
  emitCodeAlignment(Align(1), &STI);

  if (NoExecStack)
    switchSection(Ctx.getAsmInfo()->getNonexecutableStackSection(Ctx));
}

static bool HasPrefix(StringRef Name, StringRef Prefix) {
  SmallString<32> PrefixDot = Prefix;
  PrefixDot += ".";
  return Name == Prefix || Name.starts_with(PrefixDot);
}

void MC6809MCELFStreamer::changeSection(MCSection *Section, uint32_t Subsection) {
  MCELFStreamer::changeSection(Section, Subsection);
  HasBSS |= HasPrefix(Section->getName(), ".bss");
  HasDPBSS |= HasPrefix(Section->getName(), ".dp.bss");
  HasData |= HasPrefix(Section->getName(), ".data");
  HasDPData |= HasPrefix(Section->getName(), ".dp.data");
  HasDPData |= HasPrefix(Section->getName(), ".dp.rodata");
  HasInitArray |= HasPrefix(Section->getName(), ".init_array");
  HasFiniArray |= HasPrefix(Section->getName(), ".fini_array");
  MState = MXFlagUnknown;
  XState = MXFlagUnknown;
}

void MC6809MCELFStreamer::emitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI) { MCELFStreamer::emitInstruction(Inst, STI); }

void MC6809MCELFStreamer::emitValueImpl(const MCExpr *Value, unsigned Size, SMLoc Loc) {
  if (const auto *MME = dyn_cast<MC6809MCExpr>(Value)) {
    if (MME->getKind() == MC6809MCExpr::VK_MC6809_ADDR_ASCIZ) {
      emitMc6809AddrAsciz(MME->getSubExpr(), Size, Loc);
      return;
    }
  }
  MCELFStreamer::emitValueImpl(Value, Size, Loc);
}

void MC6809MCELFStreamer::emitMc6809AddrAsciz(const MCExpr *Value, unsigned Size, SMLoc Loc) {
  visitUsedExpr(*Value);
  MCDwarfLineEntry::make(this, getCurrentSectionOnly());
  MCDataFragment *DF = getOrCreateDataFragment();

  DF->getFixups().push_back(MCFixup::create(DF->getContents().size(), Value, (MCFixupKind)MC6809::AddrAsciz, Loc));
  DF->getContents().resize(DF->getContents().size() + Size, 0);
}

void MC6809MCELFStreamer::emitMappingSymbol(StringRef Name) {
  auto *Symbol = cast<MCSymbolELF>(getContext().getOrCreateSymbol(Name + "." + Twine(MappingSymbolCounter++)));
  emitLabel(Symbol);
  Symbol->setType(ELF::STT_NOTYPE);
  Symbol->setBinding(ELF::STB_LOCAL);
  Symbol->setExternal(false);
}

MCStreamer *createMC6809MCELFStreamer(const Triple & /*T*/, MCContext &Ctx, std::unique_ptr<MCAsmBackend> &&TAB, std::unique_ptr<MCObjectWriter> &&OW, std::unique_ptr<MCCodeEmitter> &&Emitter) {
  auto *S = new MC6809MCELFStreamer(Ctx, std::move(TAB), std::move(OW), std::move(Emitter));
  return S;
}

} // end namespace llvm
