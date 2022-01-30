//===--------- MC6809MCELFStreamer.cpp - MC6809 subclass of MCELFStreamer -------===//
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
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCELFStreamer.h"
#define DEBUG_TYPE "mc6809mcelfstreamer"

#include "MCTargetDesc/MC6809MCELFStreamer.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectFileInfo.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSymbol.h"

using namespace llvm;

namespace llvm {

void MC6809MCELFStreamer::initSections(bool NoExecStack,
                                    const MCSubtargetInfo &STI) {
  MCContext &Ctx = getContext();
  SwitchSection(Ctx.getObjectFileInfo()->getTextSection());
  emitCodeAlignment(1, &STI);

  if (NoExecStack)
    SwitchSection(Ctx.getAsmInfo()->getNonexecutableStackSection(Ctx));
}

void MC6809MCELFStreamer::changeSection(MCSection *Section, const MCExpr *Subsection) {
  MCELFStreamer::changeSection(Section, Subsection);
  HasInitArray |= Section->getName().startswith(".init_array");
  HasFiniArray |= Section->getName().startswith(".fini_array");
}

void MC6809MCELFStreamer::emitValueImpl(const MCExpr *Value, unsigned Size,
                                     SMLoc Loc) {
  MCELFStreamer::emitValueImpl(Value, Size, Loc);
}

MCStreamer *createMC6809MCELFStreamer(const Triple & /*T*/, MCContext &Ctx,
                                   std::unique_ptr<MCAsmBackend> &&TAB,
                                   std::unique_ptr<MCObjectWriter> &&OW,
                                   std::unique_ptr<MCCodeEmitter> &&Emitter,
                                   bool RelaxAll) {
  auto *S = new MC6809MCELFStreamer(Ctx, std::move(TAB), std::move(OW),
                                 std::move(Emitter));
  if (RelaxAll) {
    S->getAssembler().setRelaxAll(true);
  }
  return S;
}

} // end namespace llvm
