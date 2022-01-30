//===-- MC6809TargetStreamer.cpp - MC6809 Target Streamer Methods ---------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MC6809 specific target streamer methods.
//
//===----------------------------------------------------------------------===//

#include "MC6809TargetStreamer.h"

#include "MC6809MCELFStreamer.h"

#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"

namespace llvm {

MC6809TargetStreamer::MC6809TargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void MC6809TargetStreamer::finish() {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  if (hasInitArray()) {
    MCSymbol *Init = Context.getOrCreateSymbol("_init");
    OS.emitRawComment("Declaring this symbol tells the CRT that there are");
    OS.emitRawComment("initialization routines to be run in .init_array");
    stronglyReference(Init);
  }

  if (hasFiniArray()) {
    MCSymbol *Fini = Context.getOrCreateSymbol("_fini");

    OS.emitRawComment("Declaring this symbol tells the CRT that there are");
    OS.emitRawComment("finalization routines to be run in .fini_array");
    stronglyReference(Fini);
    OS.emitSymbolAttribute(Fini, MCSA_Global);
  }
}

MC6809TargetAsmStreamer::MC6809TargetAsmStreamer(MCStreamer &S)
    : MC6809TargetStreamer(S) {}
void MC6809TargetAsmStreamer::changeSection(const MCSection *CurSection,
                                         MCSection *Section,
                                         const MCExpr *SubSection,
                                         raw_ostream &OS) {
  MCTargetStreamer::changeSection(CurSection, Section, SubSection, OS);
  HasInitArray |= Section->getName().startswith(".init_array");
  HasFiniArray |= Section->getName().startswith(".fini_array");
}

void MC6809TargetAsmStreamer::stronglyReference(MCSymbol *Sym) {
  getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

MC6809TargetELFStreamer::MC6809TargetELFStreamer(MCStreamer &S,
                                           const MCSubtargetInfo &STI)
    : MC6809TargetStreamer(S) {}

bool MC6809TargetELFStreamer::hasInitArray() {
  return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasInitArray();
}
bool MC6809TargetELFStreamer::hasFiniArray() {
  return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasFiniArray();
}
void MC6809TargetELFStreamer::stronglyReference(MCSymbol *Sym) {
  auto *ES = cast<MCSymbolELF>(Sym);
  // There's an explicit check in emitSymbolAttribute to avoid accidentally
  // overriding weak->global due to a GCC corner case, but it should always be
  // safe for symbols under complete compiler control.
  if (ES->isBindingSet())
    ES->setBinding(ELF::STB_GLOBAL);
  else
    getStreamer().emitSymbolAttribute(Sym, MCSA_Global);
}

} // end namespace llvm
