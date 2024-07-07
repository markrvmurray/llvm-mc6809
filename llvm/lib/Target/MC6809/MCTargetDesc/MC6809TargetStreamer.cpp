//===-- MC6809TargetStreamer.cpp - MC6809 Target Streamer Methods ---------===//
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

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCSymbolELF.h"
#include "llvm/Support/Casting.h"

namespace llvm {

MC6809TargetStreamer::MC6809TargetStreamer(MCStreamer &S) : MCTargetStreamer(S) {}

void MC6809TargetStreamer::finish() {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();

  if (hasBSS())
    stronglyReference("__do_zero_bss", "Declaring this symbol tells the CRT that there is "
                                       "something in .bss, so it may need to be zeroed.");

  if (hasDPBSS())
    stronglyReference("__do_zero_dp_bss", "Declaring this symbol tells the CRT that there is "
                                          "something in .dp.bss, so it may need to be zeroed.");

  if (hasData())
    stronglyReference("__do_copy_data", "Declaring this symbol tells the CRT that there is something in .data, "
                                        "so it may need to be copied from LMA to VMA.");

  if (hasDPData())
    stronglyReference("__do_copy_dp_data", "Declaring this symbol tells the CRT that there is something in "
                                           ".dp.data, so it may need to be copied from LMA to VMA.");

  if (hasInitArray())
    stronglyReference("__do_init_array", "Declaring this symbol tells the CRT that there are "
                                         "initialization routines to be run in .init_array");

  if (hasFiniArray())
    stronglyReference("__do_fini_array", "Declaring this symbol tells the CRT that there are "
                                         "finalization routines to be run in .fini_array");

  bool ReferencesStackPtr = llvm::any_of(Context.getSymbols(), [](const StringMapEntry<MCSymbolTableValue> &TableEntry) { return TableEntry.getKey() == "__rc0" || TableEntry.getKey() == "__rc1"; });
  if (ReferencesStackPtr)
    stronglyReference("__do_init_stack", "Declaring this symbol tells the CRT that the stack "
                                         "pointer needs to be initialized.");
}

void MC6809TargetStreamer::stronglyReference(StringRef Name, StringRef Comment) {
  MCStreamer &OS = getStreamer();
  MCContext &Context = OS.getContext();
  MCSymbol *Sym = Context.getOrCreateSymbol(Name);
  OS.emitRawComment(Comment);
  stronglyReference(Sym);
}

static bool HasPrefix(StringRef Name, StringRef Prefix) {
  SmallString<32> PrefixDot = Prefix;
  PrefixDot += ".";
  return Name == Prefix || Name.starts_with(PrefixDot);
}

void MC6809TargetAsmStreamer::changeSection(const MCSection *CurSection, MCSection *Section, uint32_t SubSection, raw_ostream &OS) {
  MCTargetStreamer::changeSection(CurSection, Section, SubSection, OS);
  HasBSS |= HasPrefix(Section->getName(), ".bss");
  HasDPBSS |= HasPrefix(Section->getName(), ".dp.bss");
  HasData |= HasPrefix(Section->getName(), ".data");
  HasDPData |= HasPrefix(Section->getName(), ".dp.data");
  HasDPData |= HasPrefix(Section->getName(), ".dp.rodata");
  HasInitArray |= HasPrefix(Section->getName(), ".init_array");
  HasFiniArray |= HasPrefix(Section->getName(), ".fini_array");
}

void MC6809TargetAsmStreamer::stronglyReference(MCSymbol *Sym) { getStreamer().emitSymbolAttribute(Sym, MCSA_Global); }

MC6809TargetELFStreamer::MC6809TargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI) : MC6809TargetStreamer(S) {}

bool MC6809TargetELFStreamer::hasBSS() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasBSS(); }
bool MC6809TargetELFStreamer::hasDPBSS() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasDPBSS(); }
bool MC6809TargetELFStreamer::hasData() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasData(); }
bool MC6809TargetELFStreamer::hasDPData() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasDPData(); }
bool MC6809TargetELFStreamer::hasInitArray() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasInitArray(); }
bool MC6809TargetELFStreamer::hasFiniArray() { return static_cast<MC6809MCELFStreamer &>(getStreamer()).hasFiniArray(); }

bool MC6809TargetELFStreamer::emitDirectiveDirectPage(MCSymbol *Sym) {
  cast<MCSymbolELF>(Sym)->setOther(ELF::STO_MC6809_DIRECTPAGE);
  return true;
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
