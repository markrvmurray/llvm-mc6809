//===-- MC6809TargetObjectFile.cpp - MC6809 Object Files
//------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809TargetObjectFile.h"
#include "llvm/IR/GlobalObject.h"
#include "llvm/MC/SectionKind.h"

using namespace llvm;

MCSection *MC6809TargetObjectFile::getExplicitSectionGlobal(
    const GlobalObject *GO, SectionKind Kind, const TargetMachine &TM) const {
  // The direct-page zero-init region (`.dp.bss` / `.dp.bss.*`) holds the
  // static-stack frame and any direct-page .bss. A global with an explicit
  // section is never auto-classified as BSS (isSuitableForBSS bails on an
  // explicit section), so it would default to @progbits — emitting real zero
  // bytes into the image and forcing the whole .dp.bss output section loaded.
  // The generic name-based classifier only recognises a `.bss`-prefixed name;
  // it passes any other dotted name (including `.dp.bss.*`) through unchanged.
  // So restoring the BSS kind here makes it emit @nobits (zeroed at startup, no
  // image cost) — matching the write-before-read static frame and the linker's
  // zero-init directpage region.
  StringRef Name = GO->getSection();
  if (Name == ".dp.bss" || Name.starts_with(".dp.bss."))
    Kind = SectionKind::getBSS();
  return Base::getExplicitSectionGlobal(GO, Kind, TM);
}
