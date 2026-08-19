//===-- MC6809TargetObjectFile.cpp - MC6809 Object Files
//------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809TargetObjectFile.h"
#include "MC6809.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DerivedTypes.h"
#include "llvm/IR/GlobalObject.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/MC/SectionKind.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/TargetParser/Triple.h"

using namespace llvm;

static bool typeHoldsPointer(Type *Ty, unsigned Depth = 0) {
  if (Depth > 16)
    return true; // give up conservatively
  if (Ty->isPointerTy())
    return true;
  if (auto *AT = dyn_cast<ArrayType>(Ty))
    return typeHoldsPointer(AT->getElementType(), Depth + 1);
  if (auto *VT = dyn_cast<VectorType>(Ty))
    return typeHoldsPointer(VT->getElementType(), Depth + 1);
  if (auto *ST = dyn_cast<StructType>(Ty)) {
    if (ST->isOpaque())
      return true;
    for (Type *E : ST->elements())
      if (typeHoldsPointer(E, Depth + 1))
        return true;
  }
  return false;
}

bool llvm::isOS9DataAreaObject(const GlobalVariable *GV) {
  if (!GV->isConstant())
    return true;
  // A constant that holds a pointer (a `FILE *const`, a table of strings or
  // functions) cannot live in the body: its value depends on where the
  // module and the data area are placed.  Judged by type so that a
  // declaration and its definition agree; a pointer-free aggregate whose
  // initialiser still needs a relocation (a pointer-sized integer holding
  // an address) is caught by the initialiser.
  if (typeHoldsPointer(GV->getValueType()))
    return true;
  if (GV->hasInitializer() && GV->getInitializer()->needsRelocation())
    return true;
  return false;
}

MCSection *MC6809TargetObjectFile::SelectSectionForGlobal(
    const GlobalObject *GO, SectionKind Kind, const TargetMachine &TM) const {
  // OS-9: a constant that the data model keeps in the data area (see
  // isOS9DataAreaObject) goes to a writable-image section (.data.rel.ro*)
  // that the module linker script copies into the data area, never to
  // .rodata in the body.
  if (TM.getTargetTriple().isOSOS9())
    if (const auto *GV = dyn_cast<GlobalVariable>(GO))
      if (GV->isConstant() && Kind.isReadOnly() && isOS9DataAreaObject(GV))
        Kind = SectionKind::getReadOnlyWithRel();
  return Base::SelectSectionForGlobal(GO, Kind, TM);
}

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
