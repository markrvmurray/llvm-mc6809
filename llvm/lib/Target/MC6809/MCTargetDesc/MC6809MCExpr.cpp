//===-- MC6809MCExpr.cpp - MC6809 specific MC expression classes ----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809MCExpr.h"
#include "MC6809FixupKinds.h"

#include "llvm/MC/MCAsmLayout.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCStreamer.h"
#include "llvm/MC/MCValue.h"

namespace llvm {

namespace {

const struct ModifierEntry {
  const char *const Spelling;
  MC6809MCExpr::VariantKind VariantKind;
} ModifierNames[] = {
    {"mc6809_8", MC6809MCExpr::VK_MC6809_ADDR_8},
    {"mc6809_16", MC6809MCExpr::VK_MC6809_ADDR_16},
};

} // end of anonymous namespace

const MC6809MCExpr *MC6809MCExpr::create(VariantKind Kind, const MCExpr *Expr,
                                   bool Negated, MCContext &Ctx) {
  return new (Ctx) MC6809MCExpr(Kind, Expr, Negated);
}

void MC6809MCExpr::printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const {
  assert(Kind != VK_MC6809_NONE);

  if (isNegated()) {
    OS << '-';
  }

  OS << getName() << '(';
  getSubExpr()->print(OS, MAI);
  OS << ')';
}

bool MC6809MCExpr::evaluateAsConstant(int64_t &Result) const {
  MCValue Value;

  bool IsRelocatable =
      getSubExpr()->evaluateAsRelocatable(Value, nullptr, nullptr);

  if (!IsRelocatable) {
    return false;
  }

  if (Value.isAbsolute()) {
    Result = evaluateAsInt64(Value.getConstant());
    return true;
  }

  return false;
}

bool MC6809MCExpr::evaluateAsRelocatableImpl(MCValue &Result,
                                          const MCAsmLayout *Layout,
                                          const MCFixup *Fixup) const {
  MCValue Value;
  bool IsRelocatable = SubExpr->evaluateAsRelocatable(Value, Layout, Fixup);

  if (!IsRelocatable)
    return false;

  if (Value.isAbsolute()) {
    Result = MCValue::get(evaluateAsInt64(Value.getConstant()));
  } else {
    if (!Layout) {
      return false;
    }

    MCContext &Context = Layout->getAssembler().getContext();
    const MCSymbolRefExpr *Sym = Value.getSymA();
    MCSymbolRefExpr::VariantKind Modifier = Sym->getKind();
    if (Modifier != MCSymbolRefExpr::VK_None) {
      return false;
    }

    Sym = MCSymbolRefExpr::create(&Sym->getSymbol(), Modifier, Context);
    Result = MCValue::get(Sym, Value.getSymB(), Value.getConstant());
  }

  return true;
}

int64_t MC6809MCExpr::evaluateAsInt64(int64_t Value) const {
  if (Negated) {
    Value *= -1;
  }

  switch (Kind) {
  case MC6809MCExpr::VK_MC6809_ADDR_8:
    Value &= 0xff;
    break;
  case MC6809MCExpr::VK_MC6809_ADDR_16:
    Value &= 0xffff;
    break;
  default:
    llvm_unreachable("Uninitialized expression.");
  }
  return static_cast<uint64_t>(Value);
}

MC6809::Fixups MC6809MCExpr::getFixupKind() const {
  MC6809::Fixups Kind = MC6809::Fixups::LastTargetFixupKind;

  switch (getKind()) {
  case VK_MC6809_ADDR_8:
    Kind = MC6809::Addr8;
    break;
  case VK_MC6809_ADDR_16:
    Kind = MC6809::Addr16;
    break;
  default:
    llvm_unreachable("Uninitialized expression");
  }

  return Kind;
}

void MC6809MCExpr::visitUsedExpr(MCStreamer &Streamer) const {
  Streamer.visitUsedExpr(*getSubExpr());
}

const char *MC6809MCExpr::getName() const {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [this](ModifierEntry const &Mod) { return Mod.VariantKind == Kind; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->Spelling;
  }
  return nullptr;
}

MC6809MCExpr::VariantKind MC6809MCExpr::getKindByName(StringRef Name) {
  const auto &Modifier = std::find_if(
      std::begin(ModifierNames), std::end(ModifierNames),
      [&Name](ModifierEntry const &Mod) { return Mod.Spelling == Name; });

  if (Modifier != std::end(ModifierNames)) {
    return Modifier->VariantKind;
  }
  return VK_MC6809_NONE;
}

} // end of namespace llvm
