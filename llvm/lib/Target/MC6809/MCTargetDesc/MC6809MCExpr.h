//===-- MC6809MCExpr.h - MC6809 specific MC expression classes --------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_MCEXPR_H
#define LLVM_MC6809_MCEXPR_H

#include "llvm/MC/MCExpr.h"
#include "MCTargetDesc/MC6809FixupKinds.h"

namespace llvm {

/// A expression in MC6809 machine code.
class MC6809MCExpr : public MCTargetExpr {
public:
  /// Specifies the type of an expression.
  enum VariantKind {
    VK_MC6809_NONE,
    VK_MC6809_ADDR_8,
    VK_MC6809_ADDR_16
  };


  /// Creates an MC6809 machine code expression.
  static const MC6809MCExpr *create(VariantKind Kind, const MCExpr *Expr,
                                 bool isNegated, MCContext &Ctx);

  /// Gets the type of the expression.
  VariantKind getKind() const { return Kind; }
  /// Gets the name of the expression.
  MC6809::Fixups getFixupKind() const;
  const char *getName() const;
  const MCExpr *getSubExpr() const { return SubExpr; }
  /// Gets the fixup which corresponds to the expression.
  /// Evaluates the fixup as a constant value.
  bool evaluateAsConstant(int64_t &Result) const;

  bool isNegated() const { return Negated; }
  void setNegated(bool negated = true) { Negated = negated; }

  void printImpl(raw_ostream &OS, const MCAsmInfo *MAI) const override;
  bool evaluateAsRelocatableImpl(MCValue &Res, const MCAsmLayout *Layout,
                                 const MCFixup *Fixup) const override;

  void visitUsedExpr(MCStreamer &streamer) const override;

  MCFragment *findAssociatedFragment() const override {
    return getSubExpr()->findAssociatedFragment();
  }

  void fixELFSymbolsInTLSFixups(MCAssembler &Asm) const override {}

  static bool classof(const MCExpr *E) {
    return E->getKind() == MCExpr::Target;
  }

  static VariantKind getKindByName(StringRef Name);

private:
  int64_t evaluateAsInt64(int64_t Value) const;

  const VariantKind Kind;
  const MCExpr *SubExpr;
  bool Negated;

  explicit MC6809MCExpr(VariantKind Kind, const MCExpr *Expr, bool Negated)
      : Kind(Kind), SubExpr(Expr), Negated(Negated) {}
};

} // end namespace llvm

#endif // LLVM_MC6809_MCEXPR_H
