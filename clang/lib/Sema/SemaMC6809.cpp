//===------ SemaMC6809.cpp ------ MC6809 target-specific routines ---------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//  This file implements semantic analysis functions specific to MC6809.
//
//===----------------------------------------------------------------------===//

#include "clang/Sema/SemaMC6809.h"

#include "clang/AST/ASTContext.h"
#include "clang/Sema/Attr.h"

using namespace llvm;

namespace clang {

void SemaMC6809::handleInterruptAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type) << "'interrupt'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MC6809InterruptAttr>(*this, D, AL);
}

void SemaMC6809::handleInterruptNorecurseAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type) << "'interrupt_norecurse'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MC6809InterruptNorecurseAttr>(*this, D, AL);
}

void SemaMC6809::handleInterruptNoISRAttr(Decl *D, const ParsedAttr &AL) {
  if (!isFuncOrMethodForAttrSubject(D)) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type) << "'no_isr'" << ExpectedFunction;
    return;
  }

  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;

  handleSimpleAttribute<MC6809NoISRAttr>(*this, D, AL);
}

SemaMC6809::SemaMC6809(Sema &S) : SemaBase(S) {}

} // namespace clang
