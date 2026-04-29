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
#include "clang/AST/Decl.h"
#include "clang/Sema/Attr.h"
#include "clang/Sema/Sema.h"

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

// Bug #178: __attribute__((directpage)) → addrspace(1) global on
// MC6809 direct page ($00-$FF). Subjects=[Var] in Attr.td already
// restricts to variables; here we additionally enforce that the
// variable has static storage duration. Cross-TU sharing would
// complicate the link-time DP allocation model and isn't supported.
void SemaMC6809::handleDirectPageAttr(Decl *D, const ParsedAttr &AL) {
  if (!AL.checkExactlyNumArgs(SemaRef, 0))
    return;
  auto *VD = dyn_cast<VarDecl>(D);
  if (!VD) {
    Diag(D->getLocation(), diag::warn_attribute_wrong_decl_type)
        << "'directpage'" << ExpectedVariable;
    return;
  }
  if (VD->getStorageClass() != SC_Static) {
    Diag(D->getLocation(), diag::err_attribute_requires_static_storage)
        << AL << AL.getRange();
    return;
  }

  // Bug #192 follow-up: also stamp the variable's TYPE with the
  // addrspace(1) qualifier — without this, the global itself lives
  // in addrspace(1) but every load/store gets an addrspacecast back
  // to addrspace(0) (Clang's default). The MC6809 backend can't
  // legalize G_ADDRSPACE_CAST p1→p0, and even if it could, the cast
  // would lose the information that the access should select the
  // DP-mode opcode (LDAd/STAd/LDDd/STDd). Stamping the QualType
  // makes loads/stores emit directly in addrspace(1).
  QualType OrigTy = VD->getType();
  LangAS DPAddrSpace = getLangASFromTargetAS(1);
  if (OrigTy.getAddressSpace() != DPAddrSpace) {
    QualType NewTy = SemaRef.Context.getAddrSpaceQualType(OrigTy, DPAddrSpace);
    VD->setType(NewTy);
    if (TypeSourceInfo *TSI = VD->getTypeSourceInfo())
      VD->setTypeSourceInfo(SemaRef.Context.CreateTypeSourceInfo(NewTy));
  }

  handleSimpleAttribute<MC6809DirectPageAttr>(*this, D, AL);
}

SemaMC6809::SemaMC6809(Sema &S) : SemaBase(S) {}

} // namespace clang
