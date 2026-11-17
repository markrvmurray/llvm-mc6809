//===- MC6809.cpp ---------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "ABIInfoImpl.h"
#include "TargetInfo.h"

using namespace clang;
using namespace clang::CodeGen;

namespace {

class MC6809TargetCodeGenInfo : public TargetCodeGenInfo {
public:
  MC6809TargetCodeGenInfo(CodeGenTypes &CGT)
      : TargetCodeGenInfo(std::make_unique<DefaultABIInfo>(CGT)) {}

  // Bug #178: variables carrying __attribute__((directpage)) live in
  // LLVM address space 1 — the MC6809 direct page ($0000-$00FF). The
  // backend's DataLayout maps p1 to 8-bit pointers and the
  // InstructionSelector picks the DP-mode opcode (LDAd/STAd/LDDd/STDd)
  // for loads and stores against them.
  LangAS getGlobalVarAddressSpace(CodeGenModule &CGM,
                                  const VarDecl *D) const override {
    if (D && D->hasAttr<MC6809DirectPageAttr>())
      return getLangASFromTargetAS(1);
    return TargetCodeGenInfo::getGlobalVarAddressSpace(CGM, D);
  }

  // Tell the backend which functions are interrupt handlers: it returns from
  // them with RTI and has them preserve the direct-page registers.
  void setTargetAttributes(const Decl *D, llvm::GlobalValue *GV,
                           CodeGen::CodeGenModule &CGM) const override {
    if (GV->isDeclaration())
      return;
    const auto *FD = dyn_cast_or_null<FunctionDecl>(D);
    if (FD && FD->hasAttr<MC6809InterruptAttr>())
      cast<llvm::Function>(GV)->addFnAttr("interrupt");
  }
};

} // anonymous namespace

std::unique_ptr<TargetCodeGenInfo>
CodeGen::createMC6809TargetCodeGenInfo(CodeGenModule &CGM) {
  return std::make_unique<MC6809TargetCodeGenInfo>(CGM.getTypes());
}
