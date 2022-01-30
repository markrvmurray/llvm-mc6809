//===--- MC6809.h - Declare MC6809 target feature support -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares MC6809 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CLANG_LIB_BASIC_TARGETS_MC6809_H
#define LLVM_CLANG_LIB_BASIC_TARGETS_MC6809_H

#include "clang/Basic/TargetInfo.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/ADT/Triple.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Support/Compiler.h"

namespace clang {
namespace targets {

class MC6809TargetInfo : public TargetInfo {
public:
  MC6809TargetInfo(const llvm::Triple &Triple, const TargetOptions &);

  void getTargetDefines(const LangOptions &Opts,
                        MacroBuilder &Builder) const override {}

  ArrayRef<Builtin::Info> getTargetBuiltins() const override { return None; }

  BuiltinVaListKind getBuiltinVaListKind() const override {
    return TargetInfo::VoidPtrBuiltinVaList;
  }

  bool validateAsmConstraint(const char *&Name,
                             TargetInfo::ConstraintInfo &Info) const override;

  bool validateOutputSize(const llvm::StringMap<bool> &FeatureMap,
                          StringRef Constraint, unsigned Size) const override;
  bool validateInputSize(const llvm::StringMap<bool> &FeatureMap,
                         StringRef Constraint, unsigned Size) const override;
  bool validateOperandSize(const llvm::StringMap<bool> &FeatureMap,
                           StringRef Constraint, unsigned Size) const;

  const char *getClobbers() const override { return ""; }

  ArrayRef<const char *> getGCCRegNames() const override;
  ArrayRef<TargetInfo::GCCRegAlias> getGCCRegAliases() const override {
    return None;
  }
  unsigned getRegisterWidth() const override { return 8; }

  bool isValidCPUName(StringRef Name) const override;
  void fillValidCPUList(SmallVectorImpl<StringRef> &Values) const override;

  bool setCPU(const std::string &Name) override { return isValidCPUName(Name); }
};

} // namespace targets
} // namespace clang

#endif // LLVM_CLANG_LIB_BASIC_TARGETS_MC6809_H
