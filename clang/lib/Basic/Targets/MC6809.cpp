//===--- MC6809.cpp - Implement MC6809 target feature support -------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements MC6809 TargetInfo objects.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"

using namespace clang::targets;

MC6809TargetInfo::MC6809TargetInfo(const llvm::Triple &Triple, const TargetOptions &)
    : TargetInfo(Triple) {
  static const char Layout[] =
    "e-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f16:8-f32:8-f64:8-a:0-n8:16";
  resetDataLayout(Layout);

  PointerWidth = 16;
  PointerAlign = 8;
  IntWidth = 16;
  IntAlign = 8;
  LongAlign = 8;
  LongLongAlign = 8;
  ShortAccumWidth = 8;
  ShortAccumAlign = 8;
  AccumWidth = 8;
  AccumAlign = 8;
  LongAccumWidth = 16;
  LongAccumAlign = 8;
  FractWidth = FractAlign = 8;
  LongFractAlign = 8;
  AccumScale = 7;
  SuitableAlign = 8;
  DefaultAlignForAttributeAligned = 8;
  SizeType = UnsignedShort;
  PtrDiffType = SignedShort;
  IntPtrType = SignedShort;
  WCharType = UnsignedLong;
  WIntType = UnsignedLong;
  Char32Type = UnsignedLong;
  SigAtomicType = UnsignedChar;
}

bool MC6809TargetInfo::validateAsmConstraint(
    const char *&Name, TargetInfo::ConstraintInfo &Info) const {
  switch (*Name) {
  default:
    return false;
  // The Accumulators
  case 'a':
  // The Index registers.
  case 'x':
    Info.setAllowsRegister();
    return true;
  }
}

bool MC6809TargetInfo::validateOutputSize(const llvm::StringMap<bool> &FeatureMap,
                                       StringRef Constraint,
                                       unsigned Size) const {
  // Strip off constraint modifiers.
  while (Constraint[0] == '=' || Constraint[0] == '+' || Constraint[0] == '&')
    Constraint = Constraint.substr(1);

  return validateOperandSize(FeatureMap, Constraint, Size);
}

bool MC6809TargetInfo::validateInputSize(const llvm::StringMap<bool> &FeatureMap,
                                      StringRef Constraint,
                                      unsigned Size) const {
  return validateOperandSize(FeatureMap, Constraint, Size);
}

bool MC6809TargetInfo::validateOperandSize(const llvm::StringMap<bool> &FeatureMap,
                                        StringRef Constraint,
                                        unsigned Size) const {
  switch (Constraint[0]) {
  default:
    return true;
  case 'a':
  case 'x':
    return Size <= 16;
  }
}

static const char *const GCCRegNames[] = {
    "a",     "b",     "e",     "f",     "d",     "w",     "q",
    "x",     "y",     "u",     "s",
    "pc",    "dp",    "cc",    "md",    "v",     "0",
};

llvm::ArrayRef<const char *> MC6809TargetInfo::getGCCRegNames() const {
  return llvm::makeArrayRef(GCCRegNames);
}

static constexpr llvm::StringLiteral ValidCPUNames[] = {
    {"mc6809"},   {"hd6309"}};

bool MC6809TargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::find(ValidCPUNames, Name) != std::end(ValidCPUNames);
}

void MC6809TargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  Values.append(std::begin(ValidCPUNames), std::end(ValidCPUNames));
}
