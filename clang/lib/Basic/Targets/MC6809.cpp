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
#include "clang/Basic/Builtins.h"
#include "clang/Basic/MacroBuilder.h"
#include "clang/Basic/TargetBuiltins.h"
#include "clang/Basic/TargetInfo.h"

using namespace clang;
using namespace clang::targets;

static constexpr int NumMC6809Builtins =
    MC6809::LastTSBuiltin - Builtin::FirstTSBuiltin;

static constexpr llvm::StringTable MC6809BuiltinStrings =
    CLANG_BUILTIN_STR_TABLE_START
#define BUILTIN CLANG_BUILTIN_STR_TABLE
#include "clang/Basic/BuiltinsMC6809.def"
    ;

static constexpr auto MC6809BuiltinInfos =
    Builtin::MakeInfos<NumMC6809Builtins>({
#define BUILTIN CLANG_BUILTIN_ENTRY
#include "clang/Basic/BuiltinsMC6809.def"
    });

llvm::SmallVector<Builtin::InfosShard>
MC6809TargetInfo::getTargetBuiltins() const {
  return {{&MC6809BuiltinStrings, MC6809BuiltinInfos}};
}

MC6809TargetInfo::MC6809TargetInfo(const llvm::Triple &Triple, const TargetOptions &)
    : TargetInfo(Triple) {
  // Address space 1 = direct page (8-bit pointers, high byte implicit
  // in DP register). Bug #178 user `__attribute__((directpage))`
  // globals live there. Must match MC6809TargetMachine.cpp.
  static const char Layout[] =
      "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16";
  resetDataLayout(Layout);

  PointerWidth = 16;
  PointerAlign = 8;
  ShortAlign = 8;
  IntWidth = 16;
  IntAlign = 8;
  LongAlign = 8;
  LongLongAlign = 8;
  BFloat16Align = 8;
  FloatAlign = 8;
  DoubleAlign = 8;
  LongDoubleAlign = 8;
  Float128Align = 8;
  ShortAccumAlign = 8;
  AccumWidth = 16;
  AccumAlign = 8;
  LongAccumAlign = 8;
  FractWidth = FractAlign = 8;
  LongFractAlign = 8;
  AccumScale = 7;
  SuitableAlign = 8;
  DefaultAlignForAttributeAligned = 8;
  SizeType = UnsignedInt;
  PtrDiffType = SignedInt;
  IntPtrType = SignedInt;
  WCharType = UnsignedLong;
  WIntType = SignedLong;
  Char16Type = UnsignedInt;
  Char32Type = UnsignedLong;
  Int16Type = SignedInt;
  SigAtomicType = UnsignedChar;
}

bool MC6809TargetInfo::validateAsmConstraint(
    const char *&Name, TargetInfo::ConstraintInfo &Info) const {
  switch (*Name) {
  default:
    return false;
  case 'r':  // gcc6809 GENERAL_REGS: any reg, size-dependent (INDEX16 or ACC8)
  case 'q':  // gcc6809 Q_REGS: 8-bit byte registers (A or B)
  case 'A':  // gcc6809 ACC_A_REGS: A register (8-bit, high byte of D)
  case 'B':  // gcc6809 ACC_B_REGS: B register (8-bit, low byte of D)
  case 'a':  // gcc6809 A_REGS: 16-bit address registers (X, Y, U, S)
  case 'x':  // X index register (16-bit)
  case 'y':  // Y index register (16-bit)
  case 'R':  // any general register
  case 'd':  // gcc6809 D_REGS: D register (16-bit accumulator A:B)
  case 'c':  // carry flag (CC.C)
  case 'v':  // overflow flag (CC.V)
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
  // 8-bit-only constraints: individual A/B registers, q (A or B), CC flags
  case 'A': case 'B': case 'q': case 'c': case 'v':
    return Size <= 8;
  // 16-bit-max constraints: D accumulator, address regs (X/Y/U/S), general
  case 'd': case 'a': case 'x': case 'y': case 'R': case 'r':
    return Size <= 16;
  }
}

llvm::StringRef
MC6809TargetInfo::getConstraintRegister(StringRef Constraint,
                                     StringRef Expression) const {
  StringRef::iterator I, E;
  for (I = Constraint.begin(), E = Constraint.end(); I != E; ++I) {
    if (isalpha(*I))
      break;
  }
  if (I == E)
    return "";
  switch (*I) {
  case 'a':
    return "a";
  case 'x':
    return "x";
  case 'y':
    return "y";
  case 'c':
    return "c";
  case 'v':
    return "v";
  case 'd':
  case 'g':
  case 'r':
  case 'R':
    // This handles any global or local register variables that make this more
    // explicit.
    return Expression;
  }
  return "";
}

static const char *const GCCRegNames[] = {
    "a", "b", "d", "e", "f", "w", "q",
    "x", "y", "u", "s",
    "dp", "cc", "pc",
    "0", "v", "md",
};

llvm::ArrayRef<const char *> MC6809TargetInfo::getGCCRegNames() const {
  return GCCRegNames;
}

uint64_t MC6809TargetInfo::getPointerWidthV(LangAS AddrSpace) const {
  // Coerce all non-target address spaces to the default address space.
  if (!isTargetAddressSpace(AddrSpace))
    return getPointerWidthV(LangAS::FirstTargetAddressSpace);

  switch (toTargetAddressSpace(AddrSpace)) {
  case 1: // Direct page memory
    return 8;
  default:
    return 16;
  }
}

// Bug #161: keep these CPU names in lockstep with the `def : Device<...>`
// list in `llvm/lib/Target/MC6809/MC6809Devices.td`. The clang driver
// validates here; the LLVM backend resolves the same string against
// the Device list. A mismatch silently routes the user back to the
// default `mc6809` device with no HD6309 features set, hiding the
// entire HD6309 codegen path. The `getTargetDefines` switch below
// (line ~220) must use the same spellings for `__6309__` to be set.
static constexpr llvm::StringLiteral ValidCPUNames[] = {
    {"mc6809"},    {"hd6309"}};

bool MC6809TargetInfo::isValidCPUName(StringRef Name) const {
  return llvm::find(ValidCPUNames, Name) != std::end(ValidCPUNames);
}

void MC6809TargetInfo::fillValidCPUList(SmallVectorImpl<StringRef> &Values) const {
  Values.append(std::begin(ValidCPUNames), std::end(ValidCPUNames));
}

bool MC6809TargetInfo::setCPU(StringRef Name) {
  if (isValidCPUName(Name)) {
    CPUName = Name;
    return true;
  }
  return false;
}

void MC6809TargetInfo::getTargetDefines(const LangOptions &Opts,
                                     MacroBuilder &Builder) const {
  Builder.defineMacro("__motorola__");
  Builder.defineMacro("__ELF__");
  Builder.defineMacro("__SOFTFP__");
  Builder.defineMacro("__mc6809__");
  Builder.defineMacro("__MC6809__");

  // Generate instruction feature set macros.
  const auto &CPUDefines =
      llvm::StringSwitch<std::vector<std::string>>(CPUName)
          .Case("mc6809", {"6809"})
          .Case("hd6309", {"6809", "6309"})
          .Default({"6809"});
  for (const auto &CPUDefine : CPUDefines) {
    Builder.defineMacro("__" + CPUDefine + "__");
  }

  // Bug #178: convenience macros for the directpage attribute. Both
  // expand to the dedicated __attribute__((directpage)) (which Sema
  // validates is only on static-storage variables); the macro names
  // are a backward-compat surface plus the canonical short alias.
  // The earlier `__deropage` macro was a typo and is removed.
  Builder.defineMacro("__directpage", "__attribute__((directpage))");
  Builder.defineMacro("__dp", "__attribute__((directpage))");
}
