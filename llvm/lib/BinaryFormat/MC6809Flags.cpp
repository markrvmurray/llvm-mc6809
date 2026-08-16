//===-- MC6809Flags.cpp - MC6809 ELF e_flags tools ---------------------*- C++-*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "llvm/BinaryFormat/MC6809Flags.h"
#include "llvm/ADT/Enum.h"
#include "llvm/BinaryFormat/ELF.h"

namespace llvm {
namespace MC6809 {

#define ENUM_ENT(enum, altName) {{#enum, altName}, ELF::enum}

EnumStrings<unsigned, 2> getElfHeaderMC6809Flags() {
  static constexpr EnumStringDef<unsigned, 2> ElfHeaderMC6809FlagsDefs[] = {
      ENUM_ENT(EF_MC6809_ARCH_6809, "mc6809"),
      ENUM_ENT(EF_MC6809_ARCH_6309, "hd6309")};
  static constexpr auto ElfHeaderMC6809FlagsStorage =
      BUILD_ENUM_STRINGS(ElfHeaderMC6809FlagsDefs);
  return EnumStrings(ElfHeaderMC6809FlagsStorage);
}

std::string makeEFlagsString(unsigned EFlags) {
  std::string Str;
  raw_string_ostream Stream(Str);
  ScopedPrinter Printer(Stream);
  Printer.printFlags("Flags", EFlags, getElfHeaderMC6809Flags());
  Stream.flush();
  return Str;
}

} // namespace MC6809
} // namespace llvm
