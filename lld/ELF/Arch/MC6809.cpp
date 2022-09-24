//===- MC6809.cpp ------------------------------------------------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "InputFiles.h"
#include "Symbols.h"
#include "Target.h"
#include "lld/Common/ErrorHandler.h"
#include "llvm/BinaryFormat/ELF.h"
#include "llvm/BinaryFormat/MC6809Flags.h"
#include "llvm/Object/ELF.h"
#include "llvm/Support/Endian.h"

using namespace llvm;
using namespace llvm::object;
using namespace llvm::support::endian;
using namespace llvm::ELF;

namespace lld {
namespace elf {

namespace {
class MC6809 final : public TargetInfo {
public:
  MC6809();
  uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

MC6809::MC6809() {
  defaultMaxPageSize = 1;
  defaultCommonPageSize = 1;
}

static uint32_t getEFlags(InputFile *file) {
  return cast<ObjFile<ELF32LE>>(file)->getObj().getHeader().e_flags;
}

uint32_t MC6809::calcEFlags() const {
  uint32_t outputFlags = 0;

  for (InputFile *f : ctx->objectFiles) {
    const uint32_t flags = getEFlags(f);
    outputFlags |= flags;
  }

  return outputFlags;
}

RelExpr MC6809::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  return R_ABS;
}

void MC6809::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_MC6809_IMM_8:
  case R_MC6809_ADDR_8:
  case R_MC6809_OFFSET_5:
  case R_MC6809_OFFSET_8:
  case R_MC6809_PCREL_8:
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MC6809_ADDR_16:
  case R_MC6809_OFFSET_16:
  case R_MC6809_PCREL_16:
    write16le(loc, static_cast<unsigned short>(val));
    break;
  case R_MC6809_FK_DATA_4:
    write32le(loc, static_cast<unsigned long>(val));
    break;
  case R_MC6809_FK_DATA_8:
    write64le(loc, static_cast<unsigned long long>(val));
    break;
  default:
    error(getErrorLocation(loc) + "unrecognized relocation " +
          toString(rel.type));
  }
}

TargetInfo *getMC6809TargetInfo() {
  static MC6809 target;
  return &target;
}

} // namespace elf
} // namespace lld
