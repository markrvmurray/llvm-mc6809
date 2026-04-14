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
  MC6809(Ctx &ctx);
  uint32_t calcEFlags() const override;
  RelExpr getRelExpr(RelType type, const Symbol &s,
                     const uint8_t *loc) const override;
  void relocate(uint8_t *loc, const Relocation &rel,
                uint64_t val) const override;
};
} // namespace

MC6809::MC6809(Ctx &ctx) : TargetInfo(ctx) {
  defaultMaxPageSize = 1;
  defaultCommonPageSize = 1;
}

static uint32_t getEFlags(InputFile *file) {
  return cast<ObjFile<ELF32LE>>(file)->getObj().getHeader().e_flags;
}

uint32_t MC6809::calcEFlags() const {
  uint32_t outputFlags = 0;

  for (InputFile *f : ctx.objectFiles) {
    const uint32_t flags = getEFlags(f);
    outputFlags |= flags;
  }

  return outputFlags;
}

RelExpr MC6809::getRelExpr(RelType type, const Symbol &s,
                        const uint8_t *loc) const {
  switch (type) {
  default:
    return R_ABS;
  case R_MC6809_PCREL_8:
  case R_MC6809_PCREL_16:
    return R_PC;
  }
}

void MC6809::relocate(uint8_t *loc, const Relocation &rel, uint64_t val) const {
  switch (rel.type) {
  case R_MC6809_IMM_8:
  case R_MC6809_ADDR_8:
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MC6809_PCREL_8:
    checkInt(ctx, loc, val - 1, 8, rel);
    // MC6809's PC relative addressing is off by one from the standard LLVM PC
    // relative convention.
    *loc = static_cast<unsigned char>(val - 1);
    break;
  case R_MC6809_ADDR_16:
    write16be(loc, static_cast<unsigned short>(val));
    break;
  case R_MC6809_PCREL_16:
    // MC6809's PC relative addressing is off by two from the standard LLVM PC
    // relative convention.
    write16be(loc, static_cast<unsigned short>(val - 2));
    break;
  case R_MC6809_OFFSET_5:
    // 5-bit signed indexed offset, packed into the low 5 bits of the
    // postbyte at *loc. The upper 3 bits (5-bit-form marker + 2-bit
    // register selector) must be preserved.
    checkInt(ctx, loc, val, 5, rel);
    *loc = (*loc & 0xE0) | (static_cast<unsigned char>(val) & 0x1F);
    break;
  case R_MC6809_OFFSET_8:
    // 8-bit signed indexed offset. Whole byte at *loc.
    checkInt(ctx, loc, val, 8, rel);
    *loc = static_cast<unsigned char>(val);
    break;
  case R_MC6809_OFFSET_16:
    // 16-bit signed indexed offset (big-endian). Full 6809 address
    // space fits, so no bounds check.
    write16be(loc, static_cast<unsigned short>(val));
    break;
  case R_MC6809_FK_DATA_4:
    write32be(loc, static_cast<unsigned long>(val));
    break;
  case R_MC6809_FK_DATA_8:
    write64be(loc, static_cast<unsigned long long>(val));
    break;
  default:
    error(getErrorLoc(ctx, loc) + "unrecognized relocation " +
          toStr(ctx, rel.type));
  }
}

void setMC6809TargetInfo(Ctx &ctx) { ctx.target.reset(new MC6809(ctx)); }

} // namespace elf
} // namespace lld