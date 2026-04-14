//===-- MC6809ELFObjectWriter.cpp - MC6809 ELF Writer ---------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MC6809FixupKinds.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes MC6809 machine code into an ELF32 object file.
class MC6809ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  explicit MC6809ELFObjectWriter(uint8_t OSABI);

  unsigned getRelocType(const MCFixup &Fixup, const MCValue &Target,
                        bool IsPCRel) const override;
};

MC6809ELFObjectWriter::MC6809ELFObjectWriter(uint8_t OSABI)
    : MCELFObjectTargetWriter(false, OSABI, ELF::EM_MC6809, true) {}

unsigned MC6809ELFObjectWriter::getRelocType(const MCFixup &Fixup,
                                          const MCValue &Target,
                                          bool IsPCRel) const {
  unsigned Kind = Fixup.getKind();
  auto Specifier = static_cast<MC6809MCExpr::VariantKind>(Target.getSpecifier());
  switch (Kind) {
  case FK_Data_1:
    switch (Specifier) {
    default:
      llvm_unreachable("Unsupported Specifier");
    case MC6809MCExpr::VK_NONE:
    case MC6809MCExpr::VK_ADDR8:
      return ELF::R_MC6809_ADDR_8;
    case MC6809MCExpr::VK_ADDR16:
      return ELF::R_MC6809_ADDR_16;
    }
  case FK_Data_2:
    switch (Specifier) {
    default:
      llvm_unreachable("Unsupported Specifier");
    case MC6809MCExpr::VK_NONE:
    case MC6809MCExpr::VK_ADDR16:
      return ELF::R_MC6809_ADDR_16;
    }
  case MC6809::Imm8:
    return ELF::R_MC6809_IMM_8;
  case MC6809::Addr8:
    return ELF::R_MC6809_ADDR_8;
  case MC6809::Imm16:
  case MC6809::Addr16:
    return ELF::R_MC6809_ADDR_16;
  case MC6809::PCRel8:
    return ELF::R_MC6809_PCREL_8;
  case MC6809::PCRel16:
    return ELF::R_MC6809_PCREL_16;
  case MC6809::Rel5:
    return ELF::R_MC6809_OFFSET_5;
  case MC6809::Rel8:
    return ELF::R_MC6809_OFFSET_8;
  case MC6809::Rel16:
    return ELF::R_MC6809_OFFSET_16;
  case FK_Data_4:
    return ELF::R_MC6809_FK_DATA_4;
  case FK_Data_8:
    return ELF::R_MC6809_FK_DATA_8;
  default:
    llvm_unreachable("invalid fixup kind!");
  }
}

std::unique_ptr<MCObjectTargetWriter> createMC6809ELFObjectWriter(uint8_t OSABI) {
  return std::make_unique<MC6809ELFObjectWriter>(OSABI);
}

} // end of namespace llvm
