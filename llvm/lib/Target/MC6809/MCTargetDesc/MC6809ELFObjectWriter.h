//===-- MC6809ELFObjectWriter.cpp - MC6809 ELF Writer ---------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MC6809FixupKinds.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/MC/MCAssembler.h"
#include "llvm/MC/MCELFObjectWriter.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCObjectWriter.h"
#include "llvm/MC/MCSection.h"
#include "llvm/MC/MCValue.h"
#include "llvm/Support/ErrorHandling.h"

namespace llvm {

/// Writes MC6809 machine code into an ELF32 object file.
class MC6809ELFObjectWriter : public MCELFObjectTargetWriter {
public:
  explicit MC6809ELFObjectWriter(uint8_t OSABI);
  virtual ~MC6809ELFObjectWriter();
  unsigned getRelocType(MCContext &Ctx,
                        const MCValue &Target,
                        const MCFixup &Fixup,
                        bool IsPCRel) const override;
};

} // end of namespace llvm

