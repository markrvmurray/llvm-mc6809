//===--------- MC6809MCELFStreamer.h - MC6809 subclass of MCELFStreamer ----===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCELFSTREAMER_H
#define LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCELFSTREAMER_H

#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCObjectWriter.h"

namespace llvm {

class MC6809MCELFStreamer : public MCELFStreamer {
  std::unique_ptr<MCInstrInfo> MCII;

public:
  MC6809MCELFStreamer(MCContext &Context, std::unique_ptr<MCAsmBackend> TAB, std::unique_ptr<MCObjectWriter> OW, std::unique_ptr<MCCodeEmitter> Emitter)
      : MCELFStreamer(Context, std::move(TAB), std::move(OW), std::move(Emitter)), MCII(createMC6809MCInstrInfo()) {}

  void initSections(bool NoExecStack, const MCSubtargetInfo &STI) override;
  void changeSection(MCSection *Section, uint32_t Subsection = 0) override;

  void emitInstruction(const MCInst &Inst, const MCSubtargetInfo &STI) override;

  void emitValueImpl(const MCExpr *Value, unsigned Size, SMLoc Loc = SMLoc()) override;

  void emitMc6809AddrAsciz(const MCExpr *Value, unsigned Size, SMLoc Loc = SMLoc());

  void emitMappingSymbol(StringRef Name);
  //void emit816MXState(bool IsMLow, bool IsMHigh, bool IsXLow, bool IsXHigh);

  bool hasBSS() const { return HasBSS; }
  bool hasDPBSS() const { return HasDPBSS; }
  bool hasData() const { return HasData; }
  bool hasDPData() const { return HasDPData; }
  bool hasInitArray() const { return HasInitArray; }
  bool hasFiniArray() const { return HasFiniArray; }

private:
  enum MXFlagState { MXFlagUnknown, MXFlagLow, MXFlagHigh };

  int64_t MappingSymbolCounter = 0;
  bool HasBSS = false;
  bool HasDPBSS = false;
  bool HasData = false;
  bool HasDPData = false;
  bool HasInitArray = false;
  bool HasFiniArray = false;
  bool Has6309Instructions = false;
  MXFlagState MState = MXFlagUnknown;
  MXFlagState XState = MXFlagUnknown;
};

MCStreamer *createMC6809MCELFStreamer(const Triple &T, MCContext &Ctx, std::unique_ptr<MCAsmBackend> &&TAB, std::unique_ptr<MCObjectWriter> &&OW, std::unique_ptr<MCCodeEmitter> &&Emitter);

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCELFSTREAMER_H
