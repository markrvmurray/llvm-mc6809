//===-- MC6809TargetStreamer.h - MC6809 Target Streamer --------------*- C++
//-*--===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_TARGET_STREAMER_H
#define LLVM_MC6809_TARGET_STREAMER_H

#include "llvm/MC/MCELFStreamer.h"

namespace llvm {
class MCStreamer;

/// A generic MC6809 target output stream.
class MC6809TargetStreamer : public MCTargetStreamer {
public:
  explicit MC6809TargetStreamer(MCStreamer &S);

  void finish() override;

protected:
  virtual bool hasInitArray() = 0;
  virtual bool hasFiniArray() = 0;

  virtual void stronglyReference(MCSymbol *Sym) = 0;
};

/// A target streamer for textual MC6809 assembly code.
class MC6809TargetAsmStreamer final : public MC6809TargetStreamer {
public:
  explicit MC6809TargetAsmStreamer(MCStreamer &S);

private:
  void changeSection(const MCSection *CurSection, MCSection *Section,
                     const MCExpr *SubSection, raw_ostream &OS) override;

  bool hasInitArray() override { return HasInitArray; }
  bool hasFiniArray() override { return HasFiniArray; }

  void stronglyReference(MCSymbol *Sym) override;

  bool HasInitArray = false;
  bool HasFiniArray = false;
};

/// A target streamer for an MC6809 ELF object file.
class MC6809TargetELFStreamer final : public MC6809TargetStreamer {
public:
  MC6809TargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

private:
  bool hasInitArray() override;
  bool hasFiniArray() override;

  void stronglyReference(MCSymbol *Sym) override;
};

} // end namespace llvm

#endif // LLVM_MC6809_TARGET_STREAMER_H
