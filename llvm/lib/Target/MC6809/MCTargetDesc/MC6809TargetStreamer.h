//===-- MC6809TargetStreamer.h - MC6809 Target Streamer --------*- C++ -*--===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809TARGETSTREAMER_H
#define LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809TARGETSTREAMER_H

#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCSymbol.h"
#include "llvm/Support/FormattedStream.h"

namespace llvm {
class MCStreamer;

/// A generic MC6809 target output stream.
class MC6809TargetStreamer : public MCTargetStreamer {
public:
  explicit MC6809TargetStreamer(MCStreamer &S);

  void finish() override;
  virtual bool emitDirectiveDirectPage(MCSymbol *Symbol) = 0;

protected:
  virtual bool hasBSS() = 0;
  virtual bool hasDPBSS() = 0;
  virtual bool hasData() = 0;
  virtual bool hasDPData() = 0;
  virtual bool hasInitArray() = 0;
  virtual bool hasFiniArray() = 0;

  void stronglyReference(StringRef Name, StringRef Comment);
  virtual void stronglyReference(MCSymbol *Sym) = 0;
};

/// A target streamer for textual MC6809 assembly code.
/// The target streamer behind the null streamer, which parses module asm to
/// find its symbols when writing LTO bitcode. It must still be an
/// MC6809TargetStreamer, because the asm parser casts to one for .directpage.
class MC6809TargetNullStreamer final : public MC6809TargetStreamer {
public:
  explicit MC6809TargetNullStreamer(MCStreamer &S) : MC6809TargetStreamer(S) {}

  bool emitDirectiveDirectPage(MCSymbol *Symbol) override { return true; }

private:
  bool hasBSS() override { return false; }
  bool hasDPBSS() override { return false; }
  bool hasData() override { return false; }
  bool hasDPData() override { return false; }
  bool hasInitArray() override { return false; }
  bool hasFiniArray() override { return false; }

  void stronglyReference(MCSymbol *Sym) override {}
};

class MC6809TargetAsmStreamer final : public MC6809TargetStreamer {
public:
  MC6809TargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS);

private:
  formatted_raw_ostream &OS;

  bool emitDirectiveDirectPage(MCSymbol *Symbol) override {
    OS << "\t.directpage\t" << Symbol->getName() << "\n";
    return true;
  };

  void changeSection(const MCSection *CurSection, MCSection *Section, uint32_t SubSection, raw_ostream &Out) override;

  bool hasBSS() override { return HasBSS; }
  bool hasDPBSS() override { return HasDPBSS; }
  bool hasData() override { return HasData; }
  bool hasDPData() override { return HasDPData; }
  bool hasInitArray() override { return HasInitArray; }
  bool hasFiniArray() override { return HasFiniArray; }

  void stronglyReference(MCSymbol *Sym) override;

  bool HasBSS = false;
  bool HasDPBSS = false;
  bool HasData = false;
  bool HasDPData = false;
  bool HasInitArray = false;
  bool HasFiniArray = false;
};

inline MC6809TargetAsmStreamer::MC6809TargetAsmStreamer(MCStreamer &S, formatted_raw_ostream &OS) : MC6809TargetStreamer(S), OS(OS) {}

/// A target streamer for an MC6809 ELF object file.
class MC6809TargetELFStreamer final : public MC6809TargetStreamer {
public:
  MC6809TargetELFStreamer(MCStreamer &S, const MCSubtargetInfo &STI);

private:
  bool emitDirectiveDirectPage(MCSymbol *Symbol) override;
  bool hasBSS() override;
  bool hasDPBSS() override;
  bool hasData() override;
  bool hasDPData() override;
  bool hasInitArray() override;
  bool hasFiniArray() override;

  void stronglyReference(MCSymbol *Sym) override;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809TARGETSTREAMER_H
