//===-- MC6809MCInstrAnalysis.h - MC6809 instruction analysis ---*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declaration of the MC6809MCAsmInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCINSTRANALYSIS_H
#define LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCINSTRANALYSIS_H

#include "llvm/MC/MCInstrAnalysis.h"

namespace llvm {

class Triple;

class MC6809MCInstrAnalysis : public MCInstrAnalysis {
public:
  explicit MC6809MCInstrAnalysis(const MCInstrInfo *Info) : MCInstrAnalysis(Info) {}

  bool evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size, uint64_t &Target) const override;

  std::optional<uint64_t> evaluateMemoryOperandAddress(const MCInst &Inst, const MCSubtargetInfo *STI, uint64_t Addr, uint64_t Size) const override;
};

} // end namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MCTARGETDESC_MC6809MCINSTRANALYSIS_H
