//===- MC6809MachineFuctionInfo.h - MC6809 machine function info -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===-----------------------------------------------------------------------===//
//
// This file declares MC6809-specific per-machine-function information.
//
//===-----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

class MC6809Subtarget;
class GlobalValue;

struct MC6809FunctionInfo : public MachineFunctionInfo {
  MC6809FunctionInfo(const Function &F, const MC6809Subtarget *STI) {}

  /// Bug #387: for a function whose frame is laid out in static memory
  /// (see MC6809StaticStackAlloc), the global (alias) that backs this
  /// function's static-stack region. Null for ordinary dynamic-frame
  /// functions. The AsmPrinter resolves Mc6809Static frame indices through
  /// this symbol.
  const GlobalValue *StaticStackValue = nullptr;

  /// Bug #387: the next free byte offset in this function's static-stack region.
  /// processFunctionBeforeFrameFinalized seeds it after marking the spill slots
  /// that exist at frame-finalisation time; MaterializeSpills continues it for
  /// the spill slots it creates later (which frame finalisation never saw), so
  /// those late slots also land in the static frame instead of a now-shrunk
  /// dynamic frame. Only meaningful when usesStaticStack(MF).
  int64_t NextStaticStackOffset = 0;

  /// Bug #387: frame indices that processFunctionBeforeFrameFinalized found
  /// reached by at least one pseudo WITHOUT a _Sym (extended-addressing)
  /// lowering — they must stay on the dynamic frame. eliminateFrameIndex's
  /// late spill-slot marking consults this so that a slot with MIXED
  /// accessors (one _Sym-capable, one not) is not moved to the static frame
  /// on its first _Sym-capable access and then hit by the non-lowerable
  /// accessor. Only meaningful when usesStaticStack(MF).
  SmallSet<int, 8> StaticNotLowerable;

  int VarArgsStackIndex = -1;
  DenseMap<Register, size_t> CSRDPOffsets;

  /// Frame indices for the stack-backed i32 spill pseudo-registers
  /// (SPILL_Q0..31 — the only spill-register family left; the byte/word/
  /// index families spill through stock frame-index slots now). Maps
  /// SPILL_Q register → its 4-byte frame index.
  DenseMap<MCPhysReg, int> SpillRegFrameIndices;

  /// Set when spill pseudo-registers are used. Forces frame pointer (U)
  /// setup so spill accesses use U-relative addressing (stable when S moves).
  bool UsesSpillRegisters = false;

  /// For functions whose return type is too large to fit in a register
  /// (i32, structs, …), gcc6809 returns the value via an implicit sret
  /// pointer passed in IX. lowerFormalArguments stashes the vreg holding
  /// the incoming pointer here so that lowerReturn can store the result
  /// through it. Zero (no register) when the function returns a value
  /// that fits in a register or returns void.
  Register SRetReturnReg;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
