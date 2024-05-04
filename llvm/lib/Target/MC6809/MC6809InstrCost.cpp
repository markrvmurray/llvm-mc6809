//===-- MC6809InstrCost.cpp - MC6809 Instruction Cost structure -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains additional helpers for the MC6809InstrCost class.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstrCost.h"

#include "llvm/IR/Function.h"

using namespace llvm;

namespace llvm {

int64_t MC6809InstrCost::value(Mode Mode) const {
  switch (Mode) {
  case Mode::PreferBytes:
    return ((int64_t)Bytes << 32) + Cycles;
  case Mode::PreferCycles:
    return ((int64_t)Cycles << 32) + Bytes;
  case Mode::Average:
    return Bytes + Cycles;
  }
}

MC6809InstrCost::Mode MC6809InstrCost::getModeFor(const MachineFunction &MF) {
  if (MF.getFunction().hasMinSize())
    return Mode::PreferBytes;
  if (MF.getFunction().hasOptSize() || MF.getFunction().hasOptNone())
    return Mode::Average;
  return Mode::PreferCycles;
}

} // namespace llvm
