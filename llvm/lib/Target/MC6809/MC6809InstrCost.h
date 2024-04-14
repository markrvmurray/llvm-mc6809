//===-- MC6809InstrCost.h - MC6809 Instruction Cost structure ---*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the definition of the MC6809InstrCost class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INSTRCOST_H
#define LLVM_LIB_TARGET_MC6809_MC6809INSTRCOST_H

#include "llvm/CodeGen/MachineFunction.h"
#include <cstdint>

namespace llvm {

class MC6809InstrCost {
public:
  enum class Mode {
    PreferBytes,
    PreferCycles,
    Average
  };

  MC6809InstrCost() : Bytes(0), Cycles(0) {}

  MC6809InstrCost(int32_t Bytes, int32_t Cycles)
    : MC6809InstrCost(Bytes, Cycles, 256) {}

  friend MC6809InstrCost operator+(MC6809InstrCost Left,
                                const MC6809InstrCost& Right) {
    return MC6809InstrCost(Left.Bytes + Right.Bytes,
                        Left.Cycles + Right.Cycles, 1);
  }

  MC6809InstrCost& operator+=(const MC6809InstrCost& Right) {
    this->Bytes += Right.Bytes;
    this->Cycles += Right.Cycles;
    return *this;
  }

  friend MC6809InstrCost operator-(MC6809InstrCost Left,
                                const MC6809InstrCost& Right) {
    return MC6809InstrCost(Left.Bytes - Right.Bytes,
                        Left.Cycles - Right.Cycles, 1);
  }

  MC6809InstrCost& operator-=(const MC6809InstrCost& Right) {
    this->Bytes -= Right.Bytes;
    this->Cycles -= Right.Cycles;
    return *this;
  }

  friend MC6809InstrCost operator*(MC6809InstrCost Left, int Right) {
    return MC6809InstrCost(Left.Bytes * Right, Left.Cycles * Right, 1);
  }

  friend MC6809InstrCost operator/(MC6809InstrCost Left, int Right) {
    return MC6809InstrCost(Left.Bytes / Right, Left.Cycles / Right, 1);
  }

  int64_t value(Mode Mode = Mode::Average) const;

  static Mode getModeFor(const MachineFunction &MF);

private:
  MC6809InstrCost(int32_t Bytes, int32_t Cycles, int Multiplier)
    : Bytes(Bytes * Multiplier), Cycles(Cycles * Multiplier) {}

  int32_t Bytes, Cycles;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809CYCLECOST_H
