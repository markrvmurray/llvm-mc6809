//===-- MC6809MachineFuctionInfo.h - MC6809 machine function info -----*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares MC6809-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

class MC6809MachineFunctionInfo : public MachineFunctionInfo {
  int VarArgsStackIndex = -1;
  const GlobalVariable *StaticStackVariable = nullptr;

public:
  MC6809MachineFunctionInfo(MachineFunction &MF) {}

  /// Returns the fake frame index indicating the start of the varargs region of
  /// the incoming call stack.
  int getVarArgsStackIndex() const { return VarArgsStackIndex; }

  /// Sets the fake frame index indicating the start of the varargs region of
  /// the incoming call stack.
  void setVarArgsStackIndex(int Index) { VarArgsStackIndex = Index; }

  /// Returns the static stack variable allocated for this function.
  const GlobalVariable *getStaticStackVariable() const {
    return StaticStackVariable;
  }

  /// Sets the static stack variable allocated for this function.
  void setStaticStackVariable(const GlobalVariable *Var) {
    StaticStackVariable = Var;
  }
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
