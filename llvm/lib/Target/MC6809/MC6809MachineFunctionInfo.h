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

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

class MC6809Subtarget;

struct MC6809FunctionInfo : public MachineFunctionInfo {
  MC6809FunctionInfo(const Function &F, const MC6809Subtarget *STI) {}

  int VarArgsStackIndex = -1;
  const GlobalValue *StaticStackValue = nullptr;
  const GlobalValue *DirectPageStackValue = nullptr;
  DenseMap<Register, size_t> CSRDPOffsets;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
