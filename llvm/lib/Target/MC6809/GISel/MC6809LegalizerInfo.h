//===-- MC6809LegalizerInfo.h - MC6809 Legalizer ----------------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the interface that MC6809 uses to legalize generic MIR.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H

#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

class MC6809Subtarget;

class MC6809LegalizerInfo : public LegalizerInfo {
public:
  MC6809LegalizerInfo(const MC6809Subtarget &STI);

  bool legalizeIntrinsic(LegalizerHelper &Helper,
                         MachineInstr &MI) const override;

  bool legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI) const override;

};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H
