//===-- MC6809InstrBuilder.h - Functions to aid building insts --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file exposes functions that may be used to handle MC6809 instruction
// building quirks, including subtarget-dependent selection, in a cleaner way.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INSTRBUILDER_H
#define LLVM_LIB_TARGET_MC6809_MC6809INSTRBUILDER_H

#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"

namespace llvm {

#if 0
static inline unsigned getIncPseudoOpcode(const MachineIRBuilder &Builder) {
  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  return MC6809::INCAa;
}

static inline unsigned getDecPseudoOpcode(const MachineIRBuilder &Builder) {
  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  return MC6809::DECAa;
}

static inline MachineInstrBuilder buildLdImm(MachineIRBuilder &Builder, DstOp Dest) {
  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  LLT DestType = Dest.getLLTTy(*Builder.getMRI());
  assert(DestType.isByteSized() && DestType.getScalarSizeInBits() <= 16);

  if (DestType.getScalarSizeInBits() == 32)
    return Builder.buildInstr(MC6809::Load_i32_Imm, {Dest, &MC6809::ACC32RegClass}, {});
  else if (DestType.getScalarSizeInBits() == 16)
    return Builder.buildInstr(MC6809::Load_i16_Imm, {Dest, &MC6809::ACC16RegClass}, {});
  else
    return Builder.buildInstr(MC6809::Load_i8_Imm, {Dest, &MC6809::ACC8RegClass}, {});
}
#endif

} // end namespace llvm

#endif
