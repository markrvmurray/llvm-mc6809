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


} // end namespace llvm

#endif
