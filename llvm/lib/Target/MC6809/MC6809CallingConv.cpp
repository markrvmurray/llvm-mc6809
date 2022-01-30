//===-- MC6809CallingConv.cpp - MC6809 Calling Convention ------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 calling convention.
//
//===----------------------------------------------------------------------===//

#include "MC6809CallingConv.h"
#include "MC6809Subtarget.h"

#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/IR/DataLayout.h"

#include "MC6809GenCallingConv.inc"
