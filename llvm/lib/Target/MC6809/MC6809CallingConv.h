//===-- MC6809CallingConv.h - MC6809 Calling Convention-----------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 calling convention.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809CALLINGCONV_H
#define LLVM_LIB_TARGET_MC6809_MC6809CALLINGCONV_H

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/Support/MachineValueType.h"

namespace llvm {
bool RetCC_MC6809(unsigned ValNo, MVT ValVT, MVT LocVT,
                  CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                  CCState &State);
bool CC_MC6809(unsigned ValNo, MVT ValVT, MVT LocVT,
               CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
               CCState &State);
bool CC_MC6809_VarArgs(unsigned ValNo, MVT ValVT, MVT LocVT,
                       CCValAssign::LocInfo LocInfo, ISD::ArgFlagsTy ArgFlags,
                       CCState &State);
} // namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MC6809CALLINGCONV_H
