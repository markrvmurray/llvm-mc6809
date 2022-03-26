//===-- MC6809MCAsmInfo.cpp - MC6809 asm properties
//-----------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MC6809MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MC6809MCAsmInfo.h"

#include "llvm/ADT/Triple.h"

namespace llvm {

MC6809MCAsmInfo::MC6809MCAsmInfo(const Triple &TT,
                                 const MCTargetOptions &Options) {
  CodePointerSize = 2;
  CalleeSaveStackSlotSize = 0;
  SeparatorString = "\n";
  CommentString = ";";
  DollarIsHexPrefix = true;
  MaxInstLength = 3;
  SupportsDebugInformation = true;
}

} //  namespace llvm
