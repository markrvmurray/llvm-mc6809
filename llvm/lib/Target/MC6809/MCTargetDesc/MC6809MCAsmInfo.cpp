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

namespace llvm {

MC6809MCAsmInfo::MC6809MCAsmInfo(const Triple &TT, const MCTargetOptions &Options)
    : MCAsmInfoELF(Options) {
  IsLittleEndian = false;
  CodePointerSize = 2;
  // MC6809 is byte-addressable; the stack slot unit is 1 byte.
  // This drives getDataAlignmentFactor() = -1 in the CIE, which lets
  // DW_CFA_offset for the return PC encode correctly as factored
  // offset = (byte_offset / -1).  The original 0 prevented the generic
  // callee-save slot allocator from meddling, but MC6809 overrides
  // assignCalleeSavedSpillSlots entirely so the slot size doesn't affect
  // frame layout — only the DWARF CIE data alignment factor.
  CalleeSaveStackSlotSize = 1;
  SeparatorString = "\n";
  CommentString = ";";
  UseMotorolaIntegers = true;
  // Real MC6809 / HD6309 ceiling is 5 bytes:
  //   - page-1 immediate-indexed, 16-bit offset, W-register postbyte
  //     (1 opcode + 1 postbyte + 2 offset + 1 immediate-byte) = 5
  //   - HD6309 LDQ #imm32: 1 opcode + 4 immediate = 5
  // No instruction in the .td reaches 6 bytes, so MC6809 and HD6309
  // share this ceiling and no subtarget override is needed.
  MaxInstLength = 5;
  SupportsDebugInformation = true;
  // Bug #164 Tier 2: enable .cfi_startproc / .cfi_endproc and
  // .debug_frame emission without EH support.  Without this, lldb's
  // DwarfCFIException decides shouldEmitCFI = false and no frame
  // unwind information is produced even when -g is active.
  UsesCFIWithoutEH = true;
}

} //  namespace llvm
