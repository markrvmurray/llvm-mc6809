//===-- MC6809FixupKinds.cpp - MC6809 fixup kinds  ------------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MC6809AsmBackend class.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/MC6809FixupKinds.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCFixupKindInfo.h"

namespace llvm {
const MCFixupKindInfo &
MC6809FixupKinds::getFixupKindInfo(const MC6809::Fixups Kind,
                                const MCAsmBackend *Alternative) {
  const static MCFixupKindInfo Infos[MC6809::NumTargetFixupKinds] = {
      // This table *must* be in same the order of fixup_* kinds in
      // MC6809FixupKinds.h.
      //
      // name, offset, bits, flags
      {"Imm8", 0, 8, 0},            // An 8 bit immediate value.
      {"Addr8", 0, 8, 0},           // An 8 bit direct page address.
      {"Addr16", 0, 16, 0},         // A 16-bit address.
      // PCRel8 is pc-relative and requires target specific handling
      {"PCRel8", 0, 8,
       MCFixupKindInfo::FKF_IsPCRel | MCFixupKindInfo::FKF_IsTarget}};
  if (Kind < static_cast<MC6809::Fixups>(FirstTargetFixupKind)) {
    assert(Alternative &&
           "Alternative MC6809 backend expected, but none was given!");
    return Alternative->getFixupKindInfo(static_cast<MCFixupKind>(Kind));
  }
  assert(unsigned(Kind - FirstTargetFixupKind) <
             MC6809::Fixups::NumTargetFixupKinds &&
         "Invalid kind!");
  return Infos[Kind - FirstTargetFixupKind];
}

} // namespace llvm
