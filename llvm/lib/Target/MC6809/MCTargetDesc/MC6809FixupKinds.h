//===-- MC6809FixupKinds.h - MC6809 Specific Fixup Entries ------------*- C++
//-*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_FIXUP_KINDS_H
#define LLVM_MC6809_FIXUP_KINDS_H

#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCFixup.h"
#include "llvm/MC/MCFixupKindInfo.h"

namespace llvm {
namespace MC6809 {

/// The set of supported fixups.
///
/// Although most of the current fixup types reflect a unique relocation
/// one can have multiple fixup types for a given relocation and thus need
/// to be uniquely named.
///
/// \note This table *must* be in the same order of
///       MCFixupKindInfo Infos[MC6809::NumTargetFixupKinds]
///       in `MC6809AsmBackend.cpp`.
enum Fixups {
  Imm8 = FirstTargetFixupKind, // An 8 bit direct page address.
  Addr8,                       // An 8 bit direct page address.
  Addr16,                      // A 16-bit address.
  Rel5,                        // A 5-bit index relative value.
  Rel8,                        // An 8-bit index relative value.
  Rel16,                       // A 16-bit index relative value.
  PCRel8,                      // An 8-bit PC relative value.
  PCRel16,                     // A 16-bit PC relative value.
  LastTargetFixupKind,
  NumTargetFixupKinds = LastTargetFixupKind - FirstTargetFixupKind
};

namespace fixups {} // end of namespace fixups
} // namespace MC6809

class MC6809FixupKinds {
public:
  const static MCFixupKindInfo &
  getFixupKindInfo(const MC6809::Fixups Kind, const MCAsmBackend *Alternative);
};
} // namespace llvm

#endif // LLVM_MC6809_FIXUP_KINDS_H
