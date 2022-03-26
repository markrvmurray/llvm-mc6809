//===-- MC6809MCTargetDesc.h - MC6809 Target Descriptions -------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file provides MC6809 specific target descriptions.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_MCTARGET_DESC_H
#define LLVM_MC6809_MCTARGET_DESC_H

#include "llvm/ADT/Sequence.h"
#include "llvm/Support/DataTypes.h"

#include <memory>

namespace llvm {

class FeatureBitset;
class MCAsmBackend;
class MCCodeEmitter;
class MCContext;
class MCInstrInfo;
class MCObjectTargetWriter;
class MCRegisterInfo;
class MCSubtargetInfo;
class MCTargetOptions;
class StringRef;
class Target;
class Triple;
class raw_pwrite_stream;

Target &getTheMC6809Target();

MCInstrInfo *createMC6809MCInstrInfo();

/// Creates a machine code emitter for MC6809.
MCCodeEmitter *createMC6809MCCodeEmitter(const MCInstrInfo &MCII,
                                         MCContext &Ctx);

/// Creates an assembly backend for MC6809.
MCAsmBackend *createMC6809AsmBackend(const Target &T,
                                     const MCSubtargetInfo &STI,
                                     const MCRegisterInfo &MRI,
                                     const llvm::MCTargetOptions &TO);

/// Creates an ELF object writer for MC6809.
std::unique_ptr<MCObjectTargetWriter>
createMC6809ELFObjectWriter(uint8_t OSABI);

namespace MC6809_MC {
/// Makes an e_flags value based on subtarget features.
unsigned makeEFlags(const FeatureBitset &Features);
} // namespace MC6809_MC

} // end namespace llvm

#define GET_REGINFO_ENUM
#include "MC6809GenRegisterInfo.inc"

#define GET_INSTRINFO_ENUM
#include "MC6809GenInstrInfo.inc"

#define GET_SUBTARGETINFO_ENUM
#include "MC6809GenSubtargetInfo.inc"

namespace llvm {
template <> struct enum_iteration_traits<decltype(MC6809::NoRegister)> {
  static constexpr bool is_iterable = true;
};
} // namespace llvm

#endif // LLVM_MC6809_MCTARGET_DESC_H
