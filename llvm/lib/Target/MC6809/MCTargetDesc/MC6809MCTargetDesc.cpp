//===-- MC6809MCTargetDesc.cpp - MC6809 Target Descriptions
//---------------------===//
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

#include "MC6809MCTargetDesc.h"
#include "MC6809InstPrinter.h"
#include "MC6809MCAsmInfo.h"
#include "MC6809MCELFStreamer.h"
#include "MC6809TargetStreamer.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/MC/MCAsmBackend.h"
#include "llvm/MC/MCCodeEmitter.h"
#include "llvm/MC/MCELFStreamer.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/MC/MCSubtargetInfo.h"
#include "llvm/MC/TargetRegistry.h"

#define GET_INSTRINFO_MC_DESC
#include "MC6809GenInstrInfo.inc"

#define GET_SUBTARGETINFO_MC_DESC
#include "MC6809GenSubtargetInfo.inc"

#define GET_REGINFO_MC_DESC
#include "MC6809GenRegisterInfo.inc"

using namespace llvm;

namespace llvm {
namespace MC6809_MC {
/// Makes an e_flags value based on subtarget features.
unsigned makeEFlags(const FeatureBitset &Features) {
  unsigned ELFArch = 0;
  ELFArch |= ELF::EF_MC6809_ARCH_6809;
  if (Features[MC6809::Feature6309])
    ELFArch |= ELF::EF_MC6809_ARCH_6309;
  return ELFArch;
}
} // namespace MC6809_MC
} // end namespace llvm

MCInstrInfo *llvm::createMC6809MCInstrInfo() {
  MCInstrInfo *X = new MCInstrInfo();
  InitMC6809MCInstrInfo(X);

  return X;
}

static MCRegisterInfo *createMC6809MCRegisterInfo(const Triple &TT) {
  MCRegisterInfo *X = new MCRegisterInfo();
  InitMC6809MCRegisterInfo(X, 0);

  return X;
}

static MCSubtargetInfo *
createMC6809MCSubtargetInfo(const Triple &TT, StringRef CPU, StringRef FS) {
  // If we've received no advice on which CPU to use, let's use our own default.
  if (CPU.empty()) {
    CPU = "mc6809";
  }
  return createMC6809MCSubtargetInfoImpl(TT, CPU, /*TuneCPU*/ CPU, FS);
}

static MCInstPrinter *createMC6809MCInstPrinter(const Triple &T,
                                                unsigned /* SyntaxVariant */,
                                                const MCAsmInfo &MAI,
                                                const MCInstrInfo &MII,
                                                const MCRegisterInfo &MRI) {
  return new MC6809InstPrinter(MAI, MII, MRI);
}

static MCTargetStreamer *
createMC6809ObjectTargetStreamer(MCStreamer &S, const MCSubtargetInfo &STI) {
  return new MC6809TargetELFStreamer(S, STI);
}

static MCTargetStreamer *createMCAsmTargetStreamer(MCStreamer &S,
                                                   formatted_raw_ostream &OS,
                                                   MCInstPrinter *InstPrint,
                                                   bool isVerboseAsm) {
  return new MC6809TargetAsmStreamer(S);
}

extern "C" void LLVM_EXTERNAL_VISIBILITY LLVMInitializeMC6809TargetMC() {
  // Register the MC asm info.
  RegisterMCAsmInfo<MC6809MCAsmInfo> X(getTheMC6809Target());

  // Register the MC instruction info.
  TargetRegistry::RegisterMCInstrInfo(getTheMC6809Target(),
                                      createMC6809MCInstrInfo);

  // Register the MC register info.
  TargetRegistry::RegisterMCRegInfo(getTheMC6809Target(),
                                    createMC6809MCRegisterInfo);

  // Register the MC subtarget info.
  TargetRegistry::RegisterMCSubtargetInfo(getTheMC6809Target(),
                                          createMC6809MCSubtargetInfo);

  // Register the MCInstPrinter.
  TargetRegistry::RegisterMCInstPrinter(getTheMC6809Target(),
                                        createMC6809MCInstPrinter);

  // Register the MC Code Emitter
  TargetRegistry::RegisterMCCodeEmitter(getTheMC6809Target(),
                                        createMC6809MCCodeEmitter);

  // Register the obj streamer
  TargetRegistry::RegisterELFStreamer(getTheMC6809Target(),
                                      createMC6809MCELFStreamer);

  // Register the obj target streamer.
  TargetRegistry::RegisterObjectTargetStreamer(
      getTheMC6809Target(), createMC6809ObjectTargetStreamer);

  // Register the asm target streamer.
  TargetRegistry::RegisterAsmTargetStreamer(getTheMC6809Target(),
                                            createMCAsmTargetStreamer);

  // Register the asm backend (as little endian).
  TargetRegistry::RegisterMCAsmBackend(getTheMC6809Target(),
                                       createMC6809AsmBackend);
}
