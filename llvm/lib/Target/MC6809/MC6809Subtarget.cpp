//===-- MC6809Subtarget.cpp - MC6809 Subtarget Information ----------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the MC6809 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MC6809Subtarget.h"
// #include "GISel/MC6809InlineAsmLowering.h"
#include "GISel/MC6809LegalizerInfo.h"
#include "GISel/MC6809RegisterBankInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"
#include "MC6809TargetMachine.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
using namespace llvm;

#define DEBUG_TYPE "mc6809-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MC6809GenSubtargetInfo.inc"

MC6809Subtarget &MC6809Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef TuneCPU, StringRef FS) {
  if (CPU.empty())
    CPU = TargetTriple.getArchName();
  ParseSubtargetFeatures(CPU, TuneCPU, FS);
  return *this;
}

MC6809Subtarget::MC6809Subtarget(const Triple &TT, StringRef CPU, StringRef TuneCPU, StringRef FS, const MC6809TargetMachine &TM)
    : MC6809GenSubtargetInfo(TT, CPU, TuneCPU, FS),
      TargetTriple(TT),
      InstrInfo(initializeSubtargetDependencies(CPU, TuneCPU, FS)),
      TLInfo(TM, *this),
      FrameLowering(*this) {
  // Convert feature bits to e_flags
  EFlags = MC6809_MC::makeEFlags(getFeatureBits());

  CallLoweringInfo.reset(new MC6809CallLowering(getTargetLowering()));
  // InlineAsmLoweringInfo.reset(new MC6809InlineAsmLowering(*getTargetLowering()));
  Legalizer.reset(new MC6809LegalizerInfo(*this, TM));

  auto *RBI = new MC6809RegisterBankInfo(*getRegisterInfo());
  RegBankInfo.reset(RBI);
  InstSelector.reset(createMC6809InstructionSelector(TM, *this, *RBI));
}
