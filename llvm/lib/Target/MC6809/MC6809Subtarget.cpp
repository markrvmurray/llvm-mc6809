//===-- MC6809Subtarget.cpp - MC6809 Subtarget Information
//----------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the MC6809 specific subclass of TargetSubtargetInfo.
//
//===----------------------------------------------------------------------===//

#include "MC6809Subtarget.h"

#include "llvm/BinaryFormat/ELF.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineScheduler.h"
#include "llvm/MC/TargetRegistry.h"

#include "GISel/MC6809InstructionSelector.h"
#include "GISel/MC6809LegalizerInfo.h"
#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809TargetMachine.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#define DEBUG_TYPE "mc6809-subtarget"

#define GET_SUBTARGETINFO_TARGET_DESC
#define GET_SUBTARGETINFO_CTOR
#include "MC6809GenSubtargetInfo.inc"

using namespace llvm;

MC6809Subtarget::MC6809Subtarget(const Triple &TT, const std::string &CPU,
                                 const std::string &FS,
                                 const MC6809TargetMachine &TM)
    : MC6809GenSubtargetInfo(TT, CPU, /* TuneCPU */ CPU, FS), InstrInfo(),
      RegInfo(), FrameLowering(*this),
      TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)),
      CallLoweringInfo(&TLInfo), Legalizer(*this),
      InstSelector(createMC6809InstructionSelector(TM, *this, RegBankInfo)),
      InlineAsmLoweringInfo(&TLInfo) {}

MC6809Subtarget &
MC6809Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS,
                                                 const TargetMachine &TM) {
  // Parse features string.
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU, FS);

  // Convert feature bits to e_flags
  EFlags = MC6809_MC::makeEFlags(getFeatureBits());

  return *this;
}

void MC6809Subtarget::overrideSchedPolicy(MachineSchedPolicy &Policy,
                                          unsigned NumRegionInstrs) const {
  // Force register pressure tracking; by default it's disabled for small
  // regions, but it's the only 6502 scheduling concern.
  Policy.ShouldTrackPressure = true;

  Policy.OnlyBottomUp = false;
  Policy.OnlyTopDown = false;
}
