//===-- MC6809Subtarget.cpp - MC6809 Subtarget Information ----------------===//
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
#include "llvm/CodeGen/LibcallLoweringInfo.h"
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

MC6809Subtarget::MC6809Subtarget(const Triple &TT, const std::string &CPU, const std::string &FS, const MC6809TargetMachine &TM)
    : MC6809GenSubtargetInfo(TT, CPU, /* TuneCPU */ CPU, FS), InstrInfo(*this), RegInfo(), FrameLowering(), TLInfo(TM, initializeSubtargetDependencies(CPU, FS, TM)), CallLoweringInfo(&TLInfo), Legalizer(*this),
      InstSelector(createMC6809InstructionSelector(TM, *this, RegBankInfo)), InlineAsmLoweringInfo(&TLInfo) {}

MC6809Subtarget &MC6809Subtarget::initializeSubtargetDependencies(StringRef CPU, StringRef FS, const TargetMachine &TM) {
  // Parse features string, with the features the triple implies (+os9).
  ParseSubtargetFeatures(CPU, /* TuneCPU */ CPU,
                         mc6809::tripleFeatures(TM.getTargetTriple(), FS));

  return *this;
}

void MC6809Subtarget::overrideSchedPolicy(MachineSchedPolicy &Policy, const SchedRegion &Region) const {
  // Inherited comment (carried over from upstream MOS): "Force register
  // pressure tracking; by default it's disabled for small regions, but
  // it's the only 6502 scheduling concern."
  //
  // For MC6809/HD6309 we DISABLE pressure tracking instead. The
  // tracker's getUpwardPressureDelta asserts (PSet overflow) on the
  // dense overlapping super-reg implicit defs that show up in HD6309
  // libc TUs after round-18 follow-up #3 added TFM-based memcpy
  // lowering — see test-strcat_s, test-wctomb, test-uchar,
  // libc-testsuite:string. The byte-level decomposition of i32 ops
  // through ACC8/ACC16/AQ sub-register lattices, combined with LDD/LDW
  // listing AA, AB, AD, AQ (or AE, AF, AW, AQ) all under defs,
  // produces pressure-set deltas the tracker can't reconcile. Turning
  // ShouldTrackPressure off avoids the assert; we keep the rest of the
  // pre-RA scheduler enabled (latency-driven) which is enough for
  // MC6809's straight-line code where register pressure is already
  // controlled by the imaginary-register/spill infrastructure.
  Policy.ShouldTrackPressure = false;

  Policy.OnlyBottomUp = false;
  Policy.OnlyTopDown = false;
}

void MC6809Subtarget::initLibcallLoweringInfo(LibcallLoweringInfo &Info) const {
  // RTLIB::SQRT_F32 defaults to "sqrtf" (libm name). We provide __sqrtsf2
  // and __sqrtdf2 wrappers around the MC6839 FP ROM — override here so the
  // name is correct regardless of whether the legacy-PM or new-PM
  // RuntimeLibraryAnalysis was initialised with the right target triple.
  Info.setLibcallImpl(RTLIB::SQRT_F32, RTLIB::impl___sqrtsf2);
  Info.setLibcallImpl(RTLIB::SQRT_F64, RTLIB::impl___sqrtdf2);
}
