//===-- MC6809TargetMachine.h - Define TargetMachine for MC6809 -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_MC6809_TARGET_MACHINE_H
#define LLVM_MC6809_TARGET_MACHINE_H

#include "llvm/CodeGen/CodeGenTargetMachineImpl.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/Target/TargetMachine.h"

#include "MC6809FrameLowering.h"
#include "MC6809ISelLowering.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"

namespace llvm {

/// A generic MC6809 implementation.
class MC6809TargetMachine : public CodeGenTargetMachineImpl {
public:
  MC6809TargetMachine(const Target &T, const Triple &TT, StringRef CPU, StringRef FS, const TargetOptions &Options, std::optional<Reloc::Model> RM, std::optional<CodeModel::Model> CM, CodeGenOptLevel OL, bool JIT);

  const MC6809Subtarget *getSubtargetImpl() const { return &SubTarget; }
  const MC6809Subtarget *getSubtargetImpl(const Function &F) const override;

  TargetLoweringObjectFile *getObjFileLowering() const override { return this->TLOF.get(); }

  TargetTransformInfo getTargetTransformInfo(const Function &F) const override;

  bool hasNoInitSection() const override { return true; }

  void registerPassBuilderCallbacks(PassBuilder &) override;

  TargetPassConfig *createPassConfig(PassManagerBase &PM) override;

  // Bug #165 Phase D: let LLVM's standard machinery add the post-RA
  // scheduler in TargetPassConfig::addMachinePasses (right after
  // addPreSched2 returns, so MC6809PostRASpillOpt and
  // MC6809LateOptimization have already run — see the
  // pipeline-ordering comment in MC6809TargetMachine.cpp's
  // addPreSched2 body, added by #165 phase B1). Was previously
  // `true` since `6a573828ff47` (2022-01-30), inherited verbatim
  // from the llvm-mos template ("disable PostRA scheduling by
  // claiming to emit it ourselves, then never doing so") — a
  // historical no-op that we're now flipping to enable scheduling.
  bool targetSchedulesPostRAScheduling() const override { return false; };

  StringRef getSectionPrefix(const GlobalObject *GO) const override;

  MachineFunctionInfo *createMachineFunctionInfo(BumpPtrAllocator &Allocator, const Function &F, const TargetSubtargetInfo *STI) const override;

  ScheduleDAGInstrs *createMachineScheduler(MachineSchedContext *C) const override;

private:
  std::unique_ptr<TargetLoweringObjectFile> TLOF;
  MC6809Subtarget SubTarget;
  mutable StringMap<std::unique_ptr<MC6809Subtarget>> SubtargetMap;
};

} // end namespace llvm

#endif // LLVM_MC6809_TARGET_MACHINE_H
