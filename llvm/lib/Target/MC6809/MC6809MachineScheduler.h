//===-- MC6809MachineScheduler.h - MC6809 Instruction Scheduler -------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 machine instruction scheduler.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809_MACHINESCHEDULER_H
#define LLVM_LIB_TARGET_MC6809_MC6809_MACHINESCHEDULER_H

#include "llvm/CodeGen/MachineScheduler.h"

namespace llvm {

class MC6809SchedStrategy : public GenericScheduler {
public:
  MC6809SchedStrategy(const MachineSchedContext *C);

  bool tryCandidate(SchedCandidate &Cand, SchedCandidate &TryCand,
                    SchedBoundary *Zone) const override;

  int registerClassPressureDiff(const TargetRegisterClass &RC, const SUnit *SU,
                                bool IsTop) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809_MACHINESCHEDULER_H
