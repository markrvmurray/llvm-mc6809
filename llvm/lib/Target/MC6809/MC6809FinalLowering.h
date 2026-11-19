//===-- MC6809FinalLowering.h - MC6809 Final Lowering Pass ------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 final-lowering pass. It runs in
// addPreEmitPass between BranchRelaxation and NoShortBranches and hosts
// independently-gated late-stage transforms (numbered as first planned;
// 2, 3, 4 and 7 are gone -- 2 was never needed, 3 and 7 never fired on the
// C library or its tests at any of the 30 bench levels, and 4's dead
// duplicate stores (about 20 across them) are left to a dead-store rule on
// the shared MC6809ValueTracker):
//
//   1. Indexed-offset relaxation (_o16 -> _o8 -> _o5 -> _o0)
//   5. Store+reload elimination  (drop reload of slot still live in a reg)
//   6. Branch-over-branch fold   (invert cond + drop uncond when one
//                                 target is the layout successor, especially
//                                 at -O0 where BranchFolderPass doesn't run)
//
// Each class has its own minimum -O level (default -O2 for 1 and 5; 0
// for 6, a pure code-shape cleanup) and its own STATISTIC counter.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809FinalLoweringPass();

} // namespace llvm

#endif // LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H
