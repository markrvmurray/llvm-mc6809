//===-- MC6809FinalLowering.h - MC6809 Final Lowering Pass ------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 final-lowering pass (bug #149). The pass
// runs in addPreEmitPass between BranchRelaxation and NoShortBranches and
// hosts six independently-gated late-stage transforms:
//
//   1. Indexed-offset relaxation (_o16 -> _o8 -> _o5 -> _o0)
//   2. Leaf-frame elision        (drop pshs u/tfr s,u where unused)
//   3. Adjacent inc/add folding  (fold inc;inc into add #N)
//   4. Dup-store elimination     (drop store before another store, same slot)
//   5. Store+reload elimination  (drop reload of slot still live in a reg)
//   6. Branch-over-branch fold   (invert cond + drop uncond when one
//                                 target is the layout successor; fixes
//                                 bug #179, especially at -O0 where
//                                 BranchFolderPass doesn't run)
//
// Each class has its own minimum -O level (default >= -O2 for size/speed
// optimisations; Class 6 defaults to -O0 since it's a pure code-shape
// cleanup that never enlarges code) and its own STATISTIC counter so we
// can see which classes fire on which workloads.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H

#include "llvm/CodeGen/MachineFunctionPass.h"

namespace llvm {

MachineFunctionPass *createMC6809FinalLoweringPass();

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809FINALLOWERING_H
