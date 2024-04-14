//===-- MC6809NonReentrant.h - MC6809 NonReentrant Pass ---------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 NonReentrant pass.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809NORECURSE_H
#define LLVM_LIB_TARGET_MC6809_MC6809NORECURSE_H

#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/Pass.h"

namespace llvm {

ModulePass *createMC6809NonReentrantPass();

struct MC6809NonReentrantPass : PassInfoMixin<MC6809NonReentrantPass> {
  PreservedAnalyses run(Module &M, ModuleAnalysisManager &AM);
};

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809NORECURSE_H
