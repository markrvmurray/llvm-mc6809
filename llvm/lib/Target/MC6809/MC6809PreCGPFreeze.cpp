//===-- MC6809PreCGPFreeze.cpp - Pre-freeze shared icmp conditions --------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass canonicalizes icmp values that have more than one `select` user
// by inserting a `freeze` right after the icmp and redirecting all such
// select conditions to the frozen value.
//
// Why: CodeGenPrepare's select-to-branch expansion
// (CodeGenPrepare::optimizeSelectInst) creates a *fresh* `freeze` for the
// condition of each select it expands (CodeGenPrepare.cpp:
// `IB.CreateFreeze(SI->getCondition(), ...)`). It tries to merge consecutive
// selects sharing a condition into one diamond, but the merge loop stops at
// any non-select instruction. When an unrelated instruction (e.g. a GEP, a
// libcall, an intrinsic such as `llvm.umax`) sits between two selects that
// conceptually share the same boolean, CodeGenPrepare ends up producing two
// independent diamonds, each guarded by its own `freeze(cond)`.
//
// `freeze(poison)` is defined to return an arbitrary-but-fixed value, chosen
// non-deterministically. Two distinct `freeze` instructions of the same
// poison input may therefore disagree, and when they do the diamonds fork on
// a conceptually-same boolean — producing a silent miscompile. The original
// failure mode for this was BSD qsort in picolibc (bug #136), where the
// partition loop contains three or four selects on `r > d` separated by
// pointer-difference arithmetic; under -Os/-Oz CodeGenPrepare fractured the
// selects into independent diamonds and qsort mis-partitioned.
//
// Pre-freezing the icmp means the select condition is already non-poison by
// the time CodeGenPrepare runs. CodeGenPrepare still emits a `freeze` when
// expanding, but `freeze(non-poison) == non-poison`, so every downstream
// diamond observes the same boolean value.
//
// The pass is intentionally narrow:
//   * It only touches icmp instructions that have more than one `select` user
//     whose *condition* is the icmp. icmps used as operand values (rather
//     than conditions), or used by a single select, are left alone.
//   * Non-select users are not redirected. The existing behaviour for branch
//     conditions, stores, etc. is preserved.
//   * An icmp whose result is already a freeze's operand is skipped — CSE /
//     InstCombine will fold any redundancy.
//
//===----------------------------------------------------------------------===//

#include "MC6809PreCGPFreeze.h"

#include "MC6809.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Instructions.h"
#include "llvm/InitializePasses.h"
#include "llvm/Pass.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-pre-cgp-freeze"

using namespace llvm;

namespace {

struct MC6809PreCGPFreeze : public FunctionPass {
  static char ID;

  MC6809PreCGPFreeze() : FunctionPass(ID) {
    initializeMC6809PreCGPFreezePass(*PassRegistry::getPassRegistry());
  }

  bool runOnFunction(Function &F) override;

  StringRef getPassName() const override {
    return "MC6809 Pre-CodeGenPrepare Freeze Canonicalization";
  }
};

} // namespace

static unsigned countSelectConditionUsers(const ICmpInst *Cmp) {
  unsigned N = 0;
  for (const User *U : Cmp->users()) {
    const auto *SI = dyn_cast<SelectInst>(U);
    if (SI && SI->getCondition() == Cmp) {
      if (++N > 1)
        return N;
    }
  }
  return N;
}

bool MC6809PreCGPFreeze::runOnFunction(Function &F) {
  if (F.isDeclaration())
    return false;

  SmallVector<ICmpInst *, 8> Candidates;
  for (BasicBlock &BB : F)
    for (Instruction &I : BB)
      if (auto *Cmp = dyn_cast<ICmpInst>(&I))
        if (countSelectConditionUsers(Cmp) > 1)
          Candidates.push_back(Cmp);

  if (Candidates.empty())
    return false;

  for (ICmpInst *Cmp : Candidates) {
    IRBuilder<> B(Cmp->getParent(), std::next(Cmp->getIterator()));
    Value *Frozen = B.CreateFreeze(Cmp, Cmp->getName() + ".precgpfrz");

    SmallVector<SelectInst *, 4> Selects;
    for (User *U : Cmp->users())
      if (auto *SI = dyn_cast<SelectInst>(U);
          SI && SI->getCondition() == Cmp)
        Selects.push_back(SI);

    for (SelectInst *SI : Selects)
      SI->setCondition(Frozen);

    LLVM_DEBUG(dbgs() << "MC6809PreCGPFreeze: pre-froze " << *Cmp
                      << " for " << Selects.size() << " select users in @"
                      << F.getName() << "\n");
  }

  return true;
}

char MC6809PreCGPFreeze::ID = 0;

INITIALIZE_PASS(MC6809PreCGPFreeze, DEBUG_TYPE,
                "MC6809: pre-freeze icmps with multiple select users "
                "before CodeGenPrepare",
                false, false)

FunctionPass *llvm::createMC6809PreCGPFreezePass() {
  return new MC6809PreCGPFreeze();
}
