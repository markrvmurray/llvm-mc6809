//===-- MC6809NonReentrant.cpp - MC6809 NonReentrant Pass -----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 NonReentrant pass.
//
// This pass examines the full inter-procedural Module function call graph to
// identify functions that need not be reentrant. Those functions are marked
// with the nonreentrant annotation, which allows the code generator to lay
// their local stack frames out in globally static memory. This is possible
// because such functions can have at most one invocation active at any given
// time. Along the way, this pass performs a norecurse analysis as well.
//
//===----------------------------------------------------------------------===//

#include "MC6809NonReentrant.h"

#include "MC6809.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/CallGraphSCCPass.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/PassManager.h"
#include "llvm/LTO/LTO.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-nonreentrant"

using namespace llvm;

namespace {

struct MC6809NonReentrantImpl {
  CallGraph &CG;
  SmallPtrSet<const CallGraphNode *, 8> Reentrant;
  SmallPtrSet<const CallGraphNode *, 8> ReachableFromCurrentNorecurseInterrupt;
  SmallPtrSet<const CallGraphNode *, 8> ReachableFromOtherNorecurseInterrupt;
  bool HasInterrupts = false;

  MC6809NonReentrantImpl(CallGraph &CG) : CG(CG) { initializeMC6809NonReentrantPass(*PassRegistry::getPassRegistry()); }

  bool run(Module &M);

  bool runOnSCC(CallGraphSCC &SCC);
  void markReentrant(const CallGraphNode &CGN);
  void visitNorecurseInterrupt(const CallGraphNode &CGN);
};

} // namespace

static bool callsSelf(const CallGraphNode &N) {
  for (const CallGraphNode::CallRecord &CR : N)
    if (CR.second == &N)
      return true;
  return false;
}

bool MC6809NonReentrantImpl::run(Module &M) {
  LLVM_DEBUG(dbgs() << "**** MC6809 NonReentrant Pass ****\n");

  // For the conservative recursion analysis, any external call may call any
  // externally-callable function so add an edge from the calls-external node
  // to the called-by-external node.
  assert(CG.getCallsExternalNode()->empty());
  CG.getCallsExternalNode()->addCalledFunction(nullptr, CG.getExternalCallingNode());

  // Walk the callgraph in bottom-up SCC order.
  scc_iterator<CallGraph *> CGI = scc_begin(&CG);
  CallGraphSCC CurSCC(CG, &CGI);
  bool Changed = false;
  for (; !CGI.isAtEnd(); ++CGI) {
    CurSCC.initialize(*CGI);
    Changed |= runOnSCC(CurSCC);
  }

  // Mark all functions reachable from an interrupt function as non-reentrant.
  for (Function &F : M.functions()) {
    if (F.hasFnAttribute("interrupt")) {
      HasInterrupts = true;
      markReentrant(*CG[&F]);
    }
  }

  // Mark all functions reachable from multiple interrupt-norecurse functions as
  // possibly recursive.
  for (Function &F : M.functions()) {
    if (F.hasFnAttribute("interrupt-norecurse") || F.getName() == "main") {
      if (F.hasFnAttribute("interrupt-norecurse"))
        HasInterrupts = true;
      visitNorecurseInterrupt(*CG[&F]);
      for (const auto *CGN : ReachableFromCurrentNorecurseInterrupt)
        ReachableFromOtherNorecurseInterrupt.insert(CGN);
      ReachableFromCurrentNorecurseInterrupt.clear();
    }
  }

  if (HasInterrupts) {
    Changed = true;

    // Mark all libcalls as possibly recursive if we have interrupts, since
    // there's no way to tell which will actually be called by an interrupt
    // before the interrupt is compiled. But the compilation of the interrupt
    // depends on whether or not it's norecurse, so we don't have much choice
    // other than making the conservative assumption here.
    for (const char *LibcallName : lto::LTO::getRuntimeLibcallSymbols(Triple(M.getTargetTriple()))) {
      Function *Libcall = M.getFunction(LibcallName);
      if (Libcall && !Libcall->isDeclaration()) {
        LLVM_DEBUG(dbgs() << "Marking libcall as reentrant: " << Libcall->getName() << "\n");
        Reentrant.insert(CG[Libcall]);
      }
    }
  }

  // Make all norecurse functions that were not determined to be reentrant as
  // nonreentrant.
  for (Function &F : M.functions())
    if (F.doesNotRecurse() && !Reentrant.contains(CG[&F]))
      F.addFnAttr("nonreentrant");

  // Remove the artificial edge.
  CG.getCallsExternalNode()->removeAllCalledFunctions();
  return Changed;
}

bool MC6809NonReentrantImpl::runOnSCC(CallGraphSCC &SCC) {
  // All nodes in SCCs with more than one node may be recursive. It's not
  // certain since CFG analysis is conservative, but there's no more
  // information to be gleaned from looking at the call graph, and other
  // sources of information are better used making the CFG analysis less
  // conservative.
  if (!SCC.isSingular())
    return false;

  const CallGraphNode &N = **SCC.begin();

  if (!N.getFunction() || N.getFunction()->isDeclaration() || N.getFunction()->hasFnAttribute("nonreentrant") || N.getFunction()->doesNotRecurse())
    return false;

  // Since the CFG analysis is conservative, any possible indirect recursion
  // involving N would have placed in an SCC with more than one node. Thus, N
  // is recursive iff it directly calls itself.
  if (callsSelf(N))
    return false;

  LLVM_DEBUG(dbgs() << "Found new non-recursive function.\n");
  LLVM_DEBUG(N.print(dbgs()));

  // At this point, the function in N can safely be made non-reentrant.
  N.getFunction()->setDoesNotRecurse();
  return true;
}

void MC6809NonReentrantImpl::markReentrant(const CallGraphNode &CGN) {
  if (Reentrant.contains(&CGN))
    return;
  Reentrant.insert(&CGN);

  for (const auto &CallRecord : CGN)
    markReentrant(*CallRecord.second);
}

void MC6809NonReentrantImpl::visitNorecurseInterrupt(const CallGraphNode &CGN) {
  if (Reentrant.contains(&CGN))
    return;
  if (ReachableFromCurrentNorecurseInterrupt.contains(&CGN))
    return;
  ReachableFromCurrentNorecurseInterrupt.insert(&CGN);

  Function *F = CGN.getFunction();
  if (F && !F->isDeclaration() && ReachableFromOtherNorecurseInterrupt.contains(&CGN)) {
    LLVM_DEBUG(dbgs() << "Marking reachable from multiple norecurse interrupts: " << F->getName() << "\n");
    Reentrant.insert(&CGN);
  }
  for (const auto &CallRecord : CGN)
    visitNorecurseInterrupt(*CallRecord.second);
}

namespace {

struct MC6809NonReentrant : public ModulePass {
  static char ID; // Pass identification, replacement for typeid

  MC6809NonReentrant() : ModulePass(ID) { initializeMC6809NonReentrantPass(*PassRegistry::getPassRegistry()); }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &Info) const override;
};

} // namespace

bool MC6809NonReentrant::runOnModule(Module &M) {
  LLVM_DEBUG(dbgs() << "**** MC6809 NonReentrant Pass ****\n");

  CallGraph &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();
  return MC6809NonReentrantImpl(CG).run(M);
}

void MC6809NonReentrant::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<CallGraphWrapperPass>();
  AU.addPreserved<CallGraphWrapperPass>();
}

PreservedAnalyses MC6809NonReentrantPass::run(Module &M, ModuleAnalysisManager &AM) {
  LLVM_DEBUG(dbgs() << "**** MC6809 NonReentrant Pass ****\n");

  CallGraph &CG = AM.getResult<CallGraphAnalysis>(M);
  bool Changed = MC6809NonReentrantImpl(CG).run(M);
  return Changed ? PreservedAnalyses::none() : PreservedAnalyses::all();
}

char MC6809NonReentrant::ID = 0;

INITIALIZE_PASS(MC6809NonReentrant, DEBUG_TYPE, "Detect non-reentrant functions via detailed call graph analysis", false, false)

ModulePass *llvm::createMC6809NonReentrantPass() { return new MC6809NonReentrant(); }
