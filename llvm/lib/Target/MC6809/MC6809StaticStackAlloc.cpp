//===-- MC6809StaticStackAlloc.cpp - MC6809 Static Stack Allocation -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 static stack allocation pass (Bug #387).
//
// MC6809FrameLowering lowers accesses to the statically-allocable part of a
// non-reentrant function's frame as a TI_STATIC_STACK target-index operand
// carrying a per-function byte offset. This whole-program pass allocates a
// single `static_stack` global, walks the call graph to give each function a
// non-overlapping region within it (a caller is placed above its callees, so
// simultaneously-live frames never alias), creates a per-function alias into
// the global, and rewrites the target-index operands to reference it.
//
// Ported from the MOS static stack allocator; the SCC layout is identical
// because it is pure call-graph logic. Only the per-target frame-access
// lowering (which produces the target indices) differs.
//
//===----------------------------------------------------------------------===//

#include "MC6809StaticStackAlloc.h"

#include "MC6809.h"
#include "MC6809CallGraphUtils.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809Subtarget.h"

#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/IR/GlobalAlias.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/Module.h"
#include "llvm/PassRegistry.h"

#define DEBUG_TYPE "mc6809-static-stack-alloc"

using namespace llvm;

namespace {

class MC6809StaticStackAlloc : public ModulePass {
public:
  static char ID;

  MC6809StaticStackAlloc() : ModulePass(ID) {
    llvm::initializeMC6809StaticStackAllocPass(*PassRegistry::getPassRegistry());
  }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

void MC6809StaticStackAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
  AU.addRequired<CallGraphWrapperPass>();
}

bool MC6809StaticStackAlloc::runOnModule(Module &M) {
  auto &MMI = getAnalysis<MachineModuleInfoWrapperPass>().getMMI();
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  mc6809::addLibcallEdges(CG, MMI);
  mc6809::addExternalEdges(CG);

  // Extract the strongly-connected components of the call graph and remember
  // which SCC contains each node.
  DenseMap<CallGraphNode *, uint64_t> SCCID;
  struct SCC {
    SmallVector<CallGraphNode *, 1> Nodes;
    uint64_t Offset = 0;
  };
  std::vector<SCC> SCCs;
  for (auto I = scc_begin(&CG), E = scc_end(&CG); I != E; ++I) {
    SCCs.emplace_back();
    for (CallGraphNode *CGN : *I) {
      SCCID[CGN] = SCCs.size() - 1;
      SCCs.back().Nodes.push_back(CGN);
    }
  }

  // For each SCC, the set of SCCs that call into it. A caller must be placed
  // higher on the static stack than the callee, or their regions would
  // conflict while both are live.
  std::map<SCC *, SmallPtrSet<SCC *, 4>> CallerSCCs;
  for (const auto &KV : enumerate(SCCs)) {
    auto &CallerSCC = KV.value();
    for (CallGraphNode *CGN : CallerSCC.Nodes) {
      for (const auto &Edge : *CGN) {
        SCC &CalleeSCC = SCCs[SCCID[Edge.second]];
        if (&CalleeSCC != &CallerSCC)
          CallerSCCs[&CalleeSCC].insert(&CallerSCC);
      }
    }
  }

  // SCCs with no callers. The SCC graph is acyclic, so there is always one.
  std::vector<SCC *> RootSCCs;
  for (SCC &S : SCCs)
    if (CallerSCCs.find(&S) == CallerSCCs.end())
      RootSCCs.push_back(&S);

  uint64_t StackSize = 0;
  std::vector<SCC *> InterruptNorecurseSCCs;
  bool ToInterruptNorecurse = false;
  while (!RootSCCs.empty() || !InterruptNorecurseSCCs.empty()) {
    // If only interrupt-norecurse roots remain, schedule one. Its offset
    // starts at the end of everything seen so far, since it (and its
    // descendants) may interrupt any of them.
    if (RootSCCs.empty()) {
      ToInterruptNorecurse = true;
      RootSCCs.push_back(InterruptNorecurseSCCs.back());
      RootSCCs.back()->Offset = StackSize;
      InterruptNorecurseSCCs.pop_back();
      continue;
    }

    SCC &RootSCC = *RootSCCs.back();
    RootSCCs.pop_back();

    // Defer interrupt-norecurse SCCs (and their descendants) until last; they
    // conflict with everything.
    if (!ToInterruptNorecurse && RootSCC.Nodes.size() == 1 &&
        RootSCC.Nodes.front()->getFunction() &&
        RootSCC.Nodes.front()->getFunction()->hasFnAttribute(
            "interrupt-norecurse")) {
      InterruptNorecurseSCCs.push_back(&RootSCC);
      continue;
    }

    // Sum the static-stack size needed by this SCC's functions. Callee SCCs go
    // below the end of this region — true even when the SCC is internally
    // recursive (the SCCs above and below it may not be).
    const auto GetStaticStackSize = [&]() -> uint64_t {
      uint64_t Size = 0;
      for (CallGraphNode *Node : RootSCC.Nodes) {
        Function *F = Node->getFunction();
        if (!F)
          continue;
        MachineFunction *MF = MMI.getMachineFunction(*F);
        if (!MF)
          continue;
        const MC6809FrameLowering &TFL =
            *MF->getSubtarget<MC6809Subtarget>().getFrameLowering();
        Size += TFL.staticSize(MF->getFrameInfo());
      }
      return Size;
    };
    uint64_t Size = GetStaticStackSize();

    uint64_t Offset = RootSCC.Offset + Size;
    StackSize = std::max(StackSize, Offset);

    // Propagate the offset to callee SCCs; once a callee has no remaining
    // unscheduled callers it becomes a root.
    for (CallGraphNode *CGN : RootSCC.Nodes) {
      for (const auto &Edge : *CGN) {
        SCC &CalleeSCC = SCCs[SCCID[Edge.second]];
        if (&CalleeSCC != &RootSCC &&
            CallerSCCs[&CalleeSCC].contains(&RootSCC)) {
          CalleeSCC.Offset = std::max(CalleeSCC.Offset, Offset);
          CallerSCCs[&CalleeSCC].erase(&RootSCC);
          if (CallerSCCs[&CalleeSCC].empty())
            RootSCCs.push_back(&CalleeSCC);
        }
      }
    }
  }

  if (!StackSize)
    return false;

  // One global for the whole static stack. Use a zero initializer (not undef)
  // so it lands in .bss (low RAM, zeroed at startup) rather than .noinit, which
  // on MC6809 the linker script places at the top of RAM where it would overlap
  // the hardware stack — a static spill write would then clobber return
  // addresses (and vice versa). The spills always write before they read, so
  // the zero init is purely a placement lever, not a correctness requirement.
  Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), StackSize);
  GlobalVariable *Stack =
      new GlobalVariable(M, Typ, false, GlobalValue::PrivateLinkage,
                         ConstantAggregateZero::get(Typ), "static_stack");
  LLVM_DEBUG(dbgs() << *Stack << "\n");

  // A per-function alias into the global, and rewrite each function's
  // target-index operands to reference it.
  for (const SCC &S : SCCs) {
    size_t Offset = S.Offset;
    for (CallGraphNode *Node : S.Nodes) {
      Function *F = Node->getFunction();
      if (!F)
        continue;
      MachineFunction *MF = MMI.getMachineFunction(*F);
      if (!MF)
        continue;
      const MC6809FrameLowering &TFL =
          *MF->getSubtarget<MC6809Subtarget>().getFrameLowering();
      uint64_t Size = TFL.staticSize(MF->getFrameInfo());
      if (!Size)
        continue;

      Type *RegionTyp = ArrayType::get(Type::getInt8Ty(M.getContext()), Size);
      Constant *Aliasee = Stack;
      if (Offset) {
        Type *I16 = Type::getInt16Ty(Stack->getContext());
        Aliasee = ConstantExpr::getGetElementPtr(
            Stack->getValueType(), Stack,
            SmallVector<Constant *>{ConstantInt::get(I16, 0),
                                    ConstantInt::get(I16, Offset)},
            /*InBounds=*/true);
      }
      Offset += Size;
      auto *Alias = GlobalAlias::create(
          RegionTyp, Stack->getAddressSpace(), Stack->getLinkage(),
          Twine(F->getName()) + "_sstk", Aliasee, Stack->getParent());
      LLVM_DEBUG(dbgs() << *Alias << "\n");

      MC6809FunctionInfo &MFI = *MF->getInfo<MC6809FunctionInfo>();
      MFI.StaticStackValue = Alias;

      for (MachineBasicBlock &MBB : *MF)
        for (MachineInstr &MI : MBB)
          for (MachineOperand &MO : MI.operands())
            if (MO.isTargetIndex())
              MO.ChangeToGA(Alias, MO.getOffset(), MO.getTargetFlags());
    }
  }
  return true;
}

} // namespace

char MC6809StaticStackAlloc::ID = 0;

INITIALIZE_PASS(MC6809StaticStackAlloc, DEBUG_TYPE,
                "Allocate non-reentrant stack frames to static memory", false,
                false)

ModulePass *llvm::createMC6809StaticStackAllocPass() {
  return new MC6809StaticStackAlloc();
}
