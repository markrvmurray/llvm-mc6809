//===-- MC6809Internalize.cpp - MC6809 Libcall Internalization ------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 library call internalization pass.
//
// Since library calls can benefit from static stacks and direct page, it's
// ideal to include them in the main LTO unit of the program. However, library
// calls may not exist until legalization occurs, which is far after global
// interprocedural dead code elimination would typically strip them out.
//
// Accordingly, this pass runs after legalization and internalizes any library
// calls that have liveness contingent on a call being emitted by the legalizer,
// but where no calls were actually emitted. The pass then runs dead code
// elimination to strip out such functions and cleans up any data structures
// that refer to them.
//
//===----------------------------------------------------------------------===//

#include "MC6809Internalize.h"

#include "MC6809.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/IR/Module.h"
#include "llvm/Pass.h"
#include "llvm/Transforms/IPO/GlobalDCE.h"

#define DEBUG_TYPE "mc6809-internalize"

using namespace llvm;

namespace {

class MC6809Internalize : public ModulePass {
  MachineModuleInfo *MMI;

public:
  static char ID;

  MC6809Internalize() : ModulePass(ID) { llvm::initializeMC6809InternalizePass(*PassRegistry::getPassRegistry()); }

  void getAnalysisUsage(AnalysisUsage &AU) const override;

  bool runOnModule(Module &M) override;

  DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> insertDummyIRLibcalls(Module &M) const;

  void eraseDummyIRLibcalls(DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> DummyIRLibcalls, const DenseSet<Function *> &ErasedFunctions) const;
};

} // namespace

void MC6809Internalize::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
}

bool MC6809Internalize::runOnModule(Module &M) {
  MMI = &getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> DummyIRLibcalls = insertDummyIRLibcalls(M);
  if (DummyIRLibcalls.empty())
    return false;

  for (GlobalValue &GV : M.global_values()) {
    if (GV.getPartition() != "contingent")
      continue;
    // These may be used in BSS zeroing and data copying, which may not be
    // discovered until the final LTO assembly is emitted.
    if (GV.getName() == "__memset" || GV.getName() == "memcpy")
      continue;
    GV.setPartition("");
    GV.setLinkage(llvm::GlobalValue::InternalLinkage);
  }

  DenseSet<Function *> ErasedFunctions;
  for (Function &F : M)
    ErasedFunctions.insert(&F);

  GlobalDCEPass DCE;
  ModuleAnalysisManager AM;
  DCE.run(M, AM);

  for (Function &F : M)
    ErasedFunctions.erase(&F);

  for (Function *F : ErasedFunctions)
    MMI->deleteMachineFunctionFor(*F);

  eraseDummyIRLibcalls(std::move(DummyIRLibcalls), ErasedFunctions);

  return true;
}

DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> MC6809Internalize::insertDummyIRLibcalls(Module &M) const {
  DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> DummyIRLibcalls;

  for (Function &F : M) {
    const MachineFunction *MF = MMI->getMachineFunction(F);
    if (!MF)
      continue;

    for (const MachineBasicBlock &MBB : *MF) {
      for (const MachineInstr &MI : MBB) {
        if (!MI.isCall())
          continue;
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isSymbol())
            continue;

          GlobalValue *Callee = M.getFunction(MO.getSymbolName());
          if (!Callee)
            Callee = M.getNamedAlias(MO.getSymbolName());
          if (!Callee || Callee->getPartition() != "contingent")
            continue;

          IRBuilder<> Builder(&F.getEntryBlock());
          std::pair<Function *, GlobalValue *> KV = {&F, Callee};
          if (DummyIRLibcalls.contains(KV))
            continue;
          auto Res = DummyIRLibcalls.try_emplace(KV, Builder.CreateCall(FunctionType::get(Type::getVoidTy(F.getContext()),
                                                                                          /*isVarArg=*/false),
                                                                        Callee));
          (void)Res;
          assert(Res.second);
        }
      }
    }
  }

  return DummyIRLibcalls;
}

void MC6809Internalize::eraseDummyIRLibcalls(DenseMap<std::pair<Function *, GlobalValue *>, Instruction *> DummyIRLibcalls, const DenseSet<Function *> &ErasedFunctions) const {
  for (const auto &KV : DummyIRLibcalls) {
    Function *Caller = KV.first.first;
    Instruction *I = KV.second;
    if (ErasedFunctions.contains(Caller))
      continue;
    I->eraseFromParent();
  }
}

char MC6809Internalize::ID = 0;

INITIALIZE_PASS(MC6809Internalize, DEBUG_TYPE, "MC6809 internalize libcalls", false, false)

ModulePass *llvm::createMC6809InternalizePass() { return new MC6809Internalize(); }
