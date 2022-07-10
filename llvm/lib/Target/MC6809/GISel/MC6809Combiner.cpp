//===-- MC6809Combiner.cpp - MC6809 GlobalIsel Combiner
//-------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the MC6809 global machine instruction combiner.
//
// This runs between phases of the GlobalISel process to optimize away
// inefficient patterns discovered in the global machine instructions.
//
//===----------------------------------------------------------------------===//

#include "MC6809Combiner.h"

#include "MC6809.h"
#include "MC6809LegalizerInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/Combiner.h"
#include "llvm/CodeGen/GlobalISel/CombinerHelper.h"
#include "llvm/CodeGen/GlobalISel/CombinerInfo.h"
#include "llvm/CodeGen/GlobalISel/GISelChangeObserver.h"
#include "llvm/CodeGen/GlobalISel/GISelKnownBits.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/PatternMatch.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mc6809-combiner"

using namespace llvm;

class MC6809CombinerHelperState {
protected:
  CombinerHelper &Helper;

public:
  MC6809CombinerHelperState(CombinerHelper &Helper) : Helper(Helper) {}

  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool matchFoldGlobalOffset(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const;
  bool applyFoldGlobalOffset(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

  // %1 = G_GLOBAL_VALUE @foo + bar
  // %2 = COPY %1
  // =>
  // %2 = G_GLOBAL_VALUE @foo + bar
  bool matchFoldGlobalCopy(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<const MachineOperand *, MachineInstr *> &MatchInfo) const;
  bool applyFoldGlobalCopy(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<const MachineOperand *, MachineInstr *> &MatchInfo) const;

  //  %2:_(s16) = G_[SZ]EXT %1:_(s8)
  //  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
  //   =>
  //  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
  bool matchFoldPointerExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<MachineInstr *, MachineInstr *>&MatchInfo) const;
  bool applyFoldPointerExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<MachineInstr *, MachineInstr *>&MatchInfo) const;

  //  ( %1:_(s8) = COPY $af // Antipattern )
  //  %1:_(s8) = G_LOAD %5:_(p0) :: (invariant load (s8) from %fixed-stack.3, align 2)
  //  %3:_(s8) = G_ADD %1:_, %0:_ ; The known physical register is on the wrong side
  //   =>
  //  :
  //  %3:_(s8) = G_ADD %0:_, %1:_ ; Can make use of addressing modes
  bool matchSwapPhysregToLhs(MachineInstr &MI, MachineRegisterInfo &MRI, MachineInstr *&MatchInfo) const;
  bool applySwapPhysregToLhs(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, MachineInstr *&MatchInfo) const;

  //  %0:accum(s8) = COPY $ab
  //  :
  //  %11:accum(s8) = G_ADD %10:accum, %4:accum
  //  %12:accum(s8) = G_SUB %0:accum, %11:accum
  //   =>
  //  %0:accum(s8) = COPY $ab
  //  :
  //  %12:accum(s8) = G_SUB %0:accum, %4:accum
  //  %13:accum(s8) = G_SUB %12:accum, %10:accum
  bool matchSwitchAddToSubtract(MachineInstr &MI, MachineRegisterInfo &MRI, MachineInstr *&MatchInfo) const;
  bool applySwitchAddToSubtract(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, MachineInstr *&MatchInfo) const;

};
// ======================================================================

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MC6809CombinerHelperState::matchFoldGlobalOffset(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == MC6809::G_PTR_ADD);

  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineInstr *GlobalBase = getOpcodeDef(MC6809::G_GLOBAL_VALUE, Base, MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI);

  if (!GlobalBase || !ConstOffset)
    return false;
  const MachineOperand *BaseGV = &GlobalBase->getOperand(1);
  int64_t NewOffset = BaseGV->getOffset() + ConstOffset->Value.getSExtValue();
  MatchInfo = {BaseGV, NewOffset};
  return true;
}

bool MC6809CombinerHelperState::applyFoldGlobalOffset(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == MC6809::G_PTR_ADD);
  const TargetInstrInfo &TII = MIB.getTII();
  Observer.changingInstr(MI);
  MI.setDesc(TII.get(MC6809::G_GLOBAL_VALUE));
  MI.getOperand(1).ChangeToGA(MatchInfo.first->getGlobal(), MatchInfo.second, MatchInfo.first->getTargetFlags());
  MI.removeOperand(2);
  Observer.changedInstr(MI);
  return true;
}

// %1 = G_GLOBAL_VALUE @foo + bar
// %2 = COPY %1
// =>
// %2 = G_GLOBAL_VALUE @foo + bar
bool MC6809CombinerHelperState::matchFoldGlobalCopy(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<const MachineOperand *, MachineInstr *> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == COPY);

  MachineOperand *CopyDest = &MI.getOperand(0);
  MachineOperand *CopySource = &MI.getOperand(1);
  MachineInstr *GlobalBase = getOpcodeDef(MC6809::G_GLOBAL_VALUE, CopySource->getReg(), MRI);

  if (!GlobalBase)
    return false;
  MatchInfo = {CopyDest, GlobalBase};
  return true;
}

bool MC6809CombinerHelperState::applyFoldGlobalCopy(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<const MachineOperand *, MachineInstr *> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == COPY);
  const TargetInstrInfo &TII = MIB.getTII();
  Observer.changingInstr(*(MatchInfo.second));
  MatchInfo.second->getOperand(0).setReg(MatchInfo.first->getReg());
  Observer.changedInstr(*(MatchInfo.second));
  MI.eraseFromParent();
  return true;
}

//  %2:_(s16) = G_[SZ]EXT %1:_(s8)
//  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
//   =>
//  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
bool MC6809CombinerHelperState::matchFoldPointerExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI, std::pair<MachineInstr *, MachineInstr *>&MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == MC6809::G_PTR_ADD);
  if (!MI.getOperand(2).isReg())
    return false;
  Register Offset = MI.getOperand(2).getReg();
  MachineInstr *Ext = getOpcodeDef (MC6809::G_SEXT, Offset, MRI);
  if (!Ext) {
    Ext = getOpcodeDef(MC6809::G_ZEXT, Offset, MRI);
    if (!Ext)
      return false;
  }
  MatchInfo = {&MI, Ext};
  return true;
}

bool MC6809CombinerHelperState::applyFoldPointerExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, std::pair<MachineInstr *, MachineInstr *>&MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == MC6809::G_PTR_ADD);
  Observer.changingInstr(MI);
  MI.getOperand(2).ChangeToRegister(MatchInfo.second->getOperand(1).getReg(), /* isDef */ false);
  MatchInfo.second->eraseFromParent();
  Observer.changedInstr(MI);
  return true;
}

// ============================================================================
//  ( %1:_(s8) = COPY $af // Antipattern )
//  %1:_(s8) = G_LOAD %5:_(p0) :: (invariant load (s8) from %fixed-stack.3, align 2)
//  %3:_(s8) = G_ADD %1:_, %0:_ ; The known physical register is on the wrong side
//   =>
//  :
//  %3:_(s8) = G_ADD %0:_, %1:_ ; Can make use of addressing modes
bool MC6809CombinerHelperState::matchSwapPhysregToLhs(MachineInstr &MI, MachineRegisterInfo &MRI, MachineInstr *&MatchInfo) const {
  using namespace TargetOpcode;
  const MC6809Subtarget &STI = static_cast<const MC6809Subtarget &>(MI.getMF()->getSubtarget());
  if (!STI.isHD6309()) {
    assert(MI.getOpcode() == MC6809::G_ADD || MI.getOpcode() == MC6809::G_SADDO || MI.getOpcode() == MC6809::G_UADDO || MI.getOpcode() == MC6809::G_SADDE || MI.getOpcode() == MC6809::G_UADDE);
    int ArgA, ArgB;
    if (MI.getOpcode() == MC6809::G_ADD) {
      ArgA = 1;
      ArgB = 2;
    } else {
      ArgA = 2;
      ArgB = 3;
    }
    Register LHS = MI.getOperand(ArgA).getReg();
    auto CopyL = getOpcodeDef(COPY, LHS, MRI);
    if (CopyL && CopyL->getOperand(1).isReg() && CopyL->getOperand(1).getReg().isPhysical())
      return false;
    Register RHS = MI.getOperand(ArgB).getReg();
    auto CopyR = getOpcodeDef(COPY, RHS, MRI);
    if (CopyR && CopyR->getOperand(1).isReg() && CopyR->getOperand(1).getReg().isPhysical()) {
      MatchInfo = CopyR;
      return true;
    }
  }
  return false;
}

bool MC6809CombinerHelperState::applySwapPhysregToLhs(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, MachineInstr *&MatchInfo) const {
  using namespace TargetOpcode;
  int ArgA, ArgB;
  assert(MI.getOpcode() == MC6809::G_ADD || MI.getOpcode() == MC6809::G_SADDO || MI.getOpcode() == MC6809::G_UADDO || MI.getOpcode() == MC6809::G_SADDE || MI.getOpcode() == MC6809::G_UADDE);
  if (MI.getOpcode() == MC6809::G_ADD) {
    ArgA = 1;
    ArgB = 2;
  } else {
    ArgA = 2;
    ArgB = 3;
  }
  Observer.changingInstr(MI);
  auto Temp = MI.getOperand(ArgB).getReg();
  MI.getOperand(ArgB).setReg(MI.getOperand(ArgA).getReg());
  MI.getOperand(ArgA).setReg(Temp);
  Observer.changedInstr(MI);
  return true;
}

//  %0:accum(s8) = COPY $ab
//  :
//  %11:accum(s8) = G_ADD %10:accum, %4:accum
//  %12:accum(s8) = G_SUB %0:accum, %11:accum
//   =>
//  %0:accum(s8) = COPY $ab
//  :
//  %11:accum(s8) = G_SUB %0:accum, %4:accum
//  %12:accum(s8) = G_SUB %11:accum, %10:accum
bool MC6809CombinerHelperState::matchSwitchAddToSubtract(MachineInstr &MI, MachineRegisterInfo &MRI, MachineInstr *&MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_SUB);

  MachineOperand *Subtrahend = &MI.getOperand(2);
  MachineInstr *GAdd = getOpcodeDef(G_ADD, Subtrahend->getReg(), MRI);

  if (!GAdd)
    return false;
  MatchInfo = GAdd;
  return true;
}

bool MC6809CombinerHelperState::applySwitchAddToSubtract(MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB, GISelChangeObserver &Observer, MachineInstr *&MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == MC6809::G_SUB);
  assert(MatchInfo->getOpcode() == MC6809::G_ADD);
  MachineIRBuilder Builder(MI);
  Observer.changingInstr(MI);
  Builder.buildSub(MatchInfo->getOperand(0).getReg(), MI.getOperand(1).getReg(), MatchInfo->getOperand(2).getReg());
  Builder.buildSub(MI.getOperand(0).getReg(), MI.getOperand(2).getReg(), MatchInfo->getOperand(1).getReg());
  MatchInfo->eraseFromParent();
  Observer.changedInstr(MI);
  MI.eraseFromParent();
  return true;
}

#define MC6809COMBINERHELPER_GENCOMBINERHELPER_DEPS
#include "MC6809GenGICombiner.inc"
#undef MC6809COMBINERHELPER_GENCOMBINERHELPER_DEPS

namespace {
#define MC6809COMBINERHELPER_GENCOMBINERHELPER_H
#include "MC6809GenGICombiner.inc"
#undef MC6809COMBINERHELPER_GENCOMBINERHELPER_H

class MC6809CombinerInfo : public CombinerInfo {
  GISelKnownBits *KB;
  MachineDominatorTree *MDT;
  MC6809GenCombinerHelperRuleConfig GeneratedRuleCfg;

public:
  MC6809CombinerInfo(bool EnableOpt, bool OptSize, bool MinSize, GISelKnownBits *KB, MachineDominatorTree *MDT)
      : CombinerInfo(/*AllowIllegalOps*/ true, /*ShouldLegalizeIllegal*/ false, /*LegalizerInfo*/ nullptr, EnableOpt, OptSize, MinSize),
        KB(KB), MDT(MDT) {
    if (!GeneratedRuleCfg.parseCommandLineOption())
      report_fatal_error("Invalid rule identifier");
  }

  virtual bool combine(GISelChangeObserver &Observer, MachineInstr &MI, MachineIRBuilder &MIB) const override;
};

bool MC6809CombinerInfo::combine(GISelChangeObserver &Observer, MachineInstr &MI, MachineIRBuilder &MIB) const {
  const LegalizerInfo *LI = MI.getMF()->getSubtarget().getLegalizerInfo();
  CombinerHelper Helper(Observer, MIB, KB, MDT, LI);
  MC6809GenCombinerHelper Generated(GeneratedRuleCfg, Helper);
  return Generated.tryCombineAll(Observer, MI, MIB);
}

#define MC6809COMBINERHELPER_GENCOMBINERHELPER_CPP
#include "MC6809GenGICombiner.inc"
#undef MC6809COMBINERHELPER_GENCOMBINERHELPER_CPP

// Pass boilerplate
// ================

class MC6809Combiner : public MachineFunctionPass {
public:
  static char ID;

  MC6809Combiner();

  StringRef getPassName() const override { return "MC6809Combiner"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override;
};
} // end anonymous namespace

void MC6809Combiner::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.addRequired<TargetPassConfig>();
  AU.setPreservesCFG();
  getSelectionDAGFallbackAnalysisUsage(AU);
  AU.addRequired<GISelKnownBitsAnalysis>();
  AU.addPreserved<GISelKnownBitsAnalysis>();
  AU.addRequired<MachineDominatorTree>();
  AU.addPreserved<MachineDominatorTree>();
  AU.addRequired<GISelCSEAnalysisWrapperPass>();
  AU.addPreserved<GISelCSEAnalysisWrapperPass>();
  MachineFunctionPass::getAnalysisUsage(AU);
}

MC6809Combiner::MC6809Combiner() : MachineFunctionPass(ID) {
  initializeMC6809CombinerPass(*PassRegistry::getPassRegistry());
}

bool MC6809Combiner::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(MachineFunctionProperties::Property::FailedISel))
    return false;

  auto *TPC = &getAnalysis<TargetPassConfig>();

  // Enable CSE.
  GISelCSEAnalysisWrapper &Wrapper = getAnalysis<GISelCSEAnalysisWrapperPass>().getCSEWrapper();
  auto *CSEInfo = &Wrapper.get(TPC->getCSEConfig());

  const Function &F = MF.getFunction();
  bool EnableOpt = MF.getTarget().getOptLevel() != CodeGenOpt::None && !skipFunction(F);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT = &getAnalysis<MachineDominatorTree>();
  MC6809CombinerInfo PCInfo(EnableOpt, F.hasOptSize(), F.hasMinSize(), KB, MDT);
  Combiner C(PCInfo, TPC);
  return C.combineMachineInstrs(MF, CSEInfo);
}

char MC6809Combiner::ID = 0;
INITIALIZE_PASS_BEGIN(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs", false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs", false, false)

namespace llvm {
FunctionPass *createMC6809Combiner() { return new MC6809Combiner; }
} // namespace llvm
