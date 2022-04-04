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
  bool matchFoldGlobalOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;
  bool applyFoldGlobalOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB,
      GISelChangeObserver &Observer,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

//  %2:_(s16) = G_SEXT %1:_(s8)
//  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
//   =>
//  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
  bool matchFoldPointerSExtOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI,
      std::tuple<MachineInstr *> &MatchInfo) const;
  bool applyFoldPointerSExtOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &MIB,
      GISelChangeObserver &Observer,
      std::tuple<MachineInstr *> &MatchInfo) const;
};
// ======================================================================

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MC6809CombinerHelperState::matchFoldGlobalOffset(
    MachineInstr &MI, MachineRegisterInfo &MRI,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);

  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Base, MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI);

  if (!GlobalBase || !ConstOffset)
    return false;
  const MachineOperand *BaseGV = &GlobalBase->getOperand(1);
  int64_t NewOffset = BaseGV->getOffset() + ConstOffset->Value.getSExtValue();
  MatchInfo = {BaseGV, NewOffset};
  return true;
}

bool MC6809CombinerHelperState::applyFoldGlobalOffset(
    MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
    GISelChangeObserver &Observer,
    std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);
  const TargetInstrInfo &TII = B.getTII();
  Observer.changingInstr(MI);
  MI.setDesc(TII.get(TargetOpcode::G_GLOBAL_VALUE));
  MI.getOperand(1).ChangeToGA(MatchInfo.first->getGlobal(), MatchInfo.second,
                              MatchInfo.first->getTargetFlags());
  MI.RemoveOperand(2);
  Observer.changedInstr(MI);
  return true;
}

//  %2:_(s16) = G_SEXT %1:_(s8)
//  %3:_(p0) = G_PTR_ADD %0:_, %2:_(s16)
//   =>
//  %3:_(s8) = G_PTR_ADD %0:_, %1:_(s8)
bool MC6809CombinerHelperState::matchFoldPointerSExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI,
                                                           std::tuple<MachineInstr *> &MatchInfo) const {
  using namespace TargetOpcode;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOpcode() == G_PTR_ADD);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Matched PtrAdd\n";);
  Register Offset = MI.getOperand(2).getReg();
  MachineInstr *SExt = getOpcodeDef (G_SEXT, Offset, MRI);
  if (!SExt)
    return false;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Matched SExt\n";);
  std::get<0>(MatchInfo) = SExt;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit with match\n";);
  return true;
}

bool MC6809CombinerHelperState::applyFoldPointerSExtOffset(MachineInstr &MI, MachineRegisterInfo &MRI,
                                                           MachineIRBuilder &MIB, GISelChangeObserver &Observer,
                                                           std::tuple<MachineInstr *> &MatchInfo) const {
  using namespace TargetOpcode;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineInstr *SExt = std::get<0>(MatchInfo);
  assert(MI.getOpcode() == G_PTR_ADD);
  const TargetInstrInfo &TII = MIB.getTII();
  Observer.changingInstr(MI);
  MI.getOperand(2).ChangeToRegister(SExt->getOperand(1).getReg(), /* isDef */ false);
  SExt->eraseFromParent();
  Observer.changedInstr(MI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
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
  MC6809CombinerInfo(bool EnableOpt, bool OptSize, bool MinSize,
                     GISelKnownBits *KB, MachineDominatorTree *MDT)
      : CombinerInfo(/*AllowIllegalOps*/ true,
                     /*ShouldLegalizeIllegal*/ false,
                     /*LegalizerInfo*/ nullptr, EnableOpt, OptSize, MinSize),
        KB(KB), MDT(MDT) {
    if (!GeneratedRuleCfg.parseCommandLineOption())
      report_fatal_error("Invalid rule identifier");
  }

  virtual bool combine(GISelChangeObserver &Observer, MachineInstr &MI,
                       MachineIRBuilder &B) const override;
};

bool MC6809CombinerInfo::combine(GISelChangeObserver &Observer,
                                 MachineInstr &MI, MachineIRBuilder &B) const {
  const LegalizerInfo *LI = MI.getMF()->getSubtarget().getLegalizerInfo();
  CombinerHelper Helper(Observer, B, KB, MDT, LI);
  MC6809GenCombinerHelper Generated(GeneratedRuleCfg, Helper);
  return Generated.tryCombineAll(Observer, MI, B);
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
  if (MF.getProperties().hasProperty(
          MachineFunctionProperties::Property::FailedISel))
    return false;

  auto *TPC = &getAnalysis<TargetPassConfig>();

  // Enable CSE.
  GISelCSEAnalysisWrapper &Wrapper =
      getAnalysis<GISelCSEAnalysisWrapperPass>().getCSEWrapper();
  auto *CSEInfo = &Wrapper.get(TPC->getCSEConfig());

  const Function &F = MF.getFunction();
  bool EnableOpt =
      MF.getTarget().getOptLevel() != CodeGenOpt::None && !skipFunction(F);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT = &getAnalysis<MachineDominatorTree>();
  MC6809CombinerInfo PCInfo(EnableOpt, F.hasOptSize(), F.hasMinSize(), KB, MDT);
  Combiner C(PCInfo, TPC);
  return C.combineMachineInstrs(MF, CSEInfo);
}

char MC6809Combiner::ID = 0;
INITIALIZE_PASS_BEGIN(MC6809Combiner, DEBUG_TYPE,
                      "Combine MC6809 machine instrs", false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs",
                    false, false)

namespace llvm {
FunctionPass *createMC6809Combiner() { return new MC6809Combiner; }
} // namespace llvm
