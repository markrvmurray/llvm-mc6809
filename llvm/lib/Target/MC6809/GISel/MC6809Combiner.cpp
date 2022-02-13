//===-- MC6809Combiner.cpp - MC6809 GlobalIsel Combiner -------------------------===//
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

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"
#include "MC6809LegalizerInfo.h"
#include "MC6809Subtarget.h"

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
  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool applyFoldGlobalOffset(
      MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
      GISelChangeObserver &Observer,
      std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

  bool matchSBCEqual(MachineInstr &MI, MachineRegisterInfo &MRI) const;
  bool applySBCEqual(MachineInstr &MI, MachineRegisterInfo &MRI,
                     MachineIRBuilder &B, GISelChangeObserver &Observer) const;

  bool matchExtractLowBit(MachineInstr &MI, MachineRegisterInfo &MRI,
                          MachineInstr *&Shift) const;
  bool applyExtractLowBit(MachineInstr &MI, MachineRegisterInfo &MRI,
                          MachineIRBuilder &B, GISelChangeObserver &Observer,
                          MachineInstr *&Shift) const;
};

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

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
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

bool MC6809CombinerHelperState::matchSBCEqual(MachineInstr &MI,
                                           MachineRegisterInfo &MRI) const {
  assert(MI.getOpcode() == MC6809::G_SBC);
  Register LHS = MI.getOperand(5).getReg();
  Register RHS = MI.getOperand(6).getReg();
  Register CarryIn = MI.getOperand(7).getReg();

  auto ConstCarryIn = getIConstantVRegValWithLookThrough(CarryIn, MRI);
  if (!ConstCarryIn)
    return false;
  if (!ConstCarryIn->Value.isAllOnesValue())
    return false;

  if (LHS == RHS)
    return true;

  auto ConstLHS = getIConstantVRegValWithLookThrough(LHS, MRI);
  auto ConstRHS = getIConstantVRegValWithLookThrough(RHS, MRI);
  if (!ConstLHS || !ConstRHS)
    return false;

  return ConstLHS->Value == ConstRHS->Value;
}

bool MC6809CombinerHelperState::applySBCEqual(
    MachineInstr &MI, MachineRegisterInfo &MRI, MachineIRBuilder &B,
    GISelChangeObserver &Observer) const {
  LLT S1 = LLT::scalar(1);

  B.setInsertPt(*MI.getParent(), MI);
  B.buildCopy(MI.getOperand(0), B.buildConstant(LLT::scalar(8), 0));

  auto S1Zero = B.buildConstant(S1, 0);
  // C
  B.buildCopy(MI.getOperand(1), S1Zero);
  // N
  B.buildCopy(MI.getOperand(2), S1Zero);
  // V
  B.buildCopy(MI.getOperand(3), S1Zero);
  // Z
  B.buildCopy(MI.getOperand(4), B.buildConstant(S1, -1));
  MI.eraseFromParent();
  return true;
}

bool MC6809CombinerHelperState::matchExtractLowBit(MachineInstr &MI,
                                                MachineRegisterInfo &MRI,
                                                MachineInstr *&Shift) const {
  using namespace MIPatternMatch;
  Register Src;
  if (MI.getOpcode() == MC6809::G_TRUNC) {
    if (MRI.getType(MI.getOperand(0).getReg()) != LLT::scalar(1))
      return false;
    Src = MI.getOperand(1).getReg();
    Register NewSrc;
    if (mi_match(Src, MRI,
                 m_GAnd(m_Reg(NewSrc), MIPatternMatch::m_SpecificICst(1))))
      Src = NewSrc;
  } else {
    assert(MI.getOpcode() == MC6809::G_ICMP);
    ICmpInst::Predicate Pred;
    if (!mi_match(MI.getOperand(0).getReg(), MRI,
                  m_GICmp(m_Pred(Pred),
                          m_GAnd(m_Reg(Src), MIPatternMatch::m_SpecificICst(1)),
                          MIPatternMatch::m_SpecificICst(0))))
      return false;
    // The NE case handled automatically via an optimization that converts it to
    // a G_TRUNC.
    if (Pred != CmpInst::ICMP_EQ)
      return false;
  }

  for (MachineInstr &RefMI : MRI.reg_nodbg_instructions(Src)) {
    if (RefMI.getOpcode() != MC6809::G_LSHR)
      continue;
    if (RefMI.getOperand(1).getReg() != Src)
      continue;
    auto ConstAmt =
        getIConstantVRegValWithLookThrough(RefMI.getOperand(2).getReg(), MRI);
    if (!ConstAmt || !ConstAmt->Value.isOne())
      continue;
    if (!Helper.dominates(RefMI, MI) && !Helper.dominates(MI, RefMI))
      continue;
    Shift = &RefMI;
    return true;
  }

  return false;
}

bool MC6809CombinerHelperState::applyExtractLowBit(MachineInstr &MI,
                                                MachineRegisterInfo &MRI,
                                                MachineIRBuilder &B,
                                                GISelChangeObserver &Observer,
                                                MachineInstr *&Shift) const {
  assert(Shift->getOpcode() == MC6809::G_LSHR);
  LLT S1 = LLT::scalar(1);

  bool Negate = MI.getOpcode() == MC6809::G_ICMP &&
                MI.getOperand(1).getPredicate() == CmpInst::ICMP_EQ;

  if (Helper.dominates(*Shift, MI)) {
    B.setInsertPt(*Shift->getParent(), *Shift);
  } else {
    assert(Helper.dominates(MI, *Shift));
    B.setInsertPt(*MI.getParent(), MI);
  }

  auto EvenShift = B.buildInstr(MC6809::G_LSHRE, {Shift->getOperand(0), S1},
                                {Shift->getOperand(1), B.buildConstant(S1, 0)});
  if (Negate)
    B.buildNot(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  else
    B.buildCopy(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  MC6809LegalizerInfo Legalizer(B.getMF().getSubtarget<MC6809Subtarget>());
  LegalizerHelper LegalizerHelper(B.getMF(), Legalizer, Observer, B);
  B.setInsertPt(B.getMBB(), *EvenShift);
  if (!Legalizer.legalizeLshrEShlE(LegalizerHelper, MRI, *EvenShift))
    llvm_unreachable("Failed to legalize shift.");
  Shift->eraseFromParent();
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

bool MC6809CombinerInfo::combine(GISelChangeObserver &Observer, MachineInstr &MI,
                              MachineIRBuilder &B) const {
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
INITIALIZE_PASS_BEGIN(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs",
                    false, false)

namespace llvm {
FunctionPass *createMC6809Combiner() { return new MC6809Combiner; }
} // namespace llvm
