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

#include "MC6809.h"
#include "MC6809LegalizerInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/Combiner.h"
#include "llvm/CodeGen/GlobalISel/CombinerHelper.h"
#include "llvm/CodeGen/GlobalISel/CombinerInfo.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutor.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutorImpl.h"
#include "llvm/CodeGen/GlobalISel/GISelChangeObserver.h"
#include "llvm/CodeGen/GlobalISel/GISelKnownBits.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/PatternMatch.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define GET_GICOMBINER_DEPS
#include "MC6809GenGICombiner.inc"
#undef GET_GICOMBINER_DEPS

#define DEBUG_TYPE "mc6809-combiner"

using namespace llvm;

namespace {

#define GET_GICOMBINER_TYPES
#include "MC6809GenGICombiner.inc"
#undef GET_GICOMBINER_TYPES

class MC6809CombinerImpl : public Combiner {
protected:
  // TODO: Make CombinerHelper methods const.
  mutable CombinerHelper Helper;
  const MC6809CombinerImplRuleConfig &RuleConfig;
  AAResults *AA;

public:
  MC6809CombinerImpl(MachineFunction &MF, CombinerInfo &CInfo, const TargetPassConfig *TPC, bool IsPreLegalize, GISelKnownBits &KB, GISelCSEInfo *CSEInfo, const MC6809CombinerImplRuleConfig &RuleConfig, const MC6809Subtarget &STI,
                     MachineDominatorTree *MDT, const LegalizerInfo *LI, AAResults *AA);

  static const char *getName() { return "MC6809Combiner"; }

  bool tryCombineAll(MachineInstr &I) const override;

  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  bool matchFoldGlobalOffset(MachineInstr &MI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const;
  // G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
  // GLOBAL_VALUE @x + (y_const + z_const)
  void applyFoldGlobalOffset(MachineInstr &MI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const;

  bool matchExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const;
  void applyExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const;

  bool matchUAddO1(MachineInstr &MI) const;
  void applyUAddO1(MachineInstr &MI) const;

  bool matchCMPZZero(MachineInstr &MI, MachineOperand *&Zero) const;
  void applyCMPZZero(MachineInstr &MI, MachineOperand *&Zero) const;

  // G_LOAD/G_STORE pair => G_MEMCPY_INLINE
  bool matchLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const;
  void applyLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const;

  // G_STORE => G_MEMSET
  bool matchStoreToMemset(MachineInstr &MI, uint8_t &Value) const;
  void applyStoreToMemset(MachineInstr &MI, uint8_t &Value) const;

  bool matchFoldAddE(MachineInstr &MI, BuildFnTy &MatchInfo) const;
  bool matchFoldSbc(MachineInstr &MI, BuildFnTy &MatchInfo) const;
  bool matchFoldShift(MachineInstr &MI, BuildFnTy &MatchInfo) const;

  bool matchShiftUnusedCarryIn(MachineInstr &MI, BuildFnTy &MatchInfo) const;

  APInt getDemandedBits(Register R) const;
  APInt getDemandedBits(Register R, DenseMap<Register, APInt> &Cache) const;

private:
#define GET_GICOMBINER_CLASS_MEMBERS
#include "MC6809GenGICombiner.inc"
#undef GET_GICOMBINER_CLASS_MEMBERS
};

#define GET_GICOMBINER_IMPL
#include "MC6809GenGICombiner.inc"
#undef GET_GICOMBINER_IMPL

MC6809CombinerImpl::MC6809CombinerImpl(MachineFunction &MF, CombinerInfo &CInfo, const TargetPassConfig *TPC, bool IsPreLegalize, GISelKnownBits &KB, GISelCSEInfo *CSEInfo, const MC6809CombinerImplRuleConfig &RuleConfig, const MC6809Subtarget &STI,
                                       MachineDominatorTree *MDT, const LegalizerInfo *LI, AAResults *AA)
    : Combiner(MF, CInfo, TPC, &KB, CSEInfo), Helper(Observer, B, IsPreLegalize, &KB, MDT, LI), RuleConfig(RuleConfig), AA(AA),
#define GET_GICOMBINER_CONSTRUCTOR_INITS
#include "MC6809GenGICombiner.inc"
#undef GET_GICOMBINER_CONSTRUCTOR_INITS
{
}

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
bool MC6809CombinerImpl::matchFoldGlobalOffset(MachineInstr &MI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  const auto &Add = cast<GPtrAdd>(MI);
  using namespace TargetOpcode;

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Add.getBaseReg(), MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Add.getOffsetReg(), MRI);

  if (!GlobalBase || !ConstOffset)
    return false;
  const MachineOperand *BaseGV = &GlobalBase->getOperand(1);
  int64_t NewOffset = BaseGV->getOffset() + ConstOffset->Value.getSExtValue();
  MatchInfo = {BaseGV, NewOffset};
  return true;
}

// G_PTR_ADD (GLOBAL_VALUE @x + y_const), z_const =>
// GLOBAL_VALUE @x + (y_const + z_const)
void MC6809CombinerImpl::applyFoldGlobalOffset(MachineInstr &MI, std::pair<const MachineOperand *, int64_t> &MatchInfo) const {
  using namespace TargetOpcode;
  assert(MI.getOpcode() == G_PTR_ADD);
  const TargetInstrInfo &TII = B.getTII();
  Observer.changingInstr(MI);
  MI.setDesc(TII.get(TargetOpcode::G_GLOBAL_VALUE));
  MI.getOperand(1).ChangeToGA(MatchInfo.first->getGlobal(), MatchInfo.second, MatchInfo.first->getTargetFlags());
  MI.removeOperand(2);
  Observer.changedInstr(MI);
}

bool MC6809CombinerImpl::matchExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const {
  using namespace MIPatternMatch;
  Register Src;
  if (MI.getOpcode() == MC6809::G_TRUNC) {
    if (MRI.getType(MI.getOperand(0).getReg()) != LLT::scalar(1))
      return false;
    Src = MI.getOperand(1).getReg();
    Register NewSrc;
    if (mi_match(Src, MRI, m_GAnd(m_Reg(NewSrc), MIPatternMatch::m_SpecificICst(1))))
      Src = NewSrc;
  } else {
    assert(MI.getOpcode() == MC6809::G_ICMP);
    ICmpInst::Predicate Pred;
    if (!mi_match(MI.getOperand(0).getReg(), MRI, m_GICmp(m_Pred(Pred), m_GAnd(m_Reg(Src), MIPatternMatch::m_SpecificICst(1)), MIPatternMatch::m_SpecificICst(0))))
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
    auto ConstAmt = getIConstantVRegValWithLookThrough(RefMI.getOperand(2).getReg(), MRI);
    if (!ConstAmt || !ConstAmt->Value.isOne())
      continue;
    if (!Helper.dominates(RefMI, MI) && !Helper.dominates(MI, RefMI))
      continue;
    Shift = &RefMI;
    return true;
  }

  return false;
}

void MC6809CombinerImpl::applyExtractLowBit(MachineInstr &MI, MachineInstr *&Shift) const {
  assert(Shift->getOpcode() == MC6809::G_LSHR);
  LLT S1 = LLT::scalar(1);

  bool Negate = MI.getOpcode() == MC6809::G_ICMP && MI.getOperand(1).getPredicate() == CmpInst::ICMP_EQ;

  if (Helper.dominates(*Shift, MI)) {
    B.setInstrAndDebugLoc(*Shift);
  } else {
    assert(Helper.dominates(MI, *Shift));
    B.setInstrAndDebugLoc(MI);
  }

  auto EvenShift = B.buildInstr(MC6809::G_LSHRE, {Shift->getOperand(0), S1}, {Shift->getOperand(1), B.buildConstant(S1, 0)});
  if (Negate)
    B.buildNot(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  else
    B.buildCopy(MI.getOperand(0).getReg(), EvenShift.getReg(1));
  MC6809LegalizerInfo Legalizer(B.getMF().getSubtarget<MC6809Subtarget>());
  LegalizerHelper LegalizerHelper(B.getMF(), Legalizer, Observer, B);
  B.setInstrAndDebugLoc(*EvenShift);
  if (!Legalizer.legalizeLshrEShlE(LegalizerHelper, MRI, *EvenShift))
    llvm_unreachable("Failed to legalize shift.");
  Shift->eraseFromParent();
  MI.eraseFromParent();
}

// Use of the overflow flag from an increment is better done on the 6502 by
// comparing the result to zero, since this allows increment and decrement
// operators instead of just ADC.
bool MC6809CombinerImpl::matchUAddO1(MachineInstr &MI) const {
  if (MI.getOpcode() != MC6809::G_UADDO)
    return false;
  std::optional<ValueAndVReg> Val = getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!Val || !Val->Value.isOne())
    return false;
  return true;
}

void MC6809CombinerImpl::applyUAddO1(MachineInstr &MI) const {
  LLT Ty = MRI.getType(MI.getOperand(0).getReg());
  B.setInstrAndDebugLoc(MI);
  B.buildAdd(MI.getOperand(0), MI.getOperand(2), MI.getOperand(3));
  B.buildICmp(CmpInst::ICMP_EQ, MI.getOperand(1), MI.getOperand(0), B.buildConstant(Ty, 0));
  MI.eraseFromParent();
}

static bool isRepeatingBytePattern(uint64_t Value, uint32_t Bytes) {
  // Support variants like 0x01010101, 0x02020202...
  for (uint32_t I = 1; I < Bytes; I++) {
    if (((Value >> (I * 8)) & 0xFF) != (Value & 0xFF)) {
      return false;
    }
  }
  return true;
}

// G_LOAD/G_STORE pair => G_MEMCPY_INLINE
bool MC6809CombinerImpl::matchLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const {
  const auto *Store = dyn_cast<GStore>(&MI);
  if (!Store)
    return false;

  Load = dyn_cast_or_null<GLoad>(MI.getPrevNode());
  if (!Load)
    return false;
  if (Load->getDstReg() != Store->getValueReg())
    return false;
  if (!Load->isUnordered() || !Store->isUnordered())
    return false;
  if (MI.mayAlias(AA, *Load, true))
    return false;

  return true;
}

void MC6809CombinerImpl::applyLoadStoreToMemcpy(MachineInstr &MI, GLoad *&Load) const {
  const auto &Store = cast<GStore>(MI);
  B.setInstrAndDebugLoc(MI);

  B.buildInstr(MC6809::G_MEMCPY_INLINE, {}, {Store.getPointerReg(), Load->getPointerReg(), B.buildConstant(LLT::scalar(16), MRI.getType(Store.getValueReg()).getSizeInBytes())}).addMemOperand(&Store.getMMO()).addMemOperand(&Load->getMMO());

  MI.eraseFromParent();
}

// G_STORE => G_MEMSET (large constant stores of repeating bytes)
bool MC6809CombinerImpl::matchStoreToMemset(MachineInstr &MI, uint8_t &Value) const {
  const auto *Store = dyn_cast<GStore>(&MI);
  if (!Store)
    return false;

  LLT Ty = MRI.getType(Store->getValueReg());
  if (!Ty.isScalar())
    return false;

  uint32_t SrcBits = Ty.getSizeInBits();
  if ((SrcBits & 7) != 0 || SrcBits < 24)
    return false;

  uint32_t SrcBytes = SrcBits >> 3;
  auto SrcValue = getIConstantVRegValWithLookThrough(Store->getValueReg(), MRI);
  if (!SrcValue)
    return false;

  auto SrcUInt64 = SrcValue->Value.getZExtValue();
  if (!isRepeatingBytePattern(SrcUInt64, SrcBytes))
    return false;

  Value = SrcUInt64 & 0xFF;
  return true;
}

void MC6809CombinerImpl::applyStoreToMemset(MachineInstr &MI, uint8_t &Value) const {
  const auto &Store = cast<GStore>(MI);
  B.setInstrAndDebugLoc(MI);

  auto Length = MRI.getType(Store.getValueReg()).getSizeInBytes();

  B.buildInstr(MC6809::G_MEMSET, {}, {Store.getPointerReg(), B.buildConstant(LLT::scalar(8), Value), B.buildConstant(LLT::scalar(16), Length), UINT64_C(0)}).addMemOperand(&Store.getMMO());

  MI.eraseFromParent();
}

bool MC6809CombinerImpl::matchFoldAddE(MachineInstr &MI, BuildFnTy &MatchInfo) const {
  auto LHS = getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!LHS)
    return false;
  auto RHS = getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!RHS)
    return false;
  auto CIn = getIConstantVRegValWithLookThrough(MI.getOperand(4).getReg(), MRI);
  if (!CIn)
    return false;

  bool Overflow;
  APInt Result;
  if (MI.getOpcode() == MC6809::G_UADDE) {
    Result = LHS->Value.uadd_ov(RHS->Value, Overflow);
    bool O;
    Result = Result.uadd_ov(CIn->Value.zext(Result.getBitWidth()), O);
    Overflow |= O;
  } else {
    Result = LHS->Value.sadd_ov(RHS->Value, Overflow);
    bool O;
    Result = Result.sadd_ov(CIn->Value.zext(Result.getBitWidth()), O);
    Overflow |= O;
  }

  Register Dst = MI.getOperand(0).getReg();
  Register COut = MI.getOperand(1).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(COut, Overflow);
  };
  return true;
}

bool MC6809CombinerImpl::matchFoldSbc(MachineInstr &MI, BuildFnTy &MatchInfo) const {
  auto LHS = getIConstantVRegValWithLookThrough(MI.getOperand(5).getReg(), MRI);
  if (!LHS)
    return false;
  auto RHS = getIConstantVRegValWithLookThrough(MI.getOperand(6).getReg(), MRI);
  if (!RHS)
    return false;
  auto CarryIn = getIConstantVRegValWithLookThrough(MI.getOperand(7).getReg(), MRI);
  if (!CarryIn)
    return false;

  APInt NotRHS = ~RHS->Value;

  bool CarryOut;
  APInt Result;
  Result = LHS->Value.uadd_ov(~RHS->Value, CarryOut);
  bool O;
  Result = Result.uadd_ov(CarryIn->Value.zext(Result.getBitWidth()), O);
  CarryOut |= O;

  bool Overflow = LHS->Value.isNegative() == !RHS->Value.isNegative() && Result.isNegative() != LHS->Value.isNegative();

  bool Negative = Result.isNegative();
  bool Zero = Result.isZero();

  Register Dst = MI.getOperand(0).getReg();
  Register C = MI.getOperand(1).getReg();
  Register N = MI.getOperand(2).getReg();
  Register V = MI.getOperand(3).getReg();
  Register Z = MI.getOperand(4).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(C, CarryOut);
    B.buildConstant(N, Negative);
    B.buildConstant(V, Overflow);
    B.buildConstant(Z, Zero);
  };
  return true;
}

bool MC6809CombinerImpl::matchFoldShift(MachineInstr &MI, BuildFnTy &MatchInfo) const {
  auto Val = getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI);
  if (!Val)
    return false;
  assert(Val->Value.getBitWidth() == 8);
  auto CarryIn = getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (!CarryIn)
    return false;

  bool CarryOut;
  APInt Result;

  if (MI.getOpcode() == MC6809::G_LSHRE) {
    CarryOut = (Val->Value & 1).getBoolValue();
    Result = Val->Value.lshr(1) | CarryIn->Value.zext(8).shl(7);
  } else {
    CarryOut = (Val->Value & 0x80).getBoolValue();
    Result = Val->Value.shl(1) | CarryIn->Value.zext(8);
  }

  Register Dst = MI.getOperand(0).getReg();
  Register C = MI.getOperand(1).getReg();
  MatchInfo = [=](MachineIRBuilder &B) {
    B.buildConstant(Dst, Result);
    B.buildConstant(C, CarryOut);
  };
  return true;
}

bool MC6809CombinerImpl::matchShiftUnusedCarryIn(MachineInstr &MI, BuildFnTy &MatchInfo) const {
  const auto ConstCarryIn = getIConstantVRegValWithLookThrough(MI.getOperand(3).getReg(), MRI);
  if (ConstCarryIn && ConstCarryIn->Value.isZero())
    return false;

  APInt DemandedBits = getDemandedBits(MI.getOperand(0).getReg());
  assert(DemandedBits.getBitWidth() == 8);
  if (MI.getOpcode() == MC6809::G_LSHRE) {
    if ((DemandedBits & 0x80).getBoolValue())
      return false;
  } else {
    if ((DemandedBits & 1).getBoolValue())
      return false;
  }

  MatchInfo = [=, &MI](MachineIRBuilder &B) {
    Observer.changingInstr(MI);
    MI.getOperand(3).setReg(B.buildConstant(LLT::scalar(1), 0).getReg(0));
    Observer.changedInstr(MI);
  };
  return true;
}

APInt MC6809CombinerImpl::getDemandedBits(Register R) const {
  DenseMap<Register, APInt> Cache;
  return getDemandedBits(R, Cache);
}

APInt MC6809CombinerImpl::getDemandedBits(Register R, DenseMap<Register, APInt> &Cache) const {
  auto It = Cache.find(R);
  if (It != Cache.end())
    return It->second;

  uint64_t Size = MRI.getType(R).getSizeInBits();

  APInt DemandedBits = APInt::getZero(Size);
  for (const MachineOperand &Use : MRI.use_nodbg_operands(R)) {
    const MachineInstr &MI = *Use.getParent();
    switch (MI.getOpcode()) {
    default:
      DemandedBits = APInt::getAllOnes(Size);
      break;
    case MC6809::G_AND: {
      APInt Zeroes = KB->getKnownZeroes(MI.getOperand(Use.getOperandNo() == 1 ? 2 : 1).getReg());
      DemandedBits |= ~Zeroes;
      break;
    }
    case MC6809::G_OR: {
      APInt Ones = KB->getKnownOnes(MI.getOperand(Use.getOperandNo() == 1 ? 2 : 1).getReg());
      DemandedBits |= ~Ones;
      break;
    }
    case MC6809::G_LSHRE: {
      APInt DstDemandedBits = getDemandedBits(MI.getOperand(0).getReg());
      if (Use.getOperandNo() == 2) {
        APInt CarryOutDemanded = getDemandedBits(MI.getOperand(1).getReg());
        DemandedBits |= DstDemandedBits << 1 | CarryOutDemanded.zext(8);
      } else {
        assert(Use.getOperandNo() == 3);
        DemandedBits |= DstDemandedBits.lshr(7).trunc(1);
      }
      break;
    }
    case MC6809::G_SHLE: {
      APInt DstDemandedBits = getDemandedBits(MI.getOperand(0).getReg());
      if (Use.getOperandNo() == 2) {
        APInt CarryOutDemanded = getDemandedBits(MI.getOperand(1).getReg());
        DemandedBits |= DstDemandedBits.lshr(1) | (CarryOutDemanded.zext(8) << 7);
      } else {
        assert(Use.getOperandNo() == 3);
        DemandedBits |= DstDemandedBits.trunc(1);
      }
      break;
    }
    }
    if (DemandedBits.isAllOnes())
      break;
  }
  Cache.try_emplace(R, DemandedBits);
  return DemandedBits;
}

#if 0
class MC6809CombinerInfo : public CombinerInfo {
  GISelKnownBits *KB;
  MachineDominatorTree *MDT;
  AAResults *AA;
  MC6809CombinerImplRuleConfig GeneratedRuleCfg;

public:
  MC6809CombinerInfo(bool EnableOpt, bool OptSize, bool MinSize, GISelKnownBits *KB, MachineDominatorTree *MDT, AAResults *AA)
      : CombinerInfo(/*AllowIllegalOps*/ true,
                     /*ShouldLegalizeIllegal*/ false,
                     /*LegalizerInfo*/ nullptr, EnableOpt, OptSize, MinSize),
        KB(KB), MDT(MDT), AA(AA) {
    if (!GeneratedRuleCfg.parseCommandLineOption())
      report_fatal_error("Invalid rule identifier");
  }

  virtual bool combine(GISelChangeObserver &Observer, MachineInstr &MI, MachineIRBuilder &B) const override;
};

bool MC6809CombinerInfo::combine(GISelChangeObserver &Observer, MachineInstr &MI, MachineIRBuilder &B) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  const LegalizerInfo *LI = MI.getMF()->getSubtarget().getLegalizerInfo();
  bool IsPreLegalize = !MI.getMF()->getProperties().hasProperty(MachineFunctionProperties::Property::Legalized);
  CombinerHelper Helper(Observer, B, IsPreLegalize, KB, MDT, LI);
  MC6809CombinerImpl Impl(GeneratedRuleCfg, STI, Observer, B, Helper, *AA);
  Impl.setupMF(*MI.getMF(), KB, nullptr, nullptr, nullptr, AA);
  return Impl.tryCombineAll(MI);
}
#endif

#define MC6809COMBINERHELPER_GENCOMBINERHELPER_CPP
#include "MC6809GenGICombiner.inc"
#undef MC6809COMBINERHELPER_GENCOMBINERHELPER_CPP

// Pass boilerplate
// ================

class MC6809Combiner : public MachineFunctionPass {
  MC6809CombinerImplRuleConfig RuleConfig;

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
  AU.addRequired<AAResultsWrapperPass>();

  MachineFunctionPass::getAnalysisUsage(AU);
}

MC6809Combiner::MC6809Combiner() : MachineFunctionPass(ID) { initializeMC6809CombinerPass(*PassRegistry::getPassRegistry()); }

bool MC6809Combiner::runOnMachineFunction(MachineFunction &MF) {
  if (MF.getProperties().hasProperty(MachineFunctionProperties::Property::FailedISel))
    return false;

  auto &TPC = getAnalysis<TargetPassConfig>();

  // Enable CSE.
  GISelCSEAnalysisWrapper &Wrapper = getAnalysis<GISelCSEAnalysisWrapperPass>().getCSEWrapper();
  auto *CSEInfo = &Wrapper.get(TPC.getCSEConfig());

  const MC6809Subtarget &ST = MF.getSubtarget<MC6809Subtarget>();
  const auto *LI = ST.getLegalizerInfo();

  const Function &F = MF.getFunction();
  bool EnableOpt = MF.getTarget().getOptLevel() != CodeGenOptLevel::None && !skipFunction(F);
  bool IsPreLegalize = !MF.getProperties().hasProperty(MachineFunctionProperties::Property::Legalized);
  GISelKnownBits *KB = &getAnalysis<GISelKnownBitsAnalysis>().get(MF);
  MachineDominatorTree *MDT = &getAnalysis<MachineDominatorTree>();
  AAResults *AA = &getAnalysis<AAResultsWrapperPass>().getAAResults();
  CombinerInfo CInfo(
      /*AllowIllegalOps*/ IsPreLegalize, /*ShouldLegalizeIllegal*/ false,
      /*LegalizerInfo*/ nullptr, EnableOpt, F.hasOptSize(), F.hasMinSize());
  MC6809CombinerImpl Impl(MF, CInfo, &TPC, IsPreLegalize, *KB, CSEInfo, RuleConfig, ST, MDT, LI, AA);
  return Impl.combineMachineInstrs();
}

char MC6809Combiner::ID = 0;
INITIALIZE_PASS_BEGIN(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs", false, false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig)
INITIALIZE_PASS_DEPENDENCY(GISelKnownBitsAnalysis)
INITIALIZE_PASS_END(MC6809Combiner, DEBUG_TYPE, "Combine MC6809 machine instrs", false, false)

namespace llvm {
FunctionPass *createMC6809Combiner() { return new MC6809Combiner; }
} // namespace llvm
