//===-- MC6809InstructionSelector.cpp - MC6809 Instruction Selector -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 instruction selector.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstructionSelector.h"
#include "MC6809RegisterBankInfo.h"

#include <set>

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelectorImpl.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instruction.h"
#include "llvm/ObjectYAML/MachOYAML.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace MIPatternMatch;

#define DEBUG_TYPE "mc6809-isel"

namespace {

#define GET_GLOBALISEL_PREDICATE_BITSET
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATE_BITSET

class MC6809InstructionSelector : public InstructionSelector {
public:
  MC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI,
                         MC6809RegisterBankInfo &RBI);

  void setupMF(MachineFunction &MF, GISelKnownBits *KB,
               CodeGenCoverage &CovInfo, ProfileSummaryInfo *PSI,
               BlockFrequencyInfo *BFI, AAResults *AA) override;

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

private:
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo &TRI;
  const MC6809RegisterBankInfo &RBI;

  // Pre-tablegen selection functions. If these return false, fall through to
  // tablegen.
  bool selectAddSub(MachineInstr &MI);
  bool selectLogical(MachineInstr &MI);

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectFrameIndex(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectLshrShlE(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectConstant(MachineInstr &MI);
  bool selectLoad(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

  void composePtr(MachineIRBuilder &Builder, Register Dst, Register Lo,
                  Register Hi);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO,
                                const TargetRegisterClass &RegClass);

  // Select all instructions in a given span, recursively. Allows selecting an
  // instruction sequence by reducing it to a more easily selectable sequence.
  bool selectAll(MachineInstrSpan MIS);

  /// tblgen-erated 'select' implementation, used as the initial selector for
  /// the patterns that don't require complex C++.
  bool selectImpl(MachineInstr &MI, CodeGenCoverage &CoverageInfo) const;

  const TargetRegisterClass &getRegClassForType(Register Reg, MachineRegisterInfo &MRI) const;

#define GET_GLOBALISEL_PREDICATES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_DECL

#define GET_GLOBALISEL_TEMPORARIES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_DECL
};

} // namespace

#define GET_GLOBALISEL_IMPL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_IMPL

MC6809InstructionSelector::MC6809InstructionSelector(const MC6809TargetMachine &TM,
                                               MC6809Subtarget &STI,
                                               MC6809RegisterBankInfo &RBI)
    : STI(STI), TII(*STI.getInstrInfo()), TRI(*STI.getRegisterInfo()), RBI(RBI),
#define GET_GLOBALISEL_PREDICATES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_INIT
#define GET_GLOBALISEL_TEMPORARIES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_INIT
{
}

void MC6809InstructionSelector::setupMF(MachineFunction &MF, GISelKnownBits *KB,
                                     CodeGenCoverage &CovInfo,
                                     ProfileSummaryInfo *PSI,
                                     BlockFrequencyInfo *BFI, AAResults *AA) {
  InstructionSelector::setupMF(MF, KB, CovInfo, PSI, BFI, AA);

  // The machine verifier doesn't allow COPY instructions to have differing
  // types, but the various GlobalISel utilities used in the instruction
  // selector really need to be able to look through G_PTRTOINT and G_INTTOPTR
  // as if they were copies. To avoid maintaining separate versions of these, we
  // temporarily lower these to technically-illegal COPY instructions, but only
  // for the duration of this one pass.
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      switch (MI.getOpcode()) {
      case MC6809::G_PTRTOINT:
      case MC6809::G_INTTOPTR:
        MI.setDesc(TII.get(MC6809::COPY));
        break;
      }
    }
  }
}

const TargetRegisterClass &MC6809InstructionSelector::getRegClassForType(Register Reg, MachineRegisterInfo &MRI) const {
  const LLT Ty = MRI.getType(Reg);
  const unsigned TySize = Ty.getSizeInBits();

  if (RBI.getRegBank(Reg, MRI, TRI)->getID() == MC6809::ACCRegBankID) {
    switch (TySize) {
    default:
      llvm_unreachable("Register class (ACC) not available for LLT, RB combination");
    case 1:
      return MC6809::BIT1RegClass;
    case 8:
      return MC6809::ACC8RegClass;
    case 16:
      return MC6809::ACC16RegClass;
    case 32:
      return MC6809::ACC32RegClass;
    }
  }

  if (RBI.getRegBank(Reg, MRI, TRI)->getID() == MC6809::INDEXRegBankID) {
    switch (TySize) {
    default:
      llvm_unreachable("Register class (INDEX) not available for LLT, RB combination");
    case 16:
      return MC6809::INDEX16RegClass;
    }
  }

  llvm_unreachable("Unsupported register bank.");
}

bool MC6809InstructionSelector::select(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 10 : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  case MC6809::G_ADD:
  case MC6809::G_SUB:
    if (selectAddSub(MI))
      return true;
    break;
  case MC6809::G_AND:
  case MC6809::G_OR:
  case MC6809::G_XOR:
    if (selectLogical(MI))
      return true;
    break;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : MI = "; MI.dump(););
  if (selectImpl(MI, *CoverageInfo))
    return true;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 30 : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  default:
    return false;
  case MC6809::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MC6809::G_BLOCK_ADDR:
  case MC6809::G_GLOBAL_VALUE:
    return selectAddr(MI);
  case MC6809::G_LSHRE:
  case MC6809::G_SHLE:
    return selectLshrShlE(MI);
  case MC6809::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case MC6809::G_TRUNC:
    return selectTrunc(MI);
  case MC6809::G_UADDE:
  case MC6809::G_SADDE:
    return selectAddE(MI);
  case MC6809::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case MC6809::G_CONSTANT:
    return selectConstant(MI);
  case MC6809::G_LOAD:
    return selectLoad(MI);
  case MC6809::G_STORE:
    return selectStore(MI);

  case MC6809::G_BRINDIRECT:
  case MC6809::G_IMPLICIT_DEF:
  case MC6809::G_PHI:
  case MC6809::G_SEXT:
  case MC6809::G_ZEXT:
    return selectGeneric(MI);
  }
}

bool MC6809InstructionSelector::selectAddSub(MachineInstr &MI) {
#if 0
  assert(MI.getOpcode() == MC6809::G_ADD || MI.getOpcode() == MC6809::G_SUB);

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  LLT S1 = LLT::scalar(1);

  if (auto RHSConst =
          getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI)) {
    // Don't inhibit generation of INC/DEC.
    if (RHSConst->Value.abs().isOne())
      return false;
  }

  int64_t CarryInVal = MI.getOpcode() == MC6809::G_ADD ? 0 : -1;

  bool Success;

  MachineInstr *Load;

  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  unsigned Opcode;
  if (MI.getOpcode() == MC6809::G_ADD) {
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAdd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::ADCAe;
  } else {
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GSub(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::SBCAe;
  }

  if (Success) {
    Register CIn =
        Builder.buildInstr(MC6809::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Idx;
  if (MI.getOpcode() == MC6809::G_ADD) {
    Success = mi_match(
        MI.getOperand(0).getReg(), MRI,
        m_GAdd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::ADCAi_o16;
  } else {
    Success = mi_match(
        MI.getOperand(0).getReg(), MRI,
        m_GSub(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::SBCAi_o16;
  }
  if (Success) {
    Register CIn =
        Builder.buildInstr(MC6809::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute indexed instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Offset;
  if (MI.getOpcode() == MC6809::G_ADD) {
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAdd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::ADCAi_o16;
  } else {
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GSub(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::SBCAi_o16;
  }
  if (Success) {
    Register CIn =
        Builder.buildInstr(MC6809::LDCImm, {S1}, {CarryInVal}).getReg(0);
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .addUse(Offset)
                     .addUse(Idx)
                     .addUse(CIn)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect indexed instruction.");
    MI.eraseFromParent();
    return true;
  }
#endif
  return false;
}

bool MC6809InstructionSelector::selectLogical(MachineInstr &MI) {
#if 0
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);

  MachineInstr *Load;

  bool Success;
  Register Opcode;
  switch (MI.getOpcode()) {
  case MC6809::G_AND:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAnd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::ANDAbs;
    break;
  case MC6809::G_XOR:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GXor(m_Reg(LHS),
                        m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::EORAbs;
    break;
  case MC6809::G_OR:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GOr(m_Reg(LHS),
                       m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::ORAbs;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addUse(LHS)
                     .add(Addr)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Idx;
  switch (MI.getOpcode()) {
  case MC6809::G_AND:
    Success = mi_match(
        MI.getOperand(0).getReg(), MRI,
        m_GAnd(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::ANDAbsIdx;
    break;
  case MC6809::G_XOR:
    Success = mi_match(
        MI.getOperand(0).getReg(), MRI,
        m_GXor(m_Reg(LHS),
               m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::EORAbsIdx;
    break;
  case MC6809::G_OR:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GOr(m_Reg(LHS), m_all_of(m_MInstr(Load),
                                            m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::ORAAbsIdx;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable(
          "Could not constrain absolute indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Offset;
  switch (MI.getOpcode()) {
  case MC6809::G_AND:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAnd(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::ANDIndirIdx;
    break;
  case MC6809::G_XOR:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GXor(m_Reg(LHS),
                        m_all_of(m_MInstr(Load),
                                 m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::EORIndirIdx;
    break;
  case MC6809::G_OR:
    Success =
        mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GOr(m_Reg(LHS),
                       m_all_of(m_MInstr(Load),
                                m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::ORAIndirIdx;
    break;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addUse(LHS)
                     .addUse(IndirAddr)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable(
          "Could not constrain absolute indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }
#endif
  return false;
}

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();
  MachineIRBuilder Builder(MI);

  Builder.buildCopy(Dst, Src);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto Instr =
      Builder
          .buildInstr(MC6809::LDImm16, {MI.getOperand(0), &MC6809::INDEX16RegClass}, {})
          .add(MI.getOperand(1));
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

template <typename ADDR_P, typename CARRYIN_P> struct GShlE_match {
  Register &CarryOut;
  ADDR_P Addr;
  CARRYIN_P CarryIn;

  GShlE_match(Register &CarryOut, const ADDR_P &Addr, const CARRYIN_P &CarryIn)
      : CarryOut(CarryOut), Addr(Addr), CarryIn(CarryIn) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *GShlE = getOpcodeDef(MC6809::G_SHLE, Reg, MRI);
    if (!GShlE)
      return false;
    CarryOut = GShlE->getOperand(1).getReg();
    return Addr.match(MRI, GShlE->getOperand(2).getReg()) &&
           CarryIn.match(MRI, GShlE->getOperand(3).getReg());
  }
};

template <typename ADDR_P, typename CARRYIN_P>
GShlE_match<ADDR_P, CARRYIN_P> m_GShlE(Register &CarryOut, const ADDR_P &Addr,
                                       const CARRYIN_P &CarryIn) {
  return {CarryOut, Addr, CarryIn};
}

template <typename ADDR_P, typename CARRYIN_P> struct GLshrE_match {
  Register &CarryOut;
  ADDR_P Addr;
  CARRYIN_P CarryIn;

  GLshrE_match(Register &CarryOut, const ADDR_P &Addr, const CARRYIN_P &CarryIn)
      : CarryOut(CarryOut), Addr(Addr), CarryIn(CarryIn) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *GLshrE = getOpcodeDef(MC6809::G_LSHRE, Reg, MRI);
    if (!GLshrE)
      return false;
    CarryOut = GLshrE->getOperand(1).getReg();
    return Addr.match(MRI, GLshrE->getOperand(2).getReg()) &&
           CarryIn.match(MRI, GLshrE->getOperand(3).getReg());
  }
};

template <typename ADDR_P, typename CARRYIN_P>
GLshrE_match<ADDR_P, CARRYIN_P> m_GLshrE(Register &CarryOut, const ADDR_P &Addr,
                                         const CARRYIN_P &CarryIn) {
  return {CarryOut, Addr, CarryIn};
}

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
#if 0
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstr *Load;

  // Read-modify-write instruction patterns are rooted at store instructions, so
  // select one if possible. This can make an entire instruction sequence dead.
  if (MI.getOpcode() == MC6809::G_STORE_ABS) {
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                        m_SpecificICst(1))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MC6809::INCAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                        m_SpecificICst(-1))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MC6809::DECAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }
    Register CarryOut;
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                         m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Asl = Builder.buildInstr(MC6809::ASLAbs, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Asl, CarryOut, Asl.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                          m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Lsr = Builder.buildInstr(MC6809::LSRAbs, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Lsr, CarryOut, Lsr.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Lsr, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryIn;
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GShlE(CarryOut,
                         m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                         m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Rol = Builder.buildInstr(MC6809::ROLAbs, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Rol, CarryOut, Rol.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI,
                 m_GLshrE(CarryOut,
                          m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)),
                          m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Ror = Builder.buildInstr(MC6809::RORAbs, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Ror, CarryOut, Ror.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Ror, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
  } else if (MI.getOpcode() == MC6809::G_STORE_ABS_IDX) {
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    Register Idx;
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                   m_SpecificICst(1))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Inc = Builder.buildInstr(MC6809::INCAbsIdx)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      if (!constrainSelectedInstRegOperands(*Inc, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                   m_SpecificICst(-1))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Inc = Builder.buildInstr(MC6809::DECAbsIdx)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      if (!constrainSelectedInstRegOperands(*Inc, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryOut;
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GShlE(CarryOut,
                    m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                    m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Asl = Builder.buildInstr(MC6809::ASLAbsIdx, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Asl, CarryOut, Asl.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GLshrE(CarryOut,
                     m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                     m_SpecificICst(0))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Lsr = Builder.buildInstr(MC6809::LSRAbsIdx, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Lsr, CarryOut, Lsr.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Lsr, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    Register CarryIn;
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GShlE(CarryOut,
                    m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                    m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Rol = Builder.buildInstr(MC6809::ROLAbsIdx, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Rol, CarryOut, Rol.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(
            MI.getOperand(0).getReg(), MRI,
            m_GLshrE(CarryOut,
                     m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)),
                     m_Reg(CarryIn))) &&
        Addr.isIdenticalTo(MI.getOperand(1)) &&
        Idx == MI.getOperand(2).getReg()) {
      auto Ror = Builder.buildInstr(MC6809::RORAbsIdx, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .addUse(Idx)
                     .addUse(CarryIn)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Ror, CarryOut, Ror.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Ror, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
  }

  // If this isn't a STZ, emit a store pseudo.
  if (!STI.has65C02() ||
      !isOperandImmEqual(MI.getOperand(0), 0, *Builder.getMRI()))
    return selectGeneric(MI);

  // STZ

  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MC6809::G_STORE_ABS:
    Opcode = MC6809::STZAbs;
    break;
  case MC6809::G_STORE_ABS_IDX:
    Opcode = MC6809::STZAbsIdx;
    break;
  }

  MI.setDesc(TII.get(Opcode));
  MI.RemoveOperand(0);
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
    return false;
  return true;
#else
  return false;
#endif
}

bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, MRI);
  if (LoConst && HiConst) {
    uint64_t Val =
        HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
    auto Instr =
        Builder.buildInstr(MC6809::LDImm16, {Dst, &MC6809::ACC16RegClass}, {Val});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectConstant(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = Builder.getMRI()->getType(Dst);

  if (MI.getOperand(1).isCImm()) {
    uint64_t Val = MI.getOperand(1).getCImm()->getZExtValue();
    MI.getOperand(1).ChangeToImmediate(Val);
  }

  unsigned Opcode;
  if (DstTy == S8) {
    Opcode = MC6809::LDImm8;
  } else if (DstTy == S16) {
    Opcode = MC6809::LDImm16;
  } else if (DstTy == S32) {
    Opcode = MC6809::LDImm32;
  } else {
    llvm_unreachable("Can't select G_CONSTANT, unsupported type.");
  }

  MI.setDesc(TII.get(Opcode));
  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
}

bool MC6809InstructionSelector::selectLoad(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););

  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = Builder.getMRI()->getType(Dst);
  Register MemOp = MI.getOperand(1).getReg();
  if (DstTy == S8) {
    auto Instr = Builder.buildInstr(MC6809::LD8Idx, {Dst}, {MemOp});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
  } else if (DstTy == S16) {
    auto Instr = Builder.buildInstr(MC6809::LD16Idx, {Dst}, {MemOp});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
  } else if (DstTy == S32) {
    auto Instr = Builder.buildInstr(MC6809::LD32Idx, {Dst}, {MemOp});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
  }
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
  return true;
}

bool MC6809InstructionSelector::selectLshrShlE(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  unsigned ShiftOpcode, RotateOpcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Unexpected opcode.");
  case MC6809::G_SHLE:
    ShiftOpcode = MC6809::ASL;
    RotateOpcode = MC6809::ROL;
    break;
  case MC6809::G_LSHRE:
    ShiftOpcode = MC6809::LSR;
    RotateOpcode = MC6809::ROR;
    break;
  }

  MachineIRBuilder Builder(MI);
  if (mi_match(CarryIn, *Builder.getMRI(), m_SpecificICst(0))) {
    auto Asl = Builder.buildInstr(ShiftOpcode, {Dst, CarryOut}, {Src});
    if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
      return false;
  } else {
    auto Rol =
        Builder.buildInstr(RotateOpcode, {Dst, CarryOut}, {Src, CarryIn});
    if (!constrainSelectedInstRegOperands(*Rol, TII, TRI, RBI))
      return false;
  }
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectTrunc(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  LLT S16 = LLT::scalar(16);
  LLT S8 = LLT::scalar(8);
  LLT S1 = LLT::scalar(1);

  Register From = MI.getOperand(1).getReg();
  Register To = MI.getOperand(0).getReg();

  LLT FromType = Builder.getMRI()->getType(From);
  LLT ToType = Builder.getMRI()->getType(To);
  assert(FromType == S16 && ToType == S1);

  MachineInstrSpan MIS(MI, MI.getParent());
  MI.getOperand(1).setReg(Builder.buildTrunc(S8, From).getReg(0));
  selectAll(MIS);
  return true;
}

bool MC6809InstructionSelector::selectAddE(MachineInstr &MI) {
#if 0
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  LLT S1 = LLT::scalar(1);

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      assert(RConst->Value.getBitWidth() == 8);
      return Builder.buildInstr(MC6809::ADCImm, {Result, CarryOut, S1},
                                {L, RConst->Value.getZExtValue(), CarryIn});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      return Builder.buildInstr(MC6809::ADCAe)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr)
          .addUse(CarryIn);
    }
    Register Idx;
    if (mi_match(R, MRI, m_FoldedLdIdx(MI, Idx, Addr, AA))) {
      return Builder.buildInstr(MC6809::ADCAi_o16)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr)
          .addUse(Idx)
          .addUse(CarryIn);
    }
    Register Offset;
    if (mi_match(R, MRI, m_FoldedLdIndirIdx(MI, Idx, Offset, AA))) {
      return Builder.buildInstr(MC6809::ADCAi_o16I)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(Offset)
          .addUse(Idx)
          .addUse(CarryIn);
    }
    llvm_unreachable("Could not match ADC instruction.");
  }();
  if (MI.getOpcode() == MC6809::G_SADDE) {
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  } else
    assert(MI.getOpcode() == MC6809::G_UADDE);
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  return true;
#else
  return false;
#endif
}

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););
  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  MachineIRBuilder Builder(MI);

  MachineInstrBuilder LoCopy;
  MachineInstrBuilder HiCopy;
  LoCopy = Builder.buildCopy(Lo, Src);
  LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_byte);
  HiCopy = Builder.buildCopy(Hi, Src);
  HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_byte);
  constrainGenericOp(*LoCopy);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectGeneric(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););
    llvm_unreachable("Unexpected opcode.");
  case MC6809::G_SEXT:
    // XXXX: FIXME: MarkM - Check the sze and maybe use SEX32Implicit
    Opcode = MC6809::SEX16Implicit;
    break;
  case MC6809::G_BRINDIRECT:
    Opcode = MC6809::JMPIndir;
    break;
  case MC6809::G_IMPLICIT_DEF:
    Opcode = MC6809::IMPLICIT_DEF;
    break;
  case MC6809::G_PHI:
    Opcode = MC6809::PHI;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MI.getMF());
  // Establish any tied operands and known register classes.
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
    return false;
  // Make sure that the outputs have register classes.
  constrainGenericOp(MI);
  return true;
}

// Produce a pointer vreg from a low and high vreg pair.
void MC6809InstructionSelector::composePtr(MachineIRBuilder &Builder, Register Dst,
                                        Register Lo, Register Hi) {
  auto RegSeq = Builder.buildInstr(MC6809::REG_SEQUENCE)
                    .addDef(Dst)
                    .addUse(Lo)
                    .addImm(MC6809::sub_lo_byte)
                    .addUse(Hi)
                    .addImm(MC6809::sub_hi_byte);
  constrainGenericOp(*RegSeq);
}

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register
// is defined exactly once, making sure definitions are constrained suffices.
void MC6809InstructionSelector::constrainGenericOp(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.operands()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 10 : OP = " << Op << "\n";);
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() ||
        MRI.getRegClassOrNull(Op.getReg()))
      continue;
    Register Reg = Op.getReg();
    constrainOperandRegClass(Op, getRegClassForType(Reg, MRI));
  }
}

void MC6809InstructionSelector::constrainOperandRegClass(
    MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, MRI, TII, RBI, MI,
                                              RegClass, RegMO));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

bool MC6809InstructionSelector::selectAll(MachineInstrSpan MIS) {
  MachineRegisterInfo &MRI = MIS.begin()->getMF()->getRegInfo();

  // Ensure that all new generic virtual registers have a register bank.
  for (MachineInstr &MI : MIS) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : MI = "; MI.dump(););
    for (MachineOperand &MO : MI.operands()) {
      if (!MO.isReg())
        continue;
      Register Reg = MO.getReg();
      if (!MO.getReg().isVirtual())
        continue;
      if (MRI.getRegClassOrNull(MO.getReg()))
        continue;
      auto *RC = MRI.getRegClassOrNull(MO.getReg());
      MRI.setRegBank(Reg, RBI.getRegBankFromRegClass(*RC, LLT()));
    }
  }

  // Select instructions in reverse block order.
  for (MachineInstr &MI : make_early_inc_range(mbb_reverse(MIS))) {
    // We could have folded this instruction away already, making it dead.
    // If so, erase it.
    if (isTriviallyDead(MI, MRI)) {
      MI.eraseFromParent();
      continue;
    }

    if (!select(MI))
      return false;
  }
  return true;
}

InstructionSelector *llvm::createMC6809InstructionSelector(
    const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI) {
  return new MC6809InstructionSelector(TM, STI, RBI);
}
