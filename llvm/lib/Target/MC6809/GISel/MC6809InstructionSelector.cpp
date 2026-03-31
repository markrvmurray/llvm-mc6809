//===-- MC6809InstructionSelector.cpp - MC6809 Instruction Selector -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 instruction selector. Instructions selected here
// are abstract pseudo-instructions which will allow register allocation to be
// applied later.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstructionSelector.h"
#include "MC6809RegisterBankInfo.h"

#include <set>

#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutorImpl.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/InstrTypes.h"
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
  MC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI);

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

  void setupMF(MachineFunction &MF, GISelValueTracking *VT, CodeGenCoverage *CovInfo, ProfileSummaryInfo *PSI, BlockFrequencyInfo *BFI, AAResults *AA) override;

private:
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo &TRI;
  const MC6809RegisterBankInfo &RBI;

  MachineBasicBlock *MBB;
  MachineFunction *MF;
  MachineRegisterInfo *MRI;
  MachineIRBuilder MIB;

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectFrameIndex(MachineInstr &MI);
  // bool selectAddr(MachineInstr &MI);
  //bool selectPtrAdd(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);
  // bool selectConstant(MachineInstr &MI);
  // bool selectStore(MachineInstr &MI);
  // bool selectLoad(MachineInstr &MI);
  // bool selectTrunc(MachineInstr &MI);

  // bool selectExt(MachineInstr &MI);
  bool selectAddO(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectSubO(MachineInstr &MI);
  bool selectSubE(MachineInstr &MI);
  // bool selectMul(MachineInstr &MI);
  // bool selectMulExpand(MachineInstr &MI);
  // bool selectMulH(MachineInstr &MI);

  // bool selectBranch(MachineInstr &MI);
  // bool selectBrCondImm(MachineInstr &MI);
  // bool selectConditionalBranch(MachineInstr &MI, MachineFunction &MF, MachineRegisterInfo &MRI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass);

  // Select all instructions in a given span, recursively. Allows selecting an
  // instruction sequence by reducing it to a more easily selectable sequence.
  bool selectAll(MachineInstrSpan MIS);

  /// tblgenerated 'select' implementation, used as the initial selector for
  /// the patterns that don't require complex C++.
  bool selectImpl(MachineInstr &MI, CodeGenCoverage &CoverageInfo) const;

  // MachineInstr *tryFoldIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIRBuilder) const;
  // bool tryOptAndIntoCompareBranch(MachineInstr &AndInst, bool Invert, MachineBasicBlock *DstMBB, MachineIRBuilder &MIB) const;
  // bool tryOptCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const;
  // MachineInstr *emitCMN(MachineOperand &LHS, MachineOperand &RHS, MachineIRBuilder &MIRBuilder) const;
  // MachineInstr *emitIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIRBuilder) const;
  // bool selectCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const;
  // MachineInstr *emitTestBit(Register TestReg, uint64_t Bit, bool IsNegative, MachineBasicBlock *DstMBB, MachineIRBuilder &MIB) const;

  LLT S1 = LLT::scalar(1);
  LLT S2 = LLT::scalar(2);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT P = LLT::pointer(0, 16);

  ComplexRendererFns selectLSIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectLSUnmergeIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectLSFrameIndex(MachineOperand &Root) const;
  ComplexRendererFns selectAMImmediate(MachineOperand &Root) const;
  ComplexRendererFns selectAMIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectAMUnmergeIndexedImmOffset(MachineOperand &Root) const;

#define GET_GLOBALISEL_PREDICATES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_DECL

#define GET_GLOBALISEL_TEMPORARIES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_DECL
};

void MC6809InstructionSelector::setupMF(MachineFunction &MF,
                                     GISelValueTracking *VT,
                                     CodeGenCoverage *CovInfo,
                                     ProfileSummaryInfo *PSI,
                                     BlockFrequencyInfo *BFI, AAResults *AA) {
  InstructionSelector::setupMF(MF, VT, CovInfo, PSI, BFI, AA);

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

} // namespace

#define GET_GLOBALISEL_IMPL
#include "MC6809GenGlobalISel.inc"
#include "llvm/Support/FormatVariadic.h"
#undef GET_GLOBALISEL_IMPL

MC6809InstructionSelector::MC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI)
    : STI(STI), TII(*STI.getInstrInfo()), TRI(*STI.getRegisterInfo()), RBI(RBI),
#define GET_GLOBALISEL_PREDICATES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_INIT
#define GET_GLOBALISEL_TEMPORARIES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_INIT
{
}

/// Select a "register plus signed immediate offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMImmediate(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_CONSTANT) {
    return {{
        [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
    }};
  }

  return std::nullopt;
}

/// Select a "frame_index plus offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_LOAD || RootDef->getOpcode() == TargetOpcode::G_STORE) {
    MachineInstr *FrameDef = MRI.getVRegDef(RootDef->getOperand(1).getReg());
    if (FrameDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(FrameDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
      }};
    }
  }

  if (!isBaseWithConstantOffset(Root, MRI))
    return std::nullopt;

  return std::nullopt;
}

/// Select a "high word unmerged frame index plus offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMUnmergeIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_UNMERGE_VALUES) {
    unsigned mergeoffset = Root.getReg() == RootDef->getOperand(0).getReg() ? 0 : 2;
    MachineInstr *LoadDef = MRI.getVRegDef(RootDef->getOperand(2).getReg());
    if (LoadDef->getOpcode() == TargetOpcode::G_LOAD) {
      MachineInstr *FrameDef = MRI.getVRegDef(LoadDef->getOperand(1).getReg());
      if (FrameDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
        return {{
            [=](MachineInstrBuilder &MIB) { MIB.add(FrameDef->getOperand(1)); },
            [=](MachineInstrBuilder &MIB) { MIB.addImm(mergeoffset); },
        }};
      }
    }
  }

  return std::nullopt;
}

/// Select a "register plus signed immediate offset" address for a target load/store instruction
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectLSIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  auto RootDef = MRI.getVRegDef(Root.getReg());

  if (RootDef->getOpcode() == TargetOpcode::G_PTR_ADD) {
    auto OffsetDef = MRI.getVRegDef(RootDef->getOperand(2).getReg());
    if (OffsetDef->getOpcode() == TargetOpcode::G_CONSTANT) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.add(OffsetDef->getOperand(1)); },
      }};
    }
  }

  return std::nullopt;
}

/// Select a "FrameIndex + immediate offset" address for a target load/store instruction
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectLSFrameIndex(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  auto RootDef = MRI.getVRegDef(Root.getReg());

  if (RootDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
    return {{
        [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
        [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
    }};
  }

  return std::nullopt;
}

static bool shouldFoldMemAccess(const MachineInstr &Dst, const MachineInstr &Src, AAResults *AA) {
  // assert(Src.mayLoadOrStore());

  // For now, don't attempt to fold across basic block boundaries.
  if (Dst.getParent() != Src.getParent())
    return false;

  //if ((*Src.memoperands_begin())->isVolatile())
  //  return false;

  // Look for intervening instructions that cannot be folded across.
  for (const MachineInstr &I : make_range(std::next(MachineBasicBlock::const_iterator(Src)), MachineBasicBlock::const_iterator(Dst))) {
    if (I.isCall() || I.hasUnmodeledSideEffects())
      return false;
    if (I.mayLoadOrStore()) {
      if (Src.hasOrderedMemoryRef() || I.hasOrderedMemoryRef())
        return false;
      if (I.mayAlias(AA, Src, /*UseTBAA=*/true))
        return false;
      // Note: Dst may be a store, indicating that the whole sequence is a RMW
      // operation.
      if (I.mayAlias(AA, Dst, /*UseTBAA=*/true))
        return false;
    }
  }

  return true;
}

struct FoldedLdIdx_match {
  const MachineInstr &Tgt;
  MachineOperand &Ptr;
  MachineOperand &Offset;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *Unmerge;
    const MachineInstr *Load;
    const MachineInstr *FrameIndex;
    Unmerge = getOpcodeDef(MC6809::G_UNMERGE_VALUES, Reg, MRI);
    if (Unmerge) {
      Load = getOpcodeDef(MC6809::G_LOAD, Unmerge->getOperand(2).getReg(), MRI);
      if (Load) {
        FrameIndex = getOpcodeDef(MC6809::G_FRAME_INDEX, Load->getOperand(1).getReg(), MRI);
        if (FrameIndex) {
          Ptr = FrameIndex->getOperand(1);
          const LLT Ty = MRI.getType(Unmerge->getOperand(2).getReg());
          const unsigned TySize = Ty.getSizeInBits();
          if (TySize == 32) {
            if (Reg == Unmerge->getOperand(0).getReg())
              Offset = MachineOperand::CreateImm(0);
            else
              Offset = MachineOperand::CreateImm(2);
          } else if (TySize == 16) {
            if (Reg == Unmerge->getOperand(0).getReg())
              Offset = MachineOperand::CreateImm(0);
            else
              Offset = MachineOperand::CreateImm(1);
          } else
            llvm_unreachable("Impossible unmerge size");
          return true;
        }
      }
    }
    Load = getOpcodeDef(MC6809::G_LOAD, Reg, MRI);
    if (Load) {
      FrameIndex = getOpcodeDef(MC6809::G_FRAME_INDEX, Load->getOperand(1).getReg(), MRI);
      if (FrameIndex) {
        if (!shouldFoldMemAccess(Tgt, *FrameIndex, AA))
          return false;
        Ptr = FrameIndex->getOperand(1);
        Offset = MachineOperand::CreateImm(0);
        return true;
      }
#if 0
      const MachineInstr *PtrAdd = getOpcodeDef(MC6809::G_PTR_ADD, Load->getOperand(1).getReg(), MRI);
      if (PtrAdd) {
        if (!shouldFoldMemAccess(Tgt, *PtrAdd, AA))
          return false;
        Ptr = PtrAdd->getOperand(1);
        Offset = PtrAdd->getOperand(2);
        return true;
      }
#endif
    }
    return false;
  }
};

inline FoldedLdIdx_match m_FoldedLdIdx(const MachineInstr &Tgt, MachineOperand &Ptr, MachineOperand &Offset, AAResults *AA) {
  return {Tgt, Ptr, Offset, AA};
}

// Returns the widest register class that can contain values of a given type.
// Used to ensure that every virtual register gets some register class by the
// time register allocation completes.
static const TargetRegisterClass &getRegClassForType(LLT Ty) {
  if (Ty == LLT::pointer(0, 16)) {
    return MC6809::INDEX16RegClass;
  } else {
    switch (Ty.getSizeInBits()) {
    default:
      llvm_unreachable("Invalid type size.");
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
}

// Version that checks register bank for s16 values — returns INDEX16
// for values in the index bank (e.g., args/returns passed in X).
static const TargetRegisterClass &getRegClassForTypeOnBank(
    LLT Ty, const RegisterBank *RB) {
  if (Ty.getSizeInBits() == 16 && RB &&
      RB->getID() == MC6809::INDEXRegBankID) {
    return MC6809::INDEX16RegClass;
  }
  return getRegClassForType(Ty);
}
bool MC6809InstructionSelector::select(MachineInstr &MI) {
  assert(MI.getParent() && "Instruction should be in a basic block!");
  assert(MI.getParent()->getParent() && "Instruction should be in a function!");

  MBB = MI.getParent();
  MF = MBB->getParent();
  MRI = &MF->getRegInfo();

  // isPreISelOpcode is stolen from llvm-mos. Methinks it means "not a GlobalISel opcode".
  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

  if (selectImpl(MI, *CoverageInfo)) {
    return true;
  }

  switch (MI.getOpcode()) {
  default:
    return false;

  case TargetOpcode::G_FRAME_INDEX:
    return selectFrameIndex(MI);
#if 0
  case TargetOpcode::G_BLOCK_ADDR:
  case TargetOpcode::G_GLOBAL_VALUE:
    return selectAddr(MI);
#endif
  case TargetOpcode::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case TargetOpcode::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);
#if 0
  case TargetOpcode::G_TRUNC:
    return selectTrunc(MI);
#endif

#if 0
  case TargetOpcode::G_LOAD:
    return selectLoad(MI);
  case TargetOpcode::G_STORE:
    return selectStore(MI);
  case TargetOpcode::G_PTR_ADD:
    return selectPtrAdd(MI);
  case TargetOpcode::G_CONSTANT:
    return selectConstant(MI);

  case TargetOpcode::G_BRCOND:
    return selectBrCondImm(MI);

  case TargetOpcode::G_BR:
    return selectBranch(MI);

  case TargetOpcode::G_BRINDIRECT:
    return selectGeneric(MI);

  case MC6809::G_BRCOND_IMM:
    return selectBrCondImm(MI);

  case TargetOpcode::G_IMPLICIT_DEF:
#endif
  case TargetOpcode::G_PHI:
    return selectGeneric(MI);

#if 0
  case TargetOpcode::G_SEXT:
  case TargetOpcode::G_ZEXT:
  case TargetOpcode::G_ANYEXT:
    return selectExt(MI);
#endif

#if 0
  case TargetOpcode::G_ADD:
    return selectAddE(MI);
#endif

  case TargetOpcode::G_ADD:
  case TargetOpcode::G_SUB: {
    // Handle INDEX-bank i16 add/sub via LEA.
    // GlobalISel puts constants in registers (not bare immediates),
    // so TableGen patterns can't match — hand-lowering required.
    // Check DstReg for INDEX bank OR INDEX16 class (intermediate results
    // in multi-add chains may already have a class from earlier selection).
    Register DstReg = MI.getOperand(0).getReg();
    const RegisterBank *RB = MRI->getRegBankOrNull(DstReg);
    bool IsIndex = (RB && RB->getID() == MC6809::INDEXRegBankID);
    if (!IsIndex) {
      const TargetRegisterClass *RC = MRI->getRegClassOrNull(DstReg);
      IsIndex = RC && MC6809::INDEX16RegClass.hasSubClassEq(RC);
    }
    if (IsIndex) {
      Register Src1 = MI.getOperand(1).getReg();
      Register Src2 = MI.getOperand(2).getReg();
      // Look through COPYs to find the defining G_CONSTANT.
      Register Src2Origin = Src2;
      MachineInstr *Src2Def = MRI->getVRegDef(Src2Origin);
      while (Src2Def && Src2Def->getOpcode() == TargetOpcode::COPY) {
        Src2Origin = Src2Def->getOperand(1).getReg();
        if (!Src2Origin.isVirtual()) break;
        Src2Def = MRI->getVRegDef(Src2Origin);
      }
      if (Src2Def && Src2Def->getOpcode() == TargetOpcode::G_CONSTANT) {
        int64_t Offset;
        if (Src2Def->getOperand(1).isCImm())
          Offset = Src2Def->getOperand(1).getCImm()->getSExtValue();
        else
          Offset = Src2Def->getOperand(1).getImm();
        if (MI.getOpcode() == TargetOpcode::G_SUB)
          Offset = -Offset;
        auto LEA = BuildMI(*MI.getParent(), MI, MI.getDebugLoc(),
                           TII.get(MC6809::LEAPtrAdd_Imm), DstReg)
                       .addReg(Src1)
                       .addImm(Offset);
        constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        MI.eraseFromParent();
        return true;
      }

      // INDEX + INDEX (non-constant): route through ACCUM (D).
      // The 6809 has no "add two index registers" instruction.
      // Rewrite: COPY src1→D, COPY src2→D2, Push D2, Add/Sub_i16_Pull, COPY D→result.
      {
        MachineBasicBlock &MBB = *MI.getParent();
        DebugLoc DL = MI.getDebugLoc();
        bool IsSub = (MI.getOpcode() == TargetOpcode::G_SUB);

        LLT s16 = LLT::scalar(16);

        // Constrain the original INDEX-bank operands to INDEX16 register class.
        RBI.constrainGenericRegister(Src1, MC6809::INDEX16RegClass, *MRI);
        RBI.constrainGenericRegister(Src2, MC6809::INDEX16RegClass, *MRI);
        RBI.constrainGenericRegister(DstReg, MC6809::INDEX16RegClass, *MRI);

        if (!IsSub) {
          // ADD: copy one operand to D, then LEAX D,X (single instruction).
          Register AccSrc = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccSrc, &MC6809::ACC16RegClass);
          BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), AccSrc).addReg(Src2);
          auto LEA = BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Reg16), DstReg)
              .addReg(Src1)
              .addReg(AccSrc);
          constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        } else {
          // SUB: compute -b as (0 - b), then LEAX D,X.
          // Trace Src2 back to its defining load to get the memory operand,
          // so we can SUBD directly from memory: LDD #0; SUBD offset,S; LEAX D,X.
          Register AccZero = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccZero, &MC6809::ACC16RegClass);
          Register AccNegB = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccNegB, &MC6809::ACC16RegClass);

          // Find the memory source of Src2 by looking through COPYs.
          Register Src2Origin = Src2;
          MachineInstr *Src2Load = MRI->getVRegDef(Src2Origin);
          while (Src2Load && Src2Load->getOpcode() == TargetOpcode::COPY) {
            Src2Origin = Src2Load->getOperand(1).getReg();
            if (!Src2Origin.isVirtual()) break;
            Src2Load = MRI->getVRegDef(Src2Origin);
          }

          // D = 0.
          auto Zero = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Imm), AccZero)
              .addImm(0);
          constrainSelectedInstRegOperands(*Zero, TII, TRI, RBI);

          if (Src2Load && Src2Load->getOpcode() == MC6809::Load_i16_Mem) {
            // SUBD directly from memory: SUBD offset,index.
            auto SubMem = BuildMI(MBB, MI, DL, TII.get(MC6809::Sub_i16_Mem), AccNegB)
                .addReg(AccZero)
                .add(Src2Load->getOperand(1))  // index register
                .add(Src2Load->getOperand(2)); // offset
            constrainSelectedInstRegOperands(*SubMem, TII, TRI, RBI);
          } else {
            // Fallback: push and pull.
            Register AccSrc2 = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(AccSrc2, &MC6809::ACC16RegClass);
            Register StackReg = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(StackReg, &MC6809::STACK16RegClass);
            BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), AccSrc2).addReg(Src2);
            auto Push = BuildMI(MBB, MI, DL, TII.get(MC6809::Push_i16), StackReg)
                .addReg(AccSrc2);
            constrainSelectedInstRegOperands(*Push, TII, TRI, RBI);
            auto Sub = BuildMI(MBB, MI, DL, TII.get(MC6809::Sub_i16_Pull), AccNegB)
                .addReg(AccZero)
                .addReg(StackReg);
            constrainSelectedInstRegOperands(*Sub, TII, TRI, RBI);
          }

          // LEA: result = src1 + (-b).
          auto LEA = BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Reg16), DstReg)
              .addReg(Src1)
              .addReg(AccNegB);
          constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        }
        MI.eraseFromParent();
        return true;
      }
    }
    return false;  // Fall through to selectImpl for ACCUM-bank (ADDD/SUBD).
  }

  // G_ICMP: INDEX-bank CMPX/CMPY selection deferred — requires post-RA
  // peephole to replace TFR X,D + CMPD with CMPX. The compare opcode
  // maps (CompareImmediateOpcode) are ready with IX/IY/SU/SS entries.

  case TargetOpcode::G_SADDO:
  case TargetOpcode::G_UADDO:
    return selectAddO(MI);

  case TargetOpcode::G_USUBO:
  case TargetOpcode::G_SSUBO:
    return selectSubO(MI);

  case TargetOpcode::G_SADDE:
  case TargetOpcode::G_UADDE:
    return selectAddE(MI);

  case TargetOpcode::G_USUBE:
  case TargetOpcode::G_SSUBE:
    return selectSubE(MI);

  // G_MUL/G_UMULH/G_SMULH i8: handled by TableGen patterns via
  // REG_SEQUENCE + MUL_D + EXTRACT_SUBREG (no hand-lowering needed).
  }
  return false;
}

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto Instr = Builder.buildInstr(MC6809::LEA_Ptr_Imm).add(MI.getOperand(0))
                   .add(MI.getOperand(1))
                   .addImm(0);
  constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
  MI.eraseFromParent();
  return true;
}

#if 0
bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  MI.setDesc(TII.get(MC6809::Load_iPtr_Imm));
  MI.addImplicitDefUseOperands(*MF);
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  return Success;
}
#endif

#if 0
bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  auto [Dst, Lo, Hi] = MI.getFirst3Regs();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, MRI);
  if (LoConst && HiConst) {
    uint64_t Val =
        HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
    auto Instr = STI.hasSPC700()
                     ? Builder.buildInstr(MOS::LDImm16SPC700, {Dst}, {Val})
                     : Builder.buildInstr(MOS::LDImm16, {Dst, &MOS::GPRRegClass}, {Val});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
  return true;
}
#endif

bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, *MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, *MRI);
  const unsigned Size = MRI->getType(Dst).getSizeInBits();
  if (LoConst && HiConst) {
    if (Size == 16) {
      uint64_t Val = HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load_i16_Imm).addDef(Dst).addImm(Val);
      Instr->addImplicitDefUseOperands(*MF);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    } else if (Size == 32) {
      uint64_t Val = HiConst->Value.getZExtValue() << 16 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load_i32_Imm).addDef(Dst).addImm(Val);
      Instr->addImplicitDefUseOperands(*MF);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    }
    MI.eraseFromParent();
    return true;
  }
  auto RegSeq = Builder.buildInstr(MC6809::REG_SEQUENCE).addDef(Dst);
  if (Size == 16)
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_byte).addUse(Hi).addImm(MC6809::sub_hi_byte);
  else
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_word).addUse(Hi).addImm(MC6809::sub_hi_word);
  RegSeq->addImplicitDefUseOperands(*MF);
  constrainGenericOp(*RegSeq);
  MI.eraseFromParent();
  return true;
}

#if 0
bool MC6809InstructionSelector::selectPtrAdd(MachineInstr &MI) {

  unsigned Opcode;
  if (MI.getOperand(2).isCImm()) {
    uint64_t Val = MI.getOperand(2).getCImm()->getSExtValue();
    MI.getOperand(2).ChangeToImmediate(Val);
  } else if (MI.getOperand(2).isReg()) {
    const MachineInstr *GConst = getOpcodeDef(TargetOpcode::G_CONSTANT, MI.getOperand(2).getReg(), *MRI);
    if (GConst) {
      if (GConst->getOperand(1).isCImm())
        MI.getOperand(2).ChangeToImmediate(GConst->getOperand(1).getCImm()->getSExtValue());
      else
        MI.getOperand(2).ChangeToImmediate(GConst->getOperand(1).getImm());
    }
  }
  Opcode = MC6809::LEAPtrAdd_Imm;
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MF);
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  return Success;
}
#endif

bool MC6809InstructionSelector::selectAddO(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_GCst(ValReg))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = DstSize == 8 ? MC6809::AddSetCarry_i8_Imm : MC6809::AddSetCarry_i16_Imm;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addImm(Value);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA))));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::AddSetCarry_i8_Mem : MC6809::AddSetCarry_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(LHS), m_Reg(RHS))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(LHS), m_Reg(RHS)));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::AddSetCarry_i8_Reg : MC6809::AddSetCarry_i16_Reg;
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addDef(CarryOut)
                .addUse(LHS)
                .addUse(RHS);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      MachineInstr *Def1 = MRI->getVRegDef(MI.getOperand(2).getReg());
      MachineInstr *Def2 = MRI->getVRegDef(MI.getOperand(3).getReg());
  );

  return false;
}

bool MC6809InstructionSelector::selectSubO(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_GCst(ValReg))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_GCst(ValReg))) ||
            mi_match(Dst, *MRI, m_GSSubO(m_GCst(ValReg), m_Reg(Reg))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_GCst(ValReg), m_Reg(Reg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = DstSize == 8 ? MC6809::SubSetCarry_i8_Imm : MC6809::SubSetCarry_i16_Imm;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addImm(Value);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)))) ||
            mi_match(Dst, *MRI, m_GSSubO(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Reg))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Reg)));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::SubSetCarry_i8_Mem : MC6809::SubSetCarry_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(LHS), m_Reg(RHS))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(LHS), m_Reg(RHS)));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::SubSetCarry_i8_Reg : MC6809::SubSetCarry_i16_Reg;
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addDef(CarryOut)
                .addUse(LHS)
                .addUse(RHS);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      MachineInstr *Def1 = MRI->getVRegDef(MI.getOperand(2).getReg());
      MachineInstr *Def2 = MRI->getVRegDef(MI.getOperand(3).getReg());
  );

  return false;
}

bool MC6809InstructionSelector::selectAddE(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Carry;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  assert((DstSize == 8 || DstSize ==16) && "Only 8- and 16-bit adds exist");
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = DstSize == 8 ? MC6809::AddSetCarryUse_i8_Imm : MC6809::AddSetCarryUse_i16_Imm;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .addImm(Value);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::AddSetCarryUse_i8_Mem : MC6809::AddSetCarryUse_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::AddSetCarryUse_i8_Mem : MC6809::AddSetCarryUse_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry)));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::AddSetCarryUse_i8_Reg : MC6809::AddSetCarryUse_i16_Reg;
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addDef(CarryOut)
                .addUse(LHS)
                .addUse(Carry)
                .addUse(RHS);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      MachineInstr *Def1 = MRI->getVRegDef(MI.getOperand(2).getReg());
      MachineInstr *Def2 = MRI->getVRegDef(MI.getOperand(3).getReg());
      );

  return false;
}

bool MC6809InstructionSelector::selectSubE(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut, Carry;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  assert((DstSize == 8 || DstSize ==16) && "Only 8- and 16-bit adds exist");
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  CarryOut = MI.getOperand(1).getReg();

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GSSubE(m_GCst(ValReg), m_Reg(Reg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_GCst(ValReg), m_Reg(Reg), m_Reg(Carry)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = DstSize == 8 ? MC6809::SubSetCarryUse_i8_Imm : MC6809::SubSetCarryUse_i16_Imm;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .addImm(Value);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GSSubE(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Reg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Reg), m_Reg(Carry)));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::SubSetCarryUse_i8_Mem : MC6809::SubSetCarryUse_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = DstSize == 8 ? MC6809::SubSetCarryUse_i8_Mem : MC6809::SubSetCarryUse_i16_Mem;
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(CarryOut)
                     .addUse(Reg)
                     .addUse(Carry)
                     .add(Ptr)
                     .add(Offset)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry)));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::SubSetCarryUse_i8_Reg : MC6809::SubSetCarryUse_i16_Reg;
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addDef(CarryOut)
                .addUse(LHS)
                .addUse(Carry)
                .addUse(RHS);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      MachineInstr *Def1 = MRI->getVRegDef(MI.getOperand(2).getReg());
      MachineInstr *Def2 = MRI->getVRegDef(MI.getOperand(3).getReg());
      );

  return false;
}

#if 0
bool MC6809InstructionSelector::selectMul(MachineInstr &MI) {
  assert(MI.getOpcode() == TargetOpcode::G_MUL);

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg, Reg2;

  if (DstSize == 8) {
    Reg = MI.getOperand(1).getReg();
    Reg2 = MI.getOperand(2).getReg();
    Instr = Builder.buildInstr(MC6809::Mul8)
                .addDef(Dst)
                //.addDef(MRI->createGenericVirtualRegister(S8))
                .addUse(Reg)
                .addUse(Reg2);
    Instr->addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain 8-bit multiply instruction.");
  } else if (DstSize == 16) {
    std::optional<ValueAndVReg> ValReg;
    int64_t Value;
    for (int Dummy = 0; Dummy < 1; Dummy++) { // Loop abuse to allow creative use of break
      Success = mi_match(Dst, *MRI, m_GMul(m_Reg(Reg), m_GCst(ValReg))) ||
                mi_match(Dst, *MRI, m_GMul(m_GCst(ValReg), m_Reg(Reg)));
      if (Success) {
        Value = ValReg->Value.getSExtValue();
        Instr = Builder.buildInstr(MC6809::Mul16Imm)
                    .addDef(Dst)
                    //.addDef(MRI->createGenericVirtualRegister(S16))
                    .addUse(Reg)
                    .addImm(Value);
        break;
      }
      Success = mi_match(Dst, *MRI, m_GMul(m_Reg(Reg), m_MInstr(Load))) ||
                mi_match(Dst, *MRI, m_GMul(m_MInstr(Load), m_Reg(Reg)));
      if (Success && Load->getOpcode() == TargetOpcode::G_LOAD) {
        Instr = Builder.buildInstr(MC6809::Mul16Idx)
                    .addDef(Dst)
                    //.addDef(MRI->createGenericVirtualRegister(S16))
                    .addUse(Reg)
                    .add(Load->getOperand(1))
                    .addImm(0)
                    .cloneMemRefs(*Load);
        Load->eraseFromParent();
        break;
      }
      Success = mi_match(Dst, *MRI, m_GMul(m_Reg(Reg2), m_Reg(Reg)));
      if (Success) {
        auto Push = Builder.buildInstr(MC6809::PushOp16)
                        .addReg(Reg2);
        Push->addImplicitDefUseOperands(*MF);
        Instr = Builder.buildInstr(MC6809::Mul16Pop)
                    .addDef(Dst)
                    //.addDef(MRI->createGenericVirtualRegister(S16))
                    .addUse(Reg);
        break;
      }
    }
    Instr->addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain 16-bit multiply instruction.");
  }

  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectConstant(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  unsigned Opcode = (DstSize == 8) ? MC6809::Load8Imm : (DstSize == 16) ? MC6809::Load16Imm : MC6809::Load32Imm;
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MF);
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  return Success;
}
#endif

#if 0
bool MC6809InstructionSelector::selectMulExpand(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  Register Src1 = MI.getOperand(1).getReg();
  LLT Src1Ty = MRI->getType(Src1);
  Register Src2 = MI.getOperand(2).getReg();
  LLT Src2Ty = MRI->getType(Src2);

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case MC6809::G_EXPAND_MUL:
    if (DstTy == S16) {
      assert(Src1Ty == S8 && "G_EXPAND_MUL Src1 must be S8");
      assert(Src2Ty == S8 && "G_EXPAND_MUL Src2 must be S8");
      auto Instr = Builder.buildInstr(MC6809::Mul_i8_i16).addDef(Dst).addUse(Src1).addUse(Src2);
      Instr->addImplicitDefUseOperands(*MF);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain Mul_i8_i16 instruction.");
      MI.eraseFromParent();
      return true;
    }
    break;
  }
  return false;
}
#endif

#if 0
bool MC6809InstructionSelector::selectLoad(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  MachineOperand IndexOp = MI.getOperand(1);
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success = false;

  unsigned Opcode = (DstSize == 8) ? MC6809::Load_i8_Idx : (DstSize == 16) ? MC6809::Load_i16_Idx : MC6809::Load_i32_Idx;
  if (IndexOp.isReg())
    Success = mi_match(IndexOp.getReg(), *MRI,m_MInstr(Pointer));
  else if (IndexOp.isImm() || )
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
      .addDef(Dst)
      .add(Pointer->getOperand(1))
      .addImm(0)
      .cloneMemRefs(MI);
    Instr->addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain registered indexed Load instruction.");
    MI.eraseFromParent();
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    MI.addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
      llvm_unreachable("Could not constrain MemOp indexed Load instruction.");
    return true;
  }
}

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  Register Src = MI.getOperand(0).getReg();
  MachineOperand IndexOp = MI.getOperand(1);
  LLT SrcTy = MRI->getType(Src);
  const auto SrcSize = SrcTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success = false;

  unsigned Opcode = (SrcSize == 8) ? MC6809::Store_i8_Idx : (SrcSize == 16) ? MC6809::Store_i16_Idx : MC6809::Store_i32_Idx;
  if (IndexOp.isReg())
    Success = mi_match(IndexOp.getReg(), *MRI,m_MInstr(Pointer));
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addUse(Src)
                     .add(Pointer->getOperand(1))
                     .addImm(0)
                     .cloneMemRefs(MI);
    Instr->addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain register indexed Store instruction.");
    MI.eraseFromParent();
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    MI.addImplicitDefUseOperands(*MF);
    if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
      llvm_unreachable("Could not constrain MemOp indexed Store instruction.");
    return true;
  }
}

bool MC6809InstructionSelector::selectTrunc(MachineInstr &MI) {
  MachineOperand &DstOp = MI.getOperand(0);
  LLT DstType = MRI->getType(DstOp.getReg());
  MachineOperand &SrcOp = MI.getOperand(1);
  LLT SrcType = MRI->getType(SrcOp.getReg());

  /* if (DstType == S1) {
    assert((SrcType == S16 || SrcType == S8) && "Illegal source type for s1 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lsb);
    constrainGenericOp(MI);
    return true;
  } else */ if (DstType == S8) {
    assert((SrcType == S16) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_byte);
    MI.addImplicitDefUseOperands(*MF);
    constrainGenericOp(MI);
    return true;
  } else if (DstType == S16) {
    assert((SrcType == S32) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_word);
    MI.addImplicitDefUseOperands(*MF);
    constrainGenericOp(MI);
    return true;
  }
  else
    llvm_unreachable("Illegal destination type");
  return false;
}
#endif

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  auto [Lo, Hi, Src] = MI.getFirst3Regs();

  MachineIRBuilder Builder(MI);
  LLT SrcTy = MRI->getType(Src);
  assert((SrcTy == S16 || SrcTy == S32) && "The Src of G_UNMERGE_VALUES must be S16 or S32");

  MachineInstrBuilder LoCopy;
  MachineInstrBuilder HiCopy;
  LoCopy = Builder.buildCopy(Lo, Src);
  HiCopy = Builder.buildCopy(Hi, Src);
  if (SrcTy == S16) {
    LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_byte);
    HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_byte);
  } else {
    LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_word);
    HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_word);
  }
  constrainGenericOp(*LoCopy);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectGeneric(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case TargetOpcode::G_IMPLICIT_DEF:
    Opcode = MC6809::IMPLICIT_DEF;
    break;
  case TargetOpcode::G_PHI:
    Opcode = MC6809::PHI;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MF);
  // Establish any tied operands and known register classes.
  constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  // Make sure that the outputs have register classes.
  constrainGenericOp(MI);
  return true;
}

#if 0
// Given a G_SBC instruction Sbc and one of its flag output virtual registers,
// returns the flag that corresponds  to the register.
static Register getCmpFlagForRegister(const MachineInstr &Sbc, Register Reg) {
  static const Register Flags[] = {MC6809::N, MC6809::Z, MC6809::V, MC6809::C};
  // TODO: C++17 structured bindings
  for (const auto &I : zip(Flags, seq(1, 5)))
    if (Sbc.getOperand(std::get<1>(I)).getReg() == Reg)
      return std::get<0>(I);
  llvm_unreachable("Could not find register in G_SBC outputs.");
}

// Match criteria common to all CMP addressing modes.
struct FoldedLdAbs_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdAbs = getOpcodeDef(MOS::G_LOAD_ABS, Reg, MRI);
    if (!LdAbs || !shouldFoldMemAccess(Tgt, *LdAbs, AA))
      return false;
    Addr = LdAbs->getOperand(1);
    return true;
  }
};

inline FoldedLdAbs_match m_FoldedLdAbs(const MachineInstr &Tgt, MachineOperand &Addr, AAResults *AA) { return {Tgt, Addr, AA}; }

struct Cmp_match {
  Register &LHS;
  Register &Flag;

  // The matched G_SBC representing a CMP.
  MachineInstr *CondMI;

  Cmp_match(Register &LHS, Register &Flag) : LHS(LHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    CondMI = DefSrcReg->MI;
    if (CondMI->getOpcode() != MC6809::G_ICMP)
      return false;

    auto CInConst = getIConstantVRegValWithLookThrough(CondMI->getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isZero())
      return false;

    LHS = CondMI->getOperand(5).getReg();
    Flag = getCmpFlagForRegister(*CondMI, DefSrcReg->Reg);
    return Flag == MC6809::N || Flag == MC6809::Z;
  }
};

struct CMPTermZ_match : public Cmp_match {
  CMPTermZ_match(Register &LHS, Register &Flag) : Cmp_match(LHS, Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;

    auto RHSConst = getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
    return RHSConst && RHSConst->Value.isZero();
  }
};

inline CMPTermZ_match m_CMPTermZ(Register &LHS, Register &Flag) { return {LHS, Flag}; }

struct CMPTermImm_match : public Cmp_match {
  int64_t &RHS;

  CMPTermImm_match(Register &LHS, int64_t &RHS, Register &Flag) : Cmp_match(LHS, Flag), RHS(RHS) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;

    auto RHSConst = getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
    if (!RHSConst)
      return false;

    RHS = RHSConst->Value.getZExtValue();
    return true;
  }
};

// Match one of the outputs of a G_SBC to a CMPTermImm operation. LHS and RHS
// are the left and right hand side of the comparison, while Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermImm_match m_CMPTermImm(Register &LHS, int64_t &RHS, Register &Flag) { return {LHS, RHS, Flag}; }

struct CMPTermImag8_match : public Cmp_match {
  Register &RHS;

  CMPTermImag8_match(Register &LHS, Register &RHS, Register &Flag) : Cmp_match(LHS, Flag), RHS(RHS) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    RHS = CondMI->getOperand(6).getReg();
    return true;
  }
};

struct CMPTermAbs_match : public Cmp_match {
  MachineOperand &Addr;
  MachineInstr *&Load;
  AAResults *AA;

  CMPTermAbs_match(Register &LHS, MachineOperand &Addr, Register &Flag, MachineInstr *&Load, AAResults *AA) : Cmp_match(LHS, Flag), Addr(Addr), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    return mi_match(CondMI->getOperand(6).getReg(), MRI, m_all_of(m_MInstr(Load), m_FoldedLdAbs(*CondMI, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CMPTermAbs operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermAbs_match m_CMPTermAbs(Register &LHS, MachineOperand &Addr, Register &Flag, MachineInstr *&Load, AAResults *AA) { return {LHS, Addr, Flag, Load, AA}; }
#endif

#if 0
struct CMPTermIdx_match : public Cmp_match {
  MachineOperand &Addr;
  Register &Idx;
  MachineInstr *&Load;
  bool &DP;
  AAResults *AA;

  CMPTermIdx_match(Register &LHS, MachineOperand &Addr, Register &Idx, Register &Flag, MachineInstr *&Load, bool &DP, AAResults *AA) : Cmp_match(LHS, Flag), Addr(Addr), Idx(Idx), Load(Load), DP(DP), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    return mi_match(CondMI->getOperand(6).getReg(), MRI, m_all_of(m_MInstr(Load), m_FoldedLdIdx(*CondMI, Addr, Idx, DP, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CMPTermIdx operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermIdx_match m_CMPTermIdx(Register &LHS, MachineOperand &Addr, Register &Idx, Register &Flag, MachineInstr *&Load, bool &DP, AAResults *AA) { return {LHS, Addr, Idx, Flag, Load, DP, AA}; }

bool MC6809InstructionSelector::selectBrCondImm(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  Register CondReg = MI.getOperand(0).getReg();
  MachineBasicBlock *Tgt = MI.getOperand(1).getMBB();
  int64_t FlagVal = MI.getOperand(2).getImm();

  LLT S1 = LLT::scalar(1);

  MachineInstr *Compare = nullptr;
  Register Flag;

  MachineIRBuilder Builder(MI);

  MachineInstr *Load;

  Register LHS;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermZ(LHS, Flag)))
    Compare = Builder.buildInstr(MC6809::Test_i8_Reg, {S1}, {LHS});
  int64_t RHSConst;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermImm(LHS, RHSConst, Flag)))
    Compare = Builder.buildInstr(MC6809::Compare_i8_Imm, {S1}, {LHS, RHSConst});
  MachineOperand Addr = MachineOperand::CreateReg(MC6809::NoRegister, /*isDef=*/false);
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermAbs(LHS, Addr, Flag, Load, AA)))
    Compare = Builder.buildInstr(MC6809::Compare_i8_Abs, {S1}, {LHS}).add(Addr).cloneMemRefs(*Load);
  Register Idx;
  bool DP;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermIdx(LHS, Addr, Idx, Flag, Load, DP, AA))) {
    Compare = Builder.buildInstr(DP ? MC6809::CMPTermDpIdx : MC6809::CMPTermAbsIdx, {S1}, {LHS}).add(Addr).addUse(Idx).cloneMemRefs(*Load);
  }
  Register RegAddr;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermIndir(LHS, RegAddr, Flag, Load, AA))) {
    Compare = Builder.buildInstr(MC6809::CMPTermIndir, {S1}, {LHS, RegAddr}).cloneMemRefs(*Load);
  }
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermIndirIdx(LHS, RegAddr, Idx, Flag, Load, AA))) {
    Compare = Builder.buildInstr(MC6809::CMPTermIndirIdx, {S1}, {LHS, RegAddr, Idx}).cloneMemRefs(*Load);
  }
  Register RHS;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermImag8(LHS, RHS, Flag)))
    Compare = Builder.buildInstr(MC6809::CMPTermImag8, {S1}, {LHS, RHS});

  if (Compare) {
    if (!constrainSelectedInstRegOperands(*Compare, TII, TRI, RBI))
      return false;
    assert(Flag != MC6809::C);
    Builder.buildInstr(MC6809::BranchRelative).addMBB(Tgt).addUse(Flag).addImm(FlagVal);
    MI.eraseFromParent();
    return true;
  }

  auto GBR = Builder.buildInstr(MC6809::BranchRelative).addMBB(MI.getOperand(1).getMBB()).addUse(MI.getOperand(0).getReg()).addImm(MI.getOperand(2).getImm());
  if (!constrainSelectedInstRegOperands(*GBR, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}
#endif

#if 0
bool MC6809InstructionSelector::selectBranch(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto Bcc = Builder.buildInstr(MC6809::JumpRelative)
                 .addMBB(MI.getOperand(0).getMBB());
  MI.eraseFromParent();
  return constrainSelectedInstRegOperands(*Bcc, TII, TRI, RBI);
}
#endif /* 0 */

#if 0
MachineInstr *MC6809InstructionSelector::emitIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &Builder) const {

  LLT CMPTy = MRI->getType(LHS.getReg());
  const auto CMPSize = CMPTy.getSizeInBits();
  MachineInstr *Load;
  Register Reg1;
  unsigned Opcode = 0;

  if (RHS.isReg()) {
    if (mi_match(RHS.getReg(), *MRI, m_Copy(m_Reg(Reg1)))) {
      if (Reg1.isPhysical()) {
        // XXXX: FixMe: MarkM - Also invert sense of the predicate
        auto Temp = LHS.getReg();
        LHS.setReg(RHS.getReg());
        RHS.setReg(Temp);
      }
    }
  }

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  if (mi_match(RHS.getReg(), *MRI, m_GCst(ValReg))) {
    Value = ValReg->Value.getSExtValue();
    Opcode = CMPSize == 8 ? MC6809::Compare8Imm : CMPSize == 16 ? MC6809::Compare16Imm : MC6809::Compare32Imm;
    auto Instr = Builder.buildInstr(Opcode)
                .addUse(LHS.getReg())
                .addImm(Value);
    Instr->addImplicitDefUseOperands(*MF);
    return Instr;
  }

  if (mi_match(RHS.getReg(), *MRI, m_MInstr(Load))) {
    Opcode = CMPSize == 8 ? MC6809::Compare8Idx : MC6809::Compare16Idx;
    if (Load->getOpcode() == TargetOpcode::G_LOAD) {
      auto Instr = Builder.buildInstr(Opcode)
                       .addUse(LHS.getReg())
                       .add(Load->getOperand(1)) // Index
                       .addImm(0) // Offset
                       .cloneMemRefs(*Load);
      Instr->addImplicitDefUseOperands(*MF);
      return Instr;
    }
  }
  return nullptr;
}

static MC6809CC::CondCode changeICMPPredToMC6809CC(CmpInst::Predicate P) {
  switch (P) {
  default:
    llvm_unreachable("Unknown condition code!");
  case CmpInst::ICMP_NE:
    return MC6809CC::NE;
  case CmpInst::ICMP_EQ:
    return MC6809CC::EQ;
  case CmpInst::ICMP_SGT:
    return MC6809CC::GT;
  case CmpInst::ICMP_SGE:
    return MC6809CC::GE;
  case CmpInst::ICMP_SLT:
    return MC6809CC::LT;
  case CmpInst::ICMP_SLE:
    return MC6809CC::LE;
  case CmpInst::ICMP_UGT:
    return MC6809CC::HI;
  case CmpInst::ICMP_UGE:
    return MC6809CC::HS;
  case CmpInst::ICMP_ULT:
    return MC6809CC::LO;
  case CmpInst::ICMP_ULE:
    return MC6809CC::LS;
  }
}

/// Return a register which can be used as a bit to test in a BIT
static Register getTestBitReg(Register Reg, uint64_t &Bit, bool &Invert, MachineRegisterInfo &MRI) {
  assert(Reg.isValid() && "Expected valid register!");
  bool HasZext = false;
  while (MachineInstr *MI = getDefIgnoringCopies(Reg, MRI)) {
    unsigned Opc = MI->getOpcode();

    if (!MI->getOperand(0).isReg() ||
        !MRI.hasOneNonDBGUse(MI->getOperand(0).getReg()))
      break;

    // (tbz (any_ext x), b) -> (tbz x, b) if we don't use the extended bits.
    //
    // (tbz (trunc x), b) -> (tbz x, b) is always safe, because the bit number
    // on the truncated x is the same as the bit number on x.
    if (Opc == TargetOpcode::G_ANYEXT || Opc == TargetOpcode::G_ZEXT ||
        Opc == TargetOpcode::G_TRUNC) {
      if (Opc == TargetOpcode::G_ZEXT)
        HasZext = true;

      Register NextReg = MI->getOperand(1).getReg();
      // Did we find something worth folding?
      if (!NextReg.isValid() || !MRI.hasOneNonDBGUse(NextReg))
        break;

      // NextReg is worth folding. Keep looking.
      Reg = NextReg;
      continue;
    }

    // Attempt to find a suitable operation with a constant on one side.
    std::optional<uint64_t> C;
    Register TestReg;
    switch (Opc) {
    default:
      break;
    case TargetOpcode::G_AND:
    case TargetOpcode::G_XOR: {
      TestReg = MI->getOperand(1).getReg();
      Register ConstantReg = MI->getOperand(2).getReg();
      auto VRegAndVal = getIConstantVRegValWithLookThrough(ConstantReg, MRI);
      if (!VRegAndVal) {
        // AND commutes, check the other side for a constant.
        // FIXME: Can we canonicalize the constant so that it's always on the
        // same side at some point earlier?
        std::swap(ConstantReg, TestReg);
        VRegAndVal = getIConstantVRegValWithLookThrough(ConstantReg, MRI);
      }
      if (VRegAndVal) {
        if (HasZext)
          C = VRegAndVal->Value.getZExtValue();
        else
          C = VRegAndVal->Value.getSExtValue();
      }
      break;
    }
    case TargetOpcode::G_ASHR:
    case TargetOpcode::G_LSHR:
    case TargetOpcode::G_SHL: {
      TestReg = MI->getOperand(1).getReg();
      auto VRegAndVal =
          getIConstantVRegValWithLookThrough(MI->getOperand(2).getReg(), MRI);
      if (VRegAndVal)
        C = VRegAndVal->Value.getSExtValue();
      break;
    }
    }

    // Didn't find a constant or viable register. Bail out of the loop.
    if (!C || !TestReg.isValid())
      break;

    // We found a suitable instruction with a constant. Check to see if we can
    // walk through the instruction.
    Register NextReg;
    unsigned TestRegSize = MRI.getType(TestReg).getSizeInBits();
    switch (Opc) {
    default:
      break;
    case TargetOpcode::G_AND:
      // (tbz (and x, m), b) -> (tbz x, b) when the b-th bit of m is set.
      if ((*C >> Bit) & 1)
        NextReg = TestReg;
      break;
    case TargetOpcode::G_SHL:
      // (tbz (shl x, c), b) -> (tbz x, b-c) when b-c is positive and fits in
      // the type of the register.
      if (*C <= Bit && (Bit - *C) < TestRegSize) {
        NextReg = TestReg;
        Bit = Bit - *C;
      }
      break;
    case TargetOpcode::G_ASHR:
      // (tbz (ashr x, c), b) -> (tbz x, b+c) or (tbz x, msb) if b+c is > # bits
      // in x
      NextReg = TestReg;
      Bit = Bit + *C;
      if (Bit >= TestRegSize)
        Bit = TestRegSize - 1;
      break;
    case TargetOpcode::G_LSHR:
      // (tbz (lshr x, c), b) -> (tbz x, b+c) when b + c is < # bits in x
      if ((Bit + *C) < TestRegSize) {
        NextReg = TestReg;
        Bit = Bit + *C;
      }
      break;
    case TargetOpcode::G_XOR:
      // We can walk through a G_XOR by inverting whether we use tbz/tbnz when
      // appropriate.
      //
      // e.g. If x' = xor x, c, and the b-th bit is set in c then
      //
      // tbz x', b -> tbnz x, b
      //
      // Because x' only has the b-th bit set if x does not.
      if ((*C >> Bit) & 1)
        Invert = !Invert;
      NextReg = TestReg;
      break;
    }

    // Check if we found anything worth folding.
    if (!NextReg.isValid())
      return Reg;
    Reg = NextReg;
  }

  return Reg;
}

MachineInstr *MC6809InstructionSelector::emitTestBit(Register TestReg, uint64_t Bit, bool IsNegative, MachineBasicBlock *DstMBB, MachineIRBuilder &MIB) const {
  assert(TestReg.isValid());
  MachineRegisterInfo &MRI = *MIB.getMRI();
  
  // Attempt to optimize the test bit by walking over instructions.
  TestReg = getTestBitReg(TestReg, Bit, IsNegative, MRI);
  LLT Ty = MRI.getType(TestReg);
  unsigned Size = Ty.getSizeInBits();
  assert(!Ty.isVector() && "Expected a scalar!");
  assert(Bit < 64 && "Bit is too large!");

  // When the test register is a 64-bit register, we have to narrow to make
  // TBNZW work.
  bool UseWReg = Bit < 32;
  unsigned NecessarySize = UseWReg ? 32 : 64;
  if (Size != NecessarySize)
    TestReg = moveScalarRegClass(TestReg, UseWReg ? MC6809::GPR32RegClass : MC6809::GPR64RegClass, MIB);

  static const unsigned OpcTable[2][2] = {{MC6809::TBZX, MC6809::TBNZX},
                                          {MC6809::TBZW, MC6809::TBNZW}};
  unsigned Opc = OpcTable[UseWReg][IsNegative];
  auto TestBitMI =
      MIB.buildInstr(Opc).addReg(TestReg).addImm(Bit).addMBB(DstMBB);
  constrainSelectedInstRegOperands(*TestBitMI, TII, TRI, RBI);
  return &*TestBitMI;
}

bool MC6809InstructionSelector::tryOptCompareBranchFedByICmp(MachineInstr &I, MachineInstr &ICmp, MachineIRBuilder &MIB) const {
  assert(ICmp.getOpcode() == TargetOpcode::G_ICMP);
  assert(I.getOpcode() == TargetOpcode::G_BRCOND);
  MachineRegisterInfo &MRI = *MIB.getMRI();
  MachineBasicBlock *DestMBB = I.getOperand(1).getMBB();
  auto Pred = static_cast<CmpInst::Predicate>(ICmp.getOperand(1).getPredicate());
  Register LHS = ICmp.getOperand(2).getReg();
  Register RHS = ICmp.getOperand(3).getReg();

  // We're allowed to emit a TB(N)Z/CB(N)Z. Try to do that.
  auto VRegAndVal = getIConstantVRegValWithLookThrough(RHS, MRI);
  MachineInstr *AndInst = getOpcodeDef(TargetOpcode::G_AND, LHS, MRI);

  // When we can emit a TB(N)Z, prefer that.
  //
  // Handle non-commutative condition codes first.
  // Note that we don't want to do this when we have a G_AND because it can
  // become a tst. The tst will make the test bit in the TB(N)Z redundant.
  if (VRegAndVal && !AndInst) {
    int64_t C = VRegAndVal->Value.getSExtValue();

    // When we have a greater-than comparison, we can just test if the msb is
    // zero.
    if (C == -1 && Pred == CmpInst::ICMP_SGT) {
      uint64_t Bit = MRI.getType(LHS).getSizeInBits() - 1;
      emitTestBit(LHS, Bit, /*IsNegative = */ false, DestMBB, MIB);
      I.eraseFromParent();
      return true;
    }

    // When we have a less than comparison, we can just test if the msb is not
    // zero.
    if (C == 0 && Pred == CmpInst::ICMP_SLT) {
      uint64_t Bit = MRI.getType(LHS).getSizeInBits() - 1;
      emitTestBit(LHS, Bit, /*IsNegative = */ true, DestMBB, MIB);
      I.eraseFromParent();
      return true;
    }

    // Inversely, if we have a signed greater-than-or-equal comparison to zero,
    // we can test if the msb is zero.
    if (C == 0 && Pred == CmpInst::ICMP_SGE) {
      uint64_t Bit = MRI.getType(LHS).getSizeInBits() - 1;
      emitTestBit(LHS, Bit, /*IsNegative = */ false, DestMBB, MIB);
      I.eraseFromParent();
      return true;
    }
  }

  // Attempt to handle commutative condition codes. Right now, that's only
  // eq/ne.
  if (ICmpInst::isEquality(Pred)) {
    if (!VRegAndVal) {
      std::swap(RHS, LHS);
      VRegAndVal = getIConstantVRegValWithLookThrough(RHS, MRI);
      AndInst = getOpcodeDef(TargetOpcode::G_AND, LHS, MRI);
    }

    if (VRegAndVal && VRegAndVal->Value == 0) {
      // If there's a G_AND feeding into this branch, try to fold it away by
      // emitting a TB(N)Z instead.
      //
      // Note: If we have LT, then it *is* possible to fold, but it wouldn't be
      // beneficial. When we have an AND and LT, we need a TST/ANDS, so folding
      // would be redundant.
      if (AndInst &&
          tryOptAndIntoCompareBranch(
              *AndInst, /*Invert = */ Pred == CmpInst::ICMP_NE, DestMBB, MIB)) {
        I.eraseFromParent();
        return true;
      }

      // Otherwise, try to emit a CB(N)Z instead.
      auto LHSTy = MRI.getType(LHS);
      if (!LHSTy.isVector() && LHSTy.getSizeInBits() <= 64) {
        emitCBZ(LHS, /*IsNegative = */ Pred == CmpInst::ICMP_NE, DestMBB, MIB);
        I.eraseFromParent();
        return true;
      }
    }
  }

  return false;
}

bool MC6809InstructionSelector::selectCompareBranchFedByICmp(MachineInstr &I, MachineInstr &ICmp, MachineIRBuilder &MIB) const {
  assert(ICmp.getOpcode() == TargetOpcode::G_ICMP);
  assert(I.getOpcode() == TargetOpcode::G_BRCOND);
  if (tryOptCompareBranchFedByICmp(I, ICmp, MIB))
    return true;

  // Couldn't optimize. Emit a compare + a Bcc.
  MachineBasicBlock *DestMBB = I.getOperand(1).getMBB();
  auto PredOp = ICmp.getOperand(1);
  emitIntegerCompare(ICmp.getOperand(2), ICmp.getOperand(3), PredOp, MIB);
  const MC6809CC::CondCode CC = changeICMPPredToMC6809CC(static_cast<CmpInst::Predicate>(PredOp.getPredicate()));
  MIB.buildInstr(MC6809::Bcc, {}, {}).addImm(CC).addMBB(DestMBB);
  I.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectConditionalBranch(MachineInstr &MI, MachineFunction &MF, MachineRegisterInfo &MRI) {
  Register CondReg = MI.getOperand(0).getReg();
  MachineInstr *CCMI = MRI.getVRegDef(CondReg);
  // Try to select the G_BRCOND using whatever is feeding the condition if
  // possible.
  unsigned CCMIOpc = CCMI->getOpcode();
  if (CCMIOpc == TargetOpcode::G_ICMP)
    return selectCompareBranchFedByICmp(MI, *CCMI, MIB);

  // Emit a TST + Bbc.
  auto TstMI = MIB.buildInstr(MC6809::AND8Imm, {LLT::scalar(8)}, {CondReg}).addImm(1);
  constrainSelectedInstRegOperands(*TstMI, TII, TRI, RBI);
  auto Bcc = MIB.buildInstr(MC6809::ConditionalBranchRelative)
                 .addImm(MC6809CC::EQ)
                 .addMBB(MI.getOperand(1).getMBB());
  MI.eraseFromParent();
  return true;
}
#endif

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register
// is defined exactly once, making sure definitions are constrained suffices.
void MC6809InstructionSelector::constrainGenericOp(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.all_defs()) {
    if (Op.getReg().isPhysical() || MRI.getRegClassOrNull(Op.getReg()))
      continue;
    LLT Ty = MRI.getType(Op.getReg());
    const RegisterBank *RB = MRI.getRegBankOrNull(Op.getReg());
    constrainOperandRegClass(Op, getRegClassForTypeOnBank(Ty, RB));
  }
}

void MC6809InstructionSelector::constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, *MRI, TII, RBI, MI, RegClass, RegMO));
}

bool MC6809InstructionSelector::selectAll(MachineInstrSpan MIS) {
  // Ensure that all new generic virtual registers have a register bank.
  for (MachineInstr &MI : MIS) {
    for (MachineOperand &MO : MI.operands()) {
      if (!MO.isReg())
        continue;
      Register Reg = MO.getReg();
      if (!MO.getReg().isVirtual())
        continue;
      if (MRI->getRegClassOrNull(MO.getReg()))
        continue;
      auto *RC = MRI->getRegClassOrNull(MO.getReg());
      MRI->setRegBank(Reg, RBI.getRegBankFromRegClass(*RC, LLT()));
    }
  }

  for (MachineInstr &MI : MIS) {
    if (!select(MI))
      return false;
  }
  return true;
}

InstructionSelector *llvm::createMC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI) { return new MC6809InstructionSelector(TM, STI, RBI); }
