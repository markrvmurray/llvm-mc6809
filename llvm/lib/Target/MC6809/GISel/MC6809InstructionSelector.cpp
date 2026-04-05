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
#include "MC6809.h"
#include "MC6809RegisterBankInfo.h"

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
  bool selectMergeValues(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);
  bool selectAddO(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectSubO(MachineInstr &MI);
  bool selectSubE(MachineInstr &MI);
  bool selectShiftExtend(MachineInstr &MI);
  bool selectShift16(MachineInstr &MI);

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
    // Big-endian: operand 0 = lo part, operand 1 = hi part.
    // Lo part is at the HIGHER address: base + element_size.
    // Hi part is at base + 0.
    Register SrcReg = RootDef->getOperand(2).getReg();
    unsigned EltSize = MRI.getType(Root.getReg()).getSizeInBytes();
    bool IsLo = Root.getReg() == RootDef->getOperand(0).getReg();
    unsigned mergeoffset = IsLo ? EltSize : 0;
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

  // Intercept i16 shifts on 6809 before selectImpl (no i16 shift pattern).
  if (!STI.has6309()) {
    switch (MI.getOpcode()) {
    case TargetOpcode::G_SHL:
    case TargetOpcode::G_LSHR:
    case TargetOpcode::G_ASHR:
      if (MRI->getType(MI.getOperand(0).getReg()) == LLT::scalar(16))
        return selectShift16(MI);
      break;
    default:
      break;
    }
  }

  if (selectImpl(MI, *CoverageInfo)) {
    return true;
  }

  switch (MI.getOpcode()) {
  default:
    return false;

  case TargetOpcode::G_TRUNC: {
    // Hand-select G_TRUNC when the imported pattern doesn't match (e.g.,
    // when imaginary registers in ACC16 cause a synthesized 'accum' class).
    // trunc i16→i8: EXTRACT_SUBREG sub_lo_byte (for real registers) or
    // COPY with class constraint (for imaginary/spill).
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(16)) {
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      // Use ADc (AD + SPILL_D), not ACC16, because sub_lo_byte requires
      // registers that have sub-register structure (RS0-RS3 don't).
      MRI->setRegClass(SrcReg, &MC6809::ADcRegClass);
      MI.setDesc(TII.get(TargetOpcode::COPY));
      MI.getOperand(1).setSubReg(MC6809::sub_lo_byte);
      constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
      return true;
    }
    return false;
  }

  case TargetOpcode::G_CONSTANT: {
    // Pointer constants (e.g., NULL) aren't covered by imported patterns
    // because they have p0 type, not s16. Hand-select to LDX #imm.
    Register DstReg = MI.getOperand(0).getReg();
    if (!MRI->getType(DstReg).isPointer())
      return false;
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);
    MI.setDesc(TII.get(MC6809::Load_iPtr_Imm));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_GLOBAL_VALUE: {
    // Load the address of a global into an index register (LDX #addr).
    Register DstReg = MI.getOperand(0).getReg();
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);
    MI.setDesc(TII.get(MC6809::Load_iPtr_Imm));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_LOAD: {
    // Pointer (p0) loads aren't covered by imported patterns.
    Register DstReg = MI.getOperand(0).getReg();
    if (!MRI->getType(DstReg).isPointer())
      return false;
    Register AddrReg = MI.getOperand(1).getReg();
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);
    MRI->setRegClass(AddrReg, &MC6809::INDEX16RegClass);
    MI.setDesc(TII.get(MC6809::Load_iPtr_Mem));
    MI.addOperand(MachineOperand::CreateImm(0));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_STORE: {
    // Pointer (p0) stores aren't covered by imported patterns.
    Register ValReg = MI.getOperand(0).getReg();
    if (!MRI->getType(ValReg).isPointer())
      return false;
    Register AddrReg = MI.getOperand(1).getReg();
    MRI->setRegClass(ValReg, &MC6809::INDEX16RegClass);
    MRI->setRegClass(AddrReg, &MC6809::INDEX16RegClass);
    MI.setDesc(TII.get(MC6809::Store_iPtr_Mem));
    MI.addOperand(MachineOperand::CreateImm(0));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_BRJT: {
    // Branch via jump table. G_BRJT %table_ptr, %jump-table.N, %index
    // Select to BranchJumpTable pseudo (expanded post-RA to PIC sequence).
    Register IdxReg = MI.getOperand(2).getReg();
    unsigned JTI = MI.getOperand(1).getIndex();
    MRI->setRegClass(IdxReg, &MC6809::ACC16RegClass);
    BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(MC6809::BranchJumpTable))
        .addReg(IdxReg)
        .addJumpTableIndex(JTI);
    MI.eraseFromParent();
    return true;
  }

  case TargetOpcode::G_BRCOND: {
    // G_BRCOND %cond(s1), %bb.target
    // When the condition is an s1 in the accum bank (not FLAGS), the imported
    // patterns can't handle it. Emit TSTB + BNE to test the boolean byte.
    Register CondReg = MI.getOperand(0).getReg();
    MachineBasicBlock *TargetMBB = MI.getOperand(1).getMBB();
    // Set register class to ACC8 (the boolean is a byte: 0 or 1).
    MRI->setRegClass(CondReg, &MC6809::ACC8RegClass);
    // Test the boolean: Test_i8_Reg sets CC from the register value.
    Register CCReg = MRI->createVirtualRegister(&MC6809::CCondRegClass);
    BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(MC6809::Test_i8_Reg))
        .addDef(CCReg)
        .addImm(MC6809CC::NE)
        .addReg(CondReg);
    // Branch if non-zero (NE).
    BuildMI(*MBB, MI, MI.getDebugLoc(),
            TII.get(MC6809::ConditionalLongBranchRelative))
        .addImm(MC6809CC::NE)
        .addMBB(TargetMBB)
        .addReg(CCReg);
    MI.eraseFromParent();
    return true;
  }

  case TargetOpcode::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case TargetOpcode::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case TargetOpcode::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case TargetOpcode::G_IMPLICIT_DEF:
  case TargetOpcode::G_PHI:
    return selectGeneric(MI);

  case TargetOpcode::G_FREEZE: {
    // G_FREEZE is a no-op (marks value as non-poison). Lower to COPY.
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    const TargetRegisterClass *RC = MRI->getRegClassOrNull(SrcReg);
    if (!RC) {
      LLT Ty = MRI->getType(SrcReg);
      if (Ty == LLT::scalar(1))
        RC = &MC6809::BIT1RegClass;
      else if (Ty == LLT::scalar(8))
        RC = &MC6809::ACC8RegClass;
      else if (Ty == LLT::scalar(16) || Ty == LLT::pointer(0, 16))
        RC = &MC6809::ACC16RegClass;
      else if (Ty == LLT::scalar(32))
        RC = &MC6809::ACC32RegClass;
    }
    if (RC) {
      MRI->setRegClass(DstReg, RC);
      MRI->setRegClass(SrcReg, RC);
    }
    MI.setDesc(TII.get(TargetOpcode::COPY));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

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

          if (Src2Load && Src2Load->getOpcode() == MC6809::Load_i16_Mem) {
            // D = 0, then SUBD directly from memory.
            auto Zero = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Imm), AccZero)
                .addImm(0);
            constrainSelectedInstRegOperands(*Zero, TII, TRI, RBI);
            auto SubMem = BuildMI(MBB, MI, DL, TII.get(MC6809::Sub_i16_Mem), AccNegB)
                .addReg(AccZero)
                .add(Src2Load->getOperand(1))  // index register
                .add(Src2Load->getOperand(2)); // offset
            constrainSelectedInstRegOperands(*SubMem, TII, TRI, RBI);
          } else {
            // Push b FIRST (before loading 0 into D), then subtract.
            Register AccSrc2 = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(AccSrc2, &MC6809::ACC16RegClass);
            Register StackReg = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(StackReg, &MC6809::STACK16RegClass);
            BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), AccSrc2).addReg(Src2);
            auto Push = BuildMI(MBB, MI, DL, TII.get(MC6809::Push_i16), StackReg)
                .addReg(AccSrc2);
            constrainSelectedInstRegOperands(*Push, TII, TRI, RBI);
            // NOW load 0 (after b is safely on the stack).
            auto Zero = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Imm), AccZero)
                .addImm(0);
            constrainSelectedInstRegOperands(*Zero, TII, TRI, RBI);
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

  case MC6809::G_SHLE:
  case MC6809::G_LSHRE:
    return selectShiftExtend(MI);

  case TargetOpcode::G_SHL:
  case TargetOpcode::G_LSHR:
  case TargetOpcode::G_ASHR:
    // i16 shifts on 6809: no native instruction. Hand-select to byte pairs
    // (constant: LSL_i16_Reg loop, variable: libcall).
    if (!STI.has6309() && MRI->getType(MI.getOperand(0).getReg()) == LLT::scalar(16))
      return selectShift16(MI);
    break;
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
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
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
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
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
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
      );

  return false;
}

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

bool MC6809InstructionSelector::selectShiftExtend(MachineInstr &MI) {
  // Select G_SHLE (shift-left-extend) and G_LSHRE (logical-shift-right-extend).
  // These are single-bit shifts with carry in/out, produced by the shift
  // decomposition in legalizeShiftRotate for constant shifts.
  //
  // G_SHLE → LSL_i8_Reg (= ASL, shift left by 1)
  // G_LSHRE + carry from ICMP (sign test) → ASR_i8_Reg (arithmetic shift right)
  // G_LSHRE + carry = 0/undef → LSR_i8_Reg (logical shift right)
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  LLT Ty = MRI->getType(Dst);
  if (Ty != LLT::scalar(8))
    return false; // Only handle s8 for now

  // Determine the shift instruction based on carry_in source:
  // - Constant 0 / undef: first in chain → ASL/LSR/ASR
  // - From G_ICMP: ASHR sign test → ASR
  // - From another G_SHLE/G_LSHRE: carry chain → ROL/ROR
  MachineInstr *CarryDef = MRI->getVRegDef(CarryIn);
  bool IsCarryChain = CarryDef &&
      (CarryDef->getOpcode() == MC6809::G_SHLE ||
       CarryDef->getOpcode() == MC6809::G_LSHRE);

  unsigned ShiftOpc;
  if (MI.getOpcode() == MC6809::G_SHLE) {
    ShiftOpc = IsCarryChain ? MC6809::ROL_i8_Reg : MC6809::LSL_i8_Reg;
  } else {
    if (CarryDef && CarryDef->getOpcode() == TargetOpcode::G_ICMP)
      ShiftOpc = MC6809::ASR_i8_Reg;
    else if (IsCarryChain)
      ShiftOpc = MC6809::ROR_i8_Reg;
    else
      ShiftOpc = MC6809::LSR_i8_Reg;
  }

  MachineIRBuilder Builder(MI);
  auto Shift = Builder.buildInstr(ShiftOpc)
      .addDef(Dst)
      .addUse(Src)
      .addImm(1);  // shift amount = 1
  constrainSelectedInstRegOperands(*Shift, TII, TRI, RBI);

  // The carry output is implicitly in CC. Mark it as dead if unused,
  // or create a COPY from the C flag if used.
  if (MRI->use_empty(CarryOut)) {
    // Carry not used — nothing to do, it's implicit in CC
  } else {
    // Carry is used by the next shift in the chain.
    // For single-byte shifts, each iteration is independent (same constant
    // carry_in), so the carry_out is never actually consumed. But mark it
    // as defined to satisfy the register allocator.
    Builder.buildUndef(CarryOut);
  }

  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectShift16(MachineInstr &MI) {
  // Hand-select i16 shifts on 6809 into byte-pair operations.
  // SHL: ASLB + ROLA (carry from lo propagates to hi)
  // LSHR: LSRA + RORB (carry from hi propagates to lo)
  // ASHR: ASRA + RORB (sign-preserving shift of hi, carry to lo)
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  Register AmtReg = MI.getOperand(2).getReg();

  // Only handle constant shift amount
  auto Amt = getIConstantVRegValWithLookThrough(AmtReg, *MRI);
  if (!Amt)
    return false;

  uint64_t ShiftAmt = Amt->Value.getZExtValue();
  if (ShiftAmt == 0) {
    // Shift by 0 = copy
    MachineIRBuilder Builder(MI);
    Builder.buildCopy(DstReg, SrcReg);
    MI.eraseFromParent();
    return true;
  }

  unsigned ShiftOpc;
  switch (MI.getOpcode()) {
  case TargetOpcode::G_SHL:  ShiftOpc = MC6809::LSL_i16_Reg; break;
  case TargetOpcode::G_LSHR: ShiftOpc = MC6809::LSR_i16_Reg; break;
  case TargetOpcode::G_ASHR: ShiftOpc = MC6809::ASR_i16_Reg; break;
  default: return false;
  }

  MachineBasicBlock &MBB = *MI.getParent();
  const DebugLoc &DL = MI.getDebugLoc();

  // Copy source into ACC16 (D register) for shift
  Register Cur = MRI->createVirtualRegister(&MC6809::ACC16RegClass);
  BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), Cur).addReg(SrcReg);

  for (uint64_t I = 0; I < ShiftAmt; ++I) {
    Register Next = (I == ShiftAmt - 1) ? DstReg
        : MRI->createVirtualRegister(&MC6809::ACC16RegClass);
    auto &Shift = *BuildMI(MBB, MI, DL, TII.get(ShiftOpc), Next)
        .addReg(Cur)
        .addImm(1);
    constrainSelectedInstRegOperands(Shift, TII, TRI, RBI);
    Cur = Next;
  }

  MRI->setRegClass(DstReg, &MC6809::ACC16RegClass);
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
      if (MRI->getRegBankOrNull(Reg))
        continue;
      const auto *RC = MRI->getRegClassOrNull(Reg);
      if (!RC)
        continue;
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
