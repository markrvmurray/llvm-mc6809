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
  bool selectBrCondImm(MachineInstr &MI);
  bool selectSbc(MachineInstr &MI);
  bool selectFrameIndex(MachineInstr &MI);
  std::pair<Register, Register> selectFrameIndexLoHi(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  std::pair<Register, Register> selectAddrLoHi(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectLshrShlE(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
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

// Returns the widest register class that can contain values of a given type.
// Used to ensure that every virtual register gets some register class by the
// time register allocation completes.
static const TargetRegisterClass &getRegClassForType(LLT Ty) {
  switch (Ty.getSizeInBits()) {
  default:
    llvm_unreachable("Invalid type size.");
  case 1:
    return MC6809::BIT1RegClass;
  case 8:
    return MC6809::ACC8RegClass;
  case 16:
    return MC6809::ACC16RegClass;
  }
}

bool MC6809InstructionSelector::select(MachineInstr &MI) {
  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

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

  if (selectImpl(MI, *CoverageInfo))
    return true;

  switch (MI.getOpcode()) {
  default:
    return false;
  case MC6809::G_BRCOND_IMM:
    return selectBrCondImm(MI);
  case MC6809::G_SBC:
    return selectSbc(MI);
  case MC6809::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MC6809::G_BLOCK_ADDR:
  case MC6809::G_GLOBAL_VALUE:
    return selectAddr(MI);
  case MC6809::G_STORE_ABS:
  case MC6809::G_STORE_ABS_IDX:
    return selectStore(MI);
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

  case MC6809::G_BRINDIRECT:
  case MC6809::G_IMPLICIT_DEF:
  case MC6809::G_LOAD_ABS:
  case MC6809::G_LOAD_ABS_IDX:
  case MC6809::G_LOAD_INDIR_IDX:
  case MC6809::G_PHI:
  case MC6809::G_STORE_INDIR_IDX:
    return selectGeneric(MI);
  }
}

static bool shouldFoldMemAccess(const MachineInstr &Dst,
                                const MachineInstr &Src, AAResults *AA) {
  assert(Src.mayLoadOrStore());

  // For now, don't attempt to fold across basic block boundaries.
  if (Dst.getParent() != Src.getParent())
    return false;

  // Does it pay off to fold the access? Depends on the number of users.
  const auto &MRI = Dst.getMF()->getRegInfo();
  const auto Users = MRI.use_nodbg_instructions(Src.getOperand(0).getReg());
  const auto NumUsers = std::distance(Users.begin(), Users.end());

  // Looking at this pessimistically, if we don't fold the access, all
  // references may refer to an Imag8 reg that needs to be copied to/from a GPR.
  // This costs 2 bytes and 3 cycles. We also need to do the actual load/store.
  // If we do fold the access, then we get rid of both that and the load/store.
  // This makes the first reference free; as it's not any more expensive than
  // the load/store. However, for each reference past the first, we pay an
  // overhead for using the addressing over the imaginary addressing mode. This
  // cost is: Absolute: 1 byte, 1 cycle Absolute Indexed: 1 byte, 1.5 cycles
  // Indirect Indexed: 2.5 cycles
  // So, it pays off to fold k references of each addressing mode if:
  // Absolute: k*(1+1) < (2+3) = 5; 2k < 5; k < 2.5; k <= 2
  // Absolute Indexed: k*(1+1.5) < 5; 2.5k < 5; k <= 1
  // Indirect Indexed: k*(0+2.5) < 5; 2.5k < 5; k <= 1
  int MaxNumUsers;
  switch (Src.getOpcode()) {
  default:
    MaxNumUsers = 1;
    break;
  case MC6809::G_LOAD_ABS:
    MaxNumUsers = 2;
  }
  if (NumUsers > MaxNumUsers)
    return false;

  // Look for intervening instructions that cannot be folded across.
  for (const MachineInstr &I :
       make_range(std::next(MachineBasicBlock::const_iterator(Src)),
                  MachineBasicBlock::const_iterator(Dst))) {
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

struct FoldedLdAbs_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  AAResults *AA;

  FoldedLdAbs_match(const MachineInstr &Tgt, MachineOperand &Addr,
                    AAResults *AA)
      : Tgt(Tgt), Addr(Addr), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdAbs = getOpcodeDef(MC6809::G_LOAD_ABS, Reg, MRI);
    if (!LdAbs || !shouldFoldMemAccess(Tgt, *LdAbs, AA))
      return false;
    Addr = LdAbs->getOperand(1);
    return true;
  }
};

inline FoldedLdAbs_match m_FoldedLdAbs(const MachineInstr &Tgt,
                                       MachineOperand &Addr, AAResults *AA) {
  return {Tgt, Addr, AA};
}

struct FoldedLdIdx_match {
  const MachineInstr &Tgt;
  Register &Idx;
  MachineOperand &Offset;
  AAResults *AA;

  FoldedLdIdx_match(const MachineInstr &Tgt, Register &Idx, MachineOperand &Offset,
                    AAResults *AA)
      : Tgt(Tgt), Idx(Idx), Offset(Offset), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LDAbsIdx = getOpcodeDef(MC6809::G_LOAD_ABS_IDX, Reg, MRI);
    if (!LDAbsIdx || !shouldFoldMemAccess(Tgt, *LDAbsIdx, AA))
      return false;
    Idx = LDAbsIdx->getOperand(1).getReg();
    Offset = LDAbsIdx->getOperand(2);
    return true;
  }
};

inline FoldedLdIdx_match m_FoldedLdIdx(const MachineInstr &Tgt,
                                       Register &Idx, MachineOperand &Offset,
                                       AAResults *AA) {
  return {Tgt, Idx, Offset, AA};
}

struct FoldedLdIndirIdx_match {
  const MachineInstr &Tgt;
  Register &Idx;
  Register &Offset;
  AAResults *AA;

  FoldedLdIndirIdx_match(const MachineInstr &Tgt, Register &Idx, Register &Offset,
                         AAResults *AA)
      : Tgt(Tgt), Idx(Idx), Offset(Offset), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdIndirIdx =
        getOpcodeDef(MC6809::G_LOAD_INDIR_IDX, Reg, MRI);
    if (!LdIndirIdx || !shouldFoldMemAccess(Tgt, *LdIndirIdx, AA))
      return false;
    Idx = LdIndirIdx->getOperand(1).getReg();
    Offset = LdIndirIdx->getOperand(2).getReg();
    return true;
  }
};

inline FoldedLdIndirIdx_match m_FoldedLdIndirIdx(const MachineInstr &Tgt,
                                                 Register &Idx, Register &Offset,
                                                 AAResults *AA) {
  return {Tgt, Idx, Offset, AA};
}

bool MC6809InstructionSelector::selectAddSub(MachineInstr &MI) {
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

// Given a G_SBC instruction Sbc and one of its flag output virtual registers,
// returns the flag that corresponds to the register.
static Register getSbcFlagForRegister(const MachineInstr &Sbc, Register Reg) {
  static const Register Flags[] = {MC6809::C, MC6809::N, MC6809::V, MC6809::Z};
  // TODO: C++17 structured bindings
  for (const auto &I : zip(Flags, seq(1, 5)))
    if (Sbc.getOperand(std::get<1>(I)).getReg() == Reg)
      return std::get<0>(I);
  llvm_unreachable("Could not find register in G_SBC outputs.");
}

// Match criteria common to all CMP addressing modes.
struct Cmp_match {
  Register &LHS;
  Register &Flag;

  // The matched G_SBC representing a CMP.
  MachineInstr *CondMI;

  Cmp_match(Register &LHS, Register &Flag) : LHS(LHS), Flag(Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    auto DefSrcReg = getDefSrcRegIgnoringCopies(CondReg, MRI);
    CondMI = DefSrcReg->MI;
    if (CondMI->getOpcode() != MC6809::G_SBC)
      return false;

    auto CInConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(7).getReg(), MRI);
    if (!CInConst || CInConst->Value.isNullValue())
      return false;

    LHS = CondMI->getOperand(5).getReg();
    Flag = getSbcFlagForRegister(*CondMI, DefSrcReg->Reg);
    return Flag == MC6809::N || Flag == MC6809::Z;
  }
};

struct CMPTermZ_match : public Cmp_match {
  CMPTermZ_match(Register &LHS, Register &Flag) : Cmp_match(LHS, Flag) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
    return RHSConst && RHSConst->Value.isZero();
  }
};

inline CMPTermZ_match m_CMPTermZ(Register &LHS, Register &Flag) {
  return {LHS, Flag};
}

struct CMPTermImm_match : public Cmp_match {
  int64_t &RHS;

  CMPTermImm_match(Register &LHS, int64_t &RHS, Register &Flag)
      : Cmp_match(LHS, Flag), RHS(RHS) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;

    auto RHSConst =
        getIConstantVRegValWithLookThrough(CondMI->getOperand(6).getReg(), MRI);
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
inline CMPTermImm_match m_CMPTermImm(Register &LHS, int64_t &RHS,
                                     Register &Flag) {
  return {LHS, RHS, Flag};
}

struct CMPTermAbs_match : public Cmp_match {
  MachineOperand &Addr;
  MachineInstr *&Load;
  AAResults *AA;

  CMPTermAbs_match(Register &LHS, MachineOperand &Addr, Register &Flag,
                   MachineInstr *&Load, AAResults *AA)
      : Cmp_match(LHS, Flag), Addr(Addr), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    return mi_match(CondMI->getOperand(6).getReg(), MRI,
                    m_all_of(m_MInstr(Load), m_FoldedLdAbs(*CondMI, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CMPTermAbs operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermAbs_match m_CMPTermAbs(Register &LHS, MachineOperand &Addr,
                                     Register &Flag, MachineInstr *&Load,
                                     AAResults *AA) {
  return {LHS, Addr, Flag, Load, AA};
}

struct CMPTermIdx_match : public Cmp_match {
  MachineOperand &Addr;
  Register &Idx;
  MachineInstr *&Load;
  AAResults *AA;

  CMPTermIdx_match(Register &LHS, MachineOperand &Addr, Register &Idx,
                   Register &Flag, MachineInstr *&Load, AAResults *AA)
      : Cmp_match(LHS, Flag), Addr(Addr), Idx(Idx), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    return mi_match(
        CondMI->getOperand(6).getReg(), MRI,
        m_all_of(m_MInstr(Load), m_FoldedLdIdx(*CondMI, Idx, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CMPTermIdx operation. Flag is the
// physical (N, Z, V, C) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermIdx_match m_CMPTermIdx(Register &LHS, MachineOperand &Addr,
                                     Register &Idx, Register &Flag,
                                     MachineInstr *&Load, AAResults *AA) {
  return {LHS, Addr, Idx, Flag, Load, AA};
}

struct CMPTermIndir_match : public Cmp_match {
  Register &Addr;
  Register &Idx;
  MachineInstr *&Load;
  AAResults *AA;

  CMPTermIndir_match(Register &LHS, Register &Addr, Register &Idx,
                     Register &Flag, MachineInstr *&Load, AAResults *AA)
      : Cmp_match(LHS, Flag), Addr(Addr), Idx(Idx), Load(Load), AA(AA) {}

  bool match(const MachineRegisterInfo &MRI, Register CondReg) {
    if (!Cmp_match::match(MRI, CondReg))
      return false;
    return mi_match(
        CondMI->getOperand(6).getReg(), MRI,
        m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(*CondMI, Idx, Addr, AA)));
  }
};

// Match one of the outputs of a G_SBC to a CMPTermIndir operation. Flag is the
// physical (N or Z) register corresponding to the output by which the G_SBC
// was reached.
inline CMPTermIndir_match m_CMPTermIndir(Register &LHS, Register &Addr,
                                         Register &Idx, Register &Flag,
                                         MachineInstr *&Load, AAResults *AA) {
  return {LHS, Addr, Idx, Flag, Load, AA};
}

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
    Compare = Builder.buildInstr(MC6809::CMPTermZ, {S1}, {LHS});
  int64_t RHSConst;
  if (!Compare && mi_match(CondReg, MRI, m_CMPTermImm(LHS, RHSConst, Flag)))
    Compare = Builder.buildInstr(MC6809::CMPTermImm, {S1}, {LHS, RHSConst});
  MachineOperand Addr =
      MachineOperand::CreateReg(MC6809::NoRegister, /*isDef=*/false);
  if (!Compare &&
      mi_match(CondReg, MRI, m_CMPTermAbs(LHS, Addr, Flag, Load, AA)))
    Compare = Builder.buildInstr(MC6809::CMPTermAbs, {S1}, {LHS})
                  .add(Addr)
                  .cloneMemRefs(*Load);
  Register Idx;
  if (!Compare &&
      mi_match(CondReg, MRI, m_CMPTermIdx(LHS, Addr, Idx, Flag, Load, AA))) {
    Compare = Builder.buildInstr(MC6809::CMPTermIdx, {S1}, {LHS})
                  .add(Addr)
                  .addUse(Idx)
                  .cloneMemRefs(*Load);
  }
  Register RegAddr;
  if (!Compare && mi_match(CondReg, MRI,
                           m_CMPTermIndir(LHS, RegAddr, Idx, Flag, Load, AA))) {
    Compare = Builder.buildInstr(MC6809::CMPTermIndir, {S1}, {LHS, RegAddr, Idx})
                  .cloneMemRefs(*Load);
  }

  if (Compare) {
    if (!constrainSelectedInstRegOperands(*Compare, TII, TRI, RBI))
      return false;
    assert(Flag != MC6809::C);
    Builder.buildInstr(MC6809::BR).addMBB(Tgt).addUse(Flag).addImm(FlagVal);
    MI.eraseFromParent();
    return true;
  }

  auto GBR = Builder.buildInstr(MC6809::GBR)
                 .addMBB(MI.getOperand(1).getMBB())
                 .addUse(MI.getOperand(0).getReg())
                 .addImm(MI.getOperand(2).getImm());
  if (!constrainSelectedInstRegOperands(*GBR, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

// Although some G_SBC instructions can be folded in to their (branch) uses,
// others need to be selected directly.
bool MC6809InstructionSelector::selectSbc(MachineInstr &MI) {
#if 0
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineIRBuilder Builder(MI);
  const auto &MRI = *Builder.getMRI();

  Register A = MI.getOperand(0).getReg();
  Register N = MI.getOperand(2).getReg();
  Register V = MI.getOperand(3).getReg();
  Register Z = MI.getOperand(4).getReg();
  Register R = MI.getOperand(6).getReg();

  // Outputs that are unused may not need to be generated.
  if (Builder.getMRI()->use_nodbg_empty(A))
    A = MC6809::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(N))
    N = MC6809::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(V))
    V = MC6809::NoRegister;
  if (Builder.getMRI()->use_nodbg_empty(Z))
    Z = MC6809::NoRegister;

  auto CInConst =
      getIConstantVRegValWithLookThrough(MI.getOperand(7).getReg(), MRI);
  bool CInSet = CInConst && !CInConst->Value.isNullValue();

  // We can only extract one of N or Z at a time, so if both are needed,
  // arbitrarily extract out the comparison that produces Z. This case
  // should very rarely be hit, if ever.
  if (N && Z) {
    MachineInstrSpan MIS(MI, MI.getParent());
    MI.getOperand(4).setReg(Builder.getMRI()->createGenericVirtualRegister(S1));
    Builder.setInsertPt(Builder.getMBB(), std::next(Builder.getInsertPt()));
    Builder.buildInstr(MC6809::G_SBC, {S8, S1, S1, S1, Z},
                       {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
    return selectAll(MIS);
  }

  auto RConst = getIConstantVRegValWithLookThrough(R, *Builder.getMRI());
  MachineInstr *Load;
  MachineInstrBuilder Instr;
  // A CMP instruction can be used if we don't need the result, the overflow,
  // and the carry in is known to be set.
  if (!A && !V && CInSet) {
    if (!Instr && RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      Instr =
          Builder.buildInstr(MC6809::CMPNZVCImm, {MI.getOperand(1), N, Z},
                             {MI.getOperand(5), RConst->Value.getZExtValue()});
    }
    MachineOperand Addr =
        MachineOperand::CreateReg(MC6809::NoRegister, /*isDef=*/false);
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)))) {
      Instr = Builder
                  .buildInstr(MC6809::CMPNZVCAbs, {MI.getOperand(1), N, Z},
                              {MI.getOperand(5)})
                  .add(Addr)
                  .cloneMemRefs(*Load);
    }
    Register Idx;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)))) {
      Instr = Builder
                  .buildInstr(MC6809::CMPNZVCAbsIdx, {MI.getOperand(1), N, Z},
                              {MI.getOperand(5)})
                  .add(Addr)
                  .addUse(Idx)
                  .cloneMemRefs(*Load);
    }
    Register Offset;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load),
                          m_FoldedLdIndirIdx(MI, Idx, Offset, AA)))) {
      Instr = Builder
                  .buildInstr(MC6809::CMPNZVCIndirIdx, {MI.getOperand(1), N, Z},
                              {MI.getOperand(5), Idx, Offset})
                  .cloneMemRefs(*Load);
    }
    if (!Instr) {
      Instr = Builder.buildInstr(MC6809::CMPNZVCImag8, {MI.getOperand(1), N, Z},
                                 {MI.getOperand(5), MI.getOperand(6)});
    }
  } else {
    if (!Instr && RConst) {
      assert(RConst->Value.getBitWidth() == 8);
      Instr = Builder.buildInstr(
          MC6809::SBCNZImm,
          {MI.getOperand(0), MI.getOperand(1), N, MI.getOperand(3), Z},
          {MI.getOperand(5), RConst->Value.getZExtValue(), MI.getOperand(7)});
    }
    MachineOperand Addr =
        MachineOperand::CreateReg(MC6809::NoRegister, /*isDef=*/false);
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)))) {
      Instr = Builder
                  .buildInstr(MC6809::SBCNZAbs,
                              {MI.getOperand(0), MI.getOperand(1), N,
                               MI.getOperand(3), Z},
                              {MI.getOperand(5)})
                  .add(Addr)
                  .add(MI.getOperand(7))
                  .cloneMemRefs(*Load);
    }
    Register Idx;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)))) {
      Instr = Builder
                  .buildInstr(MC6809::SBCNZAbsIdx,
                              {MI.getOperand(0), MI.getOperand(1), N,
                               MI.getOperand(3), Z},
                              {MI.getOperand(5)})
                  .add(Addr)
                  .addUse(Idx)
                  .add(MI.getOperand(7))
                  .cloneMemRefs(*Load);
    }
    Register Offset;
    if (!Instr &&
        mi_match(MI.getOperand(6).getReg(), MRI,
                 m_all_of(m_MInstr(Load),
                          m_FoldedLdIndirIdx(MI, Idx, Offset, AA)))) {
      Instr =
          Builder
              .buildInstr(
                  MC6809::SBCNZIndirIdx,
                  {MI.getOperand(0), MI.getOperand(1), N, MI.getOperand(3), Z},
                  {MI.getOperand(5), RegAddr, Idx, MI.getOperand(7)})
              .cloneMemRefs(*Load);
    }
    if (!Instr) {
      Instr = Builder.buildInstr(
          MC6809::SBCNZImag8,
          {MI.getOperand(0), MI.getOperand(1), N, MI.getOperand(3), Z},
          {MI.getOperand(5), MI.getOperand(6), MI.getOperand(7)});
    }
  }
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
#else
  return false;
#endif
}

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  Register Dst = MI.getOperand(0).getReg();

  std::pair<Register, Register> LoHi = selectFrameIndexLoHi(MI);

  MachineIRBuilder Builder(MI);
  composePtr(Builder, Dst, LoHi.first, LoHi.second);
  MI.eraseFromParent();
  return true;
}

std::pair<Register, Register>
MC6809InstructionSelector::selectFrameIndexLoHi(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);

  MachineInstrBuilder LoAddr;
  MachineInstrBuilder HiAddr;

  bool IsLocal = !MI.getMF()->getFrameInfo().isFixedObjectIndex(
      MI.getOperand(1).getIndex());
  if (MI.getMF()->getFunction().doesNotRecurse() && IsLocal)
    return selectAddrLoHi(MI);

  // Otherwise a soft stack needs to be used, so frame addresses are offsets
  // from the stack/frame pointer. Record this as a pseudo, since the best
  // code to emit depends heavily on the actual offset, which isn't known
  // until FEI.
  LoAddr = Builder.buildInstr(MC6809::AddrLostk, {S8, S1, S1}, {})
               .add(MI.getOperand(1))
               .addImm(0);
  Register Carry = LoAddr.getReg(1);

  HiAddr = Builder.buildInstr(MC6809::AddrHistk, {S8, S1, S1}, {})
               .add(MI.getOperand(1))
               .addImm(0)
               .addUse(Carry);

  if (!constrainSelectedInstRegOperands(*LoAddr, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");
  if (!constrainSelectedInstRegOperands(*HiAddr, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");

  return {LoAddr.getReg(0), HiAddr.getReg(0)};
}

bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto Instr =
      Builder
          .buildInstr(MC6809::LDImm16, {MI.getOperand(0), &MC6809::ACCRegClass}, {})
          .add(MI.getOperand(1));
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  return true;
}

std::pair<Register, Register>
MC6809InstructionSelector::selectAddrLoHi(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  auto LoImm = Builder.buildInstr(MC6809::LDImm, {S8}, {}).add(MI.getOperand(1));
  LoImm->getOperand(1).setTargetFlags(MC6809::MO_LO);
  if (!constrainSelectedInstRegOperands(*LoImm, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");
  auto HiImm = Builder.buildInstr(MC6809::LDImm, {S8}, {}).add(MI.getOperand(1));
  HiImm->getOperand(1).setTargetFlags(MC6809::MO_HI);
  if (!constrainSelectedInstRegOperands(*HiImm, TII, TRI, RBI))
    llvm_unreachable("Cannot constrain instruction.");

  return {LoImm.getReg(0), HiImm.getReg(0)};
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

// Replace all uses of a given virtual register after a given instruction with a
// new one. The given machine instruction must dominate all references outside
// the containing basic block. This allows folding a multi-def machine
// instruction into a later one in the same block by rewriting all later
// references to use new vregs.
static void replaceUsesAfter(MachineBasicBlock::iterator MI, Register From,
                             Register To, const MachineRegisterInfo &MRI) {
  for (MachineInstr &I : make_range(MI, MI->getParent()->end())) {
    for (MachineOperand &Op : I.uses())
      if (Op.isReg() && Op.getReg() == From)
        Op.setReg(To);
  }
  for (MachineOperand &MO : MRI.use_nodbg_operands(From))
    if (MO.getParent()->getParent() != MI->getParent())
      MO.setReg(To);
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
        Builder.buildInstr(MC6809::LDImm16, {Dst, &MC6809::ACCRegClass}, {Val});
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      return false;
    MI.eraseFromParent();
    return true;
  }

  composePtr(Builder, Dst, Lo, Hi);
  MI.eraseFromParent();
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
}

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();

  MachineIRBuilder Builder(MI);

  MachineInstr *SrcMI = getDefIgnoringCopies(Src, *Builder.getMRI());
  Optional<std::pair<Register, Register>> LoHi;
  switch (SrcMI->getOpcode()) {
  case MC6809::G_FRAME_INDEX:
    LoHi = selectFrameIndexLoHi(*SrcMI);
    break;
  case MC6809::G_BLOCK_ADDR:
  case MC6809::G_GLOBAL_VALUE:
    LoHi = selectAddrLoHi(*SrcMI);
    break;
  }
  MachineInstrBuilder LoCopy;
  MachineInstrBuilder HiCopy;
  if (LoHi) {
    LoCopy = Builder.buildCopy(Lo, LoHi->first);
    HiCopy = Builder.buildCopy(Hi, LoHi->second);
  } else {
    LoCopy = Builder.buildCopy(Lo, Src);
    LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_byte);
    HiCopy = Builder.buildCopy(Hi, Src);
    HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_byte);
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
    llvm_unreachable("Unexpected opcode.");
  case MC6809::G_BRINDIRECT:
    Opcode = MC6809::JMPIndir;
    break;
  case MC6809::G_IMPLICIT_DEF:
    Opcode = MC6809::IMPLICIT_DEF;
    break;
  case MC6809::G_LOAD_ABS:
    Opcode = MC6809::LDAbs;
    break;
  case MC6809::G_LOAD_ABS_IDX:
    Opcode = MC6809::LDAbsIdx;
    break;
  case MC6809::G_LOAD_INDIR_IDX:
    Opcode = MC6809::LDIndirIdx;
    break;
  case MC6809::G_PHI:
    Opcode = MC6809::PHI;
    break;
  case MC6809::G_STORE_ABS:
    Opcode = MC6809::STAbs;
    break;
  case MC6809::G_STORE_ABS_IDX:
    Opcode = MC6809::STAbsIdx;
    break;
  case MC6809::G_STORE_INDIR_IDX:
    Opcode = MC6809::STIndirIdx;
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
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.operands()) {
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() ||
        MRI.getRegClassOrNull(Op.getReg()))
      continue;
    LLT Ty = MRI.getType(Op.getReg());
    constrainOperandRegClass(Op, getRegClassForType(Ty));
  }
}

void MC6809InstructionSelector::constrainOperandRegClass(
    MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, MRI, TII, RBI, MI,
                                              RegClass, RegMO));
}

bool MC6809InstructionSelector::selectAll(MachineInstrSpan MIS) {
  MachineRegisterInfo &MRI = MIS.begin()->getMF()->getRegInfo();

  // Ensure that all new generic virtual registers have a register bank.
  for (MachineInstr &MI : MIS)
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
