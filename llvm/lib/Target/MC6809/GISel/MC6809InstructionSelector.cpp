//===-- MC6809InstructionSelector.cpp - MC6809 Instruction Selector
//-------------===//
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

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

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

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

private:
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo &TRI;
  const MC6809RegisterBankInfo &RBI;

#if 0
  // Pre-tablegen selection functions. If these return false, fall through to
  // tablegen.
  bool selectAddSub(MachineInstr &MI);
  bool selectLogical(MachineInstr &MI);
#endif /* 0 */

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectFrameIndex(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectPtrAdd(MachineInstr &MI);
#if 0
  bool selectLshrShlE(MachineInstr &MI);
#endif
  bool selectMergeValues(MachineInstr &MI);
  bool selectConstant(MachineInstr &MI);
  bool selectLoad(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);
  bool selectExt(MachineInstr &MI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

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
  const TargetRegisterClass *guessRegClass(unsigned Reg, MachineRegisterInfo &MRI) const;

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

const TargetRegisterClass *MC6809InstructionSelector::guessRegClass(unsigned Reg, MachineRegisterInfo &MRI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  const RegisterBank *RegBank = RBI.getRegBank(Reg, MRI, TRI);
  assert(RegBank && "Can't get register bank for virtual register");

  const unsigned Size = MRI.getType(Reg).getSizeInBits();
  assert((RegBank->getID() == MC6809::ACCUMRegBankID || RegBank->getID() == MC6809::INDEXRegBankID) && "Unsupported register bank");

  if (RegBank->getID() == MC6809::ACCUMRegBankID) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MC6809::ACCRegBankID : Size = " << Size << "\n";);
    if (Size == 8)
      return &MC6809::ACC8RegClass;
    else if (Size == 16)
      return &MC6809::ACC16RegClass;
    else if (Size == 32)
      return &MC6809::ACC32RegClass;
    else
      llvm_unreachable("Unsupported destination size");
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Return &MC6809::INDEX16RegClass\n";);
  return &MC6809::INDEX16RegClass;
}

const TargetRegisterClass &MC6809InstructionSelector::getRegClassForType(Register Reg, MachineRegisterInfo &MRI) const {
  const LLT Ty = MRI.getType(Reg);
  const unsigned TySize = Ty.getSizeInBits();

  if (RBI.getRegBank(Reg, MRI, TRI)->getID() == MC6809::ACCUMRegBankID) {
    switch (TySize) {
    default:
      llvm_unreachable(
          "Register class (ACC) not available for LLT, RB combination");
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
      llvm_unreachable(
          "Register class (INDEX) not available for LLT, RB combination");
    case 16:
      return MC6809::INDEX16RegClass;
    }
  }

  llvm_unreachable("Unsupported register bank.");
}

bool MC6809InstructionSelector::select(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  assert(MI.getParent() && "Instruction should be in a basic block!");
  assert(MI.getParent()->getParent() && "Instruction should be in a function!");

  auto &MBB = *MI.getParent();
  auto &MF = *MBB.getParent();
  auto &MRI = MF.getRegInfo();

  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

// ============================================================================
#if 0
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
#endif

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
#if 0
  case MC6809::G_LSHRE:
  case MC6809::G_SHLE:
    return selectLshrShlE(MI);
#endif /* 0 */
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
  case MC6809::G_PTR_ADD:
    return selectPtrAdd(MI);

  case MC6809::G_BRINDIRECT:
  case MC6809::G_IMPLICIT_DEF:
  case MC6809::G_PHI:
    return selectGeneric(MI);

  case MC6809::G_SEXT:
  case MC6809::G_ZEXT:
  case MC6809::G_ANYEXT:
    return selectExt(MI);
  }
  return false;
}

bool MC6809InstructionSelector::selectExt(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : MI = ";MI.dump(););
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();
  LLT S1 = LLT::scalar(1);
  LLT S2 = LLT::scalar(2);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = Builder.getMRI()->getType(Dst);
  Register Src = MI.getOperand(1).getReg();
  LLT SrcTy = Builder.getMRI()->getType(Src);

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case MC6809::G_SEXT:
    if (DstTy == S8) {
      assert(SrcTy == S1 && "G_SEXT Src must be S1 for Dst S8");
      auto Instr1 = Builder.buildInstr(MC6809::AND8Imm)
                        .addDef(MRI.createGenericVirtualRegister(S8))
                        .addDef(MRI.createGenericVirtualRegister(S2))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addImm(1)
                        .addUse(Src);
      Register Reg1 = Instr1.getReg(0);
      auto Instr2 = Builder.buildInstr(MC6809::Load8Imm)
                        .addDef(MRI.createGenericVirtualRegister(S8))
                        .addDef(MRI.createGenericVirtualRegister(S2))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addImm(0);
      Register Reg2 = Instr2.getReg(0);
      auto Instr3 = Builder.buildInstr(MC6809::Sub8Reg)
                        .addDef(Dst)
                        .addDef(MRI.createGenericVirtualRegister(S2))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addDef(MRI.createGenericVirtualRegister(S1))
                        .addUse(Reg2)
                        .addUse(Reg1);
      if (!constrainSelectedInstRegOperands(*Instr3, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s1 -> s8 instructions.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S16) {
      assert(SrcTy == S8 && "G_SEXT Src must be S8 for Dst S16");
      auto Instr =
          Builder.buildInstr(MC6809::SEX16Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s8 -> s16 instruction.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S32) {
      assert(SrcTy == S8 && "G_SEXT Src must be S16 for Dst S32");
      auto Instr =
          Builder.buildInstr(MC6809::SEX32Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s16 -> s32 instruction.");
      MI.eraseFromParent();
      return true;
    }
    break;
  case MC6809::G_ANYEXT:
  case MC6809::G_ZEXT:
    if (DstTy == S8) {
      assert(SrcTy == S1 && "G_ZEXT Src must be S1 for Dst S8");
      auto Instr = Builder.buildInstr(MC6809::AND8Imm)
                       .addDef(Dst)
                       .addDef(MRI.createGenericVirtualRegister(S2))
                       .addDef(MRI.createGenericVirtualRegister(S1))
                       .addDef(MRI.createGenericVirtualRegister(S1))
                       .addImm(1)
                       .addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s1 -> s8 instructions.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S16) {
      assert(SrcTy == S8 && "G_ZEXT Src must be S8 for Dst S16");
      auto Instr =
          Builder.buildInstr(MC6809::ZEX16Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s8 -> s16 instruction.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S32) {
      assert(SrcTy == S8 && "G_ZEXT Src must be S16 for Dst S32");
      auto Instr =
          Builder.buildInstr(MC6809::ZEX32Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s16 -> s32 instruction.");
      MI.eraseFromParent();
      return true;
    }
    break;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : FALSE : MI = "; MI.dump(););
  return false;
}

static bool shouldFoldMemAccess(const MachineInstr &Dst, const MachineInstr &Src, AAResults *AA) {
  assert(Src.mayLoadOrStore());

  // For now, don't attempt to fold across basic block boundaries.
  if (Dst.getParent() != Src.getParent())
    return false;

  if ((*Src.memoperands_begin())->isVolatile())
    return false;

  // Does it pay off to fold the access? Depends on the number of users.
  const auto &MRI = Dst.getMF()->getRegInfo();
  const auto Users = MRI.use_nodbg_instructions(Src.getOperand(0).getReg());
  const auto NumUsers = std::distance(Users.begin(), Users.end());

  // Look for intervening instructions that cannot be folded across.
  for (const MachineInstr &MI : make_range(std::next(MachineBasicBlock::const_iterator(Src)), MachineBasicBlock::const_iterator(Dst))) {
    if (MI.isCall() || MI.hasUnmodeledSideEffects())
      return false;
    if (MI.mayLoadOrStore()) {
      if (Src.hasOrderedMemoryRef() || MI.hasOrderedMemoryRef())
        return false;
      if (MI.mayAlias(AA, Src, /*UseTBAA=*/true))
        return false;
      // Note: Dst may be a store, indicating that the whole sequence is a RMW
      // operation.
      if (MI.mayAlias(AA, Dst, /*UseTBAA=*/true))
        return false;
    }
  }
  return true;
}

#if 0
struct FoldedLdAbs_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdAbs = getOpcodeDef(MC6809::G_LOAD_ABS, Reg, MRI);
    if (!LdAbs || !shouldFoldMemAccess(Tgt, *LdAbs, AA))
      return false;
    Addr = LdAbs->getOperand(1);
    return true;
  }
};
inline FoldedLdAbs_match m_FoldedLdAbs(const MachineInstr &Tgt, MachineOperand &Addr, AAResults *AA) {
  return {Tgt, Addr, AA};
}

struct FoldedLdIdx_match {
  const MachineInstr &Tgt;
  MachineOperand &Addr;
  Register &Idx;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LDAbsIdx = getOpcodeDef(MC6809::G_LOAD_ABS_IDX, Reg, MRI);
    if (!LDAbsIdx || !shouldFoldMemAccess(Tgt, *LDAbsIdx, AA))
      return false;
    Addr = LDAbsIdx->getOperand(1);
    Idx = LDAbsIdx->getOperand(2).getReg();
    return true;
  }
};
inline FoldedLdIdx_match m_FoldedLdIdx(const MachineInstr &Tgt, MachineOperand &Addr, Register &Idx, AAResults *AA) {
  return {Tgt, Addr, Idx, AA};
}

struct FoldedLdIndirIdx_match {
  const MachineInstr &Tgt;
  Register &Addr;
  Register &Idx;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdIndirIdx = getOpcodeDef(MC6809::G_LOAD_INDIR_IDX, Reg, MRI);
    if (!LdIndirIdx || !shouldFoldMemAccess(Tgt, *LdIndirIdx, AA))
      return false;
    Addr = LdIndirIdx->getOperand(1).getReg();
    Idx = LdIndirIdx->getOperand(2).getReg();
    return true;
  }
};
inline FoldedLdIndirIdx_match m_FoldedLdIndirIdx(const MachineInstr &Tgt, Register &Addr, Register &Idx, AAResults *AA) {
  return {Tgt, Addr, Idx, AA};
}
#endif /* 0 */

#if 0
bool MC6809InstructionSelector::selectAddSub(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOpcode() == MC6809::G_ADD || MI.getOpcode() == MC6809::G_SUB);
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();
  bool Success;

  unsigned Opcode;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  MachineInstr *Load;
  LLT S1 = LLT::scalar(1);
  Register LHS;
  Register Idx;
  if (MI.getOpcode() == MC6809::G_ADD) {
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::ADCAi_o16;
  } else {
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GSub(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::SBCAi_o16;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute indexed instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Offset;
  if (MI.getOpcode() == MC6809::G_ADD) {
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::ADCAi_o16;
  } else {
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GSub(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::SBCAi_o16;
  }
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg())
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addDef(MRI.createGenericVirtualRegister(S1))
                     .addUse(LHS)
                     .addUse(Offset)
                     .addUse(Idx)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect indexed instruction.");
    MI.eraseFromParent();
    return true;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return false;
}
#endif /* 0 */

#if 0
bool MC6809InstructionSelector::selectLogical(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  Register LHS;
  MachineOperand Addr = MachineOperand::CreateReg(0, false);

  MachineInstr *Load;

  bool Success;
  Register Opcode;
  switch (MI.getOpcode()) {
  case MC6809::G_AND:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GAnd(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::ANDAbs;
    break;
  case MC6809::G_XOR:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GXor(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
    Opcode = MC6809::EORAbs;
    break;
  case MC6809::G_OR:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GOr(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
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
    Success = mi_match( MI.getOperand(0).getReg(), MRI, m_GAnd(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::ANDAbsIdx;
    break;
  case MC6809::G_XOR:
    Success = mi_match( MI.getOperand(0).getReg(), MRI, m_GXor(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
    Opcode = MC6809::EORAbsIdx;
    break;
  case MC6809::G_OR:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GOr(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA))));
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
      llvm_unreachable( "Could not constrain absolute indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }

  Register Offset;
  switch (MI.getOpcode()) {
  case MC6809::G_AND:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GAnd(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::ANDIndirIdx;
    break;
  case MC6809::G_XOR:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GXor(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
    Opcode = MC6809::EORIndirIdx;
    break;
  case MC6809::G_OR:
    Success = mi_match(MI.getOperand(0).getReg(), MRI, m_GOr(m_Reg(LHS), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdx(MI, Idx, Offset, AA))));
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
      llvm_unreachable( "Could not constrain absolute indexed logical instruction.");
    MI.eraseFromParent();
    return true;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return false;
}
#endif

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();
  Register Src = MI.getOperand(0).getReg();
  LLT PtrTy = MI.getOperand(0).isReg() ? MRI.getType(MI.getOperand(0).getReg()) : LLT{};
  if (PtrTy != LLT::pointer(0, 16)) {
    LLVM_DEBUG(dbgs() << "G_FRAME_INDEX pointer has type: " << PtrTy << ", expected: " << LLT::pointer(0, 16) << '\n');
    return false;
  }
  auto FrameIndexOp = MI.getOperand(1);

  MI.setDesc(TII.get(MC6809::LEAPtrAddImm));
  MI.addOperand(MachineOperand::CreateImm(0));

  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
}

bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  auto Instr = Builder.buildInstr(MC6809::Load16Imm, {MI.getOperand(0), &MC6809::INDEX16RegClass}, {MI.getOperand(1)});
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}

#if 0
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
    return Addr.match(MRI, GShlE->getOperand(2).getReg()) && CarryIn.match(MRI, GShlE->getOperand(3).getReg());
  }
};

template <typename ADDR_P, typename CARRYIN_P>
GShlE_match<ADDR_P, CARRYIN_P> m_GShlE(Register &CarryOut, const ADDR_P &Addr, const CARRYIN_P &CarryIn) {
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
    return Addr.match(MRI, GLshrE->getOperand(2).getReg()) && CarryIn.match(MRI, GLshrE->getOperand(3).getReg());
  }
};

template <typename ADDR_P, typename CARRYIN_P>
GLshrE_match<ADDR_P, CARRYIN_P> m_GLshrE(Register &CarryOut, const ADDR_P &Addr, const CARRYIN_P &CarryIn) {
  return {CarryOut, Addr, CarryIn};
}
#endif /* 0 */

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();
  Register Src = MI.getOperand(0).getReg();
  LLT SrcTy = MRI.getType(Src);
  const auto SrcSize = SrcTy.getSizeInBits();

  unsigned NewOpc;
  switch  (SrcSize) {
  default:
    llvm_unreachable(formatv("Cannot store register of size = {}", SrcSize).str().c_str());
  case 8:
    NewOpc = MC6809::Store8IdxZero;
    break;
  case 16:
    NewOpc = MC6809::Store16IdxZero;
    break;
  case 32:
    NewOpc = MC6809::Store32IdxZero;
    break;
  }
  MI.setDesc(TII.get(NewOpc));
  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
#if 0
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstr *Load;

  // Read-modify-write instruction patterns are rooted at store instructions, so
  // select one if possible. This can make an entire instruction sequence dead.
  if (MI.getOpcode() == MC6809::G_STORE_ABS) {
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_SpecificICst(1))) && Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MC6809::INCAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_SpecificICst(-1))) && Addr.isIdenticalTo(MI.getOperand(1))) {
      Builder.buildInstr(MC6809::DECAbs).add(Addr).cloneMergedMemRefs({&MI, Load});
      MI.eraseFromParent();
      return true;
    }
    Register CarryOut;
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GShlE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_SpecificICst(0))) && Addr.isIdenticalTo(MI.getOperand(1))) {
      auto Asl = Builder.buildInstr(MC6809::ASLAbs, {&MC6809::CcRegClass}, {})
                     .add(Addr)
                     .cloneMergedMemRefs({&MI, Load});
      replaceUsesAfter(Asl, CarryOut, Asl.getReg(0), MRI);
      if (!constrainSelectedInstRegOperands(*Asl, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GLshrE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_SpecificICst(0))) && Addr.isIdenticalTo(MI.getOperand(1))) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GShlE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_Reg(CarryIn))) && Addr.isIdenticalTo(MI.getOperand(1))) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GLshrE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA)), m_Reg(CarryIn))) && Addr.isIdenticalTo(MI.getOperand(1))) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_SpecificICst(1))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
      auto Inc = Builder.buildInstr(MC6809::INCAbsIdx)
                     .add(Addr)
                     .addUse(Idx)
                     .cloneMergedMemRefs({&MI, Load});
      if (!constrainSelectedInstRegOperands(*Inc, TII, TRI, RBI))
        return false;
      MI.eraseFromParent();
      return true;
    }
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GAdd(m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_SpecificICst(-1))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GShlE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_SpecificICst(0))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GLshrE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_SpecificICst(0))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GShlE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_Reg(CarryIn))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
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
    if (mi_match(MI.getOperand(0).getReg(), MRI, m_GLshrE(CarryOut, m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Idx, Addr, AA)), m_Reg(CarryIn))) && Addr.isIdenticalTo(MI.getOperand(1)) && Idx == MI.getOperand(2).getReg()) {
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
  if (!STI.has65C02() || !isOperandImmEqual(MI.getOperand(0), 0, *Builder.getMRI()))
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  return true;
#endif
}

bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  const MachineRegisterInfo &MRI = *Builder.getMRI();

  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, MRI);
  const unsigned Size = MRI.getType(Dst).getSizeInBits();
  if (LoConst && HiConst) {
    if (Size == 16) {
      uint64_t Val = HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load16Imm, {Dst, &MC6809::ACC16RegClass}, {Val});
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        return false;
    } else if (Size == 32) {
      uint64_t Val = HiConst->Value.getZExtValue() << 16 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load32Imm, {Dst, &MC6809::ACC32RegClass}, {Val});
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        return false;
    }
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit Const : MI = "; MI.dump(););
    return true;
  }
  auto RegSeq = Builder.buildInstr(MC6809::REG_SEQUENCE).addDef(Dst);
  if (Size == 16)
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_byte).addUse(Hi).addImm(MC6809::sub_hi_byte);
  else
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_word).addUse(Hi).addImm(MC6809::sub_hi_word);
  constrainGenericOp(*RegSeq);
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit Merged : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectPtrAdd(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";
             MI.dump(););
  MachineIRBuilder Builder(MI);
  const MachineRegisterInfo &MRI = *Builder.getMRI();
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT P = LLT::pointer(0, 16);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = Builder.getMRI()->getType(Dst);
  Register Ptr = MI.getOperand(1).getReg();
  LLT PtrTy = Builder.getMRI()->getType(Ptr);
  Register Offset = MI.getOperand(2).getReg();
  LLT OffsetTy = Builder.getMRI()->getType(Offset);
  assert(DstTy == P && PtrTy == P && "Destination and source must both be pointer types.");

  unsigned Opcode;
  if (MI.getOperand(2).isCImm()) {
    uint64_t Val = MI.getOperand(2).getCImm()->getSExtValue();
    MI.getOperand(2).ChangeToImmediate(Val);
    Opcode = MC6809::LEAPtrAddImm;
  } else {
    if (OffsetTy == S8) {
      Opcode = MC6809::LEAPtrAddReg8;
    } else if (OffsetTy == S16) {
      Opcode = MC6809::LEAPtrAddReg16;
    } else
      llvm_unreachable("Must be adding an 8- or 16-bit register quantity to the pointer.");
  }
  MI.setDesc(TII.get(Opcode));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectConstant(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
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
    Opcode = MC6809::Load8Imm;
  } else if (DstTy == S16) {
    Opcode = MC6809::Load16Imm;
  } else if (DstTy == S32) {
    Opcode = MC6809::Load32Imm;
  } else {
    llvm_unreachable("Can't select G_CONSTANT, unsupported type.");
  }

  MI.setDesc(TII.get(Opcode));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectLoad(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI.getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  unsigned NewOpc;
  switch  (DstSize) {
  default:
    llvm_unreachable(formatv("Cannot store register of size = {}", DstSize).str().c_str());
  case 8:
    NewOpc = MC6809::Load8IdxZero;
    break;
  case 16:
    NewOpc = MC6809::Load16IdxZero;
    break;
  case 32:
    NewOpc = MC6809::Load32IdxZero;
    break;
  }
  MI.setDesc(TII.get(NewOpc));
  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
}

#if 0
bool MC6809InstructionSelector::selectLshrShlE(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

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

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return false;
}
#endif

bool MC6809InstructionSelector::selectTrunc(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectAddE(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return false;
#endif
}

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter: MI = "; MI.dump(););
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter: MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectGeneric(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = ";
    // MI.dump(););
    llvm_unreachable("Select Generic - Unexpected opcode.");
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
  if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit cannot constrainSelectedInstRegOperands()\n";);
    return false;
  }
  // Make sure that the outputs have register classes.
  constrainGenericOp(MI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
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
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Attempting to constrain Op = " << Op << "\n";);
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() ||
        MRI.getRegClassOrNull(Op.getReg()))
      continue;
    constrainOperandRegClass(Op, getRegClassForType(Op.getReg(), MRI));
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constrained Op = " << Op << "\n";);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstructionSelector::constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, MRI, TII, RBI, MI, RegClass, RegMO));
}

bool MC6809InstructionSelector::selectAll(MachineInstrSpan MIS) {
  MachineRegisterInfo &MRI = MIS.begin()->getMF()->getRegInfo();

  // Ensure that all new generic virtual registers have a register bank.
  for (MachineInstr &MI : MIS) {
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

  for (MachineInstr &MI : MIS) {
    if (!select(MI))
      return false;
  }
  return true;
}

InstructionSelector *llvm::createMC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI) {
  return new MC6809InstructionSelector(TM, STI, RBI);
}
