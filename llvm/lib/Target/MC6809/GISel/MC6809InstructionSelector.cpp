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
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelectorImpl.h"
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

private:
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo &TRI;
  const MC6809RegisterBankInfo &RBI;

  MachineBasicBlock *MBB;
  MachineFunction *MF;
  MachineRegisterInfo *MRI;

  // Post-tablegen selection functions. If these return false, it is an error.
  bool selectFrameIndex(MachineInstr &MI);
  bool selectAddr(MachineInstr &MI);
  bool selectStore(MachineInstr &MI);
  bool selectPtrAdd(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectConstant(MachineInstr &MI);
  bool selectLoad(MachineInstr &MI);
  bool selectTrunc(MachineInstr &MI);

  bool selectUnMergeValues(MachineInstr &MI);
  bool selectExt(MachineInstr &MI);
  bool selectAdd(MachineInstr &MI);
  bool selectSub(MachineInstr &MI);

  bool selectBranch(MachineInstr &MI);
  bool selectConditionalBranch(MachineInstr &MI, MachineFunction &MF, MachineRegisterInfo &MRI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass);

  // Select all instructions in a given span, recursively. Allows selecting an
  // instruction sequence by reducing it to a more easily selectable sequence.
  bool selectAll(MachineInstrSpan MIS);

  /// tblgen-erated 'select' implementation, used as the initial selector for
  /// the patterns that don't require complex C++.
  bool selectImpl(MachineInstr &MI, CodeGenCoverage &CoverageInfo) const;

  const TargetRegisterClass &getRegClassForType(Register Reg) const;

  MachineInstr *tryFoldIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIRBuilder) const;
  bool tryOptAndIntoCompareBranch(MachineInstr &AndInst, bool Invert, MachineBasicBlock *DstMBB, MachineIRBuilder &MIB) const;
  bool tryOptCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const;
  MachineInstr *emitCMN(MachineOperand &LHS, MachineOperand &RHS, MachineIRBuilder &MIRBuilder) const;
  MachineInstr *emitIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIRBuilder) const;
  bool selectCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const;

  LLT S1 = LLT::scalar(1);
  LLT S2 = LLT::scalar(2);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT P = LLT::pointer(0, 16);

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

const TargetRegisterClass &MC6809InstructionSelector::getRegClassForType(Register Reg) const {
  const LLT Ty = MRI->getType(Reg);
  const unsigned TySize = Ty.getSizeInBits();

  if (RBI.getRegBank(Reg, *MRI, TRI)->getID() == MC6809::ACCUMRegBankID) {
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

  if (RBI.getRegBank(Reg, *MRI, TRI)->getID() == MC6809::INDEXRegBankID) {
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  assert(MI.getParent() && "Instruction should be in a basic block!");
  assert(MI.getParent()->getParent() && "Instruction should be in a function!");

  MBB = MI.getParent();
  MF = MBB->getParent();
  MRI = &MF->getRegInfo();

  // isPreISelOpcode is stolen from llvm-mos. Methinks it means "not a GlobalISel opcode".
  if (!MI.isPreISelOpcode()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 10 : !MI.isPreISelOpcode() : MI = ";MI.dump(););
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : before selectImpl() : MI = "; MI.dump(););
  if (selectImpl(MI, *CoverageInfo))
    return true;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 30 : before switch(MI.getOpcode()) : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  default:
    return false;
  case MC6809::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case MC6809::G_BLOCK_ADDR:
  case MC6809::G_GLOBAL_VALUE:
    return selectAddr(MI);
  case MC6809::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case MC6809::G_TRUNC:
    return selectTrunc(MI);
  case MC6809::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case MC6809::G_LOAD:
  case MC6809::G_LOAD_INDEX_OFFSET:
    return selectLoad(MI);
  case MC6809::G_STORE:
  case MC6809::G_STORE_INDEX_OFFSET:
    return selectStore(MI);
  case MC6809::G_PTR_ADD:
    return selectPtrAdd(MI);
  case MC6809::G_CONSTANT:
    return selectConstant(MI);

  case MC6809::G_BR:
    return selectBranch(MI);

  case TargetOpcode::G_BRCOND:
    return selectConditionalBranch(MI, *MF, *MRI);

  case MC6809::G_BRINDIRECT:
  case MC6809::G_IMPLICIT_DEF:
  case MC6809::G_PHI:
    return selectGeneric(MI);

  case MC6809::G_SEXT:
  case MC6809::G_ZEXT:
  case MC6809::G_ANYEXT:
    return selectExt(MI);

  case MC6809::G_ADD:
  case MC6809::G_SADDE:
  case MC6809::G_SADDO:
  case MC6809::G_UADDE:
  case MC6809::G_UADDO:
    return selectAdd(MI);
  case MC6809::G_SUB:
  case MC6809::G_USUBE:
  case MC6809::G_SSUBE:
  case MC6809::G_USUBO:
  case MC6809::G_SSUBO:
    return selectSub(MI);
  }
  return false;
}

bool MC6809InstructionSelector::selectExt(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  Register Src = MI.getOperand(1).getReg();
  LLT SrcTy = MRI->getType(Src);

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case MC6809::G_SEXT:
    if (DstTy == S8) {
      assert(SrcTy == S1 && "G_SEXT Src must be S1 for Dst S8");
      auto Instr1 = Builder.buildInstr(MC6809::AND8Imm)
                        .addDef(MRI->createGenericVirtualRegister(S8))
                        .addDef(MRI->createGenericVirtualRegister(S2))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addImm(1)
                        .addUse(Src);
      Register Reg1 = Instr1.getReg(0);
      auto Instr2 = Builder.buildInstr(MC6809::Load8Imm)
                        .addDef(MRI->createGenericVirtualRegister(S8))
                        .addDef(MRI->createGenericVirtualRegister(S2))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addImm(0);
      Register Reg2 = Instr2.getReg(0);
      auto Instr3 = Builder.buildInstr(MC6809::Sub8Reg)
                        .addDef(Dst)
                        .addDef(MRI->createGenericVirtualRegister(S2))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addDef(MRI->createGenericVirtualRegister(S1))
                        .addUse(Reg2)
                        .addUse(Reg1);
      if (!constrainSelectedInstRegOperands(*Instr3, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s1 -> s8 instructions.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S16) {
      assert(SrcTy == S8 && "G_SEXT Src must be S8 for Dst S16");
      auto Instr = Builder.buildInstr(MC6809::SEX16Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s8 -> s16 instruction.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S32) {
      assert(SrcTy == S8 && "G_SEXT Src must be S16 for Dst S32");
      auto Instr = Builder.buildInstr(MC6809::SEX32Implicit).addDef(Dst).addUse(Src);
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
                       .addDef(MRI->createGenericVirtualRegister(S2))
                       .addDef(MRI->createGenericVirtualRegister(S1))
                       .addDef(MRI->createGenericVirtualRegister(S1))
                       .addImm(1)
                       .addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s1 -> s8 instructions.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S16) {
      assert(SrcTy == S8 && "G_ZEXT Src must be S8 for Dst S16");
      auto Instr = Builder.buildInstr(MC6809::ZEX16Implicit).addDef(Dst).addUse(Src);
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain sext s8 -> s16 instruction.");
      MI.eraseFromParent();
      return true;
    }
    if (DstTy == S32) {
      assert(SrcTy == S8 && "G_ZEXT Src must be S16 for Dst S32");
      auto Instr = Builder.buildInstr(MC6809::ZEX32Implicit).addDef(Dst).addUse(Src);
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

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  MI.setDesc(TII.get(MC6809::LEAPtrAdd));
  MI.addOperand(MachineOperand::CreateImm(0));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MI.setDesc(TII.get(MC6809::LoadPtrImm));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
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
      auto Instr = Builder.buildInstr(MC6809::Load16Imm, {Dst, &MC6809::ACC16RegClass}, {Val});
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        return false;
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Instr = "; Instr->dump(););
    } else if (Size == 32) {
      uint64_t Val = HiConst->Value.getZExtValue() << 16 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load32Imm, {Dst, &MC6809::ACC32RegClass}, {Val});
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        return false;
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Instr = "; Instr->dump(););
    }
    MI.eraseFromParent();
    return true;
  }
  auto RegSeq = Builder.buildInstr(MC6809::REG_SEQUENCE).addDef(Dst);
  if (Size == 16)
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_byte).addUse(Hi).addImm(MC6809::sub_hi_byte);
  else
    RegSeq.addUse(Lo).addImm(MC6809::sub_lo_word).addUse(Hi).addImm(MC6809::sub_hi_word);
  constrainGenericOp(*RegSeq);
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit Merged : RegSeq = "; RegSeq->dump(););
  return true;
}

bool MC6809InstructionSelector::selectPtrAdd(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  unsigned Opcode;
  if (MI.getOperand(2).isCImm()) {
    uint64_t Val = MI.getOperand(2).getCImm()->getSExtValue();
    MI.getOperand(2).ChangeToImmediate(Val);
  } else if (MI.getOperand(2).isReg()) {
    const MachineInstr *GConst = getOpcodeDef(MC6809::G_CONSTANT, MI.getOperand(2).getReg(), *MRI);
    if (GConst) {
      if (GConst->getOperand(1).isCImm())
        MI.getOperand(2).ChangeToImmediate(GConst->getOperand(1).getCImm()->getSExtValue());
      else
        MI.getOperand(2).ChangeToImmediate(GConst->getOperand(1).getImm());
    }
  }
  Opcode = MC6809::LEAPtrAdd;
  MI.setDesc(TII.get(Opcode));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
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
       make_range(std::next(MachineBasicBlock::const_iterator(Src)), MachineBasicBlock::const_iterator(Dst))) {
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

struct FoldedLdIdxOffs_match {
  const MachineInstr &Tgt;
  MachineOperand &Index;
  MachineOperand &Offset;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LDIdxOffs = getOpcodeDef(MC6809::G_LOAD_INDEX_OFFSET, Reg, MRI);
    if (!LDIdxOffs || !shouldFoldMemAccess(Tgt, *LDIdxOffs, AA))
      return false;
    Index = LDIdxOffs->getOperand(1);
    Offset = LDIdxOffs->getOperand(2);
    return true;
  }
};

inline FoldedLdIdxOffs_match m_FoldedLdIdxOffs(const MachineInstr &Tgt, MachineOperand &Index, MachineOperand &Offset, AAResults *AA) {
  return {Tgt, Index, Offset, AA};
}

struct FoldedLdIndirIdxOffs_match {
  const MachineInstr &Tgt;
  MachineOperand &Index;
  MachineOperand &Offset;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *LdIndirIdxOffs = getOpcodeDef(MC6809::G_LOAD_INDIR_INDEX_OFFSET, Reg, MRI);
    if (!LdIndirIdxOffs || !shouldFoldMemAccess(Tgt, *LdIndirIdxOffs, AA))
      return false;
    Index = LdIndirIdxOffs->getOperand(1);
    Offset = LdIndirIdxOffs->getOperand(2);
    return true;
  }
};

inline FoldedLdIndirIdxOffs_match m_FoldedLdIndirIdxOffs(const MachineInstr &Tgt, MachineOperand &Index, MachineOperand &Offset, AAResults *AA) {
  return {Tgt, Index, Offset, AA};
}

bool MC6809InstructionSelector::selectAdd(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOpcode() == MC6809::G_ADD ||
         MI.getOpcode() == MC6809::G_SADDO || MI.getOpcode() == MC6809::G_UADDO ||
         MI.getOpcode() == MC6809::G_SADDE || MI.getOpcode() == MC6809::G_UADDE);

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool SetCarry = MI.getOpcode() != MC6809::G_ADD;
  bool UseCarry = MI.getOpcode() == MC6809::G_UADDE || MI.getOpcode() == MC6809::G_SADDE;
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load, *Instr1, *Instr2;
  Register Reg, Reg1, CarryIn;
  unsigned Opcode = 0;
  int LHSOp, RHSOp;

  if (SetCarry) {
    CarryOut = MI.getOperand(1).getReg();
    LHSOp = 2;
    RHSOp = 3;
  } else {
    CarryOut = MRI->createGenericVirtualRegister(S1);
    LHSOp = 1;
    RHSOp = 2;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for possible physical register in inconvenient location for ADD\n";);
  if (MI.getOperand(RHSOp).isReg()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found register on RHS operand of ADD\n";);
    if (mi_match(MI.getOperand(RHSOp).getReg(), *MRI, m_Copy(m_Reg(Reg1)))) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of register on RHS operand of ADD\n";);
      if (Reg1.isPhysical()) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of physical register on RHS operand of ADD\n";);
        auto Temp = MI.getOperand(LHSOp).getReg();
        MI.getOperand(LHSOp).setReg(MI.getOperand(RHSOp).getReg());
        MI.getOperand(RHSOp).setReg(Temp);
      }
    }
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for possible load in inconvenient location for ADD\n";);
  if (MI.getOperand(LHSOp).isReg() || MI.getOperand(RHSOp).isReg()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found registers on LHS and RHS operand of ADD\n";);
    if (mi_match(MI.getOperand(LHSOp).getReg(), *MRI, m_MInstr(Instr1)) &&
        mi_match(MI.getOperand(RHSOp).getReg(), *MRI, m_MInstr(Instr2))) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found both LHS and RHS instructions for ADD\n";);
      if (Instr1->getOpcode() == MC6809::G_LOAD || Instr1->getOpcode() == MC6809::G_LOAD_INDEX_OFFSET) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found LOAD of register on LHS operand of ADD\n";);
        if (!(Instr2->getOpcode() == MC6809::G_LOAD || Instr2->getOpcode() == MC6809::G_LOAD_INDEX_OFFSET)) {
          LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found other than LOAD of register on RHS operand of ADD\n";);
          auto Temp = MI.getOperand(LHSOp).getReg();
          MI.getOperand(LHSOp).setReg(MI.getOperand(RHSOp).getReg());
          MI.getOperand(RHSOp).setReg(Temp);
        }
      }
    }
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying immediate mode\n";);
  Optional<ValueAndVReg> ValReg;
  int64_t Value;
  if (SetCarry) {
    if (UseCarry)
      Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GSAddE(m_GCst(ValReg), m_Reg(Reg), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GUAddE(m_GCst(ValReg), m_Reg(Reg), m_Reg(CarryIn)));
    else
      Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_GCst(ValReg))) ||
                mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_GCst(ValReg))) ||
                mi_match(Dst, *MRI, m_GSAddO(m_GCst(ValReg), m_Reg(Reg))) ||
                mi_match(Dst, *MRI, m_GUAddO(m_GCst(ValReg), m_Reg(Reg)));
  } else
    Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_GCst(ValReg))) ||
              mi_match(Dst, *MRI, m_GAdd(m_GCst(ValReg), m_Reg(Reg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    if (UseCarry) {
      Opcode = DstSize == 8 ? MC6809::AddCarry8Imm : (DstSize == 16 ? MC6809::AddCarry16Imm : MC6809::AddCarry32Imm);
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addDef(CarryOut)
                       .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                       .addUse(Reg)
                       .addUse(CarryIn)
                       .addImm(Value);
    } else {
      Opcode = DstSize == 8 ? MC6809::Add8Imm : (DstSize == 16 ? MC6809::Add16Imm : MC6809::Add32Imm);
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addDef(CarryOut)
                       .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                       .addUse(Reg)
                       .addImm(Value);
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain immediate instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Immediate : Instr = "; Instr->dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Looking for load (for addressing mode)\n";);
  if (SetCarry) {
    if (UseCarry)
      Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_MInstr(Load), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GSAddE(m_MInstr(Load), m_Reg(Reg), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_MInstr(Load), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GUAddE(m_MInstr(Load), m_Reg(Reg), m_Reg(CarryIn)));
    else
      Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_MInstr(Load))) ||
                mi_match(Dst, *MRI, m_GSAddO(m_MInstr(Load), m_Reg(Reg))) ||
                mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_MInstr(Load))) ||
                mi_match(Dst, *MRI, m_GUAddO(m_MInstr(Load), m_Reg(Reg)));
  } else
    Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_MInstr(Load))) ||
              mi_match(Dst, *MRI, m_GAdd(m_MInstr(Load), m_Reg(Reg)));
  if (Success) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Maybe found load\n";);
    if (Load->getOpcode() == MC6809::G_LOAD || Load->getOpcode() == MC6809::G_LOAD_INDEX_OFFSET) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Proceeding with G_LOAD(_INDEX_OFFSET)\n";);
      if (UseCarry) {
        Opcode = DstSize == 8 ? MC6809::AddCarry8Idx : (DstSize == 16 ? MC6809::AddCarry16Idx : MC6809::AddCarry32Idx);
        Instr = Builder.buildInstr(Opcode)
                         .addDef(Dst)
                         .addDef(CarryOut)
                         .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                         .addUse(Reg)                                   // LHS
                         .addUse(CarryIn)
                         .add(Load->getOperand(1))
                         .addImm(0)
                         .cloneMemRefs(*Load);
      } else {
        Opcode = DstSize == 8 ? MC6809::Add8Idx : (DstSize == 16 ? MC6809::Add16Idx : MC6809::Add32Idx);
        Instr = Builder.buildInstr(Opcode)
                         .addDef(Dst)
                         .addDef(CarryOut)
                         .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                         .addUse(Reg)                                   // LHS
                         .add(Load->getOperand(1))
                         .addImm(0)
                         .cloneMemRefs(*Load);
      }
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain indexed add instruction.");
      MI.eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Indexed : Instr = "; Instr->dump(););
      return true;
    }
  }

  if (STI.isHD6309()) {
    Register LHS, RHS;
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying register/register mode\n";);
    if (SetCarry) {
      if (UseCarry)
        Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(CarryIn))) ||
                  mi_match(Dst, *MRI, m_GUAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(CarryIn)));
      else
        Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(LHS), m_Reg(RHS))) ||
                  mi_match(Dst, *MRI, m_GUAddO(m_Reg(LHS), m_Reg(RHS)));
    } else
        Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(LHS), m_Reg(RHS)));
    if (Success) {
      if (UseCarry) {
        Opcode = (DstSize == 8) ? MC6809::AddCarry8Reg : MC6809::AddCarry16Reg;
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(CarryOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(LHS)
                    .addUse(CarryIn)
                    .addUse(RHS);
      } else {
        Opcode = (DstSize == 8) ? MC6809::Add8Reg : MC6809::Add16Reg;
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(CarryOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(LHS)
                    .addUse(RHS);
      }
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain register/register add instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Reg/Reg : Instr = "; Instr->dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return false : MI = "; MI.dump(););
  return false;
}

#if 0
bool MC6809InstructionSelector::selectAdd32(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);

  MachineOperand Dst = MI.getOperand(0);
  MachineOperand Src = MI.getOperand(1);
  auto Carry = MRI->createGenericVirtualRegister(S1);

  MachineInstrBuilder LoAdd = Builder.buildInstr(MC6809::Add16Idx)
                                    .add(Dst)
                                    .addDef(Carry)
                                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                                    .add(Src);
  LoAdd->getOperand(0).setSubReg(MC6809::sub_lo_word);
  LoAdd->getOperand(3).setSubReg(MC6809::sub_lo_word);
  if (!constrainSelectedInstRegOperands(*LoAdd, TII, TRI, RBI))
    llvm_unreachable("Could not constrain 32-bit add low half instruction.");
  MachineInstrBuilder HiAdd = Builder.buildInstr(MC6809::AddCarry16Idx)
                                    .add(Dst)
                                    .addDef(MRI->createGenericVirtualRegister(S1)) // Carry
                                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                                    .add(Src)
                                    .addUse(Carry);
  HiAdd->getOperand(0).setSubReg(MC6809::sub_hi_word);
  HiAdd->getOperand(3).setSubReg(MC6809::sub_hi_word);
  if (!constrainSelectedInstRegOperands(*HiAdd, TII, TRI, RBI))
    llvm_unreachable("Could not constrain 32-bit add high half instruction.");
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : LoAdd = "; LoAdd->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : HiAdd = "; HiAdd->dump(););
  return true;
}
#endif

bool MC6809InstructionSelector::selectSub(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOpcode() == MC6809::G_SUB ||
         MI.getOpcode() == MC6809::G_SSUBO || MI.getOpcode() == MC6809::G_USUBO ||
         MI.getOpcode() == MC6809::G_SSUBE || MI.getOpcode() == MC6809::G_USUBE);
  
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register BorrowOut;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool SetBorrow = MI.getOpcode() != MC6809::G_SUB;
  bool UseBorrow = MI.getOpcode() == MC6809::G_USUBE || MI.getOpcode() == MC6809::G_SSUBE;
  bool Success;
  MachineInstr *Load;
  Register Reg, Reg1, BorrowIn;
  unsigned Opcode = 0;
  int LHSOp, RHSOp;

  if (SetBorrow) {
    BorrowOut = MI.getOperand(1).getReg();
    LHSOp = 2;
    RHSOp = 3;
  } else {
    BorrowOut = MRI->createGenericVirtualRegister(S1);
    LHSOp = 1;
    RHSOp = 2;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for possible physical register in inconvenient location for G_SUB\n";);
  if (MI.getOperand(RHSOp).isReg()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found register on RHS operand of G_SUB\n";);
    if (mi_match(MI.getOperand(RHSOp).getReg(), *MRI, m_Copy(m_Reg(Reg1)))) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of register on RHS operand of G_SUB\n";);
      if (Reg1.isPhysical()) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of physical register on RHS operand of G_SUB\n";);
        auto Instr1 = Builder.buildInstr(DstSize == 8 ? MC6809::PushOp8 : (DstSize == 16 ? MC6809::PushOp16 : MC6809::PushOp32))
                          .addReg(Reg1, RegState::Kill);
        auto Instr2 = Builder.buildInstr(DstSize == 8 ? MC6809::Sub8Pop : (DstSize == 16 ? MC6809::Sub16Pop : MC6809::Sub32Pop))
                          .addDef(Dst)
                          .addDef(BorrowOut)
                          .addDef(MRI->createGenericVirtualRegister(S1))
                          .addUse(MI.getOperand(LHSOp).getReg());
        if (!constrainSelectedInstRegOperands(*Instr2, TII, TRI, RBI))
          llvm_unreachable("Could not constrain evicted subtract instruction.");
        MI.eraseFromParent();
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Evict RHS for Sub(8|16|32)Pop : Instr1 = "; Instr1->dump(););
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Evict RHS for Sub(8|16|32)Pop : Instr2 = "; Instr2->dump(););
        return true;
      }
    }
  }

  MachineInstrBuilder Instr;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying immediate mode\n";);
  Optional<ValueAndVReg> ValReg;
  int64_t Value;
  if (SetBorrow) {
    Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(BorrowIn))) ||
              mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(BorrowIn))) ||
              mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_GCst(ValReg))) ||
              mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_GCst(ValReg)));
  } else
    Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    if (UseBorrow) {
      Opcode = DstSize == 8 ? MC6809::SubBorrow8Imm : (DstSize == 16 ? MC6809::SubBorrow16Imm : MC6809::SubBorrow32Imm);
      Instr = Builder.buildInstr(Opcode)
                  .addDef(Dst)
                  .addDef(BorrowOut)
                  .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                  .addUse(Reg)
                  .addUse(BorrowIn)
                  .addImm(Value);
    } else {
      Opcode = DstSize == 8 ? MC6809::Sub8Imm : (DstSize == 16 ? MC6809::Sub16Imm : MC6809::Sub32Imm);
      Instr = Builder.buildInstr(Opcode)
                  .addDef(Dst)
                  .addDef(BorrowOut)
                  .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                  .addUse(Reg)
                  .addImm(Value);
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain immediate instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Immediate : Instr = "; Instr->dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Looking for load (for addressing mode)\n";);
  if (SetBorrow)
    Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_MInstr(Load))) ||
              mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_MInstr(Load))) ||
              mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_MInstr(Load), m_Reg(BorrowIn))) ||
              mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_MInstr(Load), m_Reg(BorrowIn)));
  else
    Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_MInstr(Load)));
  if (Success) {
    if (Load->getOpcode() == MC6809::G_LOAD_INDEX_OFFSET) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Proceeding with G_LOAD_INDEX_OFFSET\n";);
      if (UseBorrow) {
        Opcode = DstSize == 8 ? MC6809::SubBorrow8Idx : (DstSize == 16 ? MC6809::SubBorrow16Idx : MC6809::SubBorrow32Idx);
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(BorrowOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(Reg)                                   // LHS
                    .addUse(BorrowIn)
                    .add(Load->getOperand(1))
                    .addImm(0)
                    .cloneMemRefs(*Load);
      } else {
        Opcode = DstSize == 8 ? MC6809::Sub8Idx : (DstSize == 16 ? MC6809::Sub16Idx : MC6809::Sub32Idx);
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(BorrowOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(Reg)                                   // LHS
                    .add(Load->getOperand(1))
                    .addImm(0)
                    .cloneMemRefs(*Load);
      }
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain indexed subtract instruction.");
      MI.eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Indexed : Instr = "; Instr->dump(););
      return true;
    }
  }

  if (STI.isHD6309()) {
    Register LHS, RHS;
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying register/register mode\n";);
    if (SetBorrow)
      Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(LHS), m_Reg(RHS))) ||
                mi_match(Dst, *MRI, m_GUSubO(m_Reg(LHS), m_Reg(RHS))) ||
                mi_match(Dst, *MRI, m_GSSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(BorrowIn))) ||
                mi_match(Dst, *MRI, m_GUSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(BorrowIn)));
    else
      Success = mi_match(Dst, *MRI, m_GSub(m_Reg(LHS), m_Reg(RHS)));
    if (Success) {
      if (UseBorrow) {
        Opcode = (DstSize == 8) ? MC6809::SubBorrow8Reg : MC6809::SubBorrow16Reg;
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(BorrowOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(LHS)
                    .addUse(BorrowIn)
                    .addUse(RHS);
      } else {
        Opcode = (DstSize == 8) ? MC6809::Sub8Reg : MC6809::Sub16Reg;
        Instr = Builder.buildInstr(Opcode)
                    .addDef(Dst)
                    .addDef(BorrowOut)
                    .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                    .addUse(LHS)
                    .addUse(RHS);
      }
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain register/register subtract instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Reg/Reg : Instr = "; Instr->dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return false : MI = "; MI.dump(););
  return false;
}

bool MC6809InstructionSelector::selectConstant(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  unsigned Opcode = (DstSize == 8) ? MC6809::Load8Imm : (DstSize == 16) ? MC6809::Load16Imm : MC6809::Load32Imm;
  MI.setDesc(TII.get(Opcode));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectLoad(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  MachineOperand IndexOp = MI.getOperand(1);
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success = false;

  unsigned Opcode = (DstSize == 8) ? MC6809::Load8Idx : (DstSize == 16) ? MC6809::Load16Idx : MC6809::Load32Idx;
  if (IndexOp.isReg())
    Success = mi_match(IndexOp.getReg(), *MRI,m_MInstr(Pointer));
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
      .addDef(Dst)
      .add(Pointer->getOperand(1))
      .addImm(0)
      .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain registered indexed Load instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    // MI.addOperand(MachineOperand::CreateImm(0));
    if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
      llvm_unreachable("Could not constrain MemOp indexed Load instruction.");
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
    return true;
  }
}

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Src = MI.getOperand(0).getReg();
  MachineOperand IndexOp = MI.getOperand(1);
  LLT SrcTy = MRI->getType(Src);
  const auto SrcSize = SrcTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success = false;

  unsigned Opcode = (SrcSize == 8) ? MC6809::Store8Idx : (SrcSize == 16) ? MC6809::Store16Idx : MC6809::Store32Idx;
  if (IndexOp.isReg())
    Success = mi_match(IndexOp.getReg(), *MRI,m_MInstr(Pointer));
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addUse(Src)
                     .add(Pointer->getOperand(1))
                     .addImm(0)
                     .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain register indexed Store instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    // MI.addOperand(MachineOperand::CreateImm(0));
    if (!constrainSelectedInstRegOperands(MI, TII, TRI, RBI))
      llvm_unreachable("Could not constrain MemOp indexed Store instruction.");
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
    return true;
  }
}

bool MC6809InstructionSelector::selectTrunc(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand &DstOp = MI.getOperand(0);
  LLT DstType = MRI->getType(DstOp.getReg());
  MachineOperand &SrcOp = MI.getOperand(1);
  LLT SrcType = MRI->getType(SrcOp.getReg());

  if (DstType == S1) {
    assert((SrcType == S16 || SrcType == S8) && "Illegal source type for s1 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lsb);
    constrainGenericOp(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit S1 : MI = "; MI.dump(););
    return true;
  } else if (DstType == S8) {
    assert((SrcType == S16) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_byte);
    constrainGenericOp(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit S8 : MI = "; MI.dump(););
    return true;
  } else if (DstType == S16) {
    assert((SrcType == S32) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_word);
    constrainGenericOp(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit S16 : MI = "; MI.dump(););
    return true;
  }
  else
    llvm_unreachable("Illegal destination type");
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit impossible : MI = "; MI.dump(););
  return false;
}

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);

  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();

  auto SrcConst = getIConstantVRegValWithLookThrough(Src, *MRI);
  const unsigned Size = MRI->getType(Lo).getSizeInBits();
  MachineInstrBuilder LoCopy;
  MachineInstrBuilder HiCopy;
  LoCopy = Builder.buildCopy(Lo, Src);
  HiCopy = Builder.buildCopy(Hi, Src);
  if (Size == 8) {
    LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_byte);
    HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_byte);
  } else if (Size == 16) {
    LoCopy->getOperand(1).setSubReg(MC6809::sub_lo_word);
    HiCopy->getOperand(1).setSubReg(MC6809::sub_hi_word);
  }
  constrainGenericOp(*LoCopy);
  constrainGenericOp(*HiCopy);
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : LoCopy = "; LoCopy->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : HiCopy = "; HiCopy->dump(););
  return true;
}

bool MC6809InstructionSelector::selectGeneric(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI.dump(););
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case MC6809::G_IMPLICIT_DEF:
    Opcode = MC6809::IMPLICIT_DEF;
    break;
  case MC6809::G_PHI:
    Opcode = MC6809::PHI;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MF);
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

bool MC6809InstructionSelector::selectBranch(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
#if 0
  MachineIRBuilder Builder(MI);
  auto Bcc = Builder.buildInstr(MC6809::JumpRelative)
                 .addMBB(MI.getOperand(0).getMBB());
  MI.eraseFromParent();
  return constrainSelectedInstRegOperands(*Bcc, TII, TRI, RBI);
#else /* 0 */
  return false;
#endif /* 0 */
}

#if 0
MachineInstr *
MC6809InstructionSelector::emitCMN(MachineOperand &LHS, MachineOperand &RHS, MachineIRBuilder &MIRBuilder) const {
  MachineRegisterInfo &MRI = MIRBuilder.getMF().getRegInfo();
  bool Is32Bit = (MRI.getType(LHS.getReg()).getSizeInBits() == 32);
  uint64_t CmpSize = MRI.getType(LHS).getSizeInBits();
  unsigned Opcode = CmpSize == 8 ? MC6809::Compare8Idx : MC6809::Compare16Idx;
  return MIRBuilder.buildInstr(Opcode, {}, {CondReg}).addImm(1);
  return emitADDS(MRI.createVirtualRegister(RC), LHS, RHS, MIRBuilder);
}

MachineInstr *MC6809InstructionSelector::tryFoldIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIRBuilder) const {
  assert(LHS.isReg() && RHS.isReg() && Predicate.isPredicate() && "Unexpected MachineOperand");
  MachineRegisterInfo &MRI = *MIRBuilder.getMRI();
  // We want to find this sort of thing:
  // x = G_SUB 0, y
  // G_ICMP z, x
  //
  // In this case, we can fold the G_SUB into the G_ICMP using a CMN instead.
  // e.g:
  //
  // cmn z, y

  // Check if the RHS or LHS of the G_ICMP is defined by a SUB
  MachineInstr *LHSDef = getDefIgnoringCopies(LHS.getReg(), MRI);
  MachineInstr *RHSDef = getDefIgnoringCopies(RHS.getReg(), MRI);
  auto P = static_cast<CmpInst::Predicate>(Predicate.getPredicate());
  // Given this:
  //
  // x = G_SUB 0, y
  // G_ICMP x, z
  //
  // Produce this:
  //
  // cmn y, z
  if (isCMN(LHSDef, P, MRI))
    return emitCMN(LHSDef->getOperand(2), RHS, MIRBuilder);

  // Same idea here, but with the RHS of the compare instead:
  //
  // Given this:
  //
  // x = G_SUB 0, y
  // G_ICMP z, x
  //
  // Produce this:
  //
  // cmn z, y
  if (isCMN(RHSDef, P, MRI))
    return emitCMN(LHS, RHSDef->getOperand(2), MIRBuilder);

  // Given this:
  //
  // z = G_AND x, y
  // G_ICMP z, 0
  //
  // Produce this if the compare is signed:
  //
  // tst x, y
  if (!CmpInst::isUnsigned(P) && LHSDef &&
      LHSDef->getOpcode() == TargetOpcode::G_AND) {
    // Make sure that the RHS is 0.
    auto ValAndVReg = getIConstantVRegValWithLookThrough(RHS.getReg(), MRI);
    if (!ValAndVReg || ValAndVReg->Value != 0)
      return nullptr;

    return emitTST(LHSDef->getOperand(1),
                   LHSDef->getOperand(2), MIRBuilder);
  }

  return nullptr;
}

bool MC6809InstructionSelector::tryOptAndIntoCompareBranch(MachineInstr &AndInst, bool Invert, MachineBasicBlock *DstMBB, MachineIRBuilder &MIB) const {
  assert(AndInst.getOpcode() == TargetOpcode::G_AND && "Expected G_AND only?");
  // Given something like this:
  //
  //  %x = ...Something...
  //  %one = G_CONSTANT i64 1
  //  %zero = G_CONSTANT i64 0
  //  %and = G_AND %x, %one
  //  %cmp = G_ICMP intpred(ne), %and, %zero
  //  %cmp_trunc = G_TRUNC %cmp
  //  G_BRCOND %cmp_trunc, %bb.3
  //
  // We want to try and fold the AND into the G_BRCOND and produce either a
  // TBNZ (when we have intpred(ne)) or a TBZ (when we have intpred(eq)).
  //
  // In this case, we'd get
  //
  // TBNZ %x %bb.3
  //

  // Check if the AND has a constant on its RHS which we can use as a mask.
  // If it's a power of 2, then it's the same as checking a specific bit.
  // (e.g, ANDing with 8 == ANDing with 000...100 == testing if bit 3 is set)
  auto MaybeBit = getIConstantVRegValWithLookThrough(
      AndInst.getOperand(2).getReg(), *MIB.getMRI());
  if (!MaybeBit)
    return false;

  int32_t Bit = MaybeBit->Value.exactLogBase2();
  if (Bit < 0)
    return false;

  Register TestReg = AndInst.getOperand(1).getReg();

  // Emit a TB(N)Z.
  emitTestBit(TestReg, Bit, Invert, DstMBB, MIB);
  return true;
}

bool MC6809InstructionSelector::tryOptCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const {
  assert(ICmp.getOpcode() == TargetOpcode::G_ICMP);
  assert(MI.getOpcode() == TargetOpcode::G_BRCOND);
  MachineRegisterInfo &MRI = *MIB.getMRI();
  MachineBasicBlock *DestMBB = MI.getOperand(1).getMBB();
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
      MI.eraseFromParent();
      return true;
    }

    // When we have a less than comparison, we can just test if the msb is not
    // zero.
    if (C == 0 && Pred == CmpInst::ICMP_SLT) {
      uint64_t Bit = MRI.getType(LHS).getSizeInBits() - 1;
      emitTestBit(LHS, Bit, /*IsNegative = */ true, DestMBB, MIB);
      MI.eraseFromParent();
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
          tryOptAndIntoCompareBranch( *AndInst, /*Invert = */ Pred == CmpInst::ICMP_NE, DestMBB, MIB)) {
        MI.eraseFromParent();
        return true;
      }

      // Otherwise, try to emit a CB(N)Z instead.
      auto LHSTy = MRI.getType(LHS);
      if (!LHSTy.isVector() && LHSTy.getSizeInBits() <= 64) {
        emitCBZ(LHS, /*IsNegative = */ Pred == CmpInst::ICMP_NE, DestMBB, MIB);
        MI.eraseFromParent();
        return true;
      }
    }
  }

  return false;
}
#endif /* 0 */

#if 0
MachineInstr *MC6809InstructionSelector::emitIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &MIB) const {
  assert(LHS.isReg() && RHS.isReg() && "Expected LHS and RHS to be registers!");
  assert(Predicate.isPredicate() && "Expected predicate?");
  MachineRegisterInfo &MRI = MIB.getMF().getRegInfo();
  LLT CmpTy = MRI.getType(LHS.getReg());
  unsigned Size = CmpTy.getSizeInBits();
  (void)Size;
  assert((Size == 8 || Size == 16) && "Expected a 8-bit or 16-bit LHS/RHS?");
  // Fold the compare into a cmn or tst if possible.
  if (auto FoldCmp = tryFoldIntegerCompare(LHS, RHS, Predicate, MIRBuilder))
    return FoldCmp;
       unsigned Opcode = Size == 8 ? MC6809::Compare8
  MIB.buildInstr(MC6809::J, {}, {}).addImm(CC).addMBB(DestMBB);
  auto Dst = MRI.cloneVirtualRegister(LHS.getReg());
  return emitSUBS(Dst, LHS, RHS, MIB);
}
#endif /* 0 */

MachineInstr *MC6809InstructionSelector::emitIntegerCompare(MachineOperand &LHS, MachineOperand &RHS, MachineOperand &Predicate, MachineIRBuilder &Builder) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : LHS = "; LHS.dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : RHS = "; RHS.dump(););

  LLT CMPTy = MRI->getType(LHS.getReg());
  const auto CMPSize = CMPTy.getSizeInBits();
  bool Invert, Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg, Reg1;
  unsigned Opcode = 0;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for possible physical register in inconvenient location for G_ICMP\n";);
  if (RHS.isReg()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found register on RHS operand of G_ICMP\n";);
    if (mi_match(RHS.getReg(), *MRI, m_Copy(m_Reg(Reg1)))) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of register on RHS operand of G_ICMP\n";);
      if (Reg1.isPhysical()) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found COPY of physical register on RHS operand of G_ICMP\n";);
        // XXXX: FixMe: MarkM - Also invert sense of the predicate
        auto Temp = LHS.getReg();
        LHS.setReg(RHS.getReg());
        RHS.setReg(Temp);
      }
    }
  }

  Optional<ValueAndVReg> ValReg;
  int64_t Value;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for constant on RHS\n";);
  if (mi_match(RHS.getReg(), *MRI, m_GCst(ValReg))) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found constant on RHS = " << ValReg->Value.getSExtValue() << "\n";);
    Value = ValReg->Value.getSExtValue();
    Opcode = CMPSize == 8 ? MC6809::Compare8Imm : MC6809::Compare16Imm;
    auto Instr = Builder.buildInstr(Opcode)
                .addUse(LHS.getReg())
                .addImm(Value);
    return Instr;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for load on RHS\n";);
  if (mi_match(RHS.getReg(), *MRI, m_MInstr(Load))) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Found load on RHS = "; Load->dump(););
    Opcode = CMPSize == 8 ? MC6809::Compare8Idx : MC6809::Compare16Idx;
    if (Load->getOpcode() == MC6809::G_LOAD_INDEX_OFFSET) {
      auto Instr = Builder.buildInstr(Opcode)
                       .addUse(LHS.getReg())
                       .add(Load->getOperand(1)) // Index
                       .add(Load->getOperand(2)) // Offset
                       .cloneMemRefs(*Load);
      return Instr;
    }
    if (Load->getOpcode() == MC6809::G_LOAD) {
      auto Instr = Builder.buildInstr(Opcode)
                       .addUse(LHS.getReg())
                       .add(Load->getOperand(1)) // Index
                       .addImm(0) // Offset
                       .cloneMemRefs(*Load);
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

bool MC6809InstructionSelector::selectCompareBranchFedByICmp(MachineInstr &MI, MachineInstr &ICmp, MachineIRBuilder &MIB) const {
  assert(ICmp.getOpcode() == TargetOpcode::G_ICMP);
  assert(MI.getOpcode() == TargetOpcode::G_BRCOND);

#if 0
  if (tryOptCompareBranchFedByICmp(MI, ICmp, MIB))
    return true;
#endif

  // Couldn't optimize. Emit a compare + a Bcc.
  MachineBasicBlock *DestMBB = MI.getOperand(1).getMBB();
  auto PredOp = ICmp.getOperand(1);
  auto CMP = emitIntegerCompare(ICmp.getOperand(2), ICmp.getOperand(3), PredOp, MIB);
  if (!CMP || !constrainSelectedInstRegOperands(*CMP, TII, TRI, RBI))
    llvm_unreachable("Could not constrain conditional branch instruction.");
  const MC6809CC::CondCode CC = changeICMPPredToMC6809CC(static_cast<CmpInst::Predicate>(PredOp.getPredicate()));
  MIB.buildInstr(MC6809::JumpConditionalRelative, {}, {}).addImm(CC).addMBB(DestMBB);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectConditionalBranch(MachineInstr &MI, MachineFunction &MF, MachineRegisterInfo &MRI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register CondReg = MI.getOperand(0).getReg();
  MachineInstr *CCMI = MRI.getVRegDef(CondReg);

  if (CCMI->getOpcode() == TargetOpcode::G_TRUNC) {
    CondReg = CCMI->getOperand(1).getReg();
    CCMI = MRI.getVRegDef(CondReg);
  }

  // Try to select the G_BRCOND using whatever is feeding the condition if
  // possible.
  unsigned CCMIOpc = CCMI->getOpcode();
  if (CCMIOpc == TargetOpcode::G_ICMP)
    return selectCompareBranchFedByICmp(MI, *CCMI, Builder);

  // Not a G_ICMP - emit a direct LSB test instead
  auto TstMI = Builder.buildInstr(MC6809::BIT8Imm, {}, {CondReg}).addImm(1);
  if (!constrainSelectedInstRegOperands(*TstMI, TII, TRI, RBI))
    llvm_unreachable("Could not constrain test bit instruction.");
  auto Bcc = Builder.buildInstr(MC6809::JumpConditionalRelative)
                 .addImm(MC6809CC::NE) // Not Equal to zero == one
                 .addMBB(MI.getOperand(1).getMBB());
  MI.eraseFromParent();
  if (!constrainSelectedInstRegOperands(*Bcc, TII, TRI, RBI))
    llvm_unreachable("Could not constrain conditional branch instruction.");
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Bcc = "; Bcc->dump(););
  return true;
}

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register
// is defined exactly once, making sure definitions are constrained suffices.
void MC6809InstructionSelector::constrainGenericOp(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  for (MachineOperand &Op : MI.operands()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Looping : Op = "; Op.dump(););
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() || MRI->getRegClassOrNull(Op.getReg()))
      continue;
    constrainOperandRegClass(Op, getRegClassForType(Op.getReg()));
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constrained : Op = "; Op.dump(););
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
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

InstructionSelector *llvm::createMC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI) {
  return new MC6809InstructionSelector(TM, STI, RBI);
}