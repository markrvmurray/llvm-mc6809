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
  bool selectAddE(MachineInstr &MI);
  bool selectAddO(MachineInstr &MI);
  bool selectSub(MachineInstr &MI);
  bool selectSubE(MachineInstr &MI);
  bool selectSubO(MachineInstr &MI);

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
    return selectLoad(MI);
  case MC6809::G_STORE:
    return selectStore(MI);
  case MC6809::G_PTR_ADD:
    return selectPtrAdd(MI);
  case MC6809::G_CONSTANT:
    return selectConstant(MI);

  case MC6809::G_BRINDIRECT:
  case MC6809::G_IMPLICIT_DEF:
  case MC6809::G_PHI:
    return selectGeneric(MI);

  case MC6809::G_SEXT:
  case MC6809::G_ZEXT:
  case MC6809::G_ANYEXT:
    return selectExt(MI);

  case MC6809::G_ADD:
    return selectAdd(MI);
  case MC6809::G_SUB:
    return selectSub(MI);
  case MC6809::G_UADDE:
  case MC6809::G_SADDE:
    return selectAdd(MI);
  case MC6809::G_USUBE:
  case MC6809::G_SSUBE:
    return selectSubE(MI);
  case MC6809::G_UADDO:
  case MC6809::G_SADDO:
    return selectAdd(MI);
  case MC6809::G_USUBO:
  case MC6809::G_SSUBO:
    return selectSubO(MI);
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
    } else if (Size == 32) {
      uint64_t Val = HiConst->Value.getZExtValue() << 16 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load32Imm, {Dst, &MC6809::ACC32RegClass}, {Val});
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        return false;
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit Merged : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectPtrAdd(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  unsigned Opcode;
  if (MI.getOperand(2).isCImm()) {
    uint64_t Val = MI.getOperand(2).getCImm()->getSExtValue();
    MI.getOperand(2).ChangeToImmediate(Val);
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
  Register CarryIn, CarryOut;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Signed = MI.getOpcode() == MC6809::G_SADDO || MI.getOpcode() == MC6809::G_SADDE;
  bool WithCarry = MI.getOpcode() == MC6809::G_SADDE || MI.getOpcode() == MC6809::G_UADDE;
  bool ExtraBits = MI.getOpcode() != MC6809::G_ADD;
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  if (ExtraBits) {
    CarryOut = MI.getOperand(1).getReg();
    if (WithCarry)
      CarryIn = MI.getOperand(4).getReg();
    else
      CarryIn = MRI->createGenericVirtualRegister(S1);
  } else {
    CarryOut = MRI->createGenericVirtualRegister(S1);
    CarryIn = MRI->createGenericVirtualRegister(S1);
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying immediate mode\n";);
  Optional<ValueAndVReg> ValReg;
  int64_t Value;
  if (ExtraBits)
    Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_GCst(ValReg))) ||
              mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_GCst(ValReg)));
  else
    Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    if (WithCarry) {
      Opcode = (DstSize == 8) ? MC6809::AddCarry8Imm : MC6809::AddCarry16Imm;
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addDef(CarryOut)
                       .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                       .addUse(Reg)
                       .addUse(CarryIn)
                       .addImm(Value);
    } else {
      Opcode = (DstSize == 8) ? MC6809::Add8Imm : MC6809::Add16Imm;
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addDef(CarryOut)
                       .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                       .addUse(Reg)
                       .addImm(Value);
    }
    if (Signed) {
      // Swap the output C and V bits
      Register Tmp = Instr.getReg(1);
      Instr->getOperand(1).setReg(Instr.getReg(2));
      Instr->getOperand(2).setReg(Tmp);
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain immediate instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Immediate : return true : MI = "; MI.dump(););
    return true;
  }

  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Looking for load (for addressing mode)\n";);
  if (ExtraBits)
    Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_MInstr(Load))) ||
              mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_MInstr(Load))) ||
              mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_MInstr(Load), m_Reg(CarryIn))) ||
              mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_MInstr(Load), m_Reg(CarryIn)));
  else
    Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_MInstr(Load)));
  if (Success) {
    if (Load->getOpcode() == MC6809::G_LOAD) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Proceeding with G_LOAD\n";);
      if (WithCarry) {
        Opcode = (DstSize == 8) ? MC6809::AddCarry8Idx : MC6809::AddCarry16Idx;
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
        Opcode = (DstSize == 8) ? MC6809::Add8Idx : MC6809::Add16Idx;
        Instr = Builder.buildInstr(Opcode)
                         .addDef(Dst)
                         .addDef(CarryOut)
                         .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                         .addUse(Reg)                                   // LHS
                         .add(Load->getOperand(1))
                         .addImm(0)
                         .cloneMemRefs(*Load);
      }
      if (Signed) {
        // Swap the output C and V bits
        Register Tmp = Instr.getReg(1);
        Instr->getOperand(1).setReg(Instr.getReg(2));
        Instr->getOperand(2).setReg(Tmp);
      }
      if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
        llvm_unreachable("Could not constrain indexed add instruction.");
      MI.eraseFromParent();
      Load->eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Add with Load found : return true : MI = "; MI.dump(););
      return true;
    }
  }

  if (STI.isHD6309()) {
    Register LHS, RHS;
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying register/register mode\n";);
    if (ExtraBits)
      Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(LHS), m_Reg(RHS))) ||
                mi_match(Dst, *MRI, m_GUAddO(m_Reg(LHS), m_Reg(RHS))) ||
                mi_match(Dst, *MRI, m_GSAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(CarryIn))) ||
                mi_match(Dst, *MRI, m_GUAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(CarryIn)));
    else
        Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(LHS), m_Reg(RHS)));
    if (Success) {
      if (WithCarry) {
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
    if (Signed) {
      // Swap the output C and V bits
      Register Tmp = Instr.getReg(1);
      Instr->getOperand(1).setReg(Instr.getReg(2));
      Instr->getOperand(2).setReg(Tmp);
    }
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain register/register add instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : register/register add found : return true : MI = "; MI.dump(););
    return true;
  }

#if 0
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying absolute mode\n";);
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
  Opcode = (DstSize == 8) ? MC6809::Add8Abs : MC6809::Add16Abs;
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1)) // CarryOut
                     .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                     .addUse(Reg)
                     .add(Addr)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Absolute : return true : MI = "; MI.dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying indexed mode\n";);
  MachineOperand Idx = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdxOffs(MI, Idx, Offset, AA))));
  Opcode = (DstSize == 8) ? MC6809::Add8Idx : MC6809::Add16Idx;
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1)) // CarryOut
                     .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                     .addUse(Reg)
                     .addUse(Idx.getReg())
                     .add(Offset)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute indexed instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Indexed : return true : MI = "; MI.dump(););
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying indirect indexed mode\n";);
  Success = mi_match(Dst, *MRI, m_GAdd(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))));
  Opcode = (DstSize == 8) ? MC6809::Add8Indir : MC6809::Add16Indir;
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1)) // CarryOut
                     .addDef(MRI->createGenericVirtualRegister(S1)) // Overflow
                     .addUse(Reg)
                     .addUse(Idx.getReg())
                     .add(Offset)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect indexed instruction.");
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Indirect Indexed : return true : MI = "; MI.dump(););
    return true;
  }
#endif

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return false : MI = "; MI.dump(););
  return false;
}

#if 0
bool MC6809InstructionSelector::selectAddE(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();
  LLT ResultTy = MRI->getType(Result);
  const auto ResultSize = ResultTy.getSizeInBits();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::AddCarry8Imm : MC6809::AddCarry16Imm;
      return Builder.buildInstr(Opcode, {Result, CarryOut, MRI.createGenericVirtualRegister(S1)}, {L, CarryIn, RConst->Value.getZExtValue()});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(L, MRI, m_FoldedLdAbs(MI, Addr, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::AddCarry8Abs : MC6809::AddCarry16Abs;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Addr);
    }
    MachineOperand Idx = MachineOperand::CreateReg(0, false);
    MachineOperand Offset = MachineOperand::CreateReg(0, false);
    if (mi_match(L, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::AddCarry8Idx : MC6809::AddCarry16Idx;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Idx)
          .add(Offset);
    }
    if (mi_match(L, MRI, m_FoldedLdIndirIdxOffs(MI,  Idx, Offset, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::AddCarry8Indir : MC6809::AddCarry16Indir;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Idx)
          .add(Offset);
    }
    llvm_unreachable("Unable to match add instruction");
  }();
  if (MI.getOpcode() == MC6809::G_SADDE) {
    // Swap C and V
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constraining : MI = "; MI.dump(););
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectAddO(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  LLT ResultTy = MRI->getType(Result);
  const auto ResultSize = ResultTy.getSizeInBits();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Add8Imm : MC6809::Add16Imm;
      return Builder.buildInstr(Opcode, {Result, CarryOut, MRI.createGenericVirtualRegister(S1)}, {L, RConst->Value.getZExtValue()});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(L, MRI, m_FoldedLdAbs(MI, Addr, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Add8Abs : MC6809::Add16Abs;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr);
    }
    MachineOperand Idx = MachineOperand::CreateReg(0, false);
    MachineOperand Offset = MachineOperand::CreateReg(0, false);
    if (mi_match(L, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Add8Idx : MC6809::Add16Idx;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Idx)
          .add(Offset);
    }

    if (mi_match(L, MRI, m_FoldedLdIndirIdxOffs(MI,  Idx, Offset, AA)))
      std::swap(L, R);
    if (mi_match(R, MRI, m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Add8Indir: MC6809::Add16Indir;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Idx)
          .add(Offset);
    }
    llvm_unreachable("Unable to match add instruction");
  }();
  if (MI.getOpcode() == MC6809::G_SADDO) {
    // Swap C and V
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constraining : MI = "; MI.dump(););
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}
#endif /* 0 */

bool MC6809InstructionSelector::selectSub(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOpcode() == MC6809::G_SUB);

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  Optional<ValueAndVReg> ValReg;
  bool Success;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Checking for subtraction <const> - <reg>\n";);
  Success = mi_match(Dst, *MRI, m_GSub(m_GCst(ValReg), m_Reg(Reg)));
  if (Success) {
    unsigned Opcode;
    Opcode = (DstSize == 8) ? MC6809::PushOp8 : MC6809::PushOp16;
    auto tmp = MRI->createGenericVirtualRegister(DstTy);
    Builder.buildInstr(Opcode)
        .addDef(tmp)
        .addReg(Reg, RegState::Kill);
    Opcode = (DstSize == 8) ? MC6809::Sub8Pop : MC6809::Sub16Pop;
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addUse(tmp)
                     .addUse(MI.getOperand(1).getReg());
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain immediate instruction.");
    MI.eraseFromParent();
    return true;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying immediate mode\n";);
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::Sub8Imm : MC6809::Sub16Imm;
    Value = ValReg->Value.getSExtValue();
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addUse(Reg)
                     .addImm(Value);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain immediate instruction.");
    MI.eraseFromParent();
    return true;
  }

#if 0
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying absolute mode\n";);
  MachineOperand Addr = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdAbs(MI, Addr, AA))));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::Sub8Abs : MC6809::Sub16Abs;
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addUse(Reg)
                     .add(Addr)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute instruction.");
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying indexed mode\n";);
  MachineOperand Idx = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdxOffs(MI, Idx, Offset, AA))));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::Sub8Idx : MC6809::Sub16Idx;
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addUse(Reg)
                     .add(Idx)
                     .add(Offset)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain absolute indexed instruction.");
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Trying indirect indexed mode\n";);
  Success = mi_match(Dst, *MRI, m_GSub(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))));
  if (Success) {
    Opcode = (DstSize == 8) ? MC6809::Sub8Indir : MC6809::Sub16Indir;
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addDef(MRI->createGenericVirtualRegister(S1))
                     .addUse(Reg)
                     .add(Idx)
                     .add(Offset)
                     .cloneMemRefs(*Load);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indirect indexed instruction.");
    MI.eraseFromParent();
    return true;
  }
#endif /* 0 */
  return false;
}

bool MC6809InstructionSelector::selectSubE(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  Register CarryIn = MI.getOperand(4).getReg();
  LLT ResultTy = MRI->getType(Result);
  const auto ResultSize = ResultTy.getSizeInBits();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::SubBorrow8Imm : MC6809::SubBorrow16Imm;
      return Builder.buildInstr(Opcode, {Result, CarryOut, MRI.createGenericVirtualRegister(S1)}, {L, CarryIn, RConst->Value.getZExtValue()});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::SubBorrow8Abs : MC6809::SubBorrow16Abs;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Addr);
    }
    MachineOperand Idx = MachineOperand::CreateReg(0, false);
    MachineOperand Offset = MachineOperand::CreateReg(0, false);
    if (mi_match(R, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::SubBorrow8Idx : MC6809::SubBorrow16Idx;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Idx)
          .add(Offset);
    }

    if (mi_match(R, MRI, m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::SubBorrow8Indir : MC6809::SubBorrow16Indir;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .addUse(CarryIn)
          .add(Idx)
          .add(Offset);
    }
    llvm_unreachable("Unable to match subtract instruction");
  }();
  if (MI.getOpcode() == MC6809::G_SSUBE) {
    // Swap C and V
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constraining : MI = "; MI.dump(););
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectSubO(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register Result = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register L = MI.getOperand(2).getReg();
  Register R = MI.getOperand(3).getReg();
  LLT ResultTy = MRI->getType(Result);
  const auto ResultSize = ResultTy.getSizeInBits();

  MachineIRBuilder Builder(MI);
  auto &MRI = *Builder.getMRI();

  MachineInstrBuilder Instr = [&]() {
    if (auto RConst = getIConstantVRegValWithLookThrough(R, MRI)) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Sub8Imm : MC6809::Sub16Imm;
      return Builder.buildInstr(Opcode, {Result, CarryOut, MRI.createGenericVirtualRegister(S1)}, {L, RConst->Value.getZExtValue()});
    }
    MachineOperand Addr = MachineOperand::CreateReg(0, false);
    if (mi_match(R, MRI, m_FoldedLdAbs(MI, Addr, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Sub8Abs : MC6809::Sub16Abs;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Addr);
    }
    MachineOperand Idx = MachineOperand::CreateReg(0, false);
    MachineOperand Offset = MachineOperand::CreateReg(0, false);
    if (mi_match(R, MRI, m_FoldedLdIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Sub8Idx : MC6809::Sub16Idx;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Idx)
          .add(Offset);
    }

    if (mi_match(R, MRI, m_FoldedLdIndirIdxOffs(MI, Idx, Offset, AA))) {
      unsigned Opcode = (ResultSize == 8) ? MC6809::Sub8Indir : MC6809::Sub16Indir;
      return Builder.buildInstr(Opcode)
          .addDef(Result)
          .addDef(CarryOut)
          .addDef(MRI.createGenericVirtualRegister(S1))
          .addUse(L)
          .add(Idx)
          .add(Offset);
    }
    llvm_unreachable("Unable to match subtract instruction");
  }();
  if (MI.getOpcode() == MC6809::G_SSUBO) {
    // Swap C and V
    Register Tmp = Instr.getReg(1);
    Instr->getOperand(1).setReg(Instr.getReg(2));
    Instr->getOperand(2).setReg(Tmp);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constraining : MI = "; MI.dump(););
  if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
    return false;

  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return true;
}

bool MC6809InstructionSelector::selectConstant(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
//  MachineIRBuilder Builder(MI);
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
  Register Index = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success;

  unsigned Opcode = (DstSize == 8) ? MC6809::Load8Idx : (DstSize == 16) ? MC6809::Load16Idx : MC6809::Load32Idx;
  Success = mi_match(Index, *MRI,m_MInstr(Pointer));
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
      .addDef(Dst)
      .add(Pointer->getOperand(1))
      .addImm(0)
      .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indexed Load instruction.");
    MI.eraseFromParent();
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    MI.addOperand(MachineOperand::CreateImm(0));
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  }
}

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Src = MI.getOperand(0).getReg();
  Register Index = MI.getOperand(1).getReg();
  LLT SrcTy = MRI->getType(Src);
  const auto SrcSize = SrcTy.getSizeInBits();
  MachineInstr *Pointer;
  bool Success;

  unsigned Opcode = (SrcSize == 8) ? MC6809::Store8Idx : (SrcSize == 16) ? MC6809::Store16Idx : MC6809::Store32Idx;
  Success = mi_match(Index, *MRI,m_MInstr(Pointer));
  if (Success) {
    auto Instr = Builder.buildInstr(Opcode)
                     .addUse(Src)
                     .add(Pointer->getOperand(1))
                     .addImm(0)
                     .cloneMemRefs(MI);
    if (!constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI))
      llvm_unreachable("Could not constrain indexed Load instruction.");
    MI.eraseFromParent();
    return true;
  } else {
    MI.setDesc(TII.get(Opcode));
    MI.addOperand(MachineOperand::CreateImm(0));
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
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