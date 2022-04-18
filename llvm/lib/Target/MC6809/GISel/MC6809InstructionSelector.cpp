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
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
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
  const TargetRegisterClass *guessRegClass(unsigned Reg) const;

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

const TargetRegisterClass *MC6809InstructionSelector::guessRegClass(unsigned Reg) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  const RegisterBank *RegBank = RBI.getRegBank(Reg, *MRI, TRI);
  assert(RegBank && "Can't get register bank for virtual register");

  const unsigned Size = MRI->getType(Reg).getSizeInBits();
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
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
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
  }
  return false;
}

bool MC6809InstructionSelector::selectExt(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 20 : MI = ";MI.dump(););
  MachineIRBuilder Builder(MI);
  LLT S1 = LLT::scalar(1);
  LLT S2 = LLT::scalar(2);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
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

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = ";MI.dump(););
  MI.setDesc(TII.get(MC6809::LEAPtrAddImm));
  MI.addOperand(MachineOperand::CreateImm(0));
  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
}

bool MC6809InstructionSelector::selectAddr(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MI.setDesc(TII.get(MC6809::Load16Imm));
  return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
}

bool MC6809InstructionSelector::selectStore(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register Idx = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  MachineInstr *PtrAdd = nullptr;
  Register IndReg;
  Register OffsReg;
  Optional<ValueAndVReg> OffsCst;
  if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_GCst(OffsCst)))) {
    dbgs() << "OINQUE DEBUG " << __func__ << " : DEBUG MATCH CONSTANT\n";
  }
  unsigned Opcode;
  if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_GCst(OffsCst))))  {
    APInt OffsVal = OffsCst->Value;
    assert(((*(OffsVal.getRawData()) & 0xFFFFFFFFFFFF0000ULL) == 0 || (*(OffsVal.getRawData()) & 0xFFFFFFFFFFFF0000ULL) == 0xFFFFFFFFFFFF0000ULL) && "Offset out of range");
    int16_t offset = (uint16_t)(*(OffsVal.getRawData()));
    PtrAdd = getOpcodeDef(MC6809::G_PTR_ADD, Idx, *MRI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : PtrAdd = "; PtrAdd->dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->Value.dump() = "; OffsCst->Value.dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->VReg = " << OffsCst->VReg << "\n";);
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot store register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = MC6809::Store8IdxImm;
      break;
    case 16:
      Opcode = MC6809::Store16IdxImm;
      break;
    case 32:
      Opcode = MC6809::Store32IdxImm;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    MI.removeOperand(1);
    MI.addOperand(PtrAdd->getOperand(1));
    MI.addOperand(MachineOperand::CreateImm(offset));
    MI.cloneMemRefs(*MF, *PtrAdd);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  } else if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_Reg(OffsReg)))) {
    PtrAdd = getOpcodeDef(MC6809::G_PTR_ADD, Idx, *MRI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : PtrAdd = "; PtrAdd->dump(););
    LLT OffsRegTy = MRI->getType(OffsReg);
    const auto OffsRegSize = OffsRegTy.getSizeInBits();
    assert((OffsRegSize == 8 || OffsRegSize == 16) && "Illegal offset register size");
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot store register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = OffsRegSize == 8 ? MC6809::Store8IdxReg8 : MC6809::Store8IdxReg16;
      break;
    case 16:
      Opcode = OffsRegSize == 8 ? MC6809::Store16IdxReg8 : MC6809::Store16IdxReg16;
      break;
    case 32:
      Opcode = OffsRegSize == 8 ? MC6809::Store32IdxReg8 : MC6809::Store32IdxReg16;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    MI.removeOperand(1);
    MI.addOperand(PtrAdd->getOperand(1));
    MI.addOperand(PtrAdd->getOperand(2));
    MI.cloneMemRefs(*MF, *PtrAdd);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  } else {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : NO MATCH\n";);
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot store register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = MC6809::Store8IdxZero;
      break;
    case 16:
      Opcode = MC6809::Store16IdxZero;
      break;
    case 32:
      Opcode = MC6809::Store32IdxZero;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : NO MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  }
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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT P = LLT::pointer(0, 16);
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  Register Ptr = MI.getOperand(1).getReg();
  LLT PtrTy = MRI->getType(Ptr);
  Register Offset = MI.getOperand(2).getReg();
  LLT OffsetTy = MRI->getType(Offset);
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
  Register Dst = MI.getOperand(0).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  Optional<ValueAndVReg> OffsCst;
  if (mi_match(Dst, *MRI, m_GCst(OffsCst)))  {
    dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->Value = "; OffsCst->Value.dump();
    dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->VReg = " << OffsCst->VReg << "\n";
  }
  unsigned Opcode;
  switch  (DstSize) {
  default:
    llvm_unreachable(formatv("Cannot load register of size = {}", DstSize).str().c_str());
  case 1:
  case 8:
    Opcode = MC6809::Load8Imm;
    break;
  case 16:
    Opcode = MC6809::Load16Imm;
    break;
  case 32:
    Opcode = MC6809::Load32Imm;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  bool Success = constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Success;
}

bool MC6809InstructionSelector::selectLoad(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register Idx = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();

  MachineInstr *PtrAdd = nullptr;
  Register IndReg;
  Register OffsReg;
  Optional<ValueAndVReg> OffsCst;
  if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_GCst(OffsCst)))) {
    dbgs() << "OINQUE DEBUG " << __func__ << " : DEBUG MATCH CONSTANT\n";
  }
  unsigned Opcode;
  if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_GCst(OffsCst))))  {
    APInt OffsVal = OffsCst->Value;
    assert(((*(OffsVal.getRawData()) & 0xFFFFFFFFFFFF0000ULL) == 0 || (*(OffsVal.getRawData()) & 0xFFFFFFFFFFFF0000ULL) == 0xFFFFFFFFFFFF0000ULL) && "Offset out of range");
    int16_t offset = (uint16_t)(*(OffsVal.getRawData()));
    PtrAdd = getOpcodeDef(MC6809::G_PTR_ADD, Idx, *MRI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : PtrAdd = "; PtrAdd->dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->Value.dump() = "; OffsCst->Value.dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : OffsCst->VReg = " << OffsCst->VReg << "\n";);
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot load register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = MC6809::Load8IdxImm;
      break;
    case 16:
      Opcode = MC6809::Load16IdxImm;
      break;
    case 32:
      Opcode = MC6809::Load32IdxImm;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    MI.removeOperand(1);
    MI.addOperand(PtrAdd->getOperand(1));
    MI.addOperand(MachineOperand::CreateImm(offset));
    MI.cloneMemRefs(*MF, *PtrAdd);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  } else if (mi_match(Idx, *MRI, m_GPtrAdd(m_Reg(IndReg), m_Reg(OffsReg)))) {
    PtrAdd = getOpcodeDef(MC6809::G_PTR_ADD, Idx, *MRI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH : PtrAdd = "; PtrAdd->dump(););
    LLT OffsRegTy = MRI->getType(OffsReg);
    const auto OffsRegSize = OffsRegTy.getSizeInBits();
    assert((OffsRegSize == 8 || OffsRegSize == 16) && "Illegal offset register size");
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot load register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = OffsRegSize == 8 ? MC6809::Load8IdxReg8 : MC6809::Load8IdxReg16;
      break;
    case 16:
      Opcode = OffsRegSize == 8 ? MC6809::Load16IdxReg8 : MC6809::Load16IdxReg16;
      break;
    case 32:
      Opcode = OffsRegSize == 8 ? MC6809::Load32IdxReg8 : MC6809::Load32IdxReg16;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    MI.removeOperand(1);
    MI.addOperand(PtrAdd->getOperand(1));
    MI.addOperand(PtrAdd->getOperand(2));
    MI.cloneMemRefs(*MF, *PtrAdd);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  } else {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : NO MATCH\n";);
    switch (DstSize) {
    default:
      llvm_unreachable(formatv("Cannot load register of size = {}", DstSize).str().c_str());
    case 1:
    case 8:
      Opcode = MC6809::Load8IdxZero;
      break;
    case 16:
      Opcode = MC6809::Load16IdxZero;
      break;
    case 32:
      Opcode = MC6809::Load32IdxZero;
      break;
    }
    MI.setDesc(TII.get(Opcode));
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : NO MATCH COMPLETE\n";);
    return constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  }
}

bool MC6809InstructionSelector::selectTrunc(MachineInstr &MI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  MachineOperand &DstOp = MI.getOperand(0);
  LLT DstType = MRI->getType(DstOp.getReg());
  MachineOperand &SrcOp = MI.getOperand(1);
  LLT SrcType = MRI->getType(SrcOp.getReg());

  if (DstType == S1) {
    assert((SrcType == S16 || SrcType == S8) && "Illegal source type for s1 destination");
    unsigned Opcode = SrcType == S8 ? MC6809::AND8Imm : MC6809::AND16Imm;
    MI.setDesc(TII.get(Opcode));
    MI.addOperand(MachineOperand::CreateImm(1));
    bool success =  constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit 0 : MI = "; MI.dump(););
    return success;
  } else if (DstType == S8) {
    assert((SrcType == S16) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_byte);
    constrainGenericOp(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit 1 : MI = "; MI.dump(););
    return true;
  } else if (DstType == S16) {
    assert((SrcType == S32) && "Illegal source type for s8 destination");
    MI.setDesc(TII.get(MC6809::COPY));
    MI.getOperand(1).setSubReg(MC6809::sub_lo_word);
    constrainGenericOp(MI);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit 2 : MI = "; MI.dump(););
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
    if (!Op.isReg() || !Op.isDef() || Op.getReg().isPhysical() ||
        MRI->getRegClassOrNull(Op.getReg()))
      continue;
    constrainOperandRegClass(Op, getRegClassForType(Op.getReg()));
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