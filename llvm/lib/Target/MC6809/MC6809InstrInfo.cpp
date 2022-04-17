//===-- MC6809InstrInfo.cpp - MC6809 Instruction Information
//--------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstrInfo.h"

#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "MC6809Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-instrinfo"

#define GET_INSTRINFO_CTOR_DTOR
#include "MC6809GenInstrInfo.inc"

void llvm::emitFrameOffset(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, const DebugLoc &DL, unsigned DestReg, unsigned SrcReg, StackOffset Offset, const TargetInstrInfo *TII, MachineInstr::MIFlag Flag) {
  int64_t Bytes = Offset.getFixed();

  if (Bytes || (!Offset && SrcReg != DestReg))
    BuildMI(MBB, MBBI, DL, TII->get(MC6809::LEAPtrAddImm), DestReg).addReg(SrcReg).addImm(Bytes);
}

MC6809InstrInfo::MC6809InstrInfo(const MC6809Subtarget &STI)
    : MC6809GenInstrInfo(/*CFSetupOpcode=*/MC6809::ADJCALLSTACKDOWN, /*CFDestroyOpcode=*/MC6809::ADJCALLSTACKUP), STI(STI) {
  LEAPtrAddImmOpcode = {
      {{MC6809::IX, -1}, MC6809::LEAXi_o16}, {{MC6809::IX, 0}, MC6809::LEAXi_o5}, {{MC6809::IX, 5}, MC6809::LEAXi_o5}, {{MC6809::IX, 8}, MC6809::LEAXi_o8}, {{MC6809::IX, 16}, MC6809::LEAXi_o16},
      {{MC6809::IY, -1}, MC6809::LEAYi_o16}, {{MC6809::IY, 0}, MC6809::LEAYi_o0}, {{MC6809::IY, 5}, MC6809::LEAYi_o5}, {{MC6809::IY, 8}, MC6809::LEAYi_o8}, {{MC6809::IY, 16}, MC6809::LEAYi_o16},
      {{MC6809::SU, -1}, MC6809::LEAUi_o16}, {{MC6809::SU, 0}, MC6809::LEAUi_o0}, {{MC6809::SU, 5}, MC6809::LEAUi_o5}, {{MC6809::SU, 8}, MC6809::LEAUi_o8}, {{MC6809::SU, 16}, MC6809::LEAUi_o16},
      {{MC6809::SS, -1}, MC6809::LEASi_o16}, {{MC6809::SS, 0}, MC6809::LEASi_o0}, {{MC6809::SS, 5}, MC6809::LEASi_o5}, {{MC6809::SS, 8}, MC6809::LEASi_o8}, {{MC6809::SS, 16}, MC6809::LEASi_o16},
  };
  LEAPtrAddRegOpcode = {
      {{MC6809::IX,MC6809::AA}, MC6809::LEAXi_oA}, {{MC6809::IX,MC6809::AB}, MC6809::LEAXi_oB}, {{MC6809::IX,MC6809::AD}, MC6809::LEAXi_oD}, {{MC6809::IX,MC6809::AE}, MC6809::LEAXi_oE}, {{MC6809::IX,MC6809::AF}, MC6809::LEAXi_oF}, {{MC6809::IX,MC6809::AW}, MC6809::LEAXi_oW},
      {{MC6809::IY,MC6809::AA}, MC6809::LEAYi_oA}, {{MC6809::IY,MC6809::AB}, MC6809::LEAYi_oB}, {{MC6809::IY,MC6809::AD}, MC6809::LEAYi_oD}, {{MC6809::IY,MC6809::AE}, MC6809::LEAYi_oE}, {{MC6809::IY,MC6809::AF}, MC6809::LEAYi_oF}, {{MC6809::IY,MC6809::AW}, MC6809::LEAYi_oW},
      {{MC6809::SU,MC6809::AA}, MC6809::LEAUi_oA}, {{MC6809::SU,MC6809::AB}, MC6809::LEAUi_oB}, {{MC6809::SU,MC6809::AD}, MC6809::LEAUi_oD}, {{MC6809::SU,MC6809::AE}, MC6809::LEAUi_oE}, {{MC6809::SU,MC6809::AF}, MC6809::LEAUi_oF}, {{MC6809::SU,MC6809::AW}, MC6809::LEAUi_oW},
      {{MC6809::SS,MC6809::AA}, MC6809::LEASi_oA}, {{MC6809::SS,MC6809::AB}, MC6809::LEASi_oB}, {{MC6809::SS,MC6809::AD}, MC6809::LEASi_oD}, {{MC6809::SS,MC6809::AE}, MC6809::LEASi_oE}, {{MC6809::SS,MC6809::AF}, MC6809::LEASi_oF}, {{MC6809::SS,MC6809::AW}, MC6809::LEASi_oW},
  };
  LoadImmediateOpcode = {
      {MC6809::AA, MC6809::LDAi8}, {MC6809::AB, MC6809::LDBi8},
      {MC6809::AE, MC6809::LDEi8}, {MC6809::AF, MC6809::LDFi8},
      {MC6809::AD, MC6809::LDDi16}, {MC6809::AW, MC6809::LDWi16},
      {MC6809::AQ, MC6809::LDQi32},
      {MC6809::IX, MC6809::LDXi16}, {MC6809::IY, MC6809::LDYi16},
      {MC6809::SU, MC6809::LDUi16}, {MC6809::SS, MC6809::LDSi16},
  };
  LoadIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::LDAi_o16}, {{MC6809::AA, 0}, MC6809::LDAi_o0}, {{MC6809::AA, 5}, MC6809::LDAi_o5}, {{MC6809::AA, 8}, MC6809::LDAi_o8}, {{MC6809::AA, 16}, MC6809::LDAi_o16},
      {{MC6809::AB, -1}, MC6809::LDBi_o16}, {{MC6809::AB, 0}, MC6809::LDBi_o0}, {{MC6809::AB, 5}, MC6809::LDBi_o5}, {{MC6809::AB, 8}, MC6809::LDBi_o8}, {{MC6809::AB, 16}, MC6809::LDBi_o16},
      {{MC6809::AD, -1}, MC6809::LDDi_o16}, {{MC6809::AD, 0}, MC6809::LDDi_o0}, {{MC6809::AD, 5}, MC6809::LDDi_o5}, {{MC6809::AD, 8}, MC6809::LDDi_o8}, {{MC6809::AD, 16}, MC6809::LDDi_o16},
      {{MC6809::AE, -1}, MC6809::LDEi_o16}, {{MC6809::AE, 0}, MC6809::LDEi_o0}, {{MC6809::AE, 5}, MC6809::LDEi_o5}, {{MC6809::AE, 8}, MC6809::LDEi_o8}, {{MC6809::AE, 16}, MC6809::LDEi_o16},
      {{MC6809::AF, -1}, MC6809::LDFi_o16}, {{MC6809::AF, 0}, MC6809::LDFi_o0}, {{MC6809::AF, 5}, MC6809::LDFi_o5}, {{MC6809::AF, 8}, MC6809::LDFi_o8}, {{MC6809::AF, 16}, MC6809::LDFi_o16},
      {{MC6809::AW, -1}, MC6809::LDWi_o16}, {{MC6809::AW, 0}, MC6809::LDWi_o0}, {{MC6809::AW, 5}, MC6809::LDWi_o5}, {{MC6809::AW, 8}, MC6809::LDWi_o8}, {{MC6809::AW, 16}, MC6809::LDWi_o16},
      {{MC6809::AQ, -1}, MC6809::LDQi_o16}, {{MC6809::AQ, 0}, MC6809::LDQi_o0}, {{MC6809::AQ, 5}, MC6809::LDQi_o5}, {{MC6809::AQ, 8}, MC6809::LDQi_o8}, {{MC6809::AQ, 16}, MC6809::LDQi_o16},
      {{MC6809::IX, -1}, MC6809::LDXi_o16}, {{MC6809::IX, 0}, MC6809::LDXi_o0}, {{MC6809::IX, 5}, MC6809::LDXi_o5}, {{MC6809::IX, 8}, MC6809::LDXi_o8}, {{MC6809::IX, 16}, MC6809::LDXi_o16},
      {{MC6809::IY, -1}, MC6809::LDYi_o16}, {{MC6809::IY, 0}, MC6809::LDYi_o0}, {{MC6809::IY, 5}, MC6809::LDYi_o5}, {{MC6809::IY, 8}, MC6809::LDYi_o8}, {{MC6809::IY, 16}, MC6809::LDYi_o16},
      {{MC6809::SU, -1}, MC6809::LDUi_o16}, {{MC6809::SU, 0}, MC6809::LDUi_o0}, {{MC6809::SU, 5}, MC6809::LDUi_o5}, {{MC6809::SU, 8}, MC6809::LDUi_o8}, {{MC6809::SU, 16}, MC6809::LDUi_o16},
      {{MC6809::SS, -1}, MC6809::LDSi_o16}, {{MC6809::SS, 0}, MC6809::LDSi_o0}, {{MC6809::SS, 5}, MC6809::LDSi_o5}, {{MC6809::SS, 8}, MC6809::LDSi_o8}, {{MC6809::SS, 16}, MC6809::LDSi_o16},
  };
  LoadIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::LDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::LDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::LDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::LDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::LDAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::LDAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::LDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::LDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::LDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::LDBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::LDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::LDBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::LDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::LDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::LDDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::LDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::LDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::LDDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::LDEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::LDEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::LDEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::LDEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::LDEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::LDEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::LDFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::LDFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::LDFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::LDFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::LDFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::LDFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::LDWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::LDWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::LDWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::LDWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::LDWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::LDWi_oW},
      {{MC6809::AQ, MC6809::AA}, MC6809::LDQi_oA}, {{MC6809::AQ, MC6809::AB}, MC6809::LDQi_oB}, {{MC6809::AQ, MC6809::AD}, MC6809::LDQi_oD}, {{MC6809::AQ, MC6809::AE}, MC6809::LDQi_oE}, {{MC6809::AQ, MC6809::AF}, MC6809::LDQi_oF}, {{MC6809::AQ, MC6809::AW}, MC6809::LDQi_oW},
      {{MC6809::IX, MC6809::AA}, MC6809::LDXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::LDXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::LDXi_oD}, {{MC6809::IX, MC6809::AE}, MC6809::LDXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::LDXi_oF}, {{MC6809::IX, MC6809::AW}, MC6809::LDXi_oW},
      {{MC6809::IY, MC6809::AA}, MC6809::LDYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::LDYi_oB}, {{MC6809::IY, MC6809::AD}, MC6809::LDYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::LDYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::LDYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::LDYi_oW},
      {{MC6809::SU, MC6809::AA}, MC6809::LDUi_oA}, {{MC6809::SU, MC6809::AB}, MC6809::LDUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::LDUi_oD}, {{MC6809::SU, MC6809::AE}, MC6809::LDUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::LDUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::LDUi_oW},
      {{MC6809::SS, MC6809::AA}, MC6809::LDSi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::LDSi_oB}, {{MC6809::SS, MC6809::AD}, MC6809::LDSi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::LDSi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::LDSi_oF}, {{MC6809::SS, MC6809::AW}, MC6809::LDSi_oW},
  };
}

unsigned MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    break;
  case MC6809::LDAi_o8:
  case MC6809::LDBi_o8:
  case MC6809::LDDi_o8:
  case MC6809::LDEi_o8:
  case MC6809::LDFi_o8:
  case MC6809::LDWi_o8:
  case MC6809::LDQi_o8:
  case MC6809::LDXi_o8:
  case MC6809::LDYi_o8:
  case MC6809::LDAi_o16:
  case MC6809::LDBi_o16:
  case MC6809::LDDi_o16:
  case MC6809::LDEi_o16:
  case MC6809::LDFi_o16:
  case MC6809::LDWi_o16:
  case MC6809::LDQi_o16:
  case MC6809::LDXi_o16:
  case MC6809::LDYi_o16:
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() && MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }

  return 0;
}

unsigned MC6809InstrInfo::isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    break;
  case MC6809::STAi_o8:
  case MC6809::STBi_o8:
  case MC6809::STDi_o8:
  case MC6809::STEi_o8:
  case MC6809::STFi_o8:
  case MC6809::STWi_o8:
  case MC6809::STQi_o8:
  case MC6809::STXi_o8:
  case MC6809::STYi_o8:
  case MC6809::STAi_o16:
  case MC6809::STBi_o16:
  case MC6809::STDi_o16:
  case MC6809::STEi_o16:
  case MC6809::STFi_o16:
  case MC6809::STWi_o16:
  case MC6809::STQi_o16:
  case MC6809::STXi_o16:
  case MC6809::STYi_o16:
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() &&
        MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  return 0;
}

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB,
                                    MachineBasicBlock::iterator I,
                                    Register DestReg, unsigned SubIdx,
                                    const MachineInstr &Orig,
                                    const TargetRegisterInfo &TRI) const {
  if (Orig.getOpcode() == MC6809::Load16Imm) {
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->RemoveOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(MC6809::Load16Imm));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig, TRI);
  }
}

MachineInstr *MC6809InstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI, unsigned Idx1, unsigned Idx2) const {
  // NOTE: This doesn't seem to actually be used anywhere.
  if (NewMI)
    report_fatal_error("NewMI is not supported");

  MachineFunction &MF = *MI.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const MC6809RegisterInfo *TRI = STI.getRegisterInfo();

  LLVM_DEBUG(dbgs() << "Commute: " << MI);

  // Determines the register class for a given virtual register constrained by a
  // target register class and all uses outside this instruction. This
  // effectively removes the constraints due to just this instruction, then
  // tries to apply the constraint for the other operand.
  const auto NewRegClass = [&](Register Reg, const TargetRegisterClass *RC) -> const TargetRegisterClass * {
    for (MachineOperand &MO : MRI.reg_nodbg_operands(Reg)) {
      MachineInstr *UseMI = MO.getParent();
      if (UseMI == &MI)
        continue;
      unsigned OpNo = &MO - &UseMI->getOperand(0);
      RC = UseMI->getRegClassConstraintEffect(OpNo, RC, this, TRI);
      if (!RC)
        return nullptr;
    }
    return RC;
  };

  const TargetRegisterClass *RegClass1 = getRegClass(MI.getDesc(), Idx1, TRI, MF);
  const TargetRegisterClass *RegClass2 = getRegClass(MI.getDesc(), Idx2, TRI, MF);
  Register Reg1 = MI.getOperand(Idx1).getReg();
  Register Reg2 = MI.getOperand(Idx2).getReg();

  // See if swapping the two operands are possible given their register classes.
  const TargetRegisterClass *Reg1Class = nullptr;
  const TargetRegisterClass *Reg2Class = nullptr;
  if (Reg1.isVirtual()) {
    Reg1Class = NewRegClass(Reg1, RegClass2);
    if (!Reg1Class)
      return nullptr;
  }
  if (Reg1.isPhysical() && !RegClass2->contains(Reg1))
    return nullptr;
  if (Reg2.isVirtual()) {
    Reg2Class = NewRegClass(Reg2, RegClass1);
    if (!Reg2Class)
      return nullptr;
  }
  if (Reg2.isPhysical() && !RegClass1->contains(Reg2))
    return nullptr;

  // If this fails, make sure to get it out of the way before rewriting reg
  // classes.
  MachineInstr *CommutedMI = TargetInstrInfo::commuteInstructionImpl(MI, NewMI, Idx1, Idx2);
  if (!CommutedMI)
    return nullptr;

  // PHI nodes keep the register classes of all their arguments. By the time the
  // two address instruction pass occurs, these phis have already been lowered
  // to copies. Changing register classes here can make those register classes
  // mismatch the new ones; to avoid this, we recompute the register classes for
  // any vregs copied into or out of a commuted vreg.
  const auto RecomputeCopyRC = [&](Register Reg) {
    for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
      if (!MI.isCopy())
        continue;
      Register Other = MI.getOperand(0).getReg() == Reg ? MI.getOperand(1).getReg() : MI.getOperand(0).getReg();
      if (!Other.isVirtual())
        continue;
      MRI.recomputeRegClass(Other);
    }
  };

  // Use the new register classes computed above, if any.
  if (Reg1Class) {
    MRI.setRegClass(Reg1, Reg1Class);
    RecomputeCopyRC(Reg1);
  }
  if (Reg2Class) {
    MRI.setRegClass(Reg2, Reg2Class);
    RecomputeCopyRC(Reg2);
  }
  return CommutedMI;
}

unsigned MC6809InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  const MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MCAsmInfo *MAI = MF->getTarget().getMCAsmInfo();

  const MCInstrDesc &MCID = MI.getDesc();

  switch (MI.getOpcode()) {
  default:
    // Return the size specified in .td file. If there's none, return 0, as we
    // can't define a default size.
    return MCID.getSize();
  case TargetOpcode::BUNDLE:
    return getInstBundleLength(MI);
  case MC6809::INLINEASM:
  case MC6809::INLINEASM_BR: {
    // If this machine instr is an inline asm, measure it.
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(), *MAI);
  }
  }
}

unsigned MC6809InstrInfo::getInstBundleLength(const MachineInstr &MI) const {
  unsigned Size = 0;
  MachineBasicBlock::const_instr_iterator I = MI.getIterator();
  MachineBasicBlock::const_instr_iterator E = MI.getParent()->instr_end();
  while (++I != E && I->isInsideBundle()) {
    assert(!I->isBundle() && "No nested bundle!");
    Size += getInstSizeInBytes(*I);
  }
  return Size;
}

// 6809 instructions aren't as regular as most commutable instructions, so this
// routine determines the commutable operands manually.
bool MC6809InstrInfo::findCommutedOpIndices(const MachineInstr &MI, unsigned &SrcOpIdx1, unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() && "MC6809InstrInfo::findCommutedOpIndices() can't handle bundles");

  // XXXX: FIXME: MarkM - Find and commute the 6809 instructions
  return false;
}

MachineBasicBlock *
MC6809InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::JumpRelative:
    return MI.getOperand(0).getMBB();
  case MC6809::JumpIndir:
    return nullptr;
  }
}

bool MC6809InstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB, SmallVectorImpl<MachineOperand> &Cond, bool AllowModify) const {
  auto I = MBB.getFirstTerminator();

  // Advance past any comparison terminators.
  while (I != MBB.end() && I->isCompare())
    ++I;

  // If no terminators, falls through.
  if (I == MBB.end())
    return false;

  // Non-branch terminators cannot be analyzed.
  if (!I->isBranch())
    return true;

  // Analyze first branch.
  auto FirstBR = I++;
  if (FirstBR->isPreISelOpcode())
    return true;
  // First branch always forms true edge, whether conditional or unconditional.
  TBB = getBranchDestBlock(*FirstBR);
  if (!TBB)
    return true;
  if (FirstBR->isConditionalBranch()) {
    Cond.push_back(FirstBR->getOperand(1));
    Cond.push_back(FirstBR->getOperand(2));
  }

  // If there's no second branch, done.
  if (I == MBB.end())
    return false;

  // Cannot analyze branch followed by non-branch.
  if (!I->isBranch())
    return true;

  auto SecondBR = I++;

  // If any instructions follow the second branch, cannot analyze.
  if (I != MBB.end())
    return true;

  // Exactly two branches present.

  // Can only analyze conditional branch followed by unconditional branch.
  if (!SecondBR->isUnconditionalBranch() || SecondBR->isPreISelOpcode())
    return true;

  // Second unconditional branch forms false edge.
  FBB = getBranchDestBlock(*SecondBR);
  if (!FBB)
    return true;
  return false;
}

unsigned MC6809InstrInfo::removeBranch(MachineBasicBlock &MBB, int *BytesRemoved) const {
  // Since analyzeBranch succeeded, we know that the only terminators are
  // comparisons and branches.

  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();

  // Advance to first branch.
  while (Begin != End && Begin->isCompare())
    ++Begin;

  // Erase all remaining terminators.
  unsigned NumRemoved = std::distance(Begin, End);
  if (BytesRemoved) {
    *BytesRemoved = 0;
    for (const auto &I : make_range(Begin, End))
      *BytesRemoved += getInstSizeInBytes(I);
  }
  MBB.erase(Begin, End);
  return NumRemoved;
}

unsigned MC6809InstrInfo::insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  // Since analyzeBranch succeeded and any existing branches were removed, the
  // only remaining terminators are comparisons.

  const MC6809Subtarget &STI = MBB.getParent()->getSubtarget<MC6809Subtarget>();

  MachineIRBuilder Builder(MBB, MBB.end());
  unsigned NumAdded = 0;
  if (BytesAdded)
    *BytesAdded = 0;

  // Unconditional branch target.
  auto *UBB = TBB;

  // Conditional branch.
  if (!Cond.empty()) {
    assert(TBB);
    // The condition stores the arguments for the Bcc and LBcc instructionis.
    assert(Cond.size() == 2);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    // Add conditional branch.
    unsigned Opcode = MC6809::Bbc;
    auto BR = Builder.buildInstr(Opcode).addMBB(TBB);
    for (const MachineOperand &Op : Cond)
      BR.add(Op);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    // For 6809, assume BRA and relax into LBRA in insertIndirectBranch if
    // necessary.
    // XXXX: FIXME: MarkM - ensure this is unconditional
    auto JMP = Builder.buildInstr(MC6809::Bbc).addMBB(UBB);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*JMP);
  }

  return NumAdded;
}

void MC6809InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB, MachineBasicBlock &NewDestBB, MachineBasicBlock &RestoreBB, const DebugLoc &DL, int64_t BrOffset, RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.
  // XXXX: FIXME: MarkM - this process is a crock; LBRA should alway work.

  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  // XXXX: FIXME: MarkM - ensure this is unconditional
  Builder.buildInstr(MC6809::Bbc).addMBB(&NewDestBB);
}

void MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg, bool KillSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  copyPhysRegImpl(Builder, DestReg, SrcReg);
}

#if 0
static Register createVReg(MachineIRBuilder &Builder, const TargetRegisterClass &RC) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Resetting NoVRegs\n";);
  Builder.getMF().getProperties().reset(MachineFunctionProperties::Property::NoVRegs);
  return Builder.getMRI()->createVirtualRegister(&RC);
}
#endif

void MC6809InstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg, Register SrcReg) const {
  if (DestReg == SrcReg)
    return;

  LLVM_DEBUG(const MC6809RegisterInfo *TRI = STI.getRegisterInfo(); dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : Src = " << TRI->getRegAsmName(SrcReg) << " : Dest = " << TRI->getRegAsmName(DestReg) << "\n";);
  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() && !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) {
    return IsClass(DestReg, Dest) && IsClass(SrcReg, Src);
  };

  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    if (AreClasses(MC6809::ADcRegClass, MC6809::ABcRegClass) ||
        AreClasses(MC6809::AWcRegClass, MC6809::AFcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    if (AreClasses(MC6809::ABcRegClass, MC6809::ADcRegClass) ||
        AreClasses(MC6809::AFcRegClass, MC6809::AWcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) ||
             AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    const MC6809RegisterInfo *TRI = STI.getRegisterInfo();
    Register SrcReg8 = TRI->getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 = TRI->getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    assert(SrcReg8 && DestReg8 && "Single-bit Src and Dst must both be LSB of 8-bit registers"); 

    const MachineInstr &MI = *Builder.getInsertPt();
    // MC6809 defines LSB writes to write the whole 8-bit register, not just
    // part of it.
    assert(!MI.readsRegister(DestReg8));
    copyPhysRegImpl(Builder, DestReg8, SrcReg8);
  } else
    llvm_unreachable("Unexpected physical register copy.");
}

const TargetRegisterClass *
MC6809InstrInfo::canFoldCopy(const MachineInstr &MI, unsigned FoldIdx) const {
  if (!MI.getMF()->getFunction().doesNotRecurse())
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MC6809::ACC8RegClass.contains(FoldReg) || MC6809::BIT1RegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MC6809::ACC8RegClass || RC == &MC6809::BIT1RegClass)
      return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  }
  return nullptr;
}

void MC6809InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg, bool isKill, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI->dump(););
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, TRI, /*IsLoad=*/false);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI->dump(););
}

void MC6809InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI->dump(););
  loadStoreRegStackSlot(MBB, MI, DestReg, false, FrameIndex, RC, TRI, /*IsLoad=*/true);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI->dump(););
}

void MC6809InstrInfo::loadStoreRegStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg, bool IsKill, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI, bool IsLoad) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
#if 0
  MachineFunction &MF = *MBB.getParent();

  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  if ((Reg.isPhysical() && MC6809::ACC16RegClass.contains(Reg)) || (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))) {
    Register Tmp = Reg;
    if (!Reg.isPhysical()) {
      assert(Reg.isVirtual());
      Tmp = MRI.createVirtualRegister(&MC6809::ACC16RegClass);
    }
    if (!IsLoad) {
      if (Tmp != Reg)
        Builder.buildCopy(Tmp, Reg);

      // The register may not have been fully defined at this point. Adding a
      // KILL here makes the entire value alive, regardless of whether or not
      // it was prior to the store. We do this because this function does not
      // have access to the detailed liveness information about the virtual
      // register in use; if we did, we'd only need to store the portion of
      // the virtual register that is actually alive.
      Builder.buildInstr(MC6809::KILL, {Tmp}, {Tmp});
    }
    loadStoreStaticStackSlot(Builder, MachineOperand::CreateReg(Tmp, IsLoad), FrameIndex, 0, MMO);
    if (IsLoad && Tmp != Reg)
      Builder.buildCopy(Reg, Tmp);
  } else {
    loadStoreStaticStackSlot(Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MMO);
  }

  LLVM_DEBUG({ dbgs() << "Inserted stack slot load/store:\n";
    for (const auto &MI : make_range(MIS.begin(), MIS.getInitial()))
      dbgs() << MI;
  });
#endif /* 0 */
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

bool MC6809InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
#if 0
  // Post RA
  case MC6809::INC:
  case MC6809::DEC:
    expandIncDec(Builder);
    break;
#endif
  case MC6809::LEAPtrAddImm:
    expandLEAPtrAddImm(Builder, MI);
    break;
  case MC6809::LEAPtrAddReg8:
  case MC6809::LEAPtrAddReg16:
    expandLEAPtrAddReg(Builder, MI);
    break;
  case MC6809::Load8Imm:
  case MC6809::Load16Imm:
  case MC6809::Load32Imm:
    expandLoadImm(Builder, MI);
    break;
  case MC6809::Load8IdxImm:
  case MC6809::Load16IdxImm:
  case MC6809::Load32IdxImm:
    expandLoadIdxImm(Builder, MI);
    break;
  case MC6809::Load8IdxReg8:
  case MC6809::Load16IdxReg8:
  case MC6809::Load32IdxReg8:
  case MC6809::Load8IdxReg16:
  case MC6809::Load16IdxReg16:
  case MC6809::Load32IdxReg16:
    expandLoadIdxReg(Builder, MI);
    break;
  case MC6809::Load8IdxZero:
  case MC6809::Load16IdxZero:
  case MC6809::Load32IdxZero:
    expandLoadIdxZero(Builder, MI);
    break;
  case MC6809::Store8IdxZero:
  case MC6809::Store16IdxZero:
  case MC6809::Store32IdxZero:
    expandStoreIdxZero(Builder, MI);
    break;
  case MC6809::Push8:
  case MC6809::Push16: {
    MI.setDesc(Builder.getTII().get(MC6809::PSHSs));
    unsigned short regList = 0;
    switch (MI.getOperand(0).getReg()) {
    case MC6809::CC:
      regList |= 1;
      break;
    case MC6809::AA:
      regList |= 2;
      break;
    case MC6809::AB:
      regList |= 4;
      break;
    case MC6809::AD:
      regList |= 6;
      break;
    case MC6809::DP:
      regList |= 8;
      break;
    case MC6809::IX:
      regList |= 16;
      break;
    case MC6809::IY:
      regList |= 32;
      break;
    case MC6809::SU:
      regList |= 64;
      break;
    case MC6809::PC:
      regList |= 128;
      break;
    }
    MI.RemoveOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
  case MC6809::Pull8:
  case MC6809::Pull16: {
    MI.setDesc(Builder.getTII().get(MC6809::PULSs));
    unsigned short regList = 0;
    switch (MI.getOperand(0).getReg()) {
    case MC6809::CC:
      regList |= 1;
      break;
    case MC6809::AA:
      regList |= 2;
      break;
    case MC6809::AB:
      regList |= 4;
      break;
    case MC6809::AD:
      regList |= 6;
      break;
    case MC6809::DP:
      regList |= 8;
      break;
    case MC6809::IX:
      regList |= 16;
      break;
    case MC6809::IY:
      regList |= 32;
      break;
    case MC6809::SU:
      regList |= 64;
      break;
    case MC6809::PC:
      regList |= 128;
      break;
    }
    MI.RemoveOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
#if 0
  case MC6809::LDImm1:
    expandLDImm1(Builder);
    break;
  case MC6809::LDImm16Remat:
    expandLDImmRemat(Builder);
    break;
  case MC6809::LDZ:
    expandLDZ(Builder);
    break;
  case MC6809::CMPNZVCImm:
  case MC6809::CMPNZVCAbs:
  case MC6809::CMPNZVCAbsIdx:
  case MC6809::CMPNZVCIndirIdx:
  case MC6809::SBCNZVCImm:
  case MC6809::SBCNZVCAbs:
  case MC6809::SBCNZVCAbsIdx:
  case MC6809::SBCNZVCIndirIdx:
    expandNZ(Builder);
    break;
  case MC6809::CMPTermImm:
  case MC6809::CMPTermAbs:
  case MC6809::CMPTermIndir:
  case MC6809::CMPTermIdx:
    expandCMPTerm(Builder);
    break;

  // Control flow
  case MC6809::GBR:
    expandGBR(Builder);
    break;
#endif
  }

  return Changed;
}

//===---------------------------------------------------------------------===//
// Post RA pseudos
//===---------------------------------------------------------------------===//

void MC6809InstrInfo::expandLoadIdxZero(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  unsigned Opcode;
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for Load(8|16|32)IdxZero.");
  case MC6809::AA:
    Opcode = MC6809::LDAi_o0;
    break;
  case MC6809::AB:
    Opcode = MC6809::LDBi_o0;
    break;
  case MC6809::AE:
    Opcode = MC6809::LDEi_o0;
    break;
  case MC6809::AF:
    Opcode = MC6809::LDFi_o0;
    break;
  case MC6809::AD:
    Opcode = MC6809::LDDi_o0;
    break;
  case MC6809::AW:
    Opcode = MC6809::LDWi_o0;
    break;
  case MC6809::IX:
    Opcode = MC6809::LDXi_o0;
    break;
  case MC6809::IY:
    Opcode = MC6809::LDYi_o0;
    break;
  case MC6809::SU:
    Opcode = MC6809::LDUi_o0;
    break;
  case MC6809::SS:
    Opcode = MC6809::LDSi_o0;
    break;
  }
  MI.setDesc(Builder.getTII().get(Opcode));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandStoreIdxZero(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  unsigned Opcode;
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for Store(8|16|32)IdxZero.");
  case MC6809::AA:
    Opcode = MC6809::STAi_o0;
    break;
  case MC6809::AB:
    Opcode = MC6809::STBi_o0;
    break;
  case MC6809::AE:
    Opcode = MC6809::STEi_o0;
    break;
  case MC6809::AF:
    Opcode = MC6809::STFi_o0;
    break;
  case MC6809::AD:
    Opcode = MC6809::STDi_o0;
    break;
  case MC6809::AW:
    Opcode = MC6809::STWi_o0;
    break;
  case MC6809::IX:
    Opcode = MC6809::STXi_o0;
    break;
  case MC6809::IY:
    Opcode = MC6809::STYi_o0;
    break;
  case MC6809::SU:
    Opcode = MC6809::STUi_o0;
    break;
  case MC6809::SS:
    Opcode = MC6809::STSi_o0;
    break;
  }
  MI.setDesc(Builder.getTII().get(Opcode));
  MI.RemoveOperand(0);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLEAPtrAddImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand IndexReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);
  uint64_t Offset = OffsetOp.getImm();
  int OffsetSize = Offset == 0 ? 0
                 : Offset < 32 ? 5
                 : Offset < 256 ? 8
                 : Offset < 65536 ? 16
                 : -1;
  RegPlusOffsetLen Lookup{IndexReg.getReg(), OffsetSize};
  auto OpcodePair = LEAPtrAddImmOpcode.find(Lookup);
  if (OpcodePair == LEAPtrAddImmOpcode.end())
    llvm_unreachable("Unexpected operand(s).");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.RemoveOperand(1);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLEAPtrAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand IndexReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);
  RegPlusReg Lookup{IndexReg.getReg(), OffsetOp.getReg()};
  auto OpcodePair = LEAPtrAddRegOpcode.find(Lookup);
  if (OpcodePair == LEAPtrAddRegOpcode.end())
    llvm_unreachable("Unexpected operand(s).");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.RemoveOperand(1);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = LoadImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == LoadImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadIdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);
  uint64_t Offset = OffsetOp.getImm();
  int OffsetSize = Offset == 0 ? 0
                 : Offset < 32 ? 5
                 : Offset < 256 ? 8
                 : Offset < 65536 ? 16
                 : -1;
  RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
  auto OpcodePair = LoadIdxImmOpcode.find(Lookup);
  if (OpcodePair == LoadIdxImmOpcode.end())
    llvm_unreachable("Unexpected operand(s).");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.RemoveOperand(0);
  MI.RemoveOperand(0);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadIdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);
  RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
  auto OpcodePair = LoadIdxRegOpcode.find(Lookup);
  if (OpcodePair == LoadIdxRegOpcode.end())
    llvm_unreachable("Unexpected operand(s).");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.RemoveOperand(1);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

bool MC6809InstrInfo::reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 2);
  auto &Val = Cond[1];
  Val.setImm(!Val.getImm());
  // Success.
  return false;
}

std::pair<unsigned, unsigned>
MC6809InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char *>>
MC6809InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {
      {MC6809::MO_LO, "lo"},
      {MC6809::MO_HI, "hi"},
      {MC6809::MO_HI_JT, "hi-jt"}};
  return Flags;
}
