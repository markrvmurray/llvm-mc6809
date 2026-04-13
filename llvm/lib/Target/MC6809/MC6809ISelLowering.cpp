//===-- MC6809ISelLowering.cpp - MC6809 DAG Lowering Implementation -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interfaces that MC6809 uses to lower LLVM code into a
// selection DAG.
//
//===----------------------------------------------------------------------===//

#include "MC6809ISelLowering.h"

#include "llvm/ADT/StringSwitch.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MC6809.h"
#include "MC6809InstrBuilder.h"
#include "MC6809InstrInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MC6809TargetMachine.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-isellowering"

MC6809TargetLowering::MC6809TargetLowering(const MC6809TargetMachine &TM, const MC6809Subtarget &STI) : TargetLowering(TM, STI) {
  addRegisterClass(MVT::i1, &MC6809::BIT1RegClass);
  addRegisterClass(MVT::i8, &MC6809::ACC8RegClass);
  addRegisterClass(MVT::i16, &MC6809::ACC16RegClass);
  if (STI.has6309())
    addRegisterClass(MVT::i32, &MC6809::ACC32RegClass);
  computeRegisterProperties(STI.getRegisterInfo());

  // Libcall name overrides (memset -> __memset, divrem names) are now
  // declared in RuntimeLibcalls.td under MC6809SystemLibrary.

  // Used in legalizer (etc.) to refer to the stack pointer.
  setStackPointerRegisterToSaveRestore(MC6809::SS);

  setMaximumJumpTableSize(std::min(256u, getMaximumJumpTableSize()));
}

MVT MC6809TargetLowering::getRegisterTypeForCallingConv(LLVMContext &Context, CallingConv::ID CC, EVT VT, const ISD::ArgFlagsTy &Flags) const {
  if (Flags.isPointer())
    return Flags.getPointerAddrSpace() == MC6809::AS_DirectPage ? MVT::i8 : MVT::i16;
  return TargetLowering::getRegisterTypeForCallingConv(Context, CC, VT, Flags);
}

unsigned MC6809TargetLowering::getNumRegistersForCallingConv(LLVMContext &Context, CallingConv::ID CC, EVT VT, const ISD::ArgFlagsTy &Flags) const {
  if (Flags.isPointer())
    return 1;
  return TargetLowering::getNumRegistersForCallingConv(Context, CC, VT, Flags);
}

unsigned MC6809TargetLowering::getNumRegistersForInlineAsm(LLVMContext &Context, EVT VT) const {
  if (VT == MVT::i16)
    return 1;
  return TargetLowering::getNumRegistersForInlineAsm(Context, VT);
}

TargetLowering::ConstraintType MC6809TargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'x':
    case 'y':
    case 'A':
    case 'B':
    case 'c':
    case 'v':
      return C_Register;
    case 'a':
    case 'd':
    case 'q':
      return C_RegisterClass;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass *> MC6809TargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI, StringRef Constraint, MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'r':
      // gcc6809: 'r' = GENERAL_REGS = G_REGS (D,X,Y,U,S,PC + A,B).
      // Map to the widest allocatable hardware class for the size.
      if (VT == MVT::i16)
        return std::make_pair(0U, &MC6809::INDEX16RegClass);
      return std::make_pair(0U, &MC6809::ACC8RegClass);
    case 'q':
      // gcc6809: 'q' = Q_REGS (A, B — 8-bit byte registers).
      return std::make_pair(0U, &MC6809::ACC8RegClass);
    case 'd':
      // gcc6809: 'd' = D_REGS (D register — 16-bit accumulator).
      return std::make_pair(MC6809::AD, &MC6809::ADcRegClass);
    case 'a':
      // gcc6809: 'a' = A_REGS (X,Y,U,S,PC — 16-bit address registers).
      return std::make_pair(0U, &MC6809::INDEX16RegClass);
    case 'x':
      // gcc6809: 'x' maps to I_REGS. We keep it as X register for
      // compatibility with existing MC6809 inline asm.
      return std::make_pair(MC6809::IX, &MC6809::IXcRegClass);
    case 'y':
      return std::make_pair(MC6809::IY, &MC6809::IXcRegClass);
    case 'A':
      // gcc6809: 'A' = ACC_A_REGS (A register only).
      return std::make_pair(MC6809::AA, &MC6809::AAcRegClass);
    case 'B':
      // gcc6809: 'B' = ACC_B_REGS (B register only).
      return std::make_pair(MC6809::AB, &MC6809::ABcRegClass);
    }
  }
  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
}

static bool is8BitIndex(Type *Ty) {
  if (!Ty)
    return false;
  return Ty == Type::getInt8Ty(Ty->getContext());
}

bool MC6809TargetLowering::isLegalAddressingMode(const DataLayout &DL, const AddrMode &AM, Type *Ty, unsigned AddrSpace, Instruction *I) const {
  if (AM.Scale > 1 || AM.Scale < 0)
    return false;

  // Any Base + Index mode can be legally selected with direct page indexed
  // addressing.
  if (AddrSpace == MC6809::AS_DirectPage)
    return true;

  if (AM.Scale) {
    assert(AM.Scale == 1);
    if (!AM.HasBaseReg) {
      // Indexed addressing mode.
      if (is8BitIndex(AM.ScaleType))
        return true;

      // Consider a reg + 8-bit offset selectable via the indirect indexed
      // addressing mode.
      return !AM.BaseGV && 0 <= AM.BaseOffs && AM.BaseOffs < 256;
    }

    // Indirect indexed addressing mode: 16-bit register + 8-bit index register.
    // Doesn't matter which is 8-bit and which is 16-bit.
    return !AM.BaseGV && !AM.BaseOffs && (is8BitIndex(AM.BaseType) || is8BitIndex(AM.ScaleType));
  }

  if (AM.HasBaseReg) {
    // Indexed addressing mode.
    if (is8BitIndex(AM.BaseType))
      return true;

    // Consider an reg + 8-bit offset selectable via the indirect indexed
    // addressing mode.
    return !AM.BaseGV && 0 <= AM.BaseOffs && AM.BaseOffs < 256;
  }

  // Any other combination of GV and BaseOffset are just global offsets.
  return true;
}

bool MC6809TargetLowering::isTruncateFree(Type *SrcTy, Type *DstTy) const {
  if (!SrcTy->isIntegerTy() || !DstTy->isIntegerTy())
    return false;
  return SrcTy->getPrimitiveSizeInBits() > DstTy->getPrimitiveSizeInBits();
}

bool MC6809TargetLowering::isZExtFree(Type *SrcTy, Type *DstTy) const {
  if (!SrcTy->isIntegerTy() || !DstTy->isIntegerTy())
    return false;
  return SrcTy->getPrimitiveSizeInBits() < DstTy->getPrimitiveSizeInBits();
}

static MachineBasicBlock *emitConditionalImm(MachineInstr &MI, MachineBasicBlock *MBB);
static MachineBasicBlock *emitSelectImm(MachineInstr &MI, MachineBasicBlock *MBB);

MachineBasicBlock *MC6809TargetLowering::EmitInstrWithCustomInserter(MachineInstr &MI, MachineBasicBlock *MBB) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad opcode in EmitInstrWithCustomInserter");
  case MC6809::ConditionalImm:
    return emitConditionalImm(MI, MBB);
  case MC6809::SelectImm:
    return emitSelectImm(MI, MBB);
  }
}

// FIXME!! MarkM: Do load Immediate of all sizes, not just i1(=i8).
static MachineBasicBlock *emitConditionalImm(MachineInstr &MI, MachineBasicBlock *MBB) {
  // To "insert" Select* instructions, we actually have to insert the triangle
  // control-flow pattern.  The incoming instructions know the destination reg
  // to set, the flag to branch on, and the true/false values to select between.
  //
  // We produce the following control flow if the flag is neither N nor Z:
  //     HeadMBB
  //     |  \
  //     |  IfFalseMBB
  //     | /
  //    TailMBB
  //
  // If the flag is N or Z, then loading the true value in HeadMBB would clobber
  // the flag before the branch. We instead emit the following:
  //     HeadMBB
  //     |  \
  //     |  IfTrueMBB
  //     |      |
  //    IfFalse |
  //     |     /
  //     |    /
  //     TailMBB
  Register Dst = MI.getOperand(0).getReg();
  int64_t Condition = MI.getOperand(1).getImm();
  Register Flag = MI.getOperand(2).getReg();
  int64_t TrueValue = MI.getOperand(3).getImm();
  int64_t FalseValue = MI.getOperand(4).getImm();

  const BasicBlock *LLVM_BB = MBB->getBasicBlock();
  MachineFunction::iterator I = ++MBB->getIterator();
  MachineIRBuilder Builder(MI);

  MachineBasicBlock *HeadMBB = MBB;
  MachineFunction *F = MBB->getParent();
  MachineRegisterInfo &MRI = F->getRegInfo();
  unsigned Opcode;
  Register Dst0;
  Register Dst1;
  Opcode = MC6809::Load_i1_Imm;
  Dst0 = MRI.createVirtualRegister(&MC6809::BIT1RegClass);
  Dst1 = MRI.createVirtualRegister(&MC6809::BIT1RegClass);

  // Split out all instructions after MI into a new basic block, updating
  // liveins.
  MachineBasicBlock *TailMBB = HeadMBB->splitAt(MI);

  // If MI is the last instruction, splitAt won't create a new block.
  // Create a merge block and update the successor's PHIs.
  if (TailMBB == HeadMBB) {
    MachineBasicBlock *OrigSucc = &*I;
    TailMBB = F->CreateMachineBasicBlock(LLVM_BB);
    F->insert(OrigSucc->getIterator(), TailMBB);
    TailMBB->addSuccessor(OrigSucc);
    HeadMBB->replaceSuccessor(OrigSucc, TailMBB);
    for (MachineInstr &Phi : OrigSucc->phis())
      for (unsigned Idx = 1; Idx < Phi.getNumOperands(); Idx += 2)
        if (Phi.getOperand(Idx + 1).getMBB() == HeadMBB)
          Phi.getOperand(Idx + 1).setMBB(TailMBB);
    Builder.setInsertPt(*TailMBB, TailMBB->end());
    Builder.buildInstr(MC6809::LongBranchRelative).addMBB(OrigSucc);
  }

  HeadMBB->removeSuccessor(TailMBB);

  // Add the false block between HeadMBB and TailMBB
  MachineBasicBlock *IfFalseMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(TailMBB->getIterator(), IfFalseMBB);
  HeadMBB->addSuccessor(IfFalseMBB);
  for (const auto &LiveIn : TailMBB->liveins())
    if (Register(LiveIn.PhysReg) != Dst)
      IfFalseMBB->addLiveIn(LiveIn);
  IfFalseMBB->addSuccessor(TailMBB);

  MachineBasicBlock *IfTrueMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(TailMBB->getIterator(), IfTrueMBB);
  IfTrueMBB->addSuccessor(TailMBB);
  // Add the unconditional branch from IfFalseMBB to TailMBB.
  Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
  Builder.buildInstr(MC6809::LongBranchRelative).addMBB(TailMBB);
  for (const auto &LiveIn : IfFalseMBB->liveins())
    IfTrueMBB->addLiveIn(LiveIn);
  Builder.setInsertPt(*HeadMBB, MI.getIterator());

  // Insert branch.
  Builder.buildInstr(MC6809::ConditionalLongBranchRelative)
      .addImm(Condition)
      .addMBB(IfTrueMBB)
      .addUse(Flag);
  HeadMBB->addSuccessor(IfTrueMBB);

  Builder.setInsertPt(*IfTrueMBB, IfTrueMBB->begin());
  // Load true value.
  Builder.buildInstr(Opcode, {Dst1}, {TrueValue});

  // Insert false load.
  Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
  Builder.buildInstr(Opcode, {Dst0}, {FalseValue});

  Builder.setInsertPt(*TailMBB, TailMBB->begin());
  Builder.buildInstr(MC6809::PHI)
      .addDef(Dst)
      .addReg(Dst0)
      .addMBB(IfFalseMBB)
      .addReg(Dst1)
      .addMBB(IfTrueMBB);

  MI.eraseFromParent();

  return TailMBB;
}

// FIXME!! MarkM: Do load Immediate of all sizes, not just i8.
static MachineBasicBlock *emitSelectImm(MachineInstr &MI, MachineBasicBlock *MBB) {
  // To "insert" Select* instructions, we actually have to insert the triangle
  // control-flow pattern.  The incoming instructions know the destination reg
  // to set, the flag to branch on, and the true/false values to select between.
  //
  // We produce the following control flow if the flag is neither N nor Z:
  //     HeadMBB
  //     |  \
  //     |  IfFalseMBB
  //     | /
  //    TailMBB
  //
  // If the flag is N or Z, then loading the true value in HeadMBB would clobber
  // the flag before the branch. We instead emit the following:
  //     HeadMBB
  //     |  \
  //     |  IfTrueMBB
  //     |      |
  //    IfFalse |
  //     |     /
  //     |    /
  //     TailMBB
  Register Dst = MI.getOperand(0).getReg();
  Register Flag = MI.getOperand(1).getReg();
  int64_t TrueValue = MI.getOperand(2).getImm();
  int64_t FalseValue = MI.getOperand(3).getImm();

  const BasicBlock *LLVM_BB = MBB->getBasicBlock();
  MachineFunction::iterator I = ++MBB->getIterator();
  MachineIRBuilder Builder(MI);

  MachineBasicBlock *HeadMBB = MBB;
  MachineFunction *F = MBB->getParent();

  // Split out all instructions after MI into a new basic block, updating
  // liveins.
  MachineBasicBlock *TailMBB = HeadMBB->splitAt(MI);

  // If MI is the last instruction, splitAt won't insert a new block. In that
  // case, the block must fall through, since there's no branch. Thus the tail
  // MBB is just the next MBB.
  if (TailMBB == HeadMBB)
    TailMBB = &*I;

  HeadMBB->removeSuccessor(TailMBB);

  // Add the false block between HeadMBB and TailMBB
  MachineBasicBlock *IfFalseMBB = F->CreateMachineBasicBlock(LLVM_BB);
  F->insert(TailMBB->getIterator(), IfFalseMBB);
  HeadMBB->addSuccessor(IfFalseMBB);
  for (const auto &LiveIn : TailMBB->liveins())
    if (Register(LiveIn.PhysReg) != Dst)
      IfFalseMBB->addLiveIn(LiveIn);
  IfFalseMBB->addSuccessor(TailMBB);

  // Add a true block if necessary to avoid clobbering NZ.
  MachineBasicBlock *IfTrueMBB = nullptr;
  if (Flag == MC6809::N || Flag == MC6809::Z) {
    IfTrueMBB = F->CreateMachineBasicBlock(LLVM_BB);
    F->insert(TailMBB->getIterator(), IfTrueMBB);
    IfTrueMBB->addSuccessor(TailMBB);

    // Add the unconditional branch from IfFalseMBB to TailMBB.
    Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
    Builder.buildInstr(MC6809::LongBranchRelative).addMBB(TailMBB);
    for (const auto &LiveIn : IfFalseMBB->liveins())
      IfTrueMBB->addLiveIn(LiveIn);

    Builder.setInsertPt(*HeadMBB, MI.getIterator());
  }

  const auto LDImm = [&Builder, &Dst](int64_t Val) {
    if (MC6809::BIT1RegClass.contains(Dst)) {
      Builder.buildInstr(MC6809::Load_i1_Imm, {Dst}, {Val});
      return;
    }

    assert(MC6809::ACC8RegClass.contains(Dst));
    Builder.buildInstr(MC6809::Load_i8_Imm, {Dst}, {Val});
  };

  if (IfTrueMBB) {
    // Insert branch.
    // FIXME!! MarkM: LongBranch here?
    Builder.buildInstr(MC6809::BranchFlagIs).addMBB(IfTrueMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(IfTrueMBB);

    Builder.setInsertPt(*IfTrueMBB, IfTrueMBB->begin());
    // Load true value.
    LDImm(TrueValue);
  } else {
    // Load true value.
    LDImm(TrueValue);

    // Insert branch.

    Builder.buildInstr(MC6809::BranchFlagIs).addMBB(TailMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(TailMBB);
  }

  // Insert false load.
  Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
  LDImm(FalseValue);

  MI.eraseFromParent();

  return TailMBB;
}
