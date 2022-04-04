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
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/SelectionDAG.h"
#include "llvm/CodeGen/TargetLoweringObjectFileImpl.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/ErrorHandling.h"

#include "MC6809.h"
#include "MC6809CallingConv.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MC6809TargetMachine.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

using namespace llvm;

//===----------------------------------------------------------------------===//
//                      Calling Convention Implementation
//===----------------------------------------------------------------------===//

/// Selects the correct CCAssignFn for a given CallingConvention value.
CCAssignFn *MC6809TargetLowering::CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg) const {
  return IsVarArg ? CC_MC6809_VarArgs : CC_MC6809;
}

CCAssignFn *MC6809TargetLowering::CCAssignFnForReturn(CallingConv::ID CC, bool IsVarArg) const {
  return IsVarArg ? CC_MC6809_VarArgs : RetCC_MC6809;
}

MC6809TargetLowering::MC6809TargetLowering(const MC6809TargetMachine &TM, const MC6809Subtarget &STI)
    : TargetLowering(TM) {
  // This is only used for CallLowering to determine how to split large
  // primitive types for the calling convention. All need to be split to 8 bits,
  // so that's all that we report here. The register class is irrelevant.
  addRegisterClass(MVT::i8, &MC6809::ACC8RegClass);
  addRegisterClass(MVT::i16, &MC6809::ACC16RegClass);
  if (STI.isHD6309())
    addRegisterClass(MVT::i32, &MC6809::ACC32RegClass);

  computeRegisterProperties(STI.getRegisterInfo());

  // The memset intrinsic takes a char, while the C memset takes an int. These
  // are different in the MC6809 calling convention, since arguments are not
  // automatically promoted to int. "memset" is the C version, and "__memset" is
  // the intrinsic version.
  setLibcallName(RTLIB::MEMSET, "__memset");

  setLibcallName(RTLIB::UDIVREM_I8, "__udivmodqi4");
  setLibcallName(RTLIB::UDIVREM_I16, "__udivmodhi4");
  setLibcallName(RTLIB::UDIVREM_I32, "__udivmodsi4");
  setLibcallName(RTLIB::UDIVREM_I64, "__udivmoddi4");
  setLibcallName(RTLIB::SDIVREM_I8, "__divmodqi4");
  setLibcallName(RTLIB::SDIVREM_I16, "__divmodhi4");
  setLibcallName(RTLIB::SDIVREM_I32, "__divmodsi4");
  setLibcallName(RTLIB::SDIVREM_I64, "__divmoddi4");

  // Used in legalizer (etc.) to refer to the stack pointer.
  setStackPointerRegisterToSaveRestore(MC6809::SS);
}

unsigned MC6809TargetLowering::getNumRegistersForInlineAsm(LLVMContext &Context, EVT VT) const {
  if (VT == MVT::i16)
    return 1;
  return TargetLowering::getNumRegistersForInlineAsm(Context, VT);
}

TargetLowering::ConstraintType
MC6809TargetLowering::getConstraintType(StringRef Constraint) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'a':
    case 'x':
    case 'y':
    case 'd':
      return C_Register;
    case 'R':
      return C_RegisterClass;
    }
  }
  return TargetLowering::getConstraintType(Constraint);
}

std::pair<unsigned, const TargetRegisterClass *>
MC6809TargetLowering::getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI, StringRef Constraint, MVT VT) const {
  if (Constraint.size() == 1) {
    switch (Constraint[0]) {
    default:
      break;
    case 'r':
      if (VT == MVT::i16)
        return std::make_pair(0U, &MC6809::ACC16RegClass);
      return std::make_pair(0U, &MC6809::ACC8RegClass);
    case 'R':
      return std::make_pair(0U, &MC6809::ACC8RegClass);
    case 'a':
      return std::make_pair(MC6809::AA, &MC6809::ACC8RegClass);
    case 'x':
      return std::make_pair(MC6809::IX, &MC6809::ACC16RegClass);
    case 'y':
      return std::make_pair(MC6809::IY, &MC6809::ACC16RegClass);
    case 'd':
      return std::make_pair(0U, &MC6809::INDEX16RegClass);
    }
  }
  if (Constraint == "{cc}")
    return std::make_pair(MC6809::CC, &MC6809::CCondRegClass);

  return TargetLowering::getRegForInlineAsmConstraint(TRI, Constraint, VT);
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

MachineBasicBlock *MC6809TargetLowering::EmitInstrWithCustomInserter(
    MachineInstr &MI, MachineBasicBlock *MBB) const {
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
  MachineIRBuilder Builder(*MBB, MI);

  MachineBasicBlock *HeadMBB = MBB;
  MachineFunction *F = MBB->getParent();

  const MC6809Subtarget &STI = F->getSubtarget<MC6809Subtarget>();

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
    if (LiveIn.PhysReg != Dst)
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
    // XXXX: FIXME: MarkM - Must make the below branch unconditional
    Builder.buildInstr(MC6809::Bbc).addMBB(TailMBB);
    for (const auto &LiveIn : IfFalseMBB->liveins())
      IfTrueMBB->addLiveIn(LiveIn);

    Builder.setInsertPt(*HeadMBB, MI.getIterator());
  }

  const auto LDImm8 = [&Builder, &Dst](int64_t Val) {
    assert(MC6809::ACC8RegClass.contains(Dst));
    Builder.buildInstr(MC6809::LDAi8, {Dst}, {Val});
  };

  if (IfTrueMBB) {
    // Insert branch.
    // XXXX: FIXME: MarkM - Must make the below branch unconditional
    Builder.buildInstr(MC6809::Bbc).addMBB(IfTrueMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(IfTrueMBB);

    Builder.setInsertPt(*IfTrueMBB, IfTrueMBB->begin());
    // Load true value.
    LDImm8(TrueValue);
  } else {
    // Load true value.
    LDImm8(TrueValue);

    // Insert branch.
    // XXXX: FIXME: MarkM - Must make the below branch unconditional
    Builder.buildInstr(MC6809::Bbc).addMBB(TailMBB).addUse(Flag).addImm(1);
    HeadMBB->addSuccessor(TailMBB);
  }

  // Insert false load.
  Builder.setInsertPt(*IfFalseMBB, IfFalseMBB->begin());
  LDImm8(FalseValue);

  MI.eraseFromParent();

  return TailMBB;
}
