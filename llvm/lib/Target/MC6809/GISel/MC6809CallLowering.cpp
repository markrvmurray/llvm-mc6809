//===-- MC6809CallLowering.cpp - MC6809 Call lowering -----------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the lowering of LLVM calls to machine code calls for
// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "MC6809CallLowering.h"

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809CallingConv.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809TargetTransformInfo.h"

#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetCallingConv.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "mc6809-call-lowering"

namespace {

} // namespace

// =========================================================================================
// ==== IMPORTANT -- lowerReturn
// =========================================================================================
bool MC6809CallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                     const Value *Val, ArrayRef<Register> VRegs,
                                     FunctionLoweringInfo &FLI) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const DataLayout &DL = MF.getDataLayout();
  auto Ret = MIRBuilder.buildInstrNoInsert(MC6809::RTSImplicit);
  bool success = true;
  if (!VRegs.empty()) {
    auto &TLI = *getTLI<MC6809TargetLowering>();
    const auto &F = MF.getFunction();
    ArgInfo OrigRetInfo(VRegs, Val->getType(), 0);
    setArgFlags(OrigRetInfo, AttributeList::ReturnIndex, DL, F);
    SmallVector<ArgInfo, 4> SplitRetInfos;
    splitToValueTypes(OrigRetInfo, SplitRetInfos, DL, F.getCallingConv());
    MC6809OutgoingValueAssigner Assigner(RetCC_MC6809);
    MC6809CallReturnHandler Handler(MIRBuilder, MF.getRegInfo(), Ret);
    success = determineAndHandleAssignments(Handler, Assigner, SplitRetInfos,
                                            MIRBuilder, F.getCallingConv(),
                                            F.isVarArg());
  }
  MIRBuilder.insertInstr(Ret);
  return success;
}

void MC6809OutgoingValueHandler::assignValueToReg(Register ValVReg, Register PhysReg,
                                               CCValAssign VA) {
  MIB.addUse(PhysReg, RegState::Implicit);
  Register ExtReg = extendRegister(ValVReg, VA);
  MIRBuilder.buildCopy(PhysReg, ExtReg);
}

void MC6809OutgoingValueHandler::assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                                                   MachinePointerInfo &MPO, CCValAssign &VA) {
}

Register MC6809OutgoingValueHandler::getStackAddress(uint64_t Size, int64_t Offset,
                                                  MachinePointerInfo &MPO,
                                                  ISD::ArgFlagsTy Flags) {
  Register SS = MC6809::SS;
  return SS;
}

// =========================================================================================
// ==== IMPORTANT -- lowerFormalArguments
// =========================================================================================
bool MC6809CallLowering::lowerFormalArguments(MachineIRBuilder &MIRBuilder,
                                              const Function &F,
                                              ArrayRef<ArrayRef<Register>> VRegs,
                                              FunctionLoweringInfo &FLI) const {
  bool success = true;
  // Quick exit if there aren't any args.
  if (!F.arg_empty()) {
    auto &TLI = *getTLI<MC6809TargetLowering>();
    MachineFunction &MF = MIRBuilder.getMF();
    const DataLayout &DL = MF.getDataLayout();
    SmallVector<ArgInfo, 8> SplitArgs;
    unsigned i = 0;
    for (auto &Arg : F.args()) {
      ArgInfo OrigArg(VRegs[i], Arg, i);
      setArgFlags(OrigArg, i + AttributeList::FirstArgIndex, DL, F);
      splitToValueTypes(OrigArg, SplitArgs, DL, F.getCallingConv());
      i++;
    }
    MC6809IncomingValueAssigner Assigner(CC_MC6809);
    MC6809FormalArgHandler Handler(MIRBuilder, MF.getRegInfo());
    success =  determineAndHandleAssignments(Handler, Assigner, SplitArgs,
                                             MIRBuilder, F.getCallingConv(),
                                             F.isVarArg());
  }
  return success;
}

void MC6809IncomingValueHandler::assignValueToReg(Register ValVReg, Register PhysReg,
                                                  CCValAssign VA) {
  MIRBuilder.getMRI()->addLiveIn(PhysReg);
  MIRBuilder.getMBB().addLiveIn(PhysReg);
  IncomingValueHandler::assignValueToReg(ValVReg, PhysReg, VA);
}

void MC6809IncomingValueHandler::assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                                                      MachinePointerInfo &MPO, CCValAssign &VA) {
}

Register MC6809IncomingValueHandler::getStackAddress(uint64_t Size, int64_t Offset,
                                                     MachinePointerInfo &MPO,
                                                     ISD::ArgFlagsTy Flags) {
  Register SS = MC6809::SS;
  return SS;
}

// =========================================================================================
// ==== IMPORTANT -- lowerCall
// =========================================================================================
bool MC6809CallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                   CallLoweringInfo &Info) const {
  return false;
}
