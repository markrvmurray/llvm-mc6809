//===-- MC6809CallLowering.cpp - MC6809 Call lowering -----------------*- C++
//-*-===//
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

#include "MC6809CallingConv.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809TargetTransformInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

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

// ===========================================================================
// ---------------------------------------------------------------------------
/// Helper class for values coming in through an ABI boundary (used for handling
/// formal arguments and call return values).
struct MC6809IncomingValueHandler : public CallLowering::IncomingValueHandler {

  MC6809IncomingValueHandler(MachineIRBuilder &MIRBuilder,
                             MachineRegisterInfo &MRI)
      : IncomingValueHandler(MIRBuilder, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    assert((Size == 1 || Size == 2 || Size == 4) && "Unsupported size");

    auto &MFI = MIRBuilder.getMF().getFrameInfo();

    // Byval is assumed to be writable memory, but other stack passed arguments
    // are not.
    const bool IsImmutable = !Flags.isByVal();

    int FI = MFI.CreateFixedObject(Size, Offset, IsImmutable);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);

    return MIRBuilder.buildFrameIndex(LLT::pointer(MPO.getAddrSpace(), 16), FI)
        .getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    if (VA.getLocInfo() == CCValAssign::SExt ||
        VA.getLocInfo() == CCValAssign::ZExt) {
      // XXXX: FIXME: MarkM: Fix for HD6309 case with AQ register
      // If the value is zero- or sign-extended, its size becomes 2 bytes, so
      // that's what we should load.
      MemTy = LLT::scalar(16);
      assert(MRI.getType(ValVReg).isScalar() && "Only scalars supported atm");

      auto LoadVReg = buildLoad(LLT::scalar(16), Addr, MemTy, MPO);
      MIRBuilder.buildTrunc(ValVReg, LoadVReg);
    } else {
      // If the value is not extended, a simple load will suffice.
      buildLoad(ValVReg, Addr, MemTy, MPO);
    }
  }

  MachineInstrBuilder buildLoad(const DstOp &Res, Register Addr, LLT MemTy,
                                MachinePointerInfo &MPO) {
    MachineFunction &MF = MIRBuilder.getMF();

    auto MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOLoad, MemTy,
                                       inferAlignFromPtrInfo(MF, MPO));
    return MIRBuilder.buildLoad(Res, Addr, *MMO);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign VA) override {
    assert(VA.isRegLoc() && "Value shouldn't be assigned to reg");
    assert(VA.getLocReg() == PhysReg && "Assigning to the wrong reg?");

    uint64_t ValSize = VA.getValVT().getFixedSizeInBits();
    uint64_t LocSize = VA.getLocVT().getFixedSizeInBits();

    assert(ValSize <= 32 && "Unsupported value size");
    assert(LocSize <= 32 && "Unsupported location size");

    markPhysRegUsed(PhysReg);
    if (ValSize == LocSize) {
      MIRBuilder.buildCopy(ValVReg, PhysReg);
    } else {
      assert(ValSize < LocSize && "Extensions not supported");

      // We cannot create a truncating copy, nor a trunc of a physical register.
      // Therefore, we need to copy the content of the physical register into a
      // virtual one and then truncate that.
      auto PhysRegToVReg = MIRBuilder.buildCopy(LLT::scalar(LocSize), PhysReg);
      MIRBuilder.buildTrunc(ValVReg, PhysRegToVReg);
    }
  }

  /// Marking a physical register as used is different between formal
  /// parameters, where it's a basic block live-in, and call returns, where it's
  /// an implicit-def of the call instruction.
  virtual void markPhysRegUsed(unsigned PhysReg) = 0;
};

// ---------------------------------------------------------------------------
struct MC6809FormalArgHandler : public MC6809IncomingValueHandler {
  MC6809FormalArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI)
      : MC6809IncomingValueHandler(MIRBuilder, MRI) {}

  void markPhysRegUsed(unsigned PhysReg) override {
    MIRBuilder.getMRI()->addLiveIn(PhysReg);
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  }
};

// ---------------------------------------------------------------------------
struct MC6809CallReturnHandler : public MC6809IncomingValueHandler {
  MC6809CallReturnHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI, MachineInstrBuilder MIB)
      : MC6809IncomingValueHandler(MIRBuilder, MRI), MIB(MIB) {}

  void markPhysRegUsed(unsigned PhysReg) override {
    MIB.addDef(PhysReg, RegState::Implicit);
  }

  MachineInstrBuilder MIB;
};

// ===========================================================================
// ---------------------------------------------------------------------------
/// Helper class for values going out through an ABI boundary (used for handling
/// function return values and call parameters).
struct MC6809OutgoingValueHandler : public CallLowering::OutgoingValueHandler {
 MC6809OutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                             MachineRegisterInfo &MRI, MachineInstrBuilder &MIB)
      : OutgoingValueHandler(MIRBuilder, MRI), MIB(MIB) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override {
    assert((Size == 1 || Size == 2 || Size == 4) && "Unsupported size");

    LLT p0 = LLT::pointer(0, 16);
    LLT s16 = LLT::scalar(16);
    auto SPReg = MIRBuilder.buildCopy(p0, Register(MC6809::SS));

    auto OffsetReg = MIRBuilder.buildConstant(s16, Offset);

    auto AddrReg = MIRBuilder.buildPtrAdd(p0, SPReg, OffsetReg);

    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);
    return AddrReg.getReg(0);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign VA) override {
    assert(VA.isRegLoc() && "Value shouldn't be assigned to reg");
    assert(VA.getLocReg() == PhysReg && "Assigning to the wrong reg?");

    assert(VA.getValVT().getSizeInBits() <= 32 && "Unsupported value size");
    assert(VA.getLocVT().getSizeInBits() <= 32 && "Unsupported location size");

    Register ExtReg = extendRegister(ValVReg, VA);
    MIRBuilder.buildCopy(PhysReg, ExtReg);
    MIB.addUse(PhysReg, RegState::Implicit);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    Register ExtReg = extendRegister(ValVReg, VA);
    auto MMO = MIRBuilder.getMF().getMachineMemOperand(
        MPO, MachineMemOperand::MOStore, MemTy, Align(1));
    MIRBuilder.buildStore(ExtReg, Addr, *MMO);
  }

  MachineInstrBuilder MIB;
};

} // end anonymous namespace

// FIXME: This should move to the MC6809Subtarget when it supports all the
// opcodes.
static inline unsigned getCallOpcode(bool isDirect) {
  if (isDirect)
    return MC6809::CallRelative;
  return MC6809::CallIndir;
}

// FIXME: This should move to the MC6809Subtarget when it supports all the
// opcodes.
static inline unsigned getReturnOpcode() { return MC6809::ReturnImplicit; }

// =========================================================================================
// ==== IMPORTANT -- lowerReturn
// =========================================================================================
/// Lower the return value for the already existing \p Ret. This assumes that
/// \p MIRBuilder's insertion point is correct.
bool MC6809CallLowering::lowerReturnVal(MachineIRBuilder &MIRBuilder,
                                        const Value *Val,
                                        ArrayRef<Register> VRegs,
                                        MachineInstrBuilder &Ret) const {
  if (!Val)
    // Nothing to do here.
    return true;

  auto &MF = MIRBuilder.getMF();
  const auto &F = MF.getFunction();

  const auto &DL = MF.getDataLayout();
  auto &TLI = *getTLI<MC6809TargetLowering>();

  ArgInfo OrigRetInfo(VRegs, Val->getType(), 0);
  setArgFlags(OrigRetInfo, AttributeList::ReturnIndex, DL, F);

  SmallVector<ArgInfo, 4> SplitRetInfos;
  splitToValueTypes(OrigRetInfo, SplitRetInfos, DL, F.getCallingConv());

  CCAssignFn *AssignFn =
      TLI.CCAssignFnForReturn(F.getCallingConv(), F.isVarArg());

  OutgoingValueAssigner RetAssigner(AssignFn);
  MC6809OutgoingValueHandler RetHandler(MIRBuilder, MF.getRegInfo(), Ret);
  return determineAndHandleAssignments(RetHandler, RetAssigner, SplitRetInfos,
                                       MIRBuilder, F.getCallingConv(),
                                       F.isVarArg());
}

bool MC6809CallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                     const Value *Val, ArrayRef<Register> VRegs,
                                     FunctionLoweringInfo &FLI) const {
  assert(!Val == VRegs.empty() && "Return value without a vreg");

  unsigned Opcode = getReturnOpcode();
  auto Ret = MIRBuilder.buildInstrNoInsert(Opcode);

  if (!lowerReturnVal(MIRBuilder, Val, VRegs, Ret))
    return false;

  MIRBuilder.insertInstr(Ret);
  return true;
}

#if 0
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
    auto RetAssignFn = TLI.CCAssignFnForReturn(F.getCallingConv(), F.isVarArg());
    OutgoingValueAssigner Assigner(RetAssignFn);
    MC6809OutgoingValueHandler Handler(MIRBuilder, MF.getRegInfo(), Ret);
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
#endif

// =========================================================================================
// ==== IMPORTANT -- lowerFormalArguments
// =========================================================================================
bool MC6809CallLowering::lowerFormalArguments(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs, FunctionLoweringInfo &FLI) const {
  auto &TLI = *getTLI<MC6809TargetLowering>();

  // Quick exit if there aren't any args
  if (F.arg_empty())
    return true;

  if (F.isVarArg())
    return false;

  auto &MF = MIRBuilder.getMF();
  auto &MBB = MIRBuilder.getMBB();
  const auto &DL = MF.getDataLayout();

  CCAssignFn *AssignFn =
      TLI.CCAssignFnForCall(F.getCallingConv(), F.isVarArg());

  OutgoingValueAssigner ArgAssigner(AssignFn);
  MC6809FormalArgHandler ArgHandler(MIRBuilder,
                                    MIRBuilder.getMF().getRegInfo());

  SmallVector<ArgInfo, 8> SplitArgInfos;
  unsigned Idx = 0;
  for (auto &Arg : F.args()) {
    ArgInfo OrigArgInfo(VRegs[Idx], Arg.getType(), Idx);

    setArgFlags(OrigArgInfo, Idx + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArgInfo, SplitArgInfos, DL, F.getCallingConv());

    Idx++;
  }

  if (!MBB.empty())
    MIRBuilder.setInstr(*MBB.begin());

  if (!determineAndHandleAssignments(ArgHandler, ArgAssigner, SplitArgInfos,
                                     MIRBuilder, F.getCallingConv(),
                                     F.isVarArg()))
    return false;

  // Move back to the end of the basic block.
  MIRBuilder.setMBB(MBB);
  return true;
}

#if 0
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
#endif

// =========================================================================================
// ==== IMPORTANT -- lowerCall
// =========================================================================================
bool MC6809CallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                   CallLoweringInfo &Info) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const auto &TLI = *getTLI<MC6809TargetLowering>();
  const auto &DL = MF.getDataLayout();
  const auto &STI = MF.getSubtarget<MC6809Subtarget>();
  const TargetRegisterInfo *TRI = STI.getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  auto CallSeqStart = MIRBuilder.buildInstr(MC6809::ADJCALLSTACKDOWN);

  // Create the call instruction so we can add the implicit uses of arg
  // registers, but don't insert it yet.
  bool IsDirect = !Info.Callee.isReg();
  auto CallOpcode = getCallOpcode(IsDirect);
  auto MIB = MIRBuilder.buildInstrNoInsert(CallOpcode);

  MIB.add(Info.Callee);
  if (!IsDirect) {
    auto CalleeReg = Info.Callee.getReg();
    if (CalleeReg && !Register::isPhysicalRegister(CalleeReg)) {
      MIB->getOperand(0).setReg(constrainOperandRegClass(
          MF, *TRI, MRI, *STI.getInstrInfo(), *STI.getRegBankInfo(),
          *MIB.getInstr(), MIB->getDesc(), Info.Callee, 0));
    }
  }

  MIB.addRegMask(TRI->getCallPreservedMask(MF, Info.CallConv));

  SmallVector<ArgInfo, 8> ArgInfos;
  for (auto Arg : Info.OrigArgs) {
    if (Arg.Flags[0].isByVal())
      return false;

    splitToValueTypes(Arg, ArgInfos, DL, Info.CallConv);
  }

  auto ArgAssignFn = TLI.CCAssignFnForCall(Info.CallConv, Info.IsVarArg);
  OutgoingValueAssigner ArgAssigner(ArgAssignFn);
  MC6809OutgoingValueHandler ArgHandler(MIRBuilder, MRI, MIB);
  if (!determineAndHandleAssignments(ArgHandler, ArgAssigner, ArgInfos,
                                     MIRBuilder, Info.CallConv, Info.IsVarArg))
    return false;

  // Now we can add the actual call instruction to the correct basic block.
  MIRBuilder.insertInstr(MIB);

  if (!Info.OrigRet.Ty->isVoidTy()) {
    ArgInfos.clear();
    splitToValueTypes(Info.OrigRet, ArgInfos, DL, Info.CallConv);
    auto RetAssignFn = TLI.CCAssignFnForReturn(Info.CallConv, Info.IsVarArg);
    OutgoingValueAssigner Assigner(RetAssignFn);
    MC6809CallReturnHandler RetHandler(MIRBuilder, MRI, MIB);
    if (!determineAndHandleAssignments(RetHandler, Assigner, ArgInfos,
                                       MIRBuilder, Info.CallConv,
                                       Info.IsVarArg))
      return false;
  }

  // We now know the size of the stack - update the ADJCALLSTACKDOWN
  // accordingly.
  CallSeqStart.addImm(ArgAssigner.StackOffset).addImm(0);

  MIRBuilder.buildInstr(MC6809::ADJCALLSTACKUP)
      .addImm(ArgAssigner.StackOffset)
      .addImm(-1ULL);

  return true;
}
