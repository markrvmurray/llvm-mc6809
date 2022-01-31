//===-- MC6809CallLowering.h - Call lowering -----------------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file describes how to lower LLVM calls to machine code calls.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H

#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"

namespace llvm {

class MC6809TargetLowering;
class MachineInstrBuilder;

// ===========================================================================
// ---------------------------------------------------------------------------
class MC6809CallLowering : public CallLowering {

public:
  MC6809CallLowering(const llvm::TargetLowering *TL) : CallLowering(TL) {}

  bool lowerReturn(MachineIRBuilder &MIRBuilder, const Value *Val,
                   ArrayRef<Register> VRegs, FunctionLoweringInfo &FLI)
                     const override;

  bool lowerFormalArguments(MachineIRBuilder &MIRBuilder, const Function &F,
                            ArrayRef<ArrayRef<Register>> VRegs,
                            FunctionLoweringInfo &FLI) const override;

  bool lowerCall(MachineIRBuilder &MIRBuilder,
                 CallLoweringInfo &Info) const override;
};

// ===========================================================================
// ---------------------------------------------------------------------------
class MC6809IncomingValueAssigner : public CallLowering::IncomingValueAssigner {

public:
  MC6809IncomingValueAssigner(CCAssignFn *AssignFn_)
      : IncomingValueAssigner(AssignFn_) {}

  bool assignArg(unsigned ValNo, EVT OrigVT, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags,
                 CCState &State) override {
    return IncomingValueAssigner::assignArg(ValNo, OrigVT, ValVT, LocVT,
                                            LocInfo, Info, Flags, State);
  }

};

// ---------------------------------------------------------------------------
class MC6809IncomingValueHandler : public CallLowering::IncomingValueHandler {

public:
  MC6809IncomingValueHandler(MachineIRBuilder &MIRBuilder,
                             MachineRegisterInfo &MRI)
      : CallLowering::IncomingValueHandler(MIRBuilder, MRI) {}

private:
  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign VA) override;

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            MachinePointerInfo &MPO, CCValAssign &VA) override;

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override;
};

// ---------------------------------------------------------------------------
class MC6809FormalArgHandler : public MC6809IncomingValueHandler {

public:
  MC6809FormalArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI)
      : MC6809IncomingValueHandler(MIRBuilder, MRI) {}
};

// ===========================================================================
// ---------------------------------------------------------------------------
class MC6809OutgoingValueAssigner : public CallLowering::OutgoingValueAssigner {

public:
  MC6809OutgoingValueAssigner(CCAssignFn *AssignFn_)
      : OutgoingValueAssigner(AssignFn_) {}

  bool assignArg(unsigned ValNo, EVT OrigVT, MVT ValVT, MVT LocVT,
                 CCValAssign::LocInfo LocInfo,
                 const CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags,
                 CCState &State) override {
    bool Res;
    Res = AssignFn(ValNo, ValVT, LocVT, LocInfo, Flags, State);
    StackOffset = State.getNextStackOffset();
    return Res;
  }

};

// ---------------------------------------------------------------------------
class MC6809OutgoingValueHandler : public CallLowering::OutgoingValueHandler {

public:
  MC6809OutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                             MachineRegisterInfo &MRI, MachineInstrBuilder &MIB)
      : CallLowering::OutgoingValueHandler(MIRBuilder, MRI), MIB(MIB) {}

private:
  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign VA) override;

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy,
                            MachinePointerInfo &MPO, CCValAssign &VA) override;

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO,
                           ISD::ArgFlagsTy Flags) override;

  MachineInstrBuilder &MIB;
};

// ---------------------------------------------------------------------------
class MC6809CallReturnHandler : public MC6809OutgoingValueHandler {

public:
  MC6809CallReturnHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                          MachineInstrBuilder &MIB)
      : MC6809OutgoingValueHandler(MIRBuilder, MRI, MIB) {}
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809CALLLOWERING_H
