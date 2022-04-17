//=== MC6809MachineFunctionInfo.h - MC6809 machine function info -*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares MC6809-specific per-machine-function information.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809MACHINEFUNCTIONINFO_H

#include "llvm/CodeGen/MachineFunction.h"

namespace llvm {

/// MC6809MachineFunctionInfo - This class is derived from MachineFunction and
/// contains private MC6809 target-specific information for each MachineFunction.
class MC6809MachineFunctionInfo : public MachineFunctionInfo {
  virtual void anchor();

  /// CalleeSavedFrameSize - Size of the callee-saved register portion of the
  /// stack frame in bytes.
  unsigned CalleeSavedFrameSize = 0;

  /// ReturnAddrIndex - FrameIndex for return slot.
  int ReturnAddrIndex = 0;

  /// VarArgsFrameIndex - FrameIndex for start of varargs area.
  int VarArgsFrameIndex = 0;

  /// SRetReturnReg - Some subtargets require that sret lowering includes
  /// returning the value of the returned struct in a register. This field
  /// holds the virtual register into which the sret argument is passed.
  Register SRetReturnReg;

  /// The number of bytes to restore to deallocate space for incoming
  /// arguments. Canonically 0 in the C calling convention, but non-zero when
  /// callee is expected to pop the args.
  unsigned ArgumentStackToRestore = 0;

  /// Number of bytes of arguments this function has on the stack. If the callee
  /// is expected to restore the argument stack this should be a multiple of 16,
  /// all usable during a tail call.
  ///
  /// The alternative would forbid tail call optimisation in some cases: if we
  /// want to transfer control from a function with 8-bytes of stack-argument
  /// space to a function with 16-bytes then misalignment of this value would
  /// make a stack adjustment necessary, which could not be undone by the
  /// callee.
  unsigned BytesInStackArgArea = 0;

  /// Space just below incoming stack pointer reserved for arguments being
  /// passed on the stack during a tail call. This will be the difference
  /// between the largest tail call argument space needed in this function and
  /// what's already available by reusing space of incoming arguments.
  unsigned TailCallReservedStack = 0;

public:
  MC6809MachineFunctionInfo() = default;

  explicit MC6809MachineFunctionInfo(MachineFunction &MF)
    : CalleeSavedFrameSize(0), ReturnAddrIndex(0), SRetReturnReg(0) {}

  unsigned getCalleeSavedFrameSize() const { return CalleeSavedFrameSize; }
  void setCalleeSavedFrameSize(unsigned bytes) { CalleeSavedFrameSize = bytes; }

  Register getSRetReturnReg() const { return SRetReturnReg; }
  void setSRetReturnReg(Register Reg) { SRetReturnReg = Reg; }

  int getRAIndex() const { return ReturnAddrIndex; }
  void setRAIndex(int Index) { ReturnAddrIndex = Index; }

  int getVarArgsFrameIndex() const { return VarArgsFrameIndex;}
  void setVarArgsFrameIndex(int Index) { VarArgsFrameIndex = Index; }

  unsigned getArgumentStackToRestore() const { return ArgumentStackToRestore; }
  void setArgumentStackToRestore(unsigned bytes) {
    ArgumentStackToRestore = bytes;
  }

  unsigned getBytesInStackArgArea() const { return BytesInStackArgArea; }
  void setBytesInStackArgArea(unsigned bytes) { BytesInStackArgArea = bytes; }

  unsigned getTailCallReservedStack() const { return TailCallReservedStack; }
  void setTailCallReservedStack(unsigned bytes) {
    TailCallReservedStack = bytes;
  }

};

} // End llvm namespace

#endif
