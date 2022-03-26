//===-- MC6809ISelLowering.h - MC6809 DAG Lowering Interface ----------*- C++
//-*-===//
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

#ifndef LLVM_LIB_TARGET_MC6809_MC6809ISELLOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809ISELLOWERING_H

#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/TargetLowering.h"

#include "llvm/Target/TargetMachine.h"

namespace llvm {

class MC6809Subtarget;
class MC6809TargetMachine;

class MC6809TargetLowering : public TargetLowering {
public:
  MC6809TargetLowering(const MC6809TargetMachine &TM,
                       const MC6809Subtarget &STI);

  bool isSelectSupported(SelectSupportKind /*kind*/) const override {
    return false;
  }

  unsigned getNumRegistersForInlineAsm(LLVMContext &Context,
                                       EVT VT) const override;

  ConstraintType getConstraintType(StringRef Constraint) const override;

  std::pair<unsigned, const TargetRegisterClass *>
  getRegForInlineAsmConstraint(const TargetRegisterInfo *TRI,
                               StringRef Constraint, MVT VT) const override;

  bool isTruncateFree(Type *SrcTy, Type *DstTy) const override;

  bool isZExtFree(Type *SrcTy, Type *DstTy) const override;

  MachineBasicBlock *
  EmitInstrWithCustomInserter(MachineInstr &MI,
                              MachineBasicBlock *MBB) const override;

  /// Selects the correct CCAssignFn for a given CallingConvention value.
  CCAssignFn *CCAssignFnForCall(CallingConv::ID CC, bool IsVarArg) const;

  /// Selects the correct CCAssignFn for a given CallingConvention value.
  CCAssignFn *CCAssignFnForReturn(CallingConv::ID CC, bool IsVarArg) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809ISELLOWERING_H
