//===- MC6809TargetTransformInfo.h - MC6809 specific TTI --------------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines a TargetTransformInfo::Concept conforming object specific
// to the MC6809 target machine. It uses the target's detailed information to
// provide more precise answers to certain TTI queries, while letting the
// target-independent and default TTI implementations handle the rest.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809TARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809TARGETTRANSFORMINFO_H

#include "MC6809TargetMachine.h"
#include "llvm/CodeGen/BasicTTIImpl.h"

namespace llvm {

class MC6809TTIImpl : public BasicTTIImplBase<MC6809TTIImpl> {
  using BaseT = BasicTTIImplBase<MC6809TTIImpl>;

  friend BaseT;

  const MC6809Subtarget *ST;
  const MC6809TargetLowering *TLI;

  const MC6809Subtarget *getST() const { return ST; }
  const MC6809TargetLowering *getTLI() const { return TLI; }

public:
  explicit MC6809TTIImpl(const MC6809TargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}

  // All div, rem, and divrem ops are libcalls, so any possible combination
  // exists.
  bool hasDivRemOp(Type *DataType, bool IsSigned) { return true; }
};

} // end namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809TARGETTRANSFORMINFO_H
