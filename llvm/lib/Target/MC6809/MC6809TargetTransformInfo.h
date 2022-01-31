//===-- MC6809TargetTransformInfo.h - MC6809 specific TTI -------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file a TargetTransformInfo::Concept conforming object specific to the
/// MC6809 target machine. It uses the target's detailed information to
/// provide more precise answers to certain TTI queries, while letting the
/// target independent and default TTI implementations handle the rest.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_POWERPC_MC6809TARGETTRANSFORMINFO_H
#define LLVM_LIB_TARGET_POWERPC_MC6809TARGETTRANSFORMINFO_H

#include "MC6809TargetMachine.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/CodeGen/BasicTTIImpl.h"
#include "llvm/CodeGen/TargetLowering.h"

namespace llvm {

class MC6809TTIImpl : public BasicTTIImplBase<MC6809TTIImpl> {
  typedef BasicTTIImplBase<MC6809TTIImpl> BaseT;
  typedef TargetTransformInfo TTI;
  friend BaseT;

  const MC6809Subtarget *ST;
  const MC6809TargetLowering *TLI;

  const MC6809Subtarget *getST() const { return ST; }
  const MC6809TargetLowering *getTLI() const { return TLI; }
  bool mightUseCTR(BasicBlock *BB, TargetLibraryInfo *LibInfo,
                   SmallPtrSetImpl<const Value *> &Visited);

public:
  explicit MC6809TTIImpl(const MC6809TargetMachine *TM, const Function &F)
      : BaseT(TM, F.getParent()->getDataLayout()), ST(TM->getSubtargetImpl(F)),
        TLI(ST->getTargetLowering()) {}
};

} // end namespace llvm

#endif
