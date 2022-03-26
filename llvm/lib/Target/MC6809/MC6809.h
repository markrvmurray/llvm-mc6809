//===-- MC6809.h - Top-level interface for MC6809 representation ------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the entry points for global functions defined in the LLVM
// MC6809 back-end.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809_H
#define LLVM_LIB_TARGET_MC6809_MC6809_H

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Pass.h"

namespace llvm {

void initializeMC6809CombinerPass(PassRegistry &);
void initializeMC6809IndexIVPass(PassRegistry &);
void initializeMC6809LateOptimizationPass(PassRegistry &);
void initializeMC6809LowerSelectPass(PassRegistry &);
void initializeMC6809NoRecursePass(PassRegistry &);
void initializeMC6809PostRAScavengingPass(PassRegistry &);
void initializeMC6809StaticStackAllocPass(PassRegistry &);

#if 0
// The behind-by-one property of the std::reverse_iterator adaptor applied by
// reverse() does not properly handle instruction erasures. This range construct
// converts the forward iterators to native reverse iterators that are not
// behind-by-one and therefore handle erasures correctly when combined with
// make_early_inc_range().
inline auto mbb_reverse(MachineBasicBlock::iterator Begin,
                        MachineBasicBlock::iterator End) {
  return make_range(MachineBasicBlock::reverse_iterator(End),
                    MachineBasicBlock::reverse_iterator(Begin));
}
template <typename ContainerTy> inline auto mbb_reverse(ContainerTy &&C) {
  return mbb_reverse(C.begin(), C.end());
}
#endif

// Enums corresponding to MC6809 condition codes
namespace MC6809CC {
// The CondCode constants map directly to the 4-bit encoding of the
// condition field for branch instructions.
enum CondCode { // Meaning
  RA,           // Always (unconditional)
  RN,           // Never (unconditional)
  HI,           // Unsigned higher
  LS,           // Unsigned lower or same
  HS,           // Carry set
  LO,           // Carry clear
  NE,           // Not equal
  EQ,           // Equal
  VC,           // No overflow
  VS,           // Overflow
  PL,           // Plus, positive or zero
  MI,           // Minus, negative
  GE,           // Greater than or equal
  LT,           // Less than
  GT,           // Greater than
  LE,           // Less than or equal
};

inline static CondCode getOppositeCondition(CondCode CC) {
  switch (CC) {
  case EQ:
    return NE;
  case NE:
    return EQ;
  case HS:
    return LO;
  case LO:
    return HS;
  case MI:
    return PL;
  case PL:
    return MI;
  case VS:
    return VC;
  case VC:
    return VS;
  case HI:
    return LS;
  case LS:
    return HI;
  case GE:
    return LT;
  case LT:
    return GE;
  case GT:
    return LE;
  case LE:
    return GT;
  case RA:
    return RN;
  case RN:
    return RA;
  }
}

/// getSwappedCondition - assume the flags are set by MI(a,b), return
/// the condition code if we modify the instructions such that flags are
/// set by MI(b,a).
inline static MC6809CC::CondCode getSwappedCondition(MC6809CC::CondCode CC) {
  switch (CC) {
  default:
    return MC6809CC::RA;
  case MC6809CC::EQ:
    return MC6809CC::EQ;
  case MC6809CC::NE:
    return MC6809CC::NE;
  case MC6809CC::HS:
    return MC6809CC::LS;
  case MC6809CC::LO:
    return MC6809CC::HI;
  case MC6809CC::HI:
    return MC6809CC::LO;
  case MC6809CC::LS:
    return MC6809CC::HS;
  case MC6809CC::GE:
    return MC6809CC::LE;
  case MC6809CC::LT:
    return MC6809CC::GT;
  case MC6809CC::GT:
    return MC6809CC::LT;
  case MC6809CC::LE:
    return MC6809CC::GE;
  }
}
} // end namespace MC6809CC

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809_H
