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

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/Pass.h"

namespace llvm {

class FunctionPass;
class InstructionSelector;
class PassRegistry;
class MC6809RegisterBankInfo;
class MC6809Subtarget;
class MC6809TargetMachine;

InstructionSelector *createMC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &, MC6809RegisterBankInfo &);

void initializeMC6809BundleCCPass(PassRegistry &);
void initializeMC6809CombinerPass(PassRegistry &);
void initializeMC6809CopyOptPass(PassRegistry &);
void initializeMC6809IncDecPhiPass(PassRegistry &);
void initializeMC6809IndexIVPass(PassRegistry &);
void initializeMC6809InsertCopiesPass(PassRegistry &);
void initializeMC6809InternalizePass(PassRegistry &);
void initializeMC6809LateOptimizationPass(PassRegistry &);
void initializeMC6809LowerSelectPass(PassRegistry &);
void initializeMC6809NonReentrantPass(PassRegistry &);
void initializeMC6809PostRAScavengingPass(PassRegistry &);
void initializeMC6809MaterializeSpillsPass(PassRegistry &);
void initializeMC6809SpillDSaveRestorePass(PassRegistry &);
void initializeMC6809ShiftRotateChainPass(PassRegistry &);
void initializeMC6809DirectPageAllocPass(PassRegistry &);

// Enums corresponding to MC6809 condition codes
namespace MC6809CC {
// The CondCode constants map directly to the 4-bit encoding of the
// condition field for branch instructions.
enum CondCode { // Meaning
  RA,           // 0 Always (unconditional)
  INVALID,
  RN = INVALID, // 1 Never (unconditional)
  HI,           // 2 Unsigned higher
  LS,           // 3 Unsigned lower or same
  CC,           // 4 Carry clear
  HS = CC,      // 4 Unsigned higher or same
  CS,           // 5 Carry set
  LO = CS,      // 5 Unsigned lower
  NE,           // 6 Not equal
  EQ,           // 7 Equal
  VC,           // 8 No overflow
  VS,           // 9 Overflow
  PL,           // 10 Plus, positive or zero
  MI,           // 11 Minus, negative
  GE,           // 12 Greater than or equal
  LT,           // 13 Less than
  GT,           // 14 Greater than
  LE,           // 15 Less than or equal
};

inline static const char *getCCString(CondCode CC) {
  switch (CC) {
  case RA:
    return "ra";
  case RN:
    return "rn";
  case HI:
    return "hi";
  case LS:
    return "ls";
  case HS:
    return "hs";
  case LO:
    return "lo";
  case NE:
    return "ne";
  case EQ:
    return "eq";
  case VC:
    return "vc";
  case VS:
    return "vs";
  case PL:
    return "pl";
  case MI:
    return "mi";
  case GE:
    return "ge";
  case LT:
    return "lt";
  case GT:
    return "gt";
  case LE:
    return "le";
  }
}

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

// The behind-by-one property of the std::reverse_iterator adaptor applied by
// reverse() does not properly handle instruction erasures. This range construct
// converts the forward iterators to native reverse iterators that are not
// behind-by-one and therefore handle erasures correctly when combined with
// make_early_inc_range().
inline auto mbb_reverse(MachineBasicBlock::iterator Begin, MachineBasicBlock::iterator End) { return make_range(MachineBasicBlock::reverse_iterator(End), MachineBasicBlock::reverse_iterator(Begin)); }
template <typename ContainerTy> inline auto mbb_reverse(ContainerTy &&C) { return mbb_reverse(C.begin(), C.end()); }

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809_H
