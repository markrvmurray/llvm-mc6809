//===-- MC6809LegalizerInfo.cpp - MC6809 Legalizer-------------------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the interface that MC6809 uses to legalize generic MIR.
//
// The "vanilla" MC6809 can handle 8- and 16-bit integers naturally.
// Pointers are 16-bit only for now. The direct page will be used in
// the future to give an 8-bit pointer capability similar to the MOS
// target's zero page, except that it can be placed on any 256-byte
// boundary.
//
//===----------------------------------------------------------------------===//

#include "MC6809LegalizerInfo.h"

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"

#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Demangle/Demangle.h"
#include "llvm/IR/GlobalVariable.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/IR/Instructions.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/MathExtras.h"

#define DEBUG_TYPE "mc6809-legaliser"

using namespace llvm;
using namespace TargetOpcode;
using namespace MIPatternMatch;

MC6809LegalizerInfo::MC6809LegalizerInfo(const MC6809Subtarget &STI) {
  using namespace LegalityPredicates;
  using namespace LegalizeMutations;

  LLT S1 = LLT::scalar(1);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT P = LLT::pointer(0, 16);

  // Constants

  if (STI.isHD6309()) {
    getActionDefinitionsBuilder(G_CONSTANT)
      .legalFor({P, S1, S8, S16, S32});
    getActionDefinitionsBuilder(G_IMPLICIT_DEF)
      .legalFor({P, S1, S8, S16, S32});
  } else {
    getActionDefinitionsBuilder(G_CONSTANT)
      .legalFor({P, S1, S8, S16});
    getActionDefinitionsBuilder(G_IMPLICIT_DEF)
      .legalFor({P, S1, S8, S16});
  }

  getActionDefinitionsBuilder({G_FRAME_INDEX, G_GLOBAL_VALUE, G_BLOCK_ADDR})
      .legalFor({P});

  // Integer Extension and Truncation

  if (STI.isHD6309()) {
    getActionDefinitionsBuilder(G_TRUNC)
      .legalFor({{S1, S8}, {S1, S16}, {S1, S32}, {S8, S16}, {S8, S32}, {S16, S32}});
    getActionDefinitionsBuilder(G_ANYEXT)
      .legalFor({{S8, S1}, {S16, S8}, {S32, S16}});
    getActionDefinitionsBuilder(G_SEXT)
      .legalFor({{S8, S1}, {S16, S8}, {S32, S16}});
    getActionDefinitionsBuilder(G_ZEXT)
      .legalFor({{S8, S1}, {S16, S8}, {S32, S16}});
  } else {
    getActionDefinitionsBuilder(G_TRUNC)
      .legalFor({{S1, S8}, {S1, S16}, {S8, S16}});
    getActionDefinitionsBuilder(G_ANYEXT)
      .legalFor({{S8, S1}, {S16, S8}});
    getActionDefinitionsBuilder(G_SEXT)
      .legalFor({{S8, S1}, {S16, S8}});
    getActionDefinitionsBuilder(G_ZEXT)
      .legalFor({{S8, S1}, {S16, S8}});
  }

  getActionDefinitionsBuilder(G_SEXT_INREG)
      .lower();

  // Type Conversions

  getActionDefinitionsBuilder(G_INTTOPTR)
      .clampScalar(1, S16, S16)
      .legalFor({{P, S16}});
  getActionDefinitionsBuilder(G_PTRTOINT)
      .clampScalar(0, S16, S16)
      .legalFor({{S16, P}});

  // Scalar Operations

  getActionDefinitionsBuilder({G_EXTRACT, G_INSERT}).lower();

  getActionDefinitionsBuilder(G_MERGE_VALUES)
      .legalForCartesianProduct({S16, P}, {S8})
      .unsupported();
  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalForCartesianProduct({S8}, {S16, P})
      .unsupported();

  getActionDefinitionsBuilder(G_BSWAP)
      .unsupported();

  getActionDefinitionsBuilder(G_BITREVERSE).lower();

  // Integer Operations

  getActionDefinitionsBuilder({G_ADD, G_SUB, G_AND, G_OR, G_XOR})
      .legalFor({S8, S16})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder(G_MUL)
      .legalFor({S8})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder({G_SDIV, G_SREM, G_UDIV, G_UREM})
      .libcall();

  getActionDefinitionsBuilder({G_SDIVREM, G_UDIVREM})
      .unsupported();

  getActionDefinitionsBuilder({G_SADDSAT, G_UADDSAT, G_SSUBSAT, G_USUBSAT, G_SSHLSAT, G_USHLSAT})
      .lower();

  getActionDefinitionsBuilder({G_SHL, G_LSHR, G_ASHR})
      .legalFor({{S8, S1}})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder({G_ROTL, G_ROTR})
      .legalFor({{S8, S8}})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder(G_ICMP)
      .legalFor({{P, P}, {S8, S8}, {S16, S16}})
      .widenScalarToNextMultipleOf(1, 8);

  getActionDefinitionsBuilder(G_SELECT)
      .legalFor({{P, S1}, {S8, S1}, {S16, S1}})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder(G_PTR_ADD)
      .legalFor({{P, S16}, {P, S8}, {P, S1}});

  getActionDefinitionsBuilder({G_SMIN, G_SMAX, G_UMIN, G_UMAX})
      .lower();

  getActionDefinitionsBuilder(G_ABS)
      .unsupported();

  // Odd operations produce a carry
  // Even operations produce and consume a carry
  getActionDefinitionsBuilder({G_UADDO, G_SADDO, G_UADDE, G_SADDE, G_USUBO, G_SSUBO, G_USUBE, G_SSUBE})
      .legalFor({{S8, S1}, {S16, S1}})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder({G_SMULO, G_UMULO})
      .widenScalarToNextMultipleOf(0, 8)
      .clampScalar(0, S8, S8)
      .lowerIf(typeIs(1, S1));

  getActionDefinitionsBuilder({G_UMULH, G_SMULH})
      .legalFor({S16})
      .lower();

  // WARNING: The default lowering of funnel shifts is terrible. Luckily, they
  // appear to mostly be rotations, which are combined away and handled
  // separately.
  getActionDefinitionsBuilder({G_FSHL, G_FSHR}).lower();

  getActionDefinitionsBuilder(
      {G_CTLZ, G_CTTZ, G_CTPOP, G_CTLZ_ZERO_UNDEF, G_CTTZ_ZERO_UNDEF})
      .lower();

  // Floating Point Operations

  getActionDefinitionsBuilder({G_FADD,       G_FSUB,
                               G_FMUL,       G_FDIV,
                               G_FMA,        G_FPOW,
                               G_FREM,       G_FCOS,
                               G_FSIN,       G_FLOG10,
                               G_FLOG,       G_FLOG2,
                               G_FEXP,       G_FEXP2,
                               G_FCEIL,      G_FFLOOR,
                               G_FMINNUM,    G_FMAXNUM,
                               G_FSQRT,      G_FRINT,
                               G_FNEARBYINT, G_INTRINSIC_ROUNDEVEN,
                               G_FPEXT,      G_FPTRUNC,
                               G_FPTOSI,     G_FPTOUI,
                               G_SITOFP,     G_UITOFP})
      .unsupported();

  // Memory Operations

  getActionDefinitionsBuilder(G_SEXTLOAD)
      .unsupported();

  getActionDefinitionsBuilder(G_ZEXTLOAD)
      .unsupported();

  getActionDefinitionsBuilder({G_LOAD, G_STORE})
      .legalForTypesWithMemDesc({{S8, P, S8, 8},
                                 {S8, P, S16, 8},
                                 {S16, P, S8, 8},
                                 {S16, P, S16, 8},
                                 {P, P, S8, 8},
                                 {P, P, S16, 8}});

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET}).libcall();

  // Control Flow

  getActionDefinitionsBuilder(G_PHI)
      .legalFor({P, S1, S8, S16})
      .widenScalarToNextMultipleOf(0, 8);

  getActionDefinitionsBuilder(G_BRCOND)
      .legalFor({S1});

  getActionDefinitionsBuilder(G_BRINDIRECT)
      .legalFor({P});

  getActionDefinitionsBuilder(G_BRJT)
      .legalFor({{P, S8}, {P, S16}});

  getActionDefinitionsBuilder(G_JUMP_TABLE)
      .legalFor({{P}, {S16}});

  // Variadic Arguments

  getActionDefinitionsBuilder({G_VASTART, G_VAARG})
      .unsupported();

  // Other Operations

  getActionDefinitionsBuilder(G_DYN_STACKALLOC)
      .unsupported();

  getActionDefinitionsBuilder(G_FREEZE)
      .unsupported();

  getLegacyLegalizerInfo().computeTables();
  verify(*STI.getInstrInfo());
}

bool MC6809LegalizerInfo::legalizeIntrinsic(LegalizerHelper &Helper,
                                         MachineInstr &MI) const {
  LLT P = LLT::pointer(0, 16);
  MachineIRBuilder &Builder = Helper.MIRBuilder;
  switch (MI.getIntrinsicID()) {
  default:
    llvm_unreachable("Invalid intrinsic.");
  }
  return false;
}

bool MC6809LegalizerInfo::legalizeCustom(LegalizerHelper &Helper,
                                      MachineInstr &MI) const {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();

  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
  }
}
