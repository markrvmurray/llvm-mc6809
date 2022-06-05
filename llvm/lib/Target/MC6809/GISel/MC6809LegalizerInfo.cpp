//===-- MC6809LegalizerInfo.cpp - MC6809 Legalizer ------------------------===//
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

#include "MC6809Subtarget.h"

#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
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
    getActionDefinitionsBuilder(G_CONSTANT).legalFor({P, S1, S8, S16, S32});
    getActionDefinitionsBuilder(G_IMPLICIT_DEF).legalFor({P, S1, S8, S16, S32});
  } else {
    getActionDefinitionsBuilder(G_CONSTANT).legalFor({P, S1, S8, S16}).maxScalar(0, S16).unsupported();
    getActionDefinitionsBuilder(G_IMPLICIT_DEF).legalFor({P, S1, S8, S16}).maxScalar(0, S16).unsupported();
  }

  getActionDefinitionsBuilder({G_FRAME_INDEX, G_GLOBAL_VALUE, G_BLOCK_ADDR})
      .legalFor({P})
      .unsupported();

  // Integer Extension and Truncation

  if (STI.isHD6309()) {
    getActionDefinitionsBuilder(G_TRUNC).legalFor(
        {{S1, S8}, {S1, S16}, {S1, S32}, {S8, S16}, {S8, S32}, {S16, S32}});
    getActionDefinitionsBuilder(G_ANYEXT).legalFor(
        {{S8, S1}, {S16, S8}, {S32, S16}});
    getActionDefinitionsBuilder(G_SEXT).legalFor(
        {{S8, S1}, {S16, S8}, {S32, S16}});
    getActionDefinitionsBuilder(G_ZEXT).legalFor(
        {{S8, S1}, {S16, S8}, {S32, S16}});
  } else {
    getActionDefinitionsBuilder(G_TRUNC).legalFor(
        {{S1, S8}, {S1, S16}, {S8, S16}});
    getActionDefinitionsBuilder(G_ANYEXT).legalFor({{S8, S1}, {S16, S8}});
    getActionDefinitionsBuilder(G_SEXT).legalFor({{S8, S1}, {S16, S8}});
    getActionDefinitionsBuilder(G_ZEXT).legalFor({{S8, S1}, {S16, S8}});
  }

  getActionDefinitionsBuilder(G_SEXT_INREG).lower();

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
      .legalFor({{S32, S16}, {S16, S8}, {P, S8}})
      .unsupported();
  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalFor({{S8, S16}, {S8, P}, {S16, S32}})
      .unsupported();

  getActionDefinitionsBuilder(G_BSWAP).unsupported();

  getActionDefinitionsBuilder(G_BITREVERSE).lower();

  // Integer Operations
  if (STI.isHD6309()) {
    getActionDefinitionsBuilder({G_ADD, G_SUB})
        .customFor({S32})
        .legalFor({S8, S16})
        .clampScalar(0, S8, S32);
  } else {
    getActionDefinitionsBuilder({G_ADD, G_SUB})
        // .lowerFor({S32})
        .legalFor({S8, S16})
        .clampScalar(0, S8, S16);
  }

  getActionDefinitionsBuilder({G_AND, G_OR, G_XOR})
      .legalFor({S8})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder(G_MUL)
      .legalFor({S8})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder({G_SDIV, G_SREM, G_UDIV, G_UREM}).libcall();

  getActionDefinitionsBuilder({G_SDIVREM, G_UDIVREM}).unsupported();

  getActionDefinitionsBuilder(
      {G_SADDSAT, G_UADDSAT, G_SSUBSAT, G_USUBSAT, G_SSHLSAT, G_USHLSAT})
      .lower();

  getActionDefinitionsBuilder({G_SHL, G_LSHR, G_ASHR})
      .legalFor({{S8, S1}})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder({G_ROTL, G_ROTR})
      .legalFor({{S8, S8}})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder(G_ICMP)
      .legalForCartesianProduct({S1}, {S8, S16, P})
      .minScalar(1, S8);

  getActionDefinitionsBuilder(G_SELECT)
      .legalFor({{P, S1}, {S8, S1}, {S16, S1}})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder(G_PTR_ADD)
      .legalFor({{P, S16}, {P, S8}, {P, S1}})
      .unsupported();

  getActionDefinitionsBuilder({G_SMIN, G_SMAX, G_UMIN, G_UMAX}).lower();

  getActionDefinitionsBuilder(G_ABS).unsupported();

  // Odd operations produce a carry
  // Even operations produce and consume a carry
  if (STI.isHD6309()) {
    getActionDefinitionsBuilder({G_UADDO, G_SADDO, G_USUBO, G_SSUBO, G_UADDE, G_SADDE, G_USUBE, G_SSUBE})
        .legalFor({{S8, S1}, {S16, S1}})
        .widenScalarToNextPow2(0);
  } else {
    getActionDefinitionsBuilder({G_UADDO, G_SADDO, G_USUBO, G_SSUBO})
        .legalFor({{S8, S1}, {S16, S1}})
        .widenScalarToNextPow2(0);
    getActionDefinitionsBuilder({G_UADDE, G_SADDE, G_USUBE, G_SSUBE})
        .legalFor({{S8, S1}, {S16, S1}})
        .widenScalarToNextPow2(0);
  }

  getActionDefinitionsBuilder({G_SMULO, G_UMULO})
      .widenScalarToNextPow2(0)
      .clampScalar(0, S8, S8)
      .lowerIf(typeIs(1, S1));

  getActionDefinitionsBuilder({G_UMULH, G_SMULH}).legalFor({S16}).lower();

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

  getActionDefinitionsBuilder({G_SEXTLOAD, G_ZEXTLOAD})
      .unsupported();

  if (STI.isHD6309()) {
    getActionDefinitionsBuilder({G_LOAD, G_STORE})
        .customFor({{S8, P}, {S16, P}, {S32, P}, {P, P}})
        .widenScalarToNextPow2(0);
  } else {
    getActionDefinitionsBuilder({G_LOAD, G_STORE})
        .customFor({{S8, P}, {S16, P}, {P, P}})
        .widenScalarToNextPow2(0);
  }

  getActionDefinitionsBuilder({G_MEMCPY, G_MEMMOVE, G_MEMSET}).libcall();

  // Control Flow

  getActionDefinitionsBuilder(G_PHI)
      .legalFor({P, S1, S8, S16})
      .widenScalarToNextPow2(0);

  getActionDefinitionsBuilder(G_BRCOND).legalFor({S1});

  getActionDefinitionsBuilder(G_BRINDIRECT).legalFor({P});

  getActionDefinitionsBuilder(G_BRJT).legalFor({{P, S8}, {P, S16}});

  getActionDefinitionsBuilder(G_JUMP_TABLE).legalFor({{P}, {S16}});

  // Variadic Arguments

  getActionDefinitionsBuilder({G_VASTART, G_VAARG}).unsupported();

  // Other Operations

  getActionDefinitionsBuilder(G_DYN_STACKALLOC).unsupported();

  getActionDefinitionsBuilder(G_FREEZE).unsupported();

  getLegacyLegalizerInfo().computeTables();
  verify(*STI.getInstrInfo());
}

bool MC6809LegalizerInfo::legalizeIntrinsic(LegalizerHelper &Helper, MachineInstr &MI) const {
  switch (MI.getIntrinsicID()) {
  default:
    llvm_unreachable("Invalid intrinsic.");
  }
}

bool MC6809LegalizerInfo::legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI) const {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Invalid opcode for custom legalization.");
  case G_LOAD:
  case G_STORE:
    return legalizeLoadStore(Helper, MRI, MI);
#if 0
  case G_PTR_ADD:
    return legalizePtrAdd(Helper, MRI, MI);
#endif /* 0 */
  // Integer Operations
  case G_ADD:
  case G_SUB:
    return legalizeAddSub(Helper, MRI, MI);
  }
}

static bool willBeStaticallyAllocated(const MachineOperand &MO) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MO = "; MO.dump(););
  assert(MO.isFI());
  if (!MO.getParent()->getMF()->getFunction().doesNotRecurse())
    return false;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : past .doesNotRecurse()\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO = "; MO.dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getIndex() = " << MO.getIndex() << "\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getParent() = "; MO.getParent()->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getParent()->getMF() = "; MO.getParent()->getMF()->dump(););
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getParent()->getMF()->getFrameInfo() = " <<  MO.getParent()->getMF()->getFrameInfo(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getParent()->getMF()->getFrameInfo().isFixedObjectIndex(MO.getIndex()) = "
                    << MO.getParent()->getMF()->getFrameInfo().isFixedObjectIndex(MO.getIndex()) << "\n";);
  return !MO.getParent()->getMF()->getFrameInfo().isFixedObjectIndex(MO.getIndex());
}

bool MC6809LegalizerInfo::tryAbsoluteAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Addr = MI.getOperand(1).getReg();
  int64_t Offset = 0;

  unsigned Opcode = IsLoad ? MC6809::G_LOAD_ABS : MC6809::G_STORE_ABS;

  for (;;) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Top of loop\n";);
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Addr, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : Constant\n";);
      Helper.Observer.changingInstr(MI);
      MI.setDesc(Builder.getTII().get(Opcode));
      MI.getOperand(1).ChangeToImmediate(Offset + ConstAddr->Value.getSExtValue());
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Constant : MI = "; MI.dump(););
      Helper.Observer.changedInstr(MI);
      return true;
    }
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Addr, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_GLOBAL_VALUE\n";);
      Helper.Observer.changingInstr(MI);
      MI.setDesc(Builder.getTII().get(Opcode));
      const MachineOperand &GV = GVAddr->getOperand(1);
      MI.getOperand(1).ChangeToGA(GV.getGlobal(), GV.getOffset() + Offset);
      Helper.Observer.changedInstr(MI);
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : G_GLOBAL_VALUE : MI = "; MI.dump(););
      return true;
    }
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Addr, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_FRAME_INDEX\n";);
      const MachineOperand &FI = FIAddr->getOperand(1);
      if (willBeStaticallyAllocated(FI)) {
        Helper.Observer.changingInstr(MI);
        MI.setDesc(Builder.getTII().get(Opcode));
        MI.getOperand(1).ChangeToFrameIndex(FI.getIndex(), FI.getOffset() + Offset);
        Helper.Observer.changedInstr(MI);
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : G_FRAME_INDEX : MI = "; MI.dump(););
        return true;
      }
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : NOT G_FRAME_INDEX\n";);
    }
    if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Addr, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_PTR_ADD\n";);
      Register Base = PtrAddAddr->getOperand(1).getReg();
      Register NewOffset = PtrAddAddr->getOperand(2).getReg();
      auto ConstOffset = getIConstantVRegValWithLookThrough(NewOffset, MRI);
      if (!ConstOffset)
        return false;
      Offset += ConstOffset->Value.getSExtValue();
      Addr = Base;
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : Continuing : G_PTR_ADD\n";);
      continue;
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bottom of loop, about to return (false)\n";);
    return false;
  }
}

bool MC6809LegalizerInfo::tryIndexedAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Index = MI.getOperand(1).getReg();
  int64_t ConstantOffset = 0;
  Register Offset = 0;

  unsigned Opcode = IsLoad ? MC6809::G_LOAD_INDEX_OFFSET : MC6809::G_STORE_INDEX_OFFSET;

  for (;;) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : Constant?\n";);
    if (auto ConstAddr = getIConstantVRegValWithLookThrough(Index, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Constant : MI = "; MI.dump(););
      assert(Offset); // Otherwise, Absolute addressing would have been selected.
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addUse(Offset)
          .addUse(Index)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Constant : MI = "; MI.dump(););
      return true;
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_GLOBAL_VALUE?\n";);
    if (const MachineInstr *GVAddr = getOpcodeDef(G_GLOBAL_VALUE, Index, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : G_GLOBAL_VALUE : MI = "; MI.dump(););
      assert(Offset); // Otherwise, Absolute addressing would have been selected.
      const MachineOperand &GV = GVAddr->getOperand(1);
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addUse(GV.getIndex())
          .addImm(GV.getOffset() + ConstantOffset)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : G_GLOBAL_VALUE : MI = "; MI.dump(););
      return true;
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_FRAME_INDEX?\n";);
    if (const MachineInstr *FIAddr = getOpcodeDef(G_FRAME_INDEX, Index, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : G_FRAME_INDEX : MI = "; MI.dump(););
      const MachineOperand &FI = FIAddr->getOperand(1);
      //      if (willBeStaticallyAllocated(FI)) {
      //        assert(Offset); // Otherwise, Absolute addressing would have been selected.
      Builder.buildInstr(Opcode)
          .add(MI.getOperand(0))
          .addFrameIndex(FI.getIndex(), FI.getOffset() + ConstantOffset)
          .addImm(0)
          .addMemOperand(*MI.memoperands_begin());
      MI.eraseFromParent();
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : G_FRAME_INDEX : MI = "; MI.dump(););
      return true;
      //      }
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_PTR_ADD?\n";);
    if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Index, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : G_PTR_ADD : MI = "; MI.dump(););
      Register Base = PtrAddAddr->getOperand(1).getReg();
      Register NewOffset = PtrAddAddr->getOperand(2).getReg();
      if (auto ConstOffset = getIConstantVRegValWithLookThrough(NewOffset, MRI)) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_PTR_ADD : ConstantOffset\n";);
        ConstantOffset += ConstOffset->Value.getSExtValue();
        Index = Base;
        continue;
      }
      if (MachineInstr *ZExtOffset = getOpcodeDef(G_ZEXT, NewOffset, MRI)) {
        LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Loop : G_PTR_ADD : G_ZEXT\n";);
        if (Offset)
          return false;
        Register Src = ZExtOffset->getOperand(1).getReg();
        LLT SrcTy = MRI.getType(Src);
        if (SrcTy.getSizeInBits() > 8)
          return false;
        if (SrcTy.getSizeInBits() < 8)
          Src = Builder.buildZExt(LLT::scalar(8), Src).getReg(0);
        assert(MRI.getType(Src) == LLT::scalar(8));
        Offset = Src;
        Index = Base;
        continue;
      }
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : End : G_PTR_ADD : MI = "; MI.dump(););
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : End loop : return false\n";);
    return false;
  }
}

bool MC6809LegalizerInfo::tryIndirectIndexedAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  bool IsLoad = MI.getOpcode() == G_LOAD;
  assert(IsLoad || MI.getOpcode() == G_STORE);

  Register Index = MI.getOperand(1).getReg();
  Register Offset = 0;

  unsigned Opcode = IsLoad ? MC6809::G_LOAD_INDIR_INDEX_OFFSET : MC6809::G_STORE_INDIR_INDEX_OFFSET;

  if (const MachineInstr *PtrAddAddr = getOpcodeDef(G_PTR_ADD, Index, MRI)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PtrAdd\n";);
    Register NewIndex = PtrAddAddr->getOperand(1).getReg();
    Register NewOffset = PtrAddAddr->getOperand(2).getReg();
    if (auto ConstOffset = getIConstantVRegValWithLookThrough(NewOffset, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PtrAdd with constant offset\n";);
      if (ConstOffset->Value.getActiveBits() <= 16) {
        Offset = Builder.buildConstant(LLT::scalar(16), ConstOffset->Value.getSExtValue())
                     .getReg(0);
        Index = NewIndex;
      } else
        llvm_unreachable("Invalid indirect index offset size");
    } else if (MachineInstr *ZExtOffset = getOpcodeDef(G_ZEXT, NewOffset, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PtrAdd with zext register offset\n";);
      Offset = ZExtOffset->getOperand(1).getReg();
      Index = NewIndex;
    } else if (MachineInstr *SExtOffset = getOpcodeDef(G_SEXT, NewOffset, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PtrAdd with sext register offset\n";);
      Offset = SExtOffset->getOperand(1).getReg();
      Index = NewIndex;
    } else if (MachineInstr *AnyExtOffset = getOpcodeDef(G_ANYEXT, NewOffset, MRI)) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PtrAdd with anyext register offset\n";);
      Offset = AnyExtOffset->getOperand(1).getReg();
      Index = NewIndex;
    }
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Building instruction\n";);
  if (!Offset)
    Offset = Builder.buildConstant(LLT::scalar(8), 0).getReg(0);
  Builder.buildInstr(Opcode)
      .add(MI.getOperand(0))
      .addUse(Index)
      .addUse(Offset)
      .addMemOperand(*MI.memoperands_begin());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return (true)\n";);
  return true;
}

bool MC6809LegalizerInfo::selectNoAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  return true;
}

bool MC6809LegalizerInfo::selectAddressingMode(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  if (tryAbsoluteAddressing(Helper, MRI, MI))
    return true;
  if (tryIndexedAddressing(Helper, MRI, MI))
    return true;
#if 0
  if (tryIndirectIndexedAddressing(Helper, MRI, MI))
    return true;
#endif /* 0 */
  return selectNoAddressing(Helper, MRI, MI);
}

bool MC6809LegalizerInfo::legalizeLoadStore(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  return selectAddressingMode(Helper, MRI, MI);
}

bool MC6809LegalizerInfo::legalizeAddSub(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  const MC6809Subtarget &STI = static_cast<const MC6809Subtarget &>(MI.getMF()->getSubtarget());

  auto &Builder = Helper.MIRBuilder;
  LLT NarrowTy;
  if (STI.isHD6309())
    NarrowTy = LLT::scalar(16);
  else
    NarrowTy = LLT::scalar(8);

  bool Success = Helper.narrowScalarAddSub(MI, 0, NarrowTy) != LegalizerHelper::UnableToLegalize;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Success = " << Success << " : MI = "; MI.dump(););
  return Success;
}

#if 0
bool MC6809LegalizerInfo::legalizePtrAdd(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const {
  MachineIRBuilder &Builder = Helper.MIRBuilder;

  Register Result = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  Register Offset = MI.getOperand(2).getReg();

  MachineInstr *GlobalBase = getOpcodeDef(G_GLOBAL_VALUE, Base, MRI);
  auto ConstOffset = getIConstantVRegValWithLookThrough(Offset, MRI);

  // Fold constant offsets into global value operand.
  if (GlobalBase && ConstOffset) {
    const MachineOperand &Op = GlobalBase->getOperand(1);
    Builder.buildInstr(G_GLOBAL_VALUE)
        .addDef(Result)
        .addGlobalAddress(Op.getGlobal(), Op.getOffset() + ConstOffset->Value.getSExtValue());
    MI.eraseFromParent();
  }
  return true;
}
#endif /* 0 */