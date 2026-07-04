//===-- MC6809LateOptimization.cpp - MC6809 Late Optimization -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 late optimization pass.
//
// This pass performs simple optimizations once pseudo-instructions have been
// fully lowered. These optimizations might otherwise increase register pressure
// and cause spills, so they're done opportunistically at the very end.
//
//===----------------------------------------------------------------------===//

#include "MC6809LateOptimization.h"

#include "MC6809.h"
#include "MC6809InstrBuilder.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

#define DEBUG_TYPE "mc6809-late-opt"

using namespace llvm;

namespace {

class MC6809LateOptimization : public MachineFunctionPass {
public:
  static char ID;

  MC6809LateOptimization() : MachineFunctionPass(ID) { llvm::initializeMC6809LateOptimizationPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
  bool tailJMP(MachineBasicBlock &MBB) const;
  bool simplifyAndZero(MachineBasicBlock &MBB) const;
  bool elideCompareZero(MachineBasicBlock &MBB) const;
  bool elideRedundantLoad(MachineBasicBlock &MBB) const;
  bool foldLEAIntoLoad(MachineBasicBlock &MBB) const;
};

bool MC6809LateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= tailJMP(MBB);
    Changed |= simplifyAndZero(MBB);
    Changed |= elideCompareZero(MBB);
    Changed |= elideRedundantLoad(MBB);
    Changed |= foldLEAIntoLoad(MBB);
  }
  return Changed;
}

// Bug #360: drop a redundant `CMPD/CMPW #0` when the immediately preceding
// instruction already produced the compared register and set its N/Z flags
// (e.g. `addd #-1; cmpd #0; bne` from a decrement loop — the ADDD's Z is
// exactly what the compare recomputes).
//
// Flag semantics gate: a compare-with-0 sets Z and N from the value and
// clears V and C. A value-producing op (ADDD/SUBD/LDD/…) sets the same Z/N
// but leaves V/C reflecting the operation, not 0. So the compare is only
// redundant for branches that test Z and/or N alone — EQ/NE (Z) and MI/PL
// (N). Conditions that consult V (signed GE/LT/GT/LE, VS/VC) or C
// (HI/LS/CC/CS) are NOT safe and are left untouched.
bool MC6809LateOptimization::elideCompareZero(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  bool Changed = false;
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &MI = *It++;
    // The redundant flag-setter: a CMPr #0 (immediate compare against zero,
    // ACC or index) or a TSTr (an inherent compare-with-zero of its
    // register). CmpReg is the register whose value the flags reflect.
    Register CmpReg;
    bool IsTst = false;
    switch (MI.getOpcode()) {
    case MC6809::CMPDi16: CmpReg = MC6809::AD; break;
    case MC6809::CMPWi16: CmpReg = MC6809::AW; break;
    case MC6809::CMPXi16: CmpReg = MC6809::IX; break;
    case MC6809::CMPYi16: CmpReg = MC6809::IY; break;
    case MC6809::TSTAa: CmpReg = MC6809::AA; IsTst = true; break;
    case MC6809::TSTBa: CmpReg = MC6809::AB; IsTst = true; break;
    case MC6809::TSTDa: CmpReg = MC6809::AD; IsTst = true; break;
    case MC6809::TSTWa: CmpReg = MC6809::AW; IsTst = true; break;
    case MC6809::TSTEa: CmpReg = MC6809::AE; IsTst = true; break;
    case MC6809::TSTFa: CmpReg = MC6809::AF; IsTst = true; break;
    default: continue;
    }
    if (!IsTst &&
        (!MI.getOperand(0).isImm() || MI.getOperand(0).getImm() != 0))
      continue;
    if (MI.getIterator() == MBB.begin())
      continue;

    auto ModifiesFlags = [&](const MachineInstr &I) {
      return I.modifiesRegister(MC6809::N, &TRI) ||
             I.modifiesRegister(MC6809::Z, &TRI) ||
             I.modifiesRegister(MC6809::V, &TRI);
    };

    // Producer search: walk back over instructions that leave the flags AND
    // CmpReg untouched, stopping at the first instruction that sets the flags
    // — that is the producer whose N/Z the compare would recompute. The skip
    // lets a flag-transparent instruction sit between producer and compare:
    //   addd #-1 ; tfr y,x ; tstd   (the tfr touches neither flags nor AD).
    // A skipped instruction that rewrites CmpReg would leave the producer's
    // flags stale for the value the compare tests, so bail in that case.
    MachineInstr *Producer = nullptr;
    for (auto PIt = MI.getIterator(); PIt != MBB.begin();) {
      MachineInstr &P = *--PIt;
      if (ModifiesFlags(P)) {
        Producer = &P;
        break;
      }
      if (P.modifiesRegister(CmpReg, &TRI))
        break;
    }
    if (!Producer)
      continue;

    // The producer must define EXACTLY the compared register, so the N/Z flags
    // it set reflect that value at the right width. A super-register def is
    // unsafe: `ldd <16-bit>; tstb` defines AD ⊇ AB but LDD's flags are the
    // 16-bit D's, not the 8-bit B's that TSTB tests (D=0x0100 -> D!=0 yet
    // B==0). `definesRegister` would accept that; an exact-width def does not.
    bool ProducerDefsExact = false;
    for (const MachineOperand &MO : Producer->operands())
      if (MO.isReg() && MO.isDef() && MO.getReg() == CmpReg) {
        ProducerDefsExact = true;
        break;
      }
    if (!ProducerDefsExact)
      continue;

    // Consumer search: walk forward over flag-transparent instructions to the
    // first that touches the flags. A flag reader there is the consuming
    // branch (its flags come from the producer once the compare is gone); a
    // flag writer means a different producer feeds the branch, so bail. The
    // skip lets flag-transparent instructions sit between compare and branch:
    //   ldd <rs0> ; tstd ; puls d ; puls d ; beq   (the puls preserve CC).
    // CmpReg being rewritten in this span is fine — the branch reads flags,
    // not CmpReg, and the producer's flags already captured its value.
    auto ConsIt = std::next(MI.getIterator());
    // Stop at the first instruction that touches ANY flag, whether it reads or
    // writes it. Dropping the compare makes the producer's N/Z/V (and, for a
    // CMPr#0, C) reach past this point instead of the compare's, so a skipped
    // instruction may only be one that consults no flag at all — a reader of V
    // or C, not just N/Z, would see the changed value.
    while (ConsIt != MBB.end() && !ModifiesFlags(*ConsIt) &&
           !ConsIt->readsRegister(MC6809::N, &TRI) &&
           !ConsIt->readsRegister(MC6809::Z, &TRI) &&
           !ConsIt->readsRegister(MC6809::V, &TRI) &&
           !ConsIt->readsRegister(MC6809::C, &TRI))
      ++ConsIt;
    if (ConsIt == MBB.end() || ModifiesFlags(*ConsIt))
      continue;
    MachineInstr &Cons = *ConsIt;

    // The consumer must be a conditional branch on an N/Z-only condition
    // (EQ/NE test Z, MI/PL test N — exactly the flags a value op sets the same
    // way a compare-with-0 would; V and C differ, so signed and unsigned
    // conditions are excluded).
    if (Cons.getNumOperands() == 0 || !Cons.getOperand(0).isImm())
      continue;
    int64_t CC = Cons.getOperand(0).getImm();
    if (CC != MC6809CC::EQ && CC != MC6809CC::NE && CC != MC6809CC::MI &&
        CC != MC6809CC::PL)
      continue;
    if (Cons.readsRegister(MC6809::C, &TRI))
      continue;

    // Every N/Z/V flag the branch reads must be (re)defined by the producer,
    // so dropping the compare leaves no flag read undefined. (This is why a
    // Z-only producer such as LEAX cannot feed a branch that also reads N/V.)
    bool ProducerCoversFlags = true;
    bool ReadsNZ = false;
    for (const MachineOperand &MO : Cons.operands()) {
      if (!MO.isReg() || !MO.isUse())
        continue;
      Register R = MO.getReg();
      if (R == MC6809::N || R == MC6809::Z)
        ReadsNZ = true;
      if ((R == MC6809::N || R == MC6809::Z || R == MC6809::V) &&
          !Producer->definesRegister(R, &TRI)) {
        ProducerCoversFlags = false;
        break;
      }
    }
    if (!ProducerCoversFlags || !ReadsNZ)
      continue;

    // The branch will now read the producer's flags; clear any stale dead
    // markers (regalloc may have flagged them when the compare was the only
    // flag consumer).
    for (MachineOperand &MO : Producer->operands())
      if (MO.isReg() && MO.isDef() &&
          (MO.getReg() == MC6809::N || MO.getReg() == MC6809::Z ||
           MO.getReg() == MC6809::NZ || MO.getReg() == MC6809::V))
        MO.setIsDead(false);

    MI.eraseFromParent();
    Changed = true;
  }
  return Changed;
}

// The index-LEA addressing suffix we can fold into a load. Constant (o5/o8/o16)
// carry an explicit (offset, base) operand pair; accumulator (oA/oB/oD and the
// HD6309 oE/oF/oW) carry only the base, with the accumulator as an implicit use.
// PC-relative (o8PC/o16PC) is intentionally absent: a global addressed
// PC-relatively is folded into the load/store at instruction selection now (the
// deferred _Sym pseudo + its post-RA expander), so no `leax sym,pc; ld ,x` pair
// survives to here. Indirect (`…I`), auto inc/dec, and LEAS/LEAU are not
// foldable. Enum values index the LoadTab columns below.
enum class IdxSuf : int {
  o5 = 0, o8, o16, oA, oB, oD, oE, oF, oW, None = -1
};

static IdxSuf leaIdxSuffix(unsigned LeaOpc) {
  switch (LeaOpc) {
  case MC6809::LEAXi_o5:    case MC6809::LEAYi_o5:    return IdxSuf::o5;
  case MC6809::LEAXi_o8:    case MC6809::LEAYi_o8:    return IdxSuf::o8;
  case MC6809::LEAXi_o16:   case MC6809::LEAYi_o16:   return IdxSuf::o16;
  case MC6809::LEAXi_oA:    case MC6809::LEAYi_oA:    return IdxSuf::oA;
  case MC6809::LEAXi_oB:    case MC6809::LEAYi_oB:    return IdxSuf::oB;
  case MC6809::LEAXi_oD:    case MC6809::LEAYi_oD:    return IdxSuf::oD;
  case MC6809::LEAXi_oE:    case MC6809::LEAYi_oE:    return IdxSuf::oE;
  case MC6809::LEAXi_oF:    case MC6809::LEAYi_oF:    return IdxSuf::oF;
  case MC6809::LEAXi_oW:    case MC6809::LEAYi_oW:    return IdxSuf::oW;
  default: return IdxSuf::None;
  }
}

// The value register a zero-offset register-indirect load (`ld ,r`) loads into.
static Register loadO0ValueReg(unsigned LoadOpc) {
  switch (LoadOpc) {
  case MC6809::LDAi_o0: return MC6809::AA;
  case MC6809::LDBi_o0: return MC6809::AB;
  case MC6809::LDDi_o0: return MC6809::AD;
  case MC6809::LDXi_o0: return MC6809::IX;
  case MC6809::LDYi_o0: return MC6809::IY;
  case MC6809::LDUi_o0: return MC6809::SU;
  default: return MC6809::NoRegister;
  }
}

// The load opcode that loads ValueReg using addressing suffix Suf. Rows are the
// value-register families; columns are the IdxSuf order {o5,o8,o16,oA,oB,oD,oE,
// oF,oW}. (HD6309 value families E/F/W are not folded — they fall to 0.)
static unsigned indexedLoadOpcode(Register ValueReg, IdxSuf Suf) {
  if (Suf == IdxSuf::None)
    return 0;
  static const unsigned LoadTab[6][9] = {
    /* AA */ {MC6809::LDAi_o5, MC6809::LDAi_o8, MC6809::LDAi_o16, MC6809::LDAi_oA, MC6809::LDAi_oB, MC6809::LDAi_oD, MC6809::LDAi_oE, MC6809::LDAi_oF, MC6809::LDAi_oW},
    /* AB */ {MC6809::LDBi_o5, MC6809::LDBi_o8, MC6809::LDBi_o16, MC6809::LDBi_oA, MC6809::LDBi_oB, MC6809::LDBi_oD, MC6809::LDBi_oE, MC6809::LDBi_oF, MC6809::LDBi_oW},
    /* AD */ {MC6809::LDDi_o5, MC6809::LDDi_o8, MC6809::LDDi_o16, MC6809::LDDi_oA, MC6809::LDDi_oB, MC6809::LDDi_oD, MC6809::LDDi_oE, MC6809::LDDi_oF, MC6809::LDDi_oW},
    /* IX */ {MC6809::LDXi_o5, MC6809::LDXi_o8, MC6809::LDXi_o16, MC6809::LDXi_oA, MC6809::LDXi_oB, MC6809::LDXi_oD, MC6809::LDXi_oE, MC6809::LDXi_oF, MC6809::LDXi_oW},
    /* IY */ {MC6809::LDYi_o5, MC6809::LDYi_o8, MC6809::LDYi_o16, MC6809::LDYi_oA, MC6809::LDYi_oB, MC6809::LDYi_oD, MC6809::LDYi_oE, MC6809::LDYi_oF, MC6809::LDYi_oW},
    /* SU */ {MC6809::LDUi_o5, MC6809::LDUi_o8, MC6809::LDUi_o16, MC6809::LDUi_oA, MC6809::LDUi_oB, MC6809::LDUi_oD, MC6809::LDUi_oE, MC6809::LDUi_oF, MC6809::LDUi_oW},
  };
  int Row;
  switch (ValueReg) {
  case MC6809::AA: Row = 0; break;
  case MC6809::AB: Row = 1; break;
  case MC6809::AD: Row = 2; break;
  case MC6809::IX: Row = 3; break;
  case MC6809::IY: Row = 4; break;
  case MC6809::SU: Row = 5; break;
  default: return 0;
  }
  return LoadTab[Row][static_cast<int>(Suf)];
}

// Bug #361: fold an address-generating index-LEA into the register-indirect load
// that immediately dereferences it:
//     leax d,x ; ldx ,x   ->   ldx d,x      (dst == base, always safe)
//     leax 4,u ; ldx ,x   ->   ldx 4,u      (base is the LEA's input, not X)
//     leax d,x ; ldy ,x   ->   ldy d,x      (dst != base — only when X dies here)
// The folded load addresses memory the way the LEA did, loading the o0 load's
// value register. Two safety regimes:
//   * dst == base (value reg == index reg == the load's base): the load
// Direct-page load / store opcode tables. Operand 0 is the direct-page slot
// (an imaginary register in the cases we handle); the value the load moves
// lands in the register returned here.
static Register dpLoadValueReg(unsigned Opc) {
  switch (Opc) {
  case MC6809::LDAd: return MC6809::AA;
  case MC6809::LDBd: return MC6809::AB;
  case MC6809::LDDd: return MC6809::AD;
  case MC6809::LDWd: return MC6809::AW;
  case MC6809::LDEd: return MC6809::AE;
  case MC6809::LDFd: return MC6809::AF;
  case MC6809::LDXd: return MC6809::IX;
  case MC6809::LDYd: return MC6809::IY;
  case MC6809::LDUd: return MC6809::SU;
  case MC6809::LDSd: return MC6809::SS;
  case MC6809::LDQd: return MC6809::AQ;
  default: return MC6809::NoRegister;
  }
}

static bool isDpSlotStore(unsigned Opc) {
  switch (Opc) {
  case MC6809::STAd: case MC6809::STBd: case MC6809::STDd:
  case MC6809::STWd: case MC6809::STEd: case MC6809::STFd:
  case MC6809::STXd: case MC6809::STYd: case MC6809::STUd:
  case MC6809::STSd: case MC6809::STQd:
    return true;
  default:
    return false;
  }
}

// Drop a direct-page load whose value register still provably holds the same
// slot's contents from an earlier identical load:
//     ldd <__rs0> ; std <__rs1> ; ldd <__rs0>   ->   ldd <__rs0> ; std <__rs1>
// The store copies D out to a DIFFERENT slot without disturbing D, so the
// reload is pure waste (memmem's outer loop reloads a 16-bit imaginary right
// after storing it elsewhere). Restricted to imaginary-register slots so
// aliasing reduces to a register-identity test.
bool MC6809LateOptimization::elideRedundantLoad(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const MachineRegisterInfo &MRI = MBB.getParent()->getRegInfo();
  if (!MRI.tracksLiveness())
    return false;
  bool Changed = false;

  auto ModifiesFlags = [&](const MachineInstr &I) {
    return I.modifiesRegister(MC6809::N, &TRI) ||
           I.modifiesRegister(MC6809::Z, &TRI) ||
           I.modifiesRegister(MC6809::V, &TRI);
  };
  auto ReadsFlags = [&](const MachineInstr &I) {
    return I.readsRegister(MC6809::N, &TRI) ||
           I.readsRegister(MC6809::Z, &TRI) ||
           I.readsRegister(MC6809::V, &TRI);
  };

  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &MI = *It++;
    Register VReg = dpLoadValueReg(MI.getOpcode());
    if (!VReg)
      continue;
    const MachineOperand &Slot = MI.getOperand(0);
    if (!Slot.isReg() || !Slot.getReg().isPhysical())
      continue;
    Register SlotReg = Slot.getReg();

    // Find an earlier identical load with the slot memory and the value
    // register both untouched in between.
    MachineInstr *Prior = nullptr;
    for (auto PIt = MI.getIterator(); PIt != MBB.begin();) {
      MachineInstr &P = *--PIt;
      if (P.getOpcode() == MI.getOpcode() && P.getOperand(0).isReg() &&
          P.getOperand(0).getReg() == SlotReg) {
        Prior = &P;
        break;
      }
      // Value register clobbered, or the slot's memory written -> the value in
      // VReg is no longer guaranteed to match the slot.
      if (P.modifiesRegister(VReg, &TRI) ||
          P.modifiesRegister(SlotReg, &TRI))
        break;
      // A direct-page slot store to a different slot is safe to walk past:
      // distinct imaginary slots are distinct addresses (a same-slot store was
      // caught by the SlotReg check above) and its only effect is that
      // disjoint memory write. These stores carry hasSideEffects in the .td,
      // so recognise them explicitly rather than lumping them in with the
      // generic store / side-effect barrier below.
      if (isDpSlotStore(P.getOpcode()))
        continue;
      // Any other store could alias the slot's address (an indexed/extended
      // store through an arbitrary pointer); a call or an instruction with
      // unmodeled side effects could touch it too.
      if (P.isCall() || P.mayStore() || P.hasUnmodeledSideEffects())
        break;
    }
    if (!Prior)
      continue;

    // The removed load's N/Z/V must be dead: walk forward to the first flag
    // access; a reader keeps them live (bail), a writer proves them dead. If
    // the walk runs off the end of the block (the redundant load is the last
    // instruction, its consumer in a successor), the flags are dead iff they
    // are not live-out of the block.
    bool FlagsDead = false;
    auto FIt = std::next(MI.getIterator());
    for (; FIt != MBB.end(); ++FIt) {
      if (ReadsFlags(*FIt))
        break;
      if (ModifiesFlags(*FIt)) {
        FlagsDead = true;
        break;
      }
    }
    if (FIt == MBB.end()) {
      LivePhysRegs LiveOut(TRI);
      LiveOut.addLiveOuts(MBB);
      FlagsDead = !LiveOut.contains(MC6809::N) &&
                  !LiveOut.contains(MC6809::Z) &&
                  !LiveOut.contains(MC6809::V) &&
                  !LiveOut.contains(MC6809::NZ);
    }
    if (!FlagsDead)
      continue;

    // Keep VReg live from the prior load to this point: clear the kill flags
    // the intervening instructions placed on it (e.g. the store's killed use),
    // and un-dead the prior load's definition.
    for (auto KIt = std::next(Prior->getIterator()); KIt != MI.getIterator();
         ++KIt)
      KIt->clearRegisterKills(VReg, &TRI);
    Prior->clearRegisterDeads(VReg);

    MI.eraseFromParent();
    Changed = true;
  }
  return Changed;
}

//     overwrites the computed address, so dropping the LEA's advance is
//     unobservable — unconditionally safe.
//   * dst != base (the base survives the load): folding drops the base's new
//     value, so it is valid ONLY if that base register is dead after the load.
//     The counter-example is a walking pointer (`leax -2,x ; ldy ,x` advancing X
//     for the next iteration) — there X is live and must NOT fold.
// Requires adjacency (load immediately follows the LEA): the load redefines
// N/Z/V, so the LEA's dropped Z def is dead and needs no preservation.
bool MC6809LateOptimization::foldLEAIntoLoad(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  bool Changed = false;
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &Lea = *It++;
    IdxSuf Suf = leaIdxSuffix(Lea.getOpcode());
    if (Suf == IdxSuf::None)
      continue;

    // The index register the LEA computes (X or Y) — its single INDEX16 def.
    Register IndexReg;
    for (const MachineOperand &MO : Lea.operands())
      if (MO.isReg() && MO.isDef() &&
          MC6809::INDEX16RegClass.contains(MO.getReg())) {
        IndexReg = MO.getReg();
        break;
      }
    if (!IndexReg)
      continue;

    // Consumer: the immediately-following (non-debug) zero-offset load whose
    // base is the LEA's output (it dereferences the computed address).
    auto NextIt = std::next(Lea.getIterator());
    while (NextIt != MBB.end() && NextIt->isDebugInstr())
      ++NextIt;
    if (NextIt == MBB.end())
      continue;
    MachineInstr &Load = *NextIt;
    Register ValueReg = loadO0ValueReg(Load.getOpcode());
    if (!ValueReg || Load.getNumOperands() == 0 ||
        !Load.getOperand(0).isReg() || Load.getOperand(0).getReg() != IndexReg)
      continue;

    // Safety. dst == base is always safe. dst != base requires the base (the
    // LEA's output) to be dead after the load: a `killed` flag on the load's
    // base use, or a definite dead liveness result.
    if (ValueReg != IndexReg) {
      bool BaseDead = Load.getOperand(0).isKill() ||
                      MBB.computeRegisterLiveness(
                          &TRI, IndexReg,
                          std::next(MachineBasicBlock::iterator(Load))) ==
                          MachineBasicBlock::LQR_Dead;
      if (!BaseDead)
        continue;
    }

    unsigned FoldedOpc = indexedLoadOpcode(ValueReg, Suf);
    if (!FoldedOpc)
      continue;

    // Build the folded load: copy the LEA's explicit operands verbatim — they
    // are exactly the load's addressing operands for the same suffix
    // ((offset imm,) base). The base is the LEA's INPUT base (e.g. U in
    // `leax 4,u`), not the index register. The value-reg def, N/Z/V defs, and
    // any accumulator-offset implicit use come from the new opcode's descriptor.
    MachineInstrBuilder MIB =
        BuildMI(MBB, Load, Load.getDebugLoc(), TII.get(FoldedOpc));
    for (const MachineOperand &MO : Lea.explicit_operands())
      MIB.add(MO);

    Lea.eraseFromParent();
    Load.eraseFromParent();
    It = std::next(MIB->getIterator());
    Changed = true;
  }
  return Changed;
}

// Bug #272 Phase B Scope A followup: convert `ANDA #0` / `ANDB #0` to
// `CLRA` / `CLRB`.  AND with immediate 0 always produces 0 regardless of
// the input, so the implicit USE of the accumulator is unnecessary —
// CLR is strictly better: 1 byte instead of 2, identical NZ/V/C flags,
// and no read of the (possibly undef) accumulator value.  Surfaced as
// a Og verifier hit at test-double-free.c:133 where the codegen emits
// `ANDA #0` to zero the high byte of a returned i16 / i8-zext value;
// post-Scope-A the implicit $aa read was no longer covered by the AQ
// over-claim and the verifier tripped on the undef read.
bool MC6809LateOptimization::simplifyAndZero(MachineBasicBlock &MBB) const {
  bool Changed = false;
  const TargetInstrInfo &TII =
      *MBB.getParent()->getSubtarget().getInstrInfo();
  for (auto It = MBB.begin(); It != MBB.end(); ) {
    auto NextIt = std::next(It);
    unsigned Opc = It->getOpcode();
    unsigned NewOpc = 0;
    if (Opc == MC6809::ANDAi8 && It->getOperand(0).isImm() &&
        It->getOperand(0).getImm() == 0)
      NewOpc = MC6809::CLRAa;
    else if (Opc == MC6809::ANDBi8 && It->getOperand(0).isImm() &&
             It->getOperand(0).getImm() == 0)
      NewOpc = MC6809::CLRBa;
    if (NewOpc != 0) {
      MachineInstr &MI = *It;
      DebugLoc DL = MI.getDebugLoc();
      BuildMI(MBB, MI, DL, TII.get(NewOpc));
      MI.eraseFromParent();
      Changed = true;
    }
    It = NextIt;
  }
  return Changed;
}

bool MC6809LateOptimization::tailJMP(MachineBasicBlock &MBB) const {
  if (MBB.size() < 2)
    return false;
  auto It = std::prev(MBB.end());
  if (It->getOpcode() != MC6809::RTSr)
    return false;
  MachineInstr &RTS = *It;
  --It;
  if (!(It->getOpcode() == MC6809::LBSRlb || It->getOpcode() == MC6809::BSRb))
    return false;
  MachineInstr &JSR = *It;
  RTS.eraseFromParent();
  JSR.setDesc(JSR.getMF()->getSubtarget().getInstrInfo()->get(MC6809::TailJump));
  return true;
}

} // namespace

char MC6809LateOptimization::ID = 0;

INITIALIZE_PASS(MC6809LateOptimization, DEBUG_TYPE, "MC6809 Late Optimizations", false, false)

MachineFunctionPass *llvm::createMC6809LateOptimizationPass() { return new MC6809LateOptimization; }
