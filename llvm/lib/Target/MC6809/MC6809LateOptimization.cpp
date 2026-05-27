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
};

bool MC6809LateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= tailJMP(MBB);
    Changed |= simplifyAndZero(MBB);
    Changed |= elideCompareZero(MBB);
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

    // Producer immediately before must define EXACTLY the compared register,
    // so the N/Z flags it set reflect that value at the right width. A
    // super-register def is unsafe: `ldd <16-bit>; tstb` defines AD ⊇ AB but
    // LDD's flags are the 16-bit D's, not the 8-bit B's that TSTB tests
    // (D=0x0100 -> D!=0 yet B==0). `definesRegister` would accept that; an
    // exact-width def operand does not.
    MachineInstr &Prev = *std::prev(MI.getIterator());
    bool PrevDefsExact = false;
    for (const MachineOperand &MO : Prev.operands())
      if (MO.isReg() && MO.isDef() && MO.getReg() == CmpReg) {
        PrevDefsExact = true;
        break;
      }
    if (!PrevDefsExact)
      continue;

    // Consumer immediately after must be a conditional branch on an N/Z-only
    // condition (EQ/NE test Z, MI/PL test N — exactly the flags a value op
    // sets the same way a compare-with-0 would; V and C differ, so signed and
    // unsigned conditions are excluded).
    auto NextIt = std::next(MI.getIterator());
    if (NextIt == MBB.end() || NextIt->getNumOperands() == 0 ||
        !NextIt->getOperand(0).isImm())
      continue;
    int64_t CC = NextIt->getOperand(0).getImm();
    if (CC != MC6809CC::EQ && CC != MC6809CC::NE && CC != MC6809CC::MI &&
        CC != MC6809CC::PL)
      continue;
    if (NextIt->readsRegister(MC6809::C, &TRI))
      continue;

    // Every N/Z/V flag the branch reads must be (re)defined by the producer,
    // so dropping the compare leaves no flag read undefined. (This is why a
    // Z-only producer such as LEAX cannot feed a branch that also reads N/V.)
    bool ProducerCoversFlags = true;
    bool ReadsNZ = false;
    for (const MachineOperand &MO : NextIt->operands()) {
      if (!MO.isReg() || !MO.isUse())
        continue;
      Register R = MO.getReg();
      if (R == MC6809::N || R == MC6809::Z)
        ReadsNZ = true;
      if ((R == MC6809::N || R == MC6809::Z || R == MC6809::V) &&
          !Prev.definesRegister(R, &TRI)) {
        ProducerCoversFlags = false;
        break;
      }
    }
    if (!ProducerCoversFlags || !ReadsNZ)
      continue;

    // The branch will now read the producer's flags; clear any stale dead
    // markers (regalloc may have flagged them when the compare was the only
    // flag consumer).
    for (MachineOperand &MO : Prev.operands())
      if (MO.isReg() && MO.isDef() &&
          (MO.getReg() == MC6809::N || MO.getReg() == MC6809::Z ||
           MO.getReg() == MC6809::NZ || MO.getReg() == MC6809::V))
        MO.setIsDead(false);

    MI.eraseFromParent();
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
