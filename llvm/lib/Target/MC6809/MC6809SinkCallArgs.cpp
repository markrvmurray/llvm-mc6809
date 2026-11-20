//===-- MC6809SinkCallArgs.cpp - Compute register arguments last ---------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 pass that moves the computation of a call's
// register arguments down to the argument copies, below the stores of its
// stack arguments.
//
// A call's argument values are usually computed before its frame opens: the
// IR computes them where the source does, and ADJCALLSTACKDOWN bounds the
// scheduler's regions, so nothing crosses it. A value bound for a register
// is then live across the stores of the stack arguments, and those stores
// need a register too. With two accumulators at most (D, and W on the
// HD6309) a byte argument in B and a word going to the stack is already
// more than the 6809 has: memset(p, c, n) copied n through a direct-page
// register, pushing and pulling D on both sides of it.
//
// So, after instruction selection, the chain that computes each register
// argument -- a load of an incoming argument, a truncation, a constant -- is
// moved to just before the first argument copy, after the stack stores. Only
// single-use chains in the call's own block move, and only instructions with
// nothing to order them: no stores, no calls, no side effects, no loads but
// invariant ones, and no physical register they read or keep. Nothing after
// selection turns into a call, so this cannot nest one call's setup in
// another's.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-sink-call-args"

using namespace llvm;

STATISTIC(NumSunk, "Instructions moved down to a call's argument copies");

static cl::opt<bool> EnableSinkCallArgs("mc6809-enable-sink-call-args", cl::init(true), cl::Hidden, cl::desc("Compute a call's register arguments after its stack arguments are stored"));

namespace {

class MC6809SinkCallArgs : public MachineFunctionPass {
public:
  static char ID;
  MC6809SinkCallArgs() : MachineFunctionPass(ID) { llvm::initializeMC6809SinkCallArgsPass(*PassRegistry::getPassRegistry()); }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override { return "MC6809 sink call arguments"; }

private:
  bool sinkForCall(MachineBasicBlock &MBB, MachineInstr &Call);
  bool collect(MachineInstr &MI, const MachineInstr &Setup, SmallPtrSetImpl<MachineInstr *> &Chain);

  const TargetInstrInfo *TII = nullptr;
  MachineRegisterInfo *MRI = nullptr;
};

} // namespace

// Whether MI may move down past the frame setup and the stack stores, and so
// may everything it depends on that is not already in place. Collects the
// instructions to move.
bool MC6809SinkCallArgs::collect(MachineInstr &MI, const MachineInstr &Setup, SmallPtrSetImpl<MachineInstr *> &Chain) {
  if (Chain.contains(&MI))
    return true;
  if (MI.isCall() || MI.isPHI() || MI.isInlineAsm() || MI.isDebugInstr() || MI.hasUnmodeledSideEffects() || MI.mayStore() || TII->isFrameInstr(MI) || MI.isTerminator())
    return false;
  if (MI.mayLoad() && !MI.isDereferenceableInvariantLoad())
    return false;

  // One virtual register out, used once; physical registers only as dead
  // implicit definitions (the flags most instructions clobber).
  Register Def;
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg())
      continue;
    if (MO.getReg().isPhysical()) {
      if (MO.isUse() || !MO.isDead())
        return false;
      continue;
    }
    if (MO.isDef()) {
      if (Def)
        return false;
      Def = MO.getReg();
    }
  }
  if (!Def || !MRI->hasOneNonDBGUse(Def))
    return false;

  Chain.insert(&MI);

  // What it reads from the same block, above the frame, moves with it when
  // it can; otherwise it stays where it is, already available.
  for (const MachineOperand &MO : MI.uses()) {
    if (!MO.isReg() || !MO.getReg().isVirtual())
      continue;
    MachineInstr *DefMI = MRI->getVRegDef(MO.getReg());
    if (!DefMI || DefMI->getParent() != MI.getParent())
      continue;
    SmallPtrSet<MachineInstr *, 8> Sub(Chain.begin(), Chain.end());
    if (collect(*DefMI, Setup, Sub))
      Chain.insert(Sub.begin(), Sub.end());
  }
  return true;
}

bool MC6809SinkCallArgs::sinkForCall(MachineBasicBlock &MBB, MachineInstr &Call) {
  // The frame setup that opens this call's sequence, in the same block.
  MachineInstr *Setup = nullptr;
  for (MachineBasicBlock::reverse_iterator I(Call.getReverseIterator()), E = MBB.rend(); ++I != E;) {
    if (I->getOpcode() == TII->getCallFrameSetupOpcode()) {
      Setup = &*I;
      break;
    }
    if (I->isCall() || I->getOpcode() == TII->getCallFrameDestroyOpcode())
      return false;
  }
  if (!Setup)
    return false;

  // The argument copies, and the first of them: everything moved goes just
  // before it, after the stack stores.
  MachineInstr *InsertBefore = nullptr;
  SmallVector<MachineInstr *, 4> ArgCopies;
  for (MachineInstr &MI : make_range(std::next(MachineBasicBlock::iterator(Setup)), MachineBasicBlock::iterator(Call))) {
    if (!MI.isCopy() || !MI.getOperand(0).getReg().isPhysical() || !MI.getOperand(1).getReg().isVirtual())
      continue;
    if (!InsertBefore)
      InsertBefore = &MI;
    ArgCopies.push_back(&MI);
  }
  if (!InsertBefore)
    return false;

  SmallPtrSet<MachineInstr *, 8> Chain;
  for (MachineInstr *Copy : ArgCopies) {
    MachineInstr *DefMI = MRI->getVRegDef(Copy->getOperand(1).getReg());
    if (!DefMI || DefMI->getParent() != &MBB)
      continue;
    // Only what is above the frame: anything already inside it is where the
    // call lowering put it.
    bool Above = false;
    for (MachineInstr &MI : make_range(MBB.begin(), MachineBasicBlock::iterator(Setup)))
      if (&MI == DefMI) {
        Above = true;
        break;
      }
    if (!Above)
      continue;
    SmallPtrSet<MachineInstr *, 8> Sub(Chain.begin(), Chain.end());
    if (collect(*DefMI, *Setup, Sub))
      Chain.insert(Sub.begin(), Sub.end());
  }
  if (Chain.empty())
    return false;

  // A moved instruction's dead physical definitions must not land on a value
  // something between its old place and the new one still reads.
  for (MachineInstr *MI : Chain) {
    for (const MachineOperand &MO : MI->operands()) {
      if (!MO.isReg() || !MO.isDef() || !MO.getReg().isPhysical())
        continue;
      for (MachineInstr &Between : make_range(std::next(MachineBasicBlock::iterator(MI)), MachineBasicBlock::iterator(InsertBefore)))
        if (!Chain.contains(&Between) && Between.readsRegister(MO.getReg(), MRI->getTargetRegisterInfo()))
          return false;
    }
  }

  // Move them in their original order, with their debug values.
  SmallVector<MachineInstr *, 8> Order;
  for (MachineInstr &MI : make_range(MBB.begin(), MachineBasicBlock::iterator(Setup)))
    if (Chain.contains(&MI))
      Order.push_back(&MI);
  for (MachineInstr *MI : Order) {
    Register Def = MI->getOperand(0).getReg();
    SmallVector<MachineInstr *, 2> Dbg;
    for (MachineInstr &U : MRI->use_instructions(Def))
      if (U.isDebugInstr() && U.getParent() == &MBB)
        Dbg.push_back(&U);
    MBB.splice(InsertBefore->getIterator(), &MBB, MI->getIterator());
    for (MachineInstr *D : Dbg)
      MBB.splice(InsertBefore->getIterator(), &MBB, D->getIterator());
    ++NumSunk;
  }
  return true;
}

bool MC6809SinkCallArgs::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableSinkCallArgs || skipFunction(MF.getFunction()))
    return false;
  TII = MF.getSubtarget().getInstrInfo();
  MRI = &MF.getRegInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    SmallVector<MachineInstr *, 4> Calls;
    for (MachineInstr &MI : MBB)
      if (MI.isCall() && !MI.isReturn())
        Calls.push_back(&MI);
    for (MachineInstr *Call : Calls)
      Changed |= sinkForCall(MBB, *Call);
  }
  return Changed;
}

char MC6809SinkCallArgs::ID = 0;

INITIALIZE_PASS(MC6809SinkCallArgs, DEBUG_TYPE, "MC6809 sink call arguments", false, false)

MachineFunctionPass *llvm::createMC6809SinkCallArgsPass() { return new MC6809SinkCallArgs(); }
