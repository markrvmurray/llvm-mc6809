//===-- MC6809InsertCopies.cpp - MC6809 Post-ISel Cleanup -----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines a post-instruction-selection cleanup pass: physical
// result registers for the i32 test pseudos, carry branches stripped of the
// value they named only for selection, and byte extracts placed at their
// users.
//
//===----------------------------------------------------------------------===//

#include "MC6809InsertCopies.h"

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-insert-copies"

static llvm::cl::opt<bool>
    PlaceByteExtracts("mc6809-place-byte-extracts", llvm::cl::init(true),
                      llvm::cl::Hidden,
                      llvm::cl::desc("Move each byte extract to just before "
                                     "its user"));

using namespace llvm;

namespace {

class MC6809InsertCopies : public MachineFunctionPass {
public:
  static char ID;

  MC6809InsertCopies() : MachineFunctionPass(ID) { llvm::initializeMC6809InsertCopiesPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809InsertCopies::runOnMachineFunction(MachineFunction &MF) {
  // Do not gate on skipFunction: at -O0 clang stamps optnone on every
  // function, which would skip the physreg pinning below.
  bool Changed = false;

  // i32 test pseudos (SignTest_i32 / EqZero_i32 / EqConst_i32) produce their
  // byte result in $ab while clobbering all of $aq (Defs=[NZ,V,C,AQ] — the
  // expansion's OR-chain / SUBW+SBCD genuinely destroys it). Every byte
  // accumulator is a sub-register of AQ, so a *virtual* ACC8 dst has no
  // register that can coexist with the declared clobber: regalloc sees the
  // dead AQ def interfering with every candidate and fails with "ran out of
  // registers" (surfaced when the byte classes lost their SPILL_* escape
  // registers). Model the hardware truth instead, x86-MUL style: pin the
  // dst operand to physical $ab (the byte the expansion leaves the result
  // in) and copy out to the original vreg. The coalescer collapses the COPY
  // whenever the consumer can take $ab directly.
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      switch (MI.getOpcode()) {
      case MC6809::SignTest_i32:
      case MC6809::EqZero_i32:
      case MC6809::EqConst_i32:
      case MC6809::CompareSet_i8_i32_Imm: {
        MachineOperand &DstMO = MI.getOperand(0);
        if (!DstMO.isReg() || !DstMO.getReg().isVirtual())
          break;
        Register DstVReg = DstMO.getReg();
        DstMO.setReg(MC6809::AB);
        BuildMI(MBB, std::next(MachineBasicBlock::iterator(MI)),
                MI.getDebugLoc(), TII.get(TargetOpcode::COPY), DstVReg)
            .addReg(MC6809::AB);
        Changed = true;
        break;
      }
      default:
        break;
      }
    }

  // A branch on a carry names the producer's carry value as an implicit use
  // only to keep the producer alive through selection (see the selector's
  // phantom-carry branch). Selected, the producer defines C, which the
  // branch reads; drop the virtual use, which would otherwise stretch a
  // live range to the block's end through PHI elimination.
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &Br : MBB.terminators()) {
      if (Br.getOpcode() != MC6809::LBlbc_OnlyC &&
          Br.getOpcode() != MC6809::LBlbc_NoC)
        continue;
      for (unsigned I = Br.getNumOperands(); I-- > Br.getNumExplicitOperands();) {
        const MachineOperand &MO = Br.getOperand(I);
        if (MO.isReg() && MO.isUse() && MO.getReg().isVirtual()) {
          Br.removeOperand(I);
          Changed = true;
        }
      }
    }

  // A byte extract (EXTRACT_LO_i16 / EXTRACT_HI_i16) is A or B only.
  // Selection emits it where the value is taken apart, so every byte of a
  // value is born at once, and three live together are more than A and B
  // can hold: a 32-bit shift by a byte, which takes one value apart and
  // builds another, ran out of registers. Move each extract to just before
  // its only user in the same block (or, for a user in a carry chain, the
  // start of the chain), when nothing in between reads a flag the extract
  // sets or touches D, and that instruction clobbers, without reading,
  // every flag the extract sets.
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &Ext : llvm::make_early_inc_range(MBB)) {
      if (!PlaceByteExtracts)
        break;
      if (Ext.getOpcode() != MC6809::EXTRACT_LO_i16 &&
          Ext.getOpcode() != MC6809::EXTRACT_HI_i16)
        continue;
      Register Def = Ext.getOperand(0).getReg();
      if (!Def.isVirtual() || !MRI.hasOneNonDBGUse(Def))
        continue;
      MachineInstr &Use = *MRI.use_instr_nodbg_begin(Def);
      if (Use.getParent() != &MBB || Use.isPHI())
        continue;
      // A user inside a carry chain (ADCA, SBCB...) takes the byte beside it
      // too, unless another of its operands is also born inside the chain
      // (`0 - x` copies its zero there): then two new bytes and the value
      // the chain carries all want A or B at once, C is live, and nothing
      // can be spilled in between. Go back to where the chain starts, the
      // instruction that sets C without reading it; if that is above the
      // extract, leave the extract where it is.
      MachineInstr *Target = &Use;
      if (Use.readsRegister(MC6809::C, &TRI)) {
        MachineInstr *Start = &Use;
        while (Start && Start->readsRegister(MC6809::C, &TRI)) {
          MachineInstr *Prev = nullptr;
          for (auto I = Start->getIterator(); I != Ext.getIterator();) {
            --I;
            if (I->modifiesRegister(MC6809::C, &TRI)) {
              Prev = &*I;
              break;
            }
          }
          Start = Prev;
        }
        // The window the byte would join: from the chain's start (or the
        // extract itself, when the chain starts above it) to the user.
        MachineInstr *From = Start ? Start : &Ext;
        bool OtherBornInChain = false;
        for (const MachineOperand &MO : Use.uses())
          if (MO.isReg() && MO.getReg().isVirtual() && MO.getReg() != Def)
            if (MachineInstr *D = MRI.getVRegDef(MO.getReg()))
              for (auto I = std::next(From->getIterator());
                   I != Use.getIterator(); ++I)
                if (&*I == D)
                  OtherBornInChain = true;
        if (OtherBornInChain)
          Target = Start;
      }
      if (!Target)
        continue;
      MachineInstr &User = *Target;
      auto IsExtract = [](const MachineInstr &I) {
        return I.getOpcode() == MC6809::EXTRACT_LO_i16 ||
               I.getOpcode() == MC6809::EXTRACT_HI_i16;
      };
      // Already in place: nothing but other extracts (two that feed the
      // same merge) stands between it and its user. Moving it past them
      // would only swap the pair back and forth.
      auto J = std::next(Ext.getIterator());
      while (J != MBB.end() && J != User.getIterator() && IsExtract(*J))
        ++J;
      if (J == User.getIterator())
        continue;
      SmallVector<Register, 4> Flags;
      for (const MachineOperand &MO : Ext.implicit_operands())
        if (MO.isReg() && MO.isDef())
          Flags.push_back(MO.getReg());
      bool Safe = llvm::all_of(Flags, [&](Register F) {
        return User.definesRegister(F, &TRI) && !User.readsRegister(F, &TRI);
      });
      for (auto I = std::next(Ext.getIterator());
           Safe && I != User.getIterator(); ++I) {
        for (Register F : Flags)
          if (I->readsRegister(F, &TRI))
            Safe = false;
        for (const MachineOperand &MO : I->operands())
          if (MO.isReg() && MO.getReg().isPhysical() &&
              TRI.regsOverlap(MO.getReg(), MC6809::AD))
            Safe = false;
      }
      if (!Safe)
        continue;
      Ext.moveBefore(&User);
      Changed = true;
    }

  return Changed;
}

} // namespace

char MC6809InsertCopies::ID = 0;

INITIALIZE_PASS(MC6809InsertCopies, DEBUG_TYPE, "MC6809 Copy Insertion", false, false)

MachineFunctionPass *llvm::createMC6809InsertCopiesPass() { return new MC6809InsertCopies; }
