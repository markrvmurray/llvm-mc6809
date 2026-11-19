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

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/SmallSet.h"
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
} // namespace

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
  // Selection emits it where the value is taken apart, so the bytes of
  // several values can be born together, and three live at once are more
  // than A and B can hold. Move the extracts of each value -- both halves
  // together, so that a value whose halves are used side by side stays in
  // D -- to just before the first of their users, when each has one user
  // in the same block, nothing in between reads a flag an extract sets or
  // touches D, and that first user clobbers, without reading, every flag
  // the extracts set.
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  auto IsExtract = [](const MachineInstr &I) {
    return I.getOpcode() == MC6809::EXTRACT_LO_i16 ||
           I.getOpcode() == MC6809::EXTRACT_HI_i16;
  };
  for (MachineBasicBlock &MBB : MF) {
    if (!PlaceByteExtracts)
      break;
    // Number the block once: the moves below only reorder extracts, which
    // never serve as anyone's first user.
    DenseMap<const MachineInstr *, unsigned> Pos;
    unsigned N = 0;
    for (MachineInstr &I : MBB)
      Pos[&I] = N++;
    SmallSet<Register, 8> Done;
    // Collect first: moving a pair down while walking the block would step
    // over the extracts it is moved past.
    SmallVector<MachineInstr *, 16> Extracts;
    for (MachineInstr &I : MBB)
      if (IsExtract(I))
        Extracts.push_back(&I);
    for (MachineInstr *ExtP : Extracts) {
      MachineInstr &Ext = *ExtP;
      Register Src = Ext.getOperand(1).getReg();
      if (!Src.isVirtual() || !Done.insert(Src).second)
        continue;
      // The value's extracts in this block, each with one user here.
      SmallVector<MachineInstr *, 2> Group;
      MachineInstr *First = nullptr;
      bool Ok = true;
      for (MachineInstr &U : MRI.use_nodbg_instructions(Src)) {
        if (!IsExtract(U) || U.getParent() != &MBB)
          continue;
        Register Def = U.getOperand(0).getReg();
        if (!Def.isVirtual() || !MRI.hasOneNonDBGUse(Def)) {
          Ok = false;
          break;
        }
        MachineInstr &User = *MRI.use_instr_nodbg_begin(Def);
        if (User.getParent() != &MBB || User.isPHI() || IsExtract(User) ||
            Pos[&User] < Pos[&U]) {
          Ok = false;
          break;
        }
        Group.push_back(&U);
        if (!First || Pos[&User] < Pos[First])
          First = &User;
      }
      if (!Ok || Group.empty() ||
          llvm::any_of(Group, [&](const MachineInstr *E) {
            return Pos[E] > Pos[First]; // only ever move an extract down
          }))
        continue;
      llvm::sort(Group, [&](const MachineInstr *L, const MachineInstr *R) {
        return Pos[L] < Pos[R];
      });
      // Already in place: nothing but extracts between them and the user.
      auto J = std::next(Group.back()->getIterator());
      while (J != First->getIterator() && IsExtract(*J))
        ++J;
      if (J == First->getIterator())
        continue;
      bool Safe = true;
      for (MachineInstr *E : Group) {
        for (const MachineOperand &MO : E->implicit_operands())
          if (MO.isReg() && MO.isDef() &&
              (!First->definesRegister(MO.getReg(), &TRI) ||
               First->readsRegister(MO.getReg(), &TRI)))
            Safe = false;
        for (auto I = std::next(E->getIterator());
             Safe && I != First->getIterator(); ++I) {
          if (IsExtract(*I))
            continue;
          for (const MachineOperand &MO : E->implicit_operands())
            if (MO.isReg() && MO.isDef() && I->readsRegister(MO.getReg(), &TRI))
              Safe = false;
          // Nothing that needs A or B for itself: a call (which clobbers
          // them, and would stretch the D-held source across it), a
          // physical register overlapping D, or a value that can only live
          // in D or Q (the 6309's Q holds A and B too).
          if (I->isCall())
            Safe = false;
          for (const MachineOperand &MO : I->operands()) {
            if (!MO.isReg() || !MO.getReg())
              continue;
            Register R = MO.getReg();
            if (R.isPhysical()) {
              if (TRI.regsOverlap(R, MC6809::AD))
                Safe = false;
            } else if (const TargetRegisterClass *RC =
                           MRI.getRegClassOrNull(R)) {
              // A value that, wherever it is put, holds both A and B (D,
              // or the 6309's Q). A single byte in A or B is ordinary
              // competition, which the allocator can settle.
              if (llvm::all_of(*RC, [&](MCPhysReg P) {
                    return TRI.regsOverlap(P, MC6809::AA) &&
                           TRI.regsOverlap(P, MC6809::AB);
                  }))
                Safe = false;
            }
          }
        }
      }
      if (!Safe)
        continue;
      for (MachineInstr *E : Group)
        E->moveBefore(First);
      Changed = true;
    }
  }

  return Changed;
}


char MC6809InsertCopies::ID = 0;

INITIALIZE_PASS(MC6809InsertCopies, DEBUG_TYPE, "MC6809 Copy Insertion", false, false)

MachineFunctionPass *llvm::createMC6809InsertCopiesPass() { return new MC6809InsertCopies; }
