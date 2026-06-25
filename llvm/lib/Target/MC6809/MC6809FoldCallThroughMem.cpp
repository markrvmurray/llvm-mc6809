//===-- MC6809FoldCallThroughMem.cpp - Fuse fn-ptr load into call --------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Fold a function-pointer load into the indirect call that consumes it, using
// MC6809's indirect-through-memory addressing (`jsr [n,r]`).
//
// An indirect call lowers to JSRi_o0 with the target forced into an index
// register. But the call clobbers the index registers, so a function pointer
// kept in memory (a function-pointer argument or local) is reloaded into an
// index register before *every* call site and never stays in a register across
// one. qsort is the archetype: its comparator argument is reloaded and called
// once per comparison —
//
//     %p = Load_iPtr_Mem %fixed-stack.0, 0   (invariant)
//     JSRi_o0 %p ; JSRi_o0 %p ; ...          (many, each reloads %p)
//
// MC6809 can read the pointer from memory as part of the call, so the register
// load is pure overhead. This pass, running on selected SSA MIR before register
// allocation, replaces each such call with the deferred JSR_iPtr_Mem pseudo
// carrying the load's addressing, then deletes the now-dead load. The pointer
// is never materialised into a register, so the allocator has one more index
// register free across the whole call-heavy region. The pseudo is expanded
// after PEI (offset resolved) to the concrete JSRi_o0I/o8I/o16I.
//
// Scope and safety:
//   * The load must be `invariant` — re-reading the pointer at each call (rather
//     than once) is only valid if the memory never changes.
//   * Every use of the loaded pointer must be an indirect call through it; if it
//     is also stored, compared, etc., the load stays.
//   * The address must be a frame index. That is the dominant case (pointer
//     args/locals live on the stack) and avoids extending a base *register*'s
//     live range to the call sites. A register base (e.g. obj->fn()) is a
//     possible follow-up.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-fold-call-mem"

using namespace llvm;

static cl::opt<bool> EnableFoldCallThroughMem(
    "mc6809-fold-call-mem", cl::init(true), cl::Hidden,
    cl::desc("Fold an invariant function-pointer load into the indirect call "
             "that uses it (jsr [n,r]), freeing the index register"));

namespace {

class MC6809FoldCallThroughMem : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldCallThroughMem() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FoldCallThroughMemPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold indirect call through memory";
  }
};

} // namespace

bool MC6809FoldCallThroughMem::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldCallThroughMem)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &TII = *MF.getSubtarget().getInstrInfo();

  // Collect foldable loads first; the rewrite erases the load and its calls
  // (which may live in other blocks), so we must not mutate while iterating.
  SmallVector<MachineInstr *, 8> Loads;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (MI.getOpcode() != MC6809::Load_iPtr_Mem)
        continue;
      // Re-reading the pointer at each call requires it to be invariant.
      if (MI.memoperands_empty() ||
          !llvm::all_of(MI.memoperands(), [](const MachineMemOperand *MMO) {
            return MMO->isInvariant();
          }))
        continue;
      // Frame-index base only: the addressing carries no base register whose
      // live range we would have to extend to the call sites.
      if (!MI.getOperand(1).isFI())
        continue;
      Loads.push_back(&MI);
    }
  }

  bool Changed = false;
  for (MachineInstr *Load : Loads) {
    Register PtrReg = Load->getOperand(0).getReg();

    // Collect the indirect-call uses of the loaded pointer. Other uses are fine
    // (e.g. qsort stores the comparator into its recursion context) — they keep
    // the load alive; we only redirect the calls to read the pointer from
    // memory. Re-reading is consistent with any retained value use because the
    // load is invariant.
    SmallVector<MachineInstr *, 8> Calls;
    for (MachineInstr &Use : MRI.use_nodbg_instructions(PtrReg))
      if (Use.getOpcode() == MC6809::JSRi_o0 && Use.getNumOperands() > 0 &&
          Use.getOperand(0).isReg() && Use.getOperand(0).getReg() == PtrReg)
        Calls.push_back(&Use);
    if (Calls.empty())
      continue;

    // The load's addressing operands (stable while we rewrite the calls).
    const MachineOperand Base = Load->getOperand(1);   // frame index
    const MachineOperand Offset = Load->getOperand(2); // sub-slot byte offset

    for (MachineInstr *Jsr : Calls) {
      MachineInstrBuilder MIB =
          BuildMI(*Jsr->getParent(), *Jsr, Jsr->getDebugLoc(),
                  TII.get(MC6809::JSR_iPtr_Mem));
      MIB.add(Base).add(Offset);
      // The call's register mask and implicit argument/clobber operands follow
      // its target register (operand 0).
      for (unsigned I = 1, E = Jsr->getNumOperands(); I < E; ++I)
        MIB.add(Jsr->getOperand(I));
      Jsr->eraseFromParent();
    }
    Changed = true;

    // The load is dead if only debug values remain; drop them with it so no
    // reference dangles.
    if (MRI.use_nodbg_empty(PtrReg)) {
      for (MachineInstr &Dbg :
           llvm::make_early_inc_range(MRI.use_instructions(PtrReg)))
        Dbg.eraseFromParent();
      Load->eraseFromParent();
    }
  }
  return Changed;
}

char MC6809FoldCallThroughMem::ID = 0;

INITIALIZE_PASS(MC6809FoldCallThroughMem, DEBUG_TYPE,
                "MC6809 fold indirect call through memory", false, false)

MachineFunctionPass *llvm::createMC6809FoldCallThroughMemPass() {
  return new MC6809FoldCallThroughMem();
}
