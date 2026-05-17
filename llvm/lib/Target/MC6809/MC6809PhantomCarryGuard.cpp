//===-- MC6809PhantomCarryGuard.cpp - PHANTOM_CARRY safety net -----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This pass enforces the PHANTOM_CARRY invariant late in the pipeline.
// Bug #302 follow-up.
//
// PHANTOM_CARRY (defined in MC6809RegisterInfo.td) is a register class
// whose eight 1-bit physregs (PHANTOM_CARRY_0..PHANTOM_CARRY_7) exist
// purely as regalloc bookkeeping slots for the scheduling-phantom carry
// vregs produced by SetCarry / SetCarryUse / SetOverflow / SetOverflowUse
// pseudos.  The phantoms have no runtime value (CC.C / CC.V is the source
// of truth) and no hardware encoding.  AsmPrinter ignores implicit
// operands at MCInst conversion, so phantoms never escape into emitted
// assembly -- AS LONG AS they remain implicit operands of pseudos
// emitted by the selector's hand-written `.addDef(CarryOut,
// RegState::ImplicitDefine)` / `.addUse(Carry, RegState::Implicit)`
// calls.
//
// If any pass promotes a phantom operand to explicit (e.g. a peephole
// folding implicit defs/uses into explicit operands), or if any
// TableGen pattern is accidentally written that references PHANTOM_CARRY
// in its `(ins ...)` / `(outs ...)` list (a violation of the invariant
// documented at PHANTOM_CARRY's class definition), the phantom physreg
// would reach MCInst conversion and either crash AsmPrinter, emit a
// 0x3FXX-encoded "garbage" MC operand, or silently emit nothing while
// the runtime semantics of the surrounding code break.
//
// This pass runs in addPreEmitPass(), after every code-shaping pass
// has run but before AsmPrinter.  It walks every MI in every MBB and
// performs three checks:
//
//   Check A (explicit-operand rejection): no explicit MachineOperand
//   may reference a PHANTOM_CARRY physreg.  Implicit operands are
//   fine.  If found, report_fatal_error with the offending MI + the
//   operand index, naming the invariant.
//
//   Check B (chain-integrity verification): for each implicit-def of
//   a PHANTOM_CARRY physreg (producer), walk forward in the same MBB
//   looking for the matching implicit-use (consumer).  Between them
//   (exclusive), assert no MI has $c in its Defs (other than a `dead`
//   def).  A live def of $c between producer and consumer would clobber
//   the carry the chain is threading -- i.e. some pass slipped an MI
//   into a forbidden window.  report_fatal_error names producer +
//   clobber + consumer.
//
// (A future Check C could cross-check the MCInst stream after
// MCInstLower for phantom MCOperands; AsmPrinter normally skips
// implicit operands at MCInst conversion so this is belt-and-braces.
// Not implemented today; the Check A early-rejection makes it
// redundant for the current MIR-level escape paths.)
//
// The pass is read-only; it never mutates MIR.  Pure invariant checking.
//
//===---------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809Subtarget.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/DebugLoc.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-phantom-carry-guard"

namespace {

class MC6809PhantomCarryGuard : public MachineFunctionPass {
public:
  static char ID;
  MC6809PhantomCarryGuard() : MachineFunctionPass(ID) {
    initializeMC6809PhantomCarryGuardPass(*PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "MC6809 PHANTOM_CARRY invariant guard";
  }

  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesAll();
    MachineFunctionPass::getAnalysisUsage(AU);
  }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

} // namespace

char MC6809PhantomCarryGuard::ID = 0;

INITIALIZE_PASS(MC6809PhantomCarryGuard, DEBUG_TYPE,
                "MC6809 PHANTOM_CARRY invariant guard", false, false)

FunctionPass *llvm::createMC6809PhantomCarryGuardPass() {
  return new MC6809PhantomCarryGuard();
}

// True if R is one of PHANTOM_CARRY_0..PHANTOM_CARRY_7.
static bool isPhantomCarry(Register R) {
  if (!R.isValid() || !R.isPhysical())
    return false;
  return MC6809::PHANTOM_CARRYRegClass.contains(R);
}

// True if MI explicitly defines $c (not via an implicit-def dead).
// Defs that are marked `dead` are not real clobbers from the chain's
// perspective -- they're conservative annotations that don't actually
// change CC.C's value.
static bool definesCarryLive(const MachineInstr &MI,
                             const TargetRegisterInfo &TRI) {
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.isDef())
      continue;
    if (MO.isDead())
      continue;
    Register R = MO.getReg();
    if (!R.isValid() || !R.isPhysical())
      continue;
    if (R == MC6809::C || TRI.regsOverlap(R, MC6809::C))
      return true;
  }
  return false;
}

bool MC6809PhantomCarryGuard::runOnMachineFunction(MachineFunction &MF) {
  const TargetRegisterInfo &TRI =
      *MF.getSubtarget().getRegisterInfo();

  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      // Check A: no explicit phantom operands.
      unsigned NumExplicit = MI.getNumExplicitOperands();
      for (unsigned I = 0; I < NumExplicit; ++I) {
        const MachineOperand &MO = MI.getOperand(I);
        if (!MO.isReg())
          continue;
        if (!isPhantomCarry(MO.getReg()))
          continue;

        std::string ErrMsg;
        raw_string_ostream OS(ErrMsg);
        OS << "MC6809PhantomCarryGuard: PHANTOM_CARRY physreg in EXPLICIT "
              "operand #"
           << I << " of MI '";
        MI.print(OS, /*IsStandalone=*/true);
        OS << "' in function '" << MF.getName()
           << "'.  Some pass promoted an implicit operand to explicit, or a "
              "TableGen pattern referenced PHANTOM_CARRY in (ins ...) / "
              "(outs ...).  See PHANTOM_CARRY invariant comment in "
              "MC6809RegisterInfo.td.";
        report_fatal_error(StringRef(ErrMsg));
      }

      // Check B: chain-integrity for each implicit-def of a phantom.
      for (const MachineOperand &MO : MI.implicit_operands()) {
        if (!MO.isReg() || !MO.isDef())
          continue;
        Register PReg = MO.getReg();
        if (!isPhantomCarry(PReg))
          continue;

        // Walk forward in the same MBB looking for the matching implicit
        // use of PReg.  Between producer (MI) and consumer (exclusive),
        // no MI may define $c live.
        auto It = std::next(MachineBasicBlock::iterator(&MI));
        auto End = MBB.end();
        bool FoundConsumer = false;
        const MachineInstr *Clobber = nullptr;

        for (; It != End; ++It) {
          // Stop walking if we find an implicit-use of PReg -- this is
          // the consumer.
          bool IsConsumer = false;
          for (const MachineOperand &CMO : It->implicit_operands()) {
            if (CMO.isReg() && CMO.isUse() && CMO.getReg() == PReg) {
              IsConsumer = true;
              break;
            }
          }
          if (IsConsumer) {
            FoundConsumer = true;
            break;
          }

          // Otherwise: this is an intervening MI.  Check if it clobbers
          // $c.
          if (!Clobber && definesCarryLive(*It, TRI)) {
            Clobber = &*It;
            // Keep walking to find the consumer, so we can report all
            // three MIs together.
          }
        }

        if (FoundConsumer && Clobber) {
          std::string ErrMsg;
          raw_string_ostream OS(ErrMsg);
          OS << "MC6809PhantomCarryGuard: carry-chain clobber detected.\n"
                "  Producer (writes "
             << printReg(PReg, &TRI) << "): ";
          MI.print(OS, /*IsStandalone=*/true);
          OS << "  Clobber (live def of $c between producer and consumer): ";
          Clobber->print(OS, /*IsStandalone=*/true);
          OS << "  Consumer (reads " << printReg(PReg, &TRI) << "): ";
          It->print(OS, /*IsStandalone=*/true);
          OS << "in function '" << MF.getName()
             << "'.  Some pass inserted an MI that overwrites CC.C between "
                "the carry-chain producer and consumer.  See PHANTOM_CARRY "
                "invariant comment in MC6809RegisterInfo.td.";
          report_fatal_error(StringRef(ErrMsg));
        }
        // If FoundConsumer is false (producer's phantom is dead), no
        // consumer means nothing to check -- the phantom was simply
        // unused and regalloc will / has discarded it.
      }
    }
  }

  return false;
}
