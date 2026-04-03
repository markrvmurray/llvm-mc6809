//===-- MC6809BundleCC.cpp - Fix CC clobbering between compare and branch --===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// On the MC6809, all load instructions set N, Z, V condition code flags.
// If the register allocator inserts a spill reload between a compare and its
// consuming conditional branch, the reload clobbers the comparison result.
//
// This post-RA pass detects the pattern and moves the offending reloads
// above the compare instruction, where they don't affect the flags that
// the branch consumes.
//
// Runs after register allocation and spill insertion, before pseudo expansion.
//
//===----------------------------------------------------------------------===//

#include "MC6809BundleCC.h"

#include "MC6809.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-bundle-cc"

using namespace llvm;

/// Check if this instruction clobbers any condition code flags.
static bool clobbersCC(const MachineInstr &MI, const TargetRegisterInfo &TRI) {
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.isDef())
      continue;
    if (TRI.regsOverlap(MO.getReg(), MC6809::CC) ||
        TRI.regsOverlap(MO.getReg(), MC6809::NZ))
      return true;
  }
  // Also check implicit defs.
  for (const MachineOperand &MO : MI.implicit_operands()) {
    if (!MO.isReg() || !MO.isDef())
      continue;
    if (TRI.regsOverlap(MO.getReg(), MC6809::CC) ||
        TRI.regsOverlap(MO.getReg(), MC6809::NZ))
      return true;
  }
  return false;
}

namespace {

class MC6809BundleCC : public MachineFunctionPass {
public:
  static char ID;

  MC6809BundleCC() : MachineFunctionPass(ID) {
    initializeMC6809BundleCCPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "MC6809 Fix CC Clobber";
  }
};

bool MC6809BundleCC::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();

  for (MachineBasicBlock &MBB : MF) {
    // Walk backwards from each conditional branch to find its CC source.
    for (auto It = MBB.end(); It != MBB.begin(); ) {
      --It;
      MachineInstr &Branch = *It;

      // Is this a conditional branch that reads CC?
      if (!Branch.isConditionalBranch())
        continue;
      bool ReadsCC = false;
      for (const MachineOperand &MO : Branch.operands()) {
        if (MO.isReg() && MO.isUse() &&
            (TRI.regsOverlap(MO.getReg(), MC6809::CC) ||
             MO.getReg() == MC6809::CC))
          ReadsCC = true;
      }
      if (!ReadsCC)
        continue;

      // Scan backwards from the branch to find the CC-defining compare.
      // Collect any CC-clobbering instructions between them.
      SmallVector<MachineInstr *, 4> ToMove;
      MachineInstr *Compare = nullptr;
      bool Abort = false;

      for (auto Scan = It; Scan != MBB.begin(); ) {
        --Scan;
        MachineInstr &SI = *Scan;

        // Is this the CC-defining instruction?
        if (SI.isCompare() || SI.definesRegister(MC6809::CC, &TRI)) {
          Compare = &SI;
          break;
        }

        // Does this instruction clobber CC?
        if (clobbersCC(SI, TRI)) {
          // Can we safely move it above the compare?
          // It's safe if it doesn't define any register used by the compare.
          // We'll check this after finding the compare.
          ToMove.push_back(&SI);
        }

        // Safety limit — don't scan too far.
        if (ToMove.size() > 8) {
          Abort = true;
          break;
        }
      }

      if (!Compare || Abort || ToMove.empty())
        continue;

      // Verify each instruction in ToMove can be safely moved before Compare.
      // It's safe if:
      // 1. It doesn't define any register that Compare reads
      // 2. It doesn't read any register that Compare defines
      // 3. It doesn't have memory dependencies with Compare
      bool Safe = true;
      for (MachineInstr *MI : ToMove) {
        for (const MachineOperand &MO : MI->operands()) {
          if (!MO.isReg())
            continue;
          if (MO.isDef()) {
            // Check if Compare reads this register.
            if (Compare->readsRegister(MO.getReg(), &TRI)) {
              Safe = false;
              break;
            }
          }
          if (MO.isUse()) {
            // Check if Compare defines this register.
            if (Compare->definesRegister(MO.getReg(), &TRI)) {
              Safe = false;
              break;
            }
          }
        }
        if (!Safe)
          break;
      }

      if (!Safe)
        continue;

      // Move the CC-clobbering instructions above the compare.
      LLVM_DEBUG(dbgs() << "FixCC: moving " << ToMove.size()
                        << " instr(s) above: " << *Compare);
      for (MachineInstr *MI : ToMove) {
        LLVM_DEBUG(dbgs() << "  moving: " << *MI);
        MI->removeFromParent();
        MBB.insert(Compare->getIterator(), MI);
      }
      Changed = true;
    }
  }

  return Changed;
}

char MC6809BundleCC::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809BundleCC, DEBUG_TYPE,
                "MC6809 Fix CC Clobber", false, false)

MachineFunctionPass *llvm::createMC6809BundleCCPass() {
  return new MC6809BundleCC();
}
