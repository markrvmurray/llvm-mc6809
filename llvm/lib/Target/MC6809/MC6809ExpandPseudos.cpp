//===-- MC6809ExpandPseudos.cpp - Bottom-up post-RA pseudo expansion ------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// The target's replacement for ExpandPostRAPseudos: the same expansions, but
// each block is expanded from its last instruction to its first. When a
// pseudo is expanded, everything after it is then concrete code, so the
// liveness of a register after it is exact -- what an expansion needs to
// know whether a staging register it borrows must be saved, without trusting
// every later pseudo to declare each register its own expansion reads.
//
// An expansion inserts instructions before and after its pseudo, never
// beyond its neighbours, and none is revisited: the walk resumes at the
// instruction that preceded the pseudo.
//
//===----------------------------------------------------------------------===//

#include "MC6809ExpandPseudos.h"

#include "MC6809.h"
#include "MC6809InstrInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-expand-pseudos"

namespace {

class MC6809ExpandPseudos : public MachineFunctionPass {
public:
  static char ID;
  MC6809ExpandPseudos() : MachineFunctionPass(ID) {
    initializeMC6809ExpandPseudosPass(*PassRegistry::getPassRegistry());
  }
  StringRef getPassName() const override {
    return "MC6809 Post-RA Pseudo Expansion";
  }
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
};

// As ExpandPostRAPseudos lowers it.
void lowerSubregToReg(MachineInstr &MI, const TargetInstrInfo &TII,
                      const TargetRegisterInfo &TRI) {
  MachineBasicBlock &MBB = *MI.getParent();
  Register DstReg = MI.getOperand(0).getReg();
  Register InsReg = MI.getOperand(1).getReg();
  Register DstSubReg = TRI.getSubReg(DstReg, MI.getOperand(2).getImm());
  if (MI.allDefsAreDead() || DstSubReg == InsReg) {
    MI.setDesc(TII.get(TargetOpcode::KILL));
    MI.removeOperand(2);
    return;
  }
  TII.copyPhysReg(MBB, MI, MI.getDebugLoc(), DstSubReg, InsReg,
                  MI.getOperand(1).isKill());
  std::prev(MI.getIterator())->addRegisterDefined(DstReg);
  MBB.erase(MI);
}

} // namespace

bool MC6809ExpandPseudos::runOnMachineFunction(MachineFunction &MF) {
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineBasicBlock::iterator It = MBB.end(); It != MBB.begin();) {
      MachineInstr &MI = *--It;
      // Resume before MI whatever its expansion inserts or erases.
      bool AtBegin = It == MBB.begin();
      MachineBasicBlock::iterator Prev = AtBegin ? It : std::prev(It);
      if (!MI.isPseudo())
        continue;
      const auto &MTII = static_cast<const MC6809InstrInfo &>(TII);
      MTII.beginExpansion(MI);
      bool Expanded = TII.expandPostRAPseudo(MI);
      if (!Expanded) {
        switch (MI.getOpcode()) {
        case TargetOpcode::SUBREG_TO_REG:
          lowerSubregToReg(MI, TII, TRI);
          Expanded = true;
          break;
        case TargetOpcode::COPY:
          TII.lowerCopy(&MI);
          Expanded = true;
          break;
        default:
          break;
        }
      }
      MTII.endExpansion();
      Changed |= Expanded;
      if (AtBegin)
        break;
      It = std::next(Prev);
    }
  }
  return Changed;
}

char MC6809ExpandPseudos::ID = 0;

INITIALIZE_PASS(MC6809ExpandPseudos, DEBUG_TYPE,
                "MC6809 bottom-up post-RA pseudo expansion", false, false)

MachineFunctionPass *llvm::createMC6809ExpandPseudosPass() {
  return new MC6809ExpandPseudos();
}
