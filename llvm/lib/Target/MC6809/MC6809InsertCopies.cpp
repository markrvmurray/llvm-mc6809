//===-- MC6809InsertCopies.cpp - MC6809 Copy Insertion --------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 copy insertion pass.
//
// Register coalescing can tightly restrict the register classes of virtual
// registers in the name of avoiding copies. This is usually a good thing, but
// occasionally it's better (or at least not any worse) to copy, since it allows
// use of a faster addressing mode. This pass finds likely candidates for this
// and inserts copies to widen the register classes to include the fastest
// possible operands.
//
//===----------------------------------------------------------------------===//

#include "MC6809InsertCopies.h"

#include "MC6809.h"
#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"

#define DEBUG_TYPE "mc6809-insert-copies"

using namespace llvm;

namespace {

class MC6809InsertCopies : public MachineFunctionPass {
public:
  static char ID;

  MC6809InsertCopies() : MachineFunctionPass(ID) { llvm::initializeMC6809InsertCopiesPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
};

bool MC6809InsertCopies::runOnMachineFunction(MachineFunction &MF) {
  if (skipFunction(MF.getFunction()))
    return false;

#if 0
  MachineRegisterInfo &MRI = MF.getRegInfo();
#endif

  bool Changed = false;
#if 0
  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock::iterator Next;
    for (auto I = MBB.begin(), E = MBB.end(); I != E; I = Next) {
      Next = std::next(I);
      const TargetRegisterClass *WideRC;
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __FILE__ << " : " << __func__ << " : SEE MOS! Shifts, incs, decs may need RMW work" << '\n';);
#if 0
      switch (I->getOpcode()) {
      default:
        continue;
      case MC6809::ASL:
      case MC6809::LSR:
      case MC6809::ROL:
      case MC6809::ROR:
        WideRC = &MC6809::AImag8RegClass;
        break;
      case MC6809::INC:
      case MC6809::DEC:
      case MC6809::IncNMC6809:
      case MC6809::DecNMC6809:
      case MC6809::IncMB:
      case MC6809::DecMB:
      case MC6809::DecDcpMB:
        WideRC = &MC6809::Anyi8RegClass;
        break;
      }
#else
      WideRC = &MC6809::ACC8RegClass;
#endif

      for (unsigned Idx = 0, EndIdx = I->getNumExplicitDefs(); Idx != EndIdx; ++Idx) {
        MachineOperand &DstOp = I->getOperand(Idx);
        if (!DstOp.isReg() || !DstOp.isTied() || !DstOp.getReg().isVirtual())
          continue;
        MachineOperand &SrcOp = I->getOperand(I->findTiedOperandIdx(Idx));

        const TargetRegisterClass *SrcRC = MRI.getRegClass(SrcOp.getReg());
        const TargetRegisterClass *DstRC = MRI.getRegClass(DstOp.getReg());

        // This may be an unrelated tied register; if so, ignore.
        if (!SrcRC->hasSuperClassEq(WideRC) || !DstRC->hasSuperClassEq(WideRC))
          continue;

        // Avoid copying to and from Imag8 just to make the regclass wider. This
        // could produce LDA ASL STA patterns, when it'd be better to just ASL.
        if (SrcRC == &MC6809::Imag8RegClass && DstRC == &MC6809::Imag8RegClass)
          continue;

        if (SrcRC != WideRC) {
          Changed = true;
          MachineIRBuilder Builder(MBB, I);
          SrcOp.setReg(Builder.buildCopy(WideRC, SrcOp).getReg(0));
        }
        if (DstRC != WideRC) {
          Changed = true;
          Register NewDst = MRI.createVirtualRegister(WideRC);
          MachineIRBuilder Builder(MBB, Next);
          Builder.buildCopy(DstOp, NewDst);
          DstOp.setReg(NewDst);
        }
      }
    }
  }
#endif
  return Changed;
}

} // namespace

char MC6809InsertCopies::ID = 0;

INITIALIZE_PASS(MC6809InsertCopies, DEBUG_TYPE, "MC6809 Copy Insertion", false, false)

MachineFunctionPass *llvm::createMC6809InsertCopiesPass() { return new MC6809InsertCopies; }
