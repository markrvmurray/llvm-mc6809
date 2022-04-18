//===-- MC6809FrameLowering.cpp - MC6809 Frame Information ----------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "MC6809FrameLowering.h"
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809Subtarget.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/Function.h"
#include "llvm/Target/TargetOptions.h"
#include "llvm/Support/FormatVariadic.h"

#define DEBUG_TYPE "mc6809-framelowering"

using namespace llvm;

bool MC6809FrameLowering::hasFP(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  return (MF.getTarget().Options.DisableFramePointerElim(MF) || MF.getFrameInfo().hasVarSizedObjects() || MFI.isFrameAddressTaken());
}

bool MC6809FrameLowering::hasReservedCallFrame(const MachineFunction &MF) const {
  return !MF.getFrameInfo().hasVarSizedObjects();
}

void MC6809FrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MF = "; MF.dump(););
  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MC6809MachineFunctionInfo *MC6809FI = MF.getInfo<MC6809MachineFunctionInfo>();
  const MC6809InstrInfo &TII = *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());

  MachineBasicBlock::iterator MBBI = MBB.begin();
  DebugLoc DL = MBBI != MBB.end() ? MBBI->getDebugLoc() : DebugLoc();

  // Get the number of bytes to allocate from the FrameInfo.
  uint64_t StackSize = MFI.getStackSize();

  uint64_t NumBytes = 0;
  if (hasFP(MF)) {
    // Calculate required stack adjustment
    uint64_t FrameSize = StackSize - 2;
    NumBytes = FrameSize - MC6809FI->getCalleeSavedFrameSize();

    // Get the offset of the stack slot for the EBP register... which is
    // guaranteed to be the last slot by processFunctionBeforeFrameFinalized.
    // Update the frame offset adjustment.
    MFI.setOffsetAdjustment(-NumBytes);

    // Save FP into the appropriate stack slot...
    BuildMI(MBB, MBBI, DL, TII.get(MC6809::Push16)).addReg(MC6809::SU, RegState::Kill);

    // Update FP with the new base value...
    BuildMI(MBB, MBBI, DL, TII.get(MC6809::TFRp), MC6809::SU).addReg(MC6809::SS);

    // Mark the FramePtr as live-in in every block except the entry.
    for (MachineBasicBlock &MBBJ : llvm::drop_begin(MF))
      MBBJ.addLiveIn(MC6809::SU);

  } else
    NumBytes = StackSize - MC6809FI->getCalleeSavedFrameSize();

  // Skip the callee-saved push instructions.
  while (MBBI != MBB.end() && (MBBI->getOpcode() == MC6809::Push8 || MBBI->getOpcode() == MC6809::Push16))
    ++MBBI;

  if (MBBI != MBB.end())
    DL = MBBI->getDebugLoc();

  if (NumBytes) { // adjust stack pointer: SS -= numbytes
    // If there is an SUB16ri of SP immediately before this instruction, merge
    // the two.
    //NumBytes -= mergeSPUpdates(MBB, MBBI, true);
    // If there is an ADD16ri or SUB16ri of SP immediately after this
    // instruction, merge the two instructions.
    // mergeSPUpdatesDown(MBB, MBBI, &NumBytes);

    BuildMI(MBB, MBBI, DL, TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(-NumBytes);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MF = "; MF.dump(););
}

void MC6809FrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  MC6809MachineFunctionInfo *MC6809FI = MF.getInfo<MC6809MachineFunctionInfo>();
  const MC6809InstrInfo &TII = *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());

  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  unsigned RetOpcode = MBBI->getOpcode();
  DebugLoc DL = MBBI->getDebugLoc();

  switch (RetOpcode) {
  case MC6809::ReturnImplicit:
  case MC6809::ReturnIRQImplicit: break;  // These are ok
  default:
    llvm_unreachable("Can only insert epilog into returning blocks");
  }

  // Get the number of bytes to allocate from the FrameInfo
  uint64_t StackSize = MFI.getStackSize();
  unsigned CSSize = MC6809FI->getCalleeSavedFrameSize();
  uint64_t NumBytes = 0;

  if (hasFP(MF)) {
    // Calculate required stack adjustment
    uint64_t FrameSize = StackSize - 2;
    NumBytes = FrameSize - CSSize;

    // pop FP.
    BuildMI(MBB, MBBI, DL, TII.get(MC6809::Pull16), MC6809::SU);
  } else
    NumBytes = StackSize - CSSize;

  // Skip the callee-saved pop instructions.
  while (MBBI != MBB.begin()) {
    MachineBasicBlock::iterator PI = std::prev(MBBI);
    unsigned Opc = PI->getOpcode();
    if (Opc != MC6809::Pull8 && Opc != MC6809::Pull16 && !PI->isTerminator())
      break;
    --MBBI;
  }

  DL = MBBI->getDebugLoc();

  // If there is an ADD16ri or SUB16ri of SP immediately before this
  // instruction, merge the two instructions.
  //if (NumBytes || MFI.hasVarSizedObjects())
  //  mergeSPUpdatesUp(MBB, MBBI, StackPtr, &NumBytes);

  if (MFI.hasVarSizedObjects()) {
    BuildMI(MBB, MBBI, DL, TII.get(MC6809::TFRp), MC6809::SS).addReg(MC6809::SU);
    if (CSSize) {
      BuildMI(MBB, MBBI, DL, TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(-CSSize);
    }
  } else {
    // adjust stack pointer back: SP += numbytes
    if (NumBytes) {
      BuildMI(MBB, MBBI, DL, TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(NumBytes);
    }
  }
}

// FIXME: Can we eleminate these in favour of generic code?
bool MC6809FrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  MC6809MachineFunctionInfo *MFI = MF.getInfo<MC6809MachineFunctionInfo>();
  MFI->setCalleeSavedFrameSize(CSI.size() * 2);

  for (const CalleeSavedInfo &I : llvm::reverse(CSI)) {
    Register Reg = I.getReg();
    // Add the callee-saved register as live-in. It's killed at the spill.
    MBB.addLiveIn(Reg);
    unsigned Opcode;
    switch (I.getReg()) {
    case MC6809::AA:
    case MC6809::AB:
    case MC6809::AE:
    case MC6809::AF:
      Opcode = MC6809::Push8;
      break;
    case MC6809::AD:
    case MC6809::IX:
    case MC6809::IY:
    case MC6809::SU:
      Opcode = MC6809::Push16;
      break;
    default:
      llvm_unreachable("Unknown push register type");
    }
    BuildMI(MBB, MI, DL, TII.get(Opcode)).addUse(Reg, RegState::Kill);
  }
  return true;
}

bool MC6809FrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  if (CSI.empty())
    return false;

  DebugLoc DL;
  if (MI != MBB.end()) DL = MI->getDebugLoc();

  MachineFunction &MF = *MBB.getParent();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  for (const CalleeSavedInfo &I : CSI) {
    unsigned Opcode;
    switch (I.getReg()) {
    case MC6809::AA:
    case MC6809::AB:
    case MC6809::AE:
    case MC6809::AF:
      Opcode = MC6809::Pull8;
      break;
    case MC6809::AD:
    case MC6809::IX:
    case MC6809::IY:
    case MC6809::SU:
      Opcode = MC6809::Pull16;
      break;
    default:
      llvm_unreachable("Unknown pull register type");
    }
    BuildMI(MBB, MI, DL, TII.get(Opcode)).addDef(I.getReg());
  }

  return true;
}

MachineBasicBlock::iterator MC6809FrameLowering::eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB, MachineBasicBlock::iterator I) const {
  const MC6809InstrInfo &TII = *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());
  if (!hasReservedCallFrame(MF)) {
    // If the stack pointer can be changed after prologue, turn the
    // adjcallstackup instruction into a 'sub SP, <amt>' and the
    // adjcallstackdown instruction into 'add SP, <amt>'
    // TODO: consider using push / pop instead of sub + store / add
    MachineInstr &Old = *I;
    uint64_t Amount = TII.getFrameSize(Old);
    if (Amount != 0) {
      // We need to keep the stack aligned properly.  To do this, we round the
      // amount of space needed for the outgoing arguments up to the next
      // alignment boundary.
      Amount = alignTo(Amount, getStackAlign());

      MachineInstr *New = nullptr;
      if (Old.getOpcode() == TII.getCallFrameSetupOpcode()) {
        New = BuildMI(MF, Old.getDebugLoc(), TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(-Amount);
      } else {
        assert(Old.getOpcode() == TII.getCallFrameDestroyOpcode());
        // factor out the amount the callee already popped.
        Amount -= TII.getFramePoppedByCallee(Old);
        if (Amount)
          New = BuildMI(MF, Old.getDebugLoc(), TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(Amount);
      }

      if (New) {
        // The SRW implicit def is dead.
        New->getOperand(3).setIsDead();

        // Replace the pseudo instruction with a new instruction...
        MBB.insert(I, New);
      }
    }
  } else if (I->getOpcode() == TII.getCallFrameDestroyOpcode()) {
    // If we are performing frame pointer elimination and if the callee pops
    // something off the stack pointer, add it back.
    if (uint64_t CalleeAmt = TII.getFramePoppedByCallee(*I)) {
      MachineInstr &Old = *I;
      MachineInstr *New = BuildMI(MF, Old.getDebugLoc(), TII.get(MC6809::LEAPtrAddImm), MC6809::SS).addReg(MC6809::SS).addImm(-CalleeAmt);
      // The SRW implicit def is dead.
      New->getOperand(3).setIsDead();

      MBB.insert(I, New);
    }
  }

  return MBB.erase(I);
}

void
MC6809FrameLowering::processFunctionBeforeFrameFinalized(MachineFunction &MF, RegScavenger *) const {
  // Create a frame entry for the FP register that must be saved.
  if (hasFP(MF)) {
    int FrameIdx = MF.getFrameInfo().CreateFixedObject(2, -4, true);
    (void)FrameIdx;
    assert(FrameIdx == MF.getFrameInfo().getObjectIndexBegin() && "Slot for FP register must be last in order to be found!");
  }
}