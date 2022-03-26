//===-- MC6809FrameLowering.cpp - MC6809 Frame Information
//------------------===//
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

#include "MC6809.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-framelowering"

using namespace llvm;

static cl::opt<bool>
    DisableLeafProc("disable-mc6809-leaf-proc", cl::init(false),
                    cl::desc("Disable MC6809 leaf procedure optimization."),
                    cl::Hidden);

MC6809FrameLowering::MC6809FrameLowering(const MC6809Subtarget &ST)
    : TargetFrameLowering(TargetFrameLowering::StackGrowsDown, Align(8), 0,
                          Align(8)) {}

void MC6809FrameLowering::emitSPAdjustment(MachineFunction &MF,
                                           MachineBasicBlock &MBB,
                                           MachineBasicBlock::iterator MBBI,
                                           int NumBytes) const {

  DebugLoc dl;
  const MC6809InstrInfo &TII =
      *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());

  BuildMI(MBB, MBBI, dl, TII.get(MC6809::LEASi_o16), MC6809::SS)
      .addReg(MC6809::SS)
      .addImm(NumBytes);
  return;
}

void MC6809FrameLowering::emitPrologue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MC6809MachineFunctionInfo *FuncInfo = MF.getInfo<MC6809MachineFunctionInfo>();

  assert(&MF.front() == &MBB && "Shrink-wrapping not yet supported");
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const MC6809Subtarget &Subtarget = MF.getSubtarget<MC6809Subtarget>();
  const MC6809InstrInfo &TII =
      *static_cast<const MC6809InstrInfo *>(Subtarget.getInstrInfo());
  const MC6809RegisterInfo &RegInfo =
      *static_cast<const MC6809RegisterInfo *>(Subtarget.getRegisterInfo());
  MachineBasicBlock::iterator MBBI = MBB.begin();
  // Debug location must be unknown since the first debug location is used
  // to determine the end of the prologue.
  DebugLoc dl;

#if 0
  // Get the number of bytes to allocate from the FrameInfo
  int NumBytes = (int) MFI.getStackSize();

  // The MC6809 ABI is a bit odd in that it requires a reserved 92-byte
  // (128 in v9) area in the user's stack, starting at %sp. Thus, the
  // first part of the stack that can actually be used is located at
  // %sp + 92.
  //
  // We therefore need to add that offset to the total stack size
  // after all the stack objects are placed by
  // PrologEpilogInserter calculateFrameObjectOffsets. However, since the stack needs to be
  // aligned *after* the extra size is added, we need to disable
  // calculateFrameObjectOffsets's built-in stack alignment, by having
  // targetHandlesStackFrameRounding return true.


  // Add the extra call frame stack size, if needed. (This is the same
  // code as in PrologEpilogInserter, but also gets disabled by
  // targetHandlesStackFrameRounding)
  if (MFI.adjustsStack() && hasReservedCallFrame(MF))
    NumBytes += MFI.getMaxCallFrameSize();

  // Adds the MC6809 subtarget-specific spill area to the stack
  // size. Also ensures target-required alignment.
  NumBytes = Subtarget.getAdjustedFrameSize(NumBytes);

  // Finally, ensure that the size is sufficiently aligned for the
  // data on the stack.
  NumBytes = alignTo(NumBytes, MFI.getMaxAlign());

  // Update stack size with corrected value.
  MFI.setStackSize(NumBytes);

  emitSPAdjustment(MF, MBB, MBBI, -NumBytes);

  unsigned regFP = RegInfo.getDwarfRegNum(MC6809::I6, true);

  // Emit ".cfi_def_cfa_register 30".
  unsigned CFIIndex =
      MF.addFrameInst(MCCFIInstruction::createDefCfaRegister(nullptr, regFP));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  // Emit ".cfi_window_save".
  CFIIndex = MF.addFrameInst(MCCFIInstruction::createWindowSave(nullptr));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);

  unsigned regInRA = RegInfo.getDwarfRegNum(MC6809::I7, true);
  unsigned regOutRA = RegInfo.getDwarfRegNum(MC6809::O7, true);
  // Emit ".cfi_register 15, 31".
  CFIIndex = MF.addFrameInst(
      MCCFIInstruction::createRegister(nullptr, regOutRA, regInRA));
  BuildMI(MBB, MBBI, dl, TII.get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex);
#endif
}

MachineBasicBlock::iterator MC6809FrameLowering::eliminateCallFramePseudoInstr(
    MachineFunction &MF, MachineBasicBlock &MBB,
    MachineBasicBlock::iterator I) const {
  if (!hasReservedCallFrame(MF)) {
    MachineInstr &MI = *I;
    int Size = MI.getOperand(0).getImm();
    if (MI.getOpcode() == MC6809::ADJCALLSTACKDOWN)
      Size = -Size;

    if (Size)
      emitSPAdjustment(MF, MBB, I, Size);
  }
  return MBB.erase(I);
}

void MC6809FrameLowering::emitEpilogue(MachineFunction &MF,
                                       MachineBasicBlock &MBB) const {
  MC6809MachineFunctionInfo *FuncInfo = MF.getInfo<MC6809MachineFunctionInfo>();
  MachineBasicBlock::iterator MBBI = MBB.getLastNonDebugInstr();
  const MC6809InstrInfo &TII =
      *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());
  DebugLoc dl = MBBI->getDebugLoc();
  // assert(MBBI->getOpcode() == MC6809::RTSr && "Can only put epilog before
  // 'rts' instruction!");
  MachineFrameInfo &MFI = MF.getFrameInfo();

  int NumBytes = (int)MFI.getStackSize();
  if (NumBytes == 0)
    return;

  emitSPAdjustment(MF, MBB, MBBI, NumBytes);
}

bool MC6809FrameLowering::hasReservedCallFrame(
    const MachineFunction &MF) const {
  // Reserve call frame if there are no variable sized objects on the stack.
  return !MF.getFrameInfo().hasVarSizedObjects();
}

// hasFP - Return true if the specified function should have a dedicated frame
// pointer register.  This is true if the function has variable sized allocas or
// if frame pointer elimination is disabled.
bool MC6809FrameLowering::hasFP(const MachineFunction &MF) const {
  const TargetRegisterInfo *RegInfo = MF.getSubtarget().getRegisterInfo();

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  return MF.getTarget().Options.DisableFramePointerElim(MF) ||
         RegInfo->hasStackRealignment(MF) || MFI.hasVarSizedObjects() ||
         MFI.isFrameAddressTaken();
}

StackOffset
MC6809FrameLowering::getFrameIndexReference(const MachineFunction &MF, int FI,
                                            Register &FrameReg) const {
  const MC6809Subtarget &Subtarget = MF.getSubtarget<MC6809Subtarget>();
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const MC6809RegisterInfo *RegInfo = Subtarget.getRegisterInfo();
  const MC6809MachineFunctionInfo *FuncInfo =
      MF.getInfo<MC6809MachineFunctionInfo>();
  bool isFixed = MFI.isFixedObjectIndex(FI);

  // Addressable stack objects are accessed using neg. offsets from
  // %fp, or positive offsets from %sp.
  bool UseFP;

  // MC6809 uses FP-based references in general, even when "hasFP" is
  // false. That function is rather a misnomer, because %fp is
  // actually always available
  if (isFixed) {
    // Otherwise, argument access should always use %fp.
    UseFP = true;
  } else if (RegInfo->hasStackRealignment(MF)) {
    // If there is dynamic stack realignment, all local object
    // references need to be via %sp, to take account of the
    // re-alignment.
    UseFP = false;
  } else {
    // Finally, default to using %fp.
    UseFP = true;
  }

  int64_t FrameOffset = MF.getFrameInfo().getObjectOffset(FI);

  if (UseFP) {
    FrameReg = RegInfo->getFrameRegister(MF);
    return StackOffset::getFixed(FrameOffset);
  } else {
    FrameReg = MC6809::SS; // %sp
    return StackOffset::getFixed(FrameOffset +
                                 MF.getFrameInfo().getStackSize());
  }
}

static bool LLVM_ATTRIBUTE_UNUSED
verifyLeafProcRegUse(MachineRegisterInfo *MRI) {
  return true;
}

void MC6809FrameLowering::determineCalleeSaves(MachineFunction &MF,
                                               BitVector &SavedRegs,
                                               RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);
}
