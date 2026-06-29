//===- MC6809FrameLowering.h - Define frame lowering for MC6809 -*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 declaration of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809FRAMELOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809FRAMELOWERING_H

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {

class MC6809FrameLowering : public TargetFrameLowering {
public:
  MC6809FrameLowering();

  bool assignCalleeSavedSpillSlots(MachineFunction &MF, const TargetRegisterInfo *TRI, std::vector<CalleeSavedInfo> &CSI) const override;

  bool enableShrinkWrapping(const MachineFunction &MF) const override;

  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const override;

  bool restoreCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const override;

  bool enableCalleeSaveSkip(const MachineFunction &) const override;

  void determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs, RegScavenger *RS) const override;

  void processFunctionBeforeFrameFinalized(MachineFunction &MF, RegScavenger *RS = nullptr) const override;

  MachineBasicBlock::iterator eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const override;

  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  bool hasFP(const MachineFunction &MF) const;

  // Computes bytes occupied by callee-saved registers pushed on the hardware
  // stack outside MachineFrameInfo's modeled stack objects.
  uint64_t getHardStackCalleeSavedSize(const MachineFunction &MF) const;

  // Computes the size of the static stack.
  uint64_t staticSize(const MachineFrameInfo &MFI) const;

  // Bug #387: true when this function's frame should be laid out in static
  // memory rather than on the dynamic stack — gated on the "static-stack"
  // subtarget feature, a real optimisation level, and the "nonreentrant"
  // attribute computed by MC6809NonReentrant.
  bool usesStaticStack(const MachineFunction &MF) const;

  bool isSupportedStackID(TargetStackID::Value ID) const override;

  // Return whether or not the function is a direct ISR.
  bool isISR(const MachineFunction &MF) const;

private:
  bool hasFPImpl(const MachineFunction &MF) const override;

  void offsetSP(MachineIRBuilder &Builder, int64_t Offset) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809FRAMELOWERING_H
