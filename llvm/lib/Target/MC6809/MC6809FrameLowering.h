//===-- MC6809TargetFrameLowering.h - Frame lowering for MC6809 -*- C++ -*-===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This class implements mc6809-specific bits of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809FRAMELOWERING_H
#define LLVM_LIB_TARGET_MC6809_MC6809FRAMELOWERING_H

#include "llvm/CodeGen/TargetFrameLowering.h"

namespace llvm {
class MC6809Subtarget;
class MC6809InstrInfo;
class MC6809RegisterInfo;

class MC6809FrameLowering : public TargetFrameLowering {
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo *TRI;

  bool Is24Bit;
  unsigned SlotSize;

public:
  explicit MC6809FrameLowering(const MC6809Subtarget &STI);

  /// MC6809 has no alignment requirements.
  bool targetHandlesStackFrameRounding() const override { return true; }

  /// emitProlog/emitEpilog - These methods insert prolog and epilog code into
  /// the function.
  void emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const override;
  void emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const override;

  bool
  assignCalleeSavedSpillSlots(MachineFunction &MF,
                              const TargetRegisterInfo *TRI,
                              std::vector<CalleeSavedInfo> &CSI) const override;
  bool spillCalleeSavedRegisters(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator MI,
                                 ArrayRef<CalleeSavedInfo> CSI,
                                 const TargetRegisterInfo *TRI) const override;
  bool
  restoreCalleeSavedRegisters(MachineBasicBlock &MBB,
                              MachineBasicBlock::iterator MI,
                              MutableArrayRef<CalleeSavedInfo> CSI,
                              const TargetRegisterInfo *TRI) const override;

  void processFunctionBeforeFrameFinalized(
      MachineFunction &MF, RegScavenger *RS = nullptr) const override;

  MachineBasicBlock::iterator
  eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB,
                                MachineBasicBlock::iterator MI) const override;

  bool hasFP(const MachineFunction &MF) const override;
  bool isFPSaved(const MachineFunction &MF) const;
  bool hasReservedCallFrame(const MachineFunction &MF) const override;
  bool needsFrameIndexResolution(const MachineFunction &MF) const override;
  unsigned getSlotSize() const { return SlotSize; }

  enum StackAdjustmentMethod {
    SAM_None,
    SAM_Tiny,
    SAM_All,
    SAM_Small,
    SAM_Medium,
    SAM_Large
  };
  StackAdjustmentMethod getOptimalStackAdjustmentMethod(
      MachineFunction &MF, int Offset, int FPOffset = -1,
      bool ScratchIsIndex = false, bool UnknownOffset = false) const;

private:
  void BuildStackAdjustment(MachineFunction &MF, MachineBasicBlock &MBB,
                            MachineBasicBlock::iterator MBBI,
                            const DebugLoc &DL, Register ScratchReg, int Offset,
                            int FPOffset = -1,
                            MachineInstr::MIFlag Flag = MachineInstr::NoFlags,
                            bool UnknownOffset = false) const;

  void
  shadowCalleeSavedRegisters(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator MI, const DebugLoc &DL,
                             MachineInstr::MIFlag Flag,
                             const std::vector<CalleeSavedInfo> &CSI) const;
};
} // End llvm namespace

#endif
