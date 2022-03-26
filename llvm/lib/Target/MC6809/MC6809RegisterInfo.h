//===-- MC6809RegisterInfo.h - MC6809 Register Information Impl -------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809REGISTERINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809REGISTERINFO_H

#include "llvm/CodeGen/TargetRegisterInfo.h"

#define GET_REGINFO_HEADER
#include "MC6809GenRegisterInfo.inc"

namespace llvm {

class MC6809RegisterInfo : public MC6809GenRegisterInfo {

public:
  MC6809RegisterInfo();

  const MCPhysReg *getCalleeSavedRegs(const MachineFunction *MF) const override;

  const uint32_t *getCallPreservedMask(const MachineFunction &MF,
                                       CallingConv::ID) const override;

  const TargetRegisterClass *
  getLargestLegalSuperClass(const TargetRegisterClass *RC,
                            const MachineFunction &) const override;

  const TargetRegisterClass *
  getCrossCopyRegClass(const TargetRegisterClass *RC) const override;

  unsigned getCSRFirstUseCost(const MachineFunction &MF) const override;

  bool requiresRegisterScavenging(const MachineFunction &MF) const override {
    // Saving/restoring to stack may require temporary registers.
    return true;
  }

  bool requiresFrameIndexScavenging(const MachineFunction &MF) const override {
    // Saving/restoring to stack may require temporary registers.
    return true;
  }

  bool saveScavengerRegister(MachineBasicBlock &MBB,
                             MachineBasicBlock::iterator I,
                             MachineBasicBlock::iterator &UseMI,
                             const TargetRegisterClass *RC,
                             Register Reg) const override;

  bool canSaveScavengerRegister(Register Reg) const override;

  void eliminateFrameIndex(MachineBasicBlock::iterator MI, int SPAdj,
                           unsigned FIOperandNum,
                           RegScavenger *RS = nullptr) const override;

  Register getFrameRegister(const MachineFunction &MF) const override;

  bool
  getRegAllocationHints(Register VirtReg, ArrayRef<MCPhysReg> Order,
                        SmallVectorImpl<MCPhysReg> &Hints,
                        const MachineFunction &MF,
                        const VirtRegMap *VRM = nullptr,
                        const LiveRegMatrix *Matrix = nullptr) const override;

  BitVector getReservedRegs(const MachineFunction &MF) const override;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809REGISTERINFO_H
