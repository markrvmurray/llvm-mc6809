//===-- MC6809LegalizerInfo.h - MC6809 Legalizer ----------------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file declares the interface that MC6809 uses to legalize generic MIR.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H

#include "llvm/CodeGen/GlobalISel/LegalizerInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"

namespace llvm {

class MC6809Subtarget;

class MC6809LegalizerInfo : public LegalizerInfo {
public:
  MC6809LegalizerInfo(const MC6809Subtarget &STI);

  bool legalizeIntrinsic(LegalizerHelper &Helper, MachineInstr &MI) const override;

  bool legalizeCustom(LegalizerHelper &Helper, MachineInstr &MI) const override;

private:
  // Memory Operations
  bool legalizeLoadStore(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  // bool legalizePtrAdd(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool selectAddressingMode(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool tryAbsoluteAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool tryIndexedAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool tryIndirectIndexedAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool selectNoAddressing(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;

  // Memory Operations
  bool legalizeAddSub(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool legalizeSubO(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
  bool legalizeSubE(LegalizerHelper &Helper, MachineRegisterInfo &MRI, MachineInstr &MI) const;
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809LEGALIZERINFO_H