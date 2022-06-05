//===-- MC6809InstrInfo.cpp - MC6809 Instruction Information --------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstrInfo.h"

#include "MC6809RegisterInfo.h"

#include "MC6809Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-instrinfo"

#define GET_INSTRINFO_CTOR_DTOR
#include "MC6809GenInstrInfo.inc"

void llvm::emitFrameOffset(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, const DebugLoc &DL, unsigned DestReg, unsigned SrcReg, StackOffset Offset, const TargetInstrInfo *TII, MachineInstr::MIFlag Flag) {
  int64_t Bytes = Offset.getFixed();

  if (Bytes || (!Offset && SrcReg != DestReg))
    BuildMI(MBB, MBBI, DL, TII->get(MC6809::LEAPtrAdd), DestReg).addReg(SrcReg).addImm(Bytes);
}

MC6809InstrInfo::MC6809InstrInfo(const MC6809Subtarget &STI)
    : MC6809GenInstrInfo(/*CFSetupOpcode=*/MC6809::ADJCALLSTACKDOWN, /*CFDestroyOpcode=*/MC6809::ADJCALLSTACKUP), STI(STI) {
  LEAPtrAddImmOpcode = {
      {{MC6809::IX, -1}, MC6809::LEAXi_o16}, {{MC6809::IX, 0}, MC6809::LEAXi_o0}, {{MC6809::IX, 5}, MC6809::LEAXi_o5}, {{MC6809::IX, 8}, MC6809::LEAXi_o8}, {{MC6809::IX, 16}, MC6809::LEAXi_o16},
      {{MC6809::IY, -1}, MC6809::LEAYi_o16}, {{MC6809::IY, 0}, MC6809::LEAYi_o0}, {{MC6809::IY, 5}, MC6809::LEAYi_o5}, {{MC6809::IY, 8}, MC6809::LEAYi_o8}, {{MC6809::IY, 16}, MC6809::LEAYi_o16},
      {{MC6809::SU, -1}, MC6809::LEAUi_o16}, {{MC6809::SU, 0}, MC6809::LEAUi_o0}, {{MC6809::SU, 5}, MC6809::LEAUi_o5}, {{MC6809::SU, 8}, MC6809::LEAUi_o8}, {{MC6809::SU, 16}, MC6809::LEAUi_o16},
      {{MC6809::SS, -1}, MC6809::LEASi_o16}, {{MC6809::SS, 0}, MC6809::LEASi_o0}, {{MC6809::SS, 5}, MC6809::LEASi_o5}, {{MC6809::SS, 8}, MC6809::LEASi_o8}, {{MC6809::SS, 16}, MC6809::LEASi_o16},
  };
  LEAPtrAddRegOpcode = {
      {{MC6809::IX,MC6809::AA}, MC6809::LEAXi_oA}, {{MC6809::IX,MC6809::AB}, MC6809::LEAXi_oB}, {{MC6809::IX,MC6809::AD}, MC6809::LEAXi_oD}, {{MC6809::IX,MC6809::AE}, MC6809::LEAXi_oE}, {{MC6809::IX,MC6809::AF}, MC6809::LEAXi_oF}, {{MC6809::IX,MC6809::AW}, MC6809::LEAXi_oW},
      {{MC6809::IY,MC6809::AA}, MC6809::LEAYi_oA}, {{MC6809::IY,MC6809::AB}, MC6809::LEAYi_oB}, {{MC6809::IY,MC6809::AD}, MC6809::LEAYi_oD}, {{MC6809::IY,MC6809::AE}, MC6809::LEAYi_oE}, {{MC6809::IY,MC6809::AF}, MC6809::LEAYi_oF}, {{MC6809::IY,MC6809::AW}, MC6809::LEAYi_oW},
      {{MC6809::SU,MC6809::AA}, MC6809::LEAUi_oA}, {{MC6809::SU,MC6809::AB}, MC6809::LEAUi_oB}, {{MC6809::SU,MC6809::AD}, MC6809::LEAUi_oD}, {{MC6809::SU,MC6809::AE}, MC6809::LEAUi_oE}, {{MC6809::SU,MC6809::AF}, MC6809::LEAUi_oF}, {{MC6809::SU,MC6809::AW}, MC6809::LEAUi_oW},
      {{MC6809::SS,MC6809::AA}, MC6809::LEASi_oA}, {{MC6809::SS,MC6809::AB}, MC6809::LEASi_oB}, {{MC6809::SS,MC6809::AD}, MC6809::LEASi_oD}, {{MC6809::SS,MC6809::AE}, MC6809::LEASi_oE}, {{MC6809::SS,MC6809::AF}, MC6809::LEASi_oF}, {{MC6809::SS,MC6809::AW}, MC6809::LEASi_oW},
  };
  LoadImmediateOpcode = {
      {MC6809::AA, MC6809::LDAi8}, {MC6809::AB, MC6809::LDBi8},
      {MC6809::AE, MC6809::LDEi8}, {MC6809::AF, MC6809::LDFi8},
      {MC6809::AD, MC6809::LDDi16}, {MC6809::AW, MC6809::LDWi16},
      {MC6809::AQ, MC6809::LDQi32},
      {MC6809::IX, MC6809::LDXi16}, {MC6809::IY, MC6809::LDYi16},
      {MC6809::SU, MC6809::LDUi16}, {MC6809::SS, MC6809::LDSi16},
  };
  LoadIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::LDAi_o16}, {{MC6809::AA, 0}, MC6809::LDAi_o0}, {{MC6809::AA, 5}, MC6809::LDAi_o5}, {{MC6809::AA, 8}, MC6809::LDAi_o8}, {{MC6809::AA, 16}, MC6809::LDAi_o16},
      {{MC6809::AB, -1}, MC6809::LDBi_o16}, {{MC6809::AB, 0}, MC6809::LDBi_o0}, {{MC6809::AB, 5}, MC6809::LDBi_o5}, {{MC6809::AB, 8}, MC6809::LDBi_o8}, {{MC6809::AB, 16}, MC6809::LDBi_o16},
      {{MC6809::AD, -1}, MC6809::LDDi_o16}, {{MC6809::AD, 0}, MC6809::LDDi_o0}, {{MC6809::AD, 5}, MC6809::LDDi_o5}, {{MC6809::AD, 8}, MC6809::LDDi_o8}, {{MC6809::AD, 16}, MC6809::LDDi_o16},
      {{MC6809::AE, -1}, MC6809::LDEi_o16}, {{MC6809::AE, 0}, MC6809::LDEi_o0}, {{MC6809::AE, 5}, MC6809::LDEi_o5}, {{MC6809::AE, 8}, MC6809::LDEi_o8}, {{MC6809::AE, 16}, MC6809::LDEi_o16},
      {{MC6809::AF, -1}, MC6809::LDFi_o16}, {{MC6809::AF, 0}, MC6809::LDFi_o0}, {{MC6809::AF, 5}, MC6809::LDFi_o5}, {{MC6809::AF, 8}, MC6809::LDFi_o8}, {{MC6809::AF, 16}, MC6809::LDFi_o16},
      {{MC6809::AW, -1}, MC6809::LDWi_o16}, {{MC6809::AW, 0}, MC6809::LDWi_o0}, {{MC6809::AW, 5}, MC6809::LDWi_o5}, {{MC6809::AW, 8}, MC6809::LDWi_o8}, {{MC6809::AW, 16}, MC6809::LDWi_o16},
      {{MC6809::AQ, -1}, MC6809::LDQi_o16}, {{MC6809::AQ, 0}, MC6809::LDQi_o0}, {{MC6809::AQ, 5}, MC6809::LDQi_o5}, {{MC6809::AQ, 8}, MC6809::LDQi_o8}, {{MC6809::AQ, 16}, MC6809::LDQi_o16},
      {{MC6809::IX, -1}, MC6809::LDXi_o16}, {{MC6809::IX, 0}, MC6809::LDXi_o0}, {{MC6809::IX, 5}, MC6809::LDXi_o5}, {{MC6809::IX, 8}, MC6809::LDXi_o8}, {{MC6809::IX, 16}, MC6809::LDXi_o16},
      {{MC6809::IY, -1}, MC6809::LDYi_o16}, {{MC6809::IY, 0}, MC6809::LDYi_o0}, {{MC6809::IY, 5}, MC6809::LDYi_o5}, {{MC6809::IY, 8}, MC6809::LDYi_o8}, {{MC6809::IY, 16}, MC6809::LDYi_o16},
      {{MC6809::SU, -1}, MC6809::LDUi_o16}, {{MC6809::SU, 0}, MC6809::LDUi_o0}, {{MC6809::SU, 5}, MC6809::LDUi_o5}, {{MC6809::SU, 8}, MC6809::LDUi_o8}, {{MC6809::SU, 16}, MC6809::LDUi_o16},
      {{MC6809::SS, -1}, MC6809::LDSi_o16}, {{MC6809::SS, 0}, MC6809::LDSi_o0}, {{MC6809::SS, 5}, MC6809::LDSi_o5}, {{MC6809::SS, 8}, MC6809::LDSi_o8}, {{MC6809::SS, 16}, MC6809::LDSi_o16},
  };
  LoadIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::LDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::LDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::LDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::LDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::LDAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::LDAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::LDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::LDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::LDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::LDBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::LDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::LDBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::LDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::LDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::LDDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::LDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::LDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::LDDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::LDEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::LDEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::LDEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::LDEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::LDEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::LDEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::LDFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::LDFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::LDFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::LDFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::LDFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::LDFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::LDWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::LDWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::LDWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::LDWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::LDWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::LDWi_oW},
      {{MC6809::AQ, MC6809::AA}, MC6809::LDQi_oA}, {{MC6809::AQ, MC6809::AB}, MC6809::LDQi_oB}, {{MC6809::AQ, MC6809::AD}, MC6809::LDQi_oD}, {{MC6809::AQ, MC6809::AE}, MC6809::LDQi_oE}, {{MC6809::AQ, MC6809::AF}, MC6809::LDQi_oF}, {{MC6809::AQ, MC6809::AW}, MC6809::LDQi_oW},
      {{MC6809::IX, MC6809::AA}, MC6809::LDXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::LDXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::LDXi_oD}, {{MC6809::IX, MC6809::AE}, MC6809::LDXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::LDXi_oF}, {{MC6809::IX, MC6809::AW}, MC6809::LDXi_oW},
      {{MC6809::IY, MC6809::AA}, MC6809::LDYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::LDYi_oB}, {{MC6809::IY, MC6809::AD}, MC6809::LDYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::LDYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::LDYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::LDYi_oW},
      {{MC6809::SU, MC6809::AA}, MC6809::LDUi_oA}, {{MC6809::SU, MC6809::AB}, MC6809::LDUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::LDUi_oD}, {{MC6809::SU, MC6809::AE}, MC6809::LDUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::LDUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::LDUi_oW},
      {{MC6809::SS, MC6809::AA}, MC6809::LDSi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::LDSi_oB}, {{MC6809::SS, MC6809::AD}, MC6809::LDSi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::LDSi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::LDSi_oF}, {{MC6809::SS, MC6809::AW}, MC6809::LDSi_oW},
  };
  StoreIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::STAi_o16}, {{MC6809::AA, 0}, MC6809::STAi_o0}, {{MC6809::AA, 5}, MC6809::STAi_o5}, {{MC6809::AA, 8}, MC6809::STAi_o8}, {{MC6809::AA, 16}, MC6809::STAi_o16},
      {{MC6809::AB, -1}, MC6809::STBi_o16}, {{MC6809::AB, 0}, MC6809::STBi_o0}, {{MC6809::AB, 5}, MC6809::STBi_o5}, {{MC6809::AB, 8}, MC6809::STBi_o8}, {{MC6809::AB, 16}, MC6809::STBi_o16},
      {{MC6809::AD, -1}, MC6809::STDi_o16}, {{MC6809::AD, 0}, MC6809::STDi_o0}, {{MC6809::AD, 5}, MC6809::STDi_o5}, {{MC6809::AD, 8}, MC6809::STDi_o8}, {{MC6809::AD, 16}, MC6809::STDi_o16},
      {{MC6809::AE, -1}, MC6809::STEi_o16}, {{MC6809::AE, 0}, MC6809::STEi_o0}, {{MC6809::AE, 5}, MC6809::STEi_o5}, {{MC6809::AE, 8}, MC6809::STEi_o8}, {{MC6809::AE, 16}, MC6809::STEi_o16},
      {{MC6809::AF, -1}, MC6809::STFi_o16}, {{MC6809::AF, 0}, MC6809::STFi_o0}, {{MC6809::AF, 5}, MC6809::STFi_o5}, {{MC6809::AF, 8}, MC6809::STFi_o8}, {{MC6809::AF, 16}, MC6809::STFi_o16},
      {{MC6809::AW, -1}, MC6809::STWi_o16}, {{MC6809::AW, 0}, MC6809::STWi_o0}, {{MC6809::AW, 5}, MC6809::STWi_o5}, {{MC6809::AW, 8}, MC6809::STWi_o8}, {{MC6809::AW, 16}, MC6809::STWi_o16},
      {{MC6809::AQ, -1}, MC6809::STQi_o16}, {{MC6809::AQ, 0}, MC6809::STQi_o0}, {{MC6809::AQ, 5}, MC6809::STQi_o5}, {{MC6809::AQ, 8}, MC6809::STQi_o8}, {{MC6809::AQ, 16}, MC6809::STQi_o16},
      {{MC6809::IX, -1}, MC6809::STXi_o16}, {{MC6809::IX, 0}, MC6809::STXi_o0}, {{MC6809::IX, 5}, MC6809::STXi_o5}, {{MC6809::IX, 8}, MC6809::STXi_o8}, {{MC6809::IX, 16}, MC6809::STXi_o16},
      {{MC6809::IY, -1}, MC6809::STYi_o16}, {{MC6809::IY, 0}, MC6809::STYi_o0}, {{MC6809::IY, 5}, MC6809::STYi_o5}, {{MC6809::IY, 8}, MC6809::STYi_o8}, {{MC6809::IY, 16}, MC6809::STYi_o16},
      {{MC6809::SU, -1}, MC6809::STUi_o16}, {{MC6809::SU, 0}, MC6809::STUi_o0}, {{MC6809::SU, 5}, MC6809::STUi_o5}, {{MC6809::SU, 8}, MC6809::STUi_o8}, {{MC6809::SU, 16}, MC6809::STUi_o16},
      {{MC6809::SS, -1}, MC6809::STSi_o16}, {{MC6809::SS, 0}, MC6809::STSi_o0}, {{MC6809::SS, 5}, MC6809::STSi_o5}, {{MC6809::SS, 8}, MC6809::STSi_o8}, {{MC6809::SS, 16}, MC6809::STSi_o16},
  };
  StoreIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::STAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::STAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::STAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::STAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::STAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::STAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::STBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::STBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::STBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::STBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::STBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::STBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::STDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::STDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::STDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::STDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::STDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::STDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::STEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::STEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::STEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::STEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::STEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::STEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::STFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::STFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::STFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::STFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::STFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::STFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::STWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::STWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::STWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::STWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::STWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::STWi_oW},
      {{MC6809::AQ, MC6809::AA}, MC6809::STQi_oA}, {{MC6809::AQ, MC6809::AB}, MC6809::STQi_oB}, {{MC6809::AQ, MC6809::AD}, MC6809::STQi_oD}, {{MC6809::AQ, MC6809::AE}, MC6809::STQi_oE}, {{MC6809::AQ, MC6809::AF}, MC6809::STQi_oF}, {{MC6809::AQ, MC6809::AW}, MC6809::STQi_oW},
      {{MC6809::IX, MC6809::AA}, MC6809::STXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::STXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::STXi_oD}, {{MC6809::IX, MC6809::AE}, MC6809::STXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::STXi_oF}, {{MC6809::IX, MC6809::AW}, MC6809::STXi_oW},
      {{MC6809::IY, MC6809::AA}, MC6809::STYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::STYi_oB}, {{MC6809::IY, MC6809::AD}, MC6809::STYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::STYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::STYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::STYi_oW},
      {{MC6809::SU, MC6809::AA}, MC6809::STUi_oA}, {{MC6809::SU, MC6809::AB}, MC6809::STUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::STUi_oD}, {{MC6809::SU, MC6809::AE}, MC6809::STUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::STUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::STUi_oW},
      {{MC6809::SS, MC6809::AA}, MC6809::STSi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::STSi_oB}, {{MC6809::SS, MC6809::AD}, MC6809::STSi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::STSi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::STSi_oF}, {{MC6809::SS, MC6809::AW}, MC6809::STSi_oW},
  };
  AddImmediateOpcode = {
      {{MC6809::AA}, MC6809::ADDAi8},
      {{MC6809::AB}, MC6809::ADDBi8},
      {{MC6809::AE}, MC6809::ADDEi8},
      {{MC6809::AF}, MC6809::ADDFi8},
      {{MC6809::AD}, MC6809::ADDDi16},
      {{MC6809::AW}, MC6809::ADDWi16},
  };
  AddCarryImmediateOpcode = {
      {{MC6809::AA}, MC6809::ADCAi8},
      {{MC6809::AB}, MC6809::ADCBi8},
      {{MC6809::AD}, MC6809::ADCDi16},
  };
  AddIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ADDAi_o16}, {{MC6809::AA, 0}, MC6809::ADDAi_o0}, {{MC6809::AA, 5}, MC6809::ADDAi_o5}, {{MC6809::AA, 8}, MC6809::ADDAi_o8}, {{MC6809::AA, 16}, MC6809::ADDAi_o16},
      {{MC6809::AB, -1}, MC6809::ADDBi_o16}, {{MC6809::AB, 0}, MC6809::ADDBi_o0}, {{MC6809::AB, 5}, MC6809::ADDBi_o5}, {{MC6809::AB, 8}, MC6809::ADDBi_o8}, {{MC6809::AB, 16}, MC6809::ADDBi_o16},
      {{MC6809::AD, -1}, MC6809::ADDDi_o16}, {{MC6809::AD, 0}, MC6809::ADDDi_o0}, {{MC6809::AD, 5}, MC6809::ADDDi_o5}, {{MC6809::AD, 8}, MC6809::ADDDi_o8}, {{MC6809::AD, 16}, MC6809::ADDDi_o16},
      {{MC6809::AE, -1}, MC6809::ADDEi_o16}, {{MC6809::AE, 0}, MC6809::ADDEi_o0}, {{MC6809::AE, 5}, MC6809::ADDEi_o5}, {{MC6809::AE, 8}, MC6809::ADDEi_o8}, {{MC6809::AE, 16}, MC6809::ADDEi_o16},
      {{MC6809::AF, -1}, MC6809::ADDFi_o16}, {{MC6809::AF, 0}, MC6809::ADDFi_o0}, {{MC6809::AF, 5}, MC6809::ADDFi_o5}, {{MC6809::AF, 8}, MC6809::ADDFi_o8}, {{MC6809::AF, 16}, MC6809::ADDFi_o16},
      {{MC6809::AW, -1}, MC6809::ADDWi_o16}, {{MC6809::AW, 0}, MC6809::ADDWi_o0}, {{MC6809::AW, 5}, MC6809::ADDWi_o5}, {{MC6809::AW, 8}, MC6809::ADDWi_o8}, {{MC6809::AW, 16}, MC6809::ADDWi_o16},
  };
  AddCarryIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ADCAi_o16}, {{MC6809::AA, 0}, MC6809::ADCAi_o0}, {{MC6809::AA, 5}, MC6809::ADCAi_o5}, {{MC6809::AA, 8}, MC6809::ADCAi_o8}, {{MC6809::AA, 16}, MC6809::ADCAi_o16},
      {{MC6809::AB, -1}, MC6809::ADCBi_o16}, {{MC6809::AB, 0}, MC6809::ADCBi_o0}, {{MC6809::AB, 5}, MC6809::ADCBi_o5}, {{MC6809::AB, 8}, MC6809::ADCBi_o8}, {{MC6809::AB, 16}, MC6809::ADCBi_o16},
      {{MC6809::AD, -1}, MC6809::ADCDi_o16}, {{MC6809::AD, 0}, MC6809::ADCDi_o0}, {{MC6809::AD, 5}, MC6809::ADCDi_o5}, {{MC6809::AD, 8}, MC6809::ADCDi_o8}, {{MC6809::AD, 16}, MC6809::ADCDi_o16},
  };
  AddIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ADDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ADDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ADDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ADDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ADDAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::ADDAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::ADDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ADDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ADDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ADDBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::ADDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ADDBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::ADDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ADDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ADDDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::ADDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ADDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ADDDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::ADDEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::ADDEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::ADDEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::ADDEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::ADDEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::ADDEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::ADDFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::ADDFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::ADDFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::ADDFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::ADDFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::ADDFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::ADDWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::ADDWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::ADDWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::ADDWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::ADDWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::ADDWi_oW},
  };
  AddCarryIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ADCAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ADCAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ADCAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ADCAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ADCAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::ADCAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::ADCBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ADCBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ADCBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ADCBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::ADCBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ADCBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::ADCDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ADCDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ADCDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::ADCDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ADCDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ADCDi_oW},
  };
  SubImmediateOpcode = {
      {{MC6809::AA}, MC6809::SUBAi8},
      {{MC6809::AB}, MC6809::SUBBi8},
      {{MC6809::AE}, MC6809::SUBEi8},
      {{MC6809::AF}, MC6809::SUBFi8},
      {{MC6809::AD}, MC6809::SUBDi16},
      {{MC6809::AW}, MC6809::SUBWi16},
  };
  SubIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::SUBAi_o16}, {{MC6809::AA, 0}, MC6809::SUBAi_o0}, {{MC6809::AA, 5}, MC6809::SUBAi_o5}, {{MC6809::AA, 8}, MC6809::SUBAi_o8}, {{MC6809::AA, 16}, MC6809::SUBAi_o16},
      {{MC6809::AB, -1}, MC6809::SUBBi_o16}, {{MC6809::AB, 0}, MC6809::SUBBi_o0}, {{MC6809::AB, 5}, MC6809::SUBBi_o5}, {{MC6809::AB, 8}, MC6809::SUBBi_o8}, {{MC6809::AB, 16}, MC6809::SUBBi_o16},
      {{MC6809::AD, -1}, MC6809::SUBDi_o16}, {{MC6809::AD, 0}, MC6809::SUBDi_o0}, {{MC6809::AD, 5}, MC6809::SUBDi_o5}, {{MC6809::AD, 8}, MC6809::SUBDi_o8}, {{MC6809::AD, 16}, MC6809::SUBDi_o16},
      {{MC6809::AE, -1}, MC6809::SUBEi_o16}, {{MC6809::AE, 0}, MC6809::SUBEi_o0}, {{MC6809::AE, 5}, MC6809::SUBEi_o5}, {{MC6809::AE, 8}, MC6809::SUBEi_o8}, {{MC6809::AE, 16}, MC6809::SUBEi_o16},
      {{MC6809::AF, -1}, MC6809::SUBFi_o16}, {{MC6809::AF, 0}, MC6809::SUBFi_o0}, {{MC6809::AF, 5}, MC6809::SUBFi_o5}, {{MC6809::AF, 8}, MC6809::SUBFi_o8}, {{MC6809::AF, 16}, MC6809::SUBFi_o16},
      {{MC6809::AW, -1}, MC6809::SUBWi_o16}, {{MC6809::AW, 0}, MC6809::SUBWi_o0}, {{MC6809::AW, 5}, MC6809::SUBWi_o5}, {{MC6809::AW, 8}, MC6809::SUBWi_o8}, {{MC6809::AW, 16}, MC6809::SUBWi_o16},
  };
  SubIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::SUBAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::SUBAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::SUBAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::SUBAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::SUBAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::SUBAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::SUBBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::SUBBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::SUBBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::SUBBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::SUBBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::SUBBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::SUBDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::SUBDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::SUBDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::SUBDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::SUBDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::SUBDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::SUBEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::SUBEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::SUBEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::SUBEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::SUBEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::SUBEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::SUBFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::SUBFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::SUBFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::SUBFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::SUBFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::SUBFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::SUBWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::SUBWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::SUBWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::SUBWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::SUBWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::SUBWi_oW},
  };
  SubPopOpcode = {
      {{MC6809::AA}, MC6809::SUBAi_Inc1},
      {{MC6809::AB}, MC6809::SUBBi_Inc1},
      {{MC6809::AE}, MC6809::SUBEi_Inc1},
      {{MC6809::AF}, MC6809::SUBFi_Inc1},
      {{MC6809::AD}, MC6809::SUBDi_Inc2},
      {{MC6809::AW}, MC6809::SUBWi_Inc2},
  };
  SubBorrowImmediateOpcode = {
      {{MC6809::AA}, MC6809::SBCAi8},
      {{MC6809::AB}, MC6809::SBCBi8},
      {{MC6809::AD}, MC6809::SBCDi16},
  };
  SubBorrowIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::SBCAi_o16}, {{MC6809::AA, 0}, MC6809::SBCAi_o0}, {{MC6809::AA, 5}, MC6809::SBCAi_o5}, {{MC6809::AA, 8}, MC6809::SBCAi_o8}, {{MC6809::AA, 16}, MC6809::SBCAi_o16},
      {{MC6809::AB, -1}, MC6809::SBCBi_o16}, {{MC6809::AB, 0}, MC6809::SBCBi_o0}, {{MC6809::AB, 5}, MC6809::SBCBi_o5}, {{MC6809::AB, 8}, MC6809::SBCBi_o8}, {{MC6809::AB, 16}, MC6809::SBCBi_o16},
      {{MC6809::AD, -1}, MC6809::SBCDi_o16}, {{MC6809::AD, 0}, MC6809::SBCDi_o0}, {{MC6809::AD, 5}, MC6809::SBCDi_o5}, {{MC6809::AD, 8}, MC6809::SBCDi_o8}, {{MC6809::AD, 16}, MC6809::SBCDi_o16},
  };
  SubBorrowIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::SBCAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::SBCAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::SBCAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::SBCAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::SBCAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::SBCAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::SBCBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::SBCBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::SBCBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::SBCBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::SBCBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::SBCBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::SBCDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::SBCDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::SBCDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::SBCDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::SBCDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::SBCDi_oW},
  };
  SubBorrowPopOpcode = {
      {{MC6809::AA}, MC6809::SBCAi_Inc1},
      {{MC6809::AB}, MC6809::SBCBi_Inc1},
      {{MC6809::AD}, MC6809::SBCDi_Inc2},
  };
  CompareImmediateOpcode = {
      {{MC6809::AA}, MC6809::CMPAi8},
      {{MC6809::AB}, MC6809::CMPBi8},
      {{MC6809::AE}, MC6809::CMPEi8},
      {{MC6809::AF}, MC6809::CMPFi8},
      {{MC6809::AD}, MC6809::CMPDi16},
      {{MC6809::AW}, MC6809::CMPWi16},
  };
  CompareIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::CMPAi_o16}, {{MC6809::AA, 0}, MC6809::CMPAi_o0}, {{MC6809::AA, 5}, MC6809::CMPAi_o5}, {{MC6809::AA, 8}, MC6809::CMPAi_o8}, {{MC6809::AA, 16}, MC6809::CMPAi_o16},
      {{MC6809::AB, -1}, MC6809::CMPBi_o16}, {{MC6809::AB, 0}, MC6809::CMPBi_o0}, {{MC6809::AB, 5}, MC6809::CMPBi_o5}, {{MC6809::AB, 8}, MC6809::CMPBi_o8}, {{MC6809::AB, 16}, MC6809::CMPBi_o16},
      {{MC6809::AD, -1}, MC6809::CMPDi_o16}, {{MC6809::AD, 0}, MC6809::CMPDi_o0}, {{MC6809::AD, 5}, MC6809::CMPDi_o5}, {{MC6809::AD, 8}, MC6809::CMPDi_o8}, {{MC6809::AD, 16}, MC6809::CMPDi_o16},
      {{MC6809::AE, -1}, MC6809::CMPEi_o16}, {{MC6809::AE, 0}, MC6809::CMPEi_o0}, {{MC6809::AE, 5}, MC6809::CMPEi_o5}, {{MC6809::AE, 8}, MC6809::CMPEi_o8}, {{MC6809::AE, 16}, MC6809::CMPEi_o16},
      {{MC6809::AF, -1}, MC6809::CMPFi_o16}, {{MC6809::AF, 0}, MC6809::CMPFi_o0}, {{MC6809::AF, 5}, MC6809::CMPFi_o5}, {{MC6809::AF, 8}, MC6809::CMPFi_o8}, {{MC6809::AF, 16}, MC6809::CMPFi_o16},
      {{MC6809::AW, -1}, MC6809::CMPWi_o16}, {{MC6809::AW, 0}, MC6809::CMPWi_o0}, {{MC6809::AW, 5}, MC6809::CMPWi_o5}, {{MC6809::AW, 8}, MC6809::CMPWi_o8}, {{MC6809::AW, 16}, MC6809::CMPWi_o16},
  };
  CompareIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::CMPAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::CMPAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::CMPAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::CMPAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::CMPAi_oF}, {{MC6809::AA, MC6809::AW}, MC6809::CMPAi_oW},
      {{MC6809::AB, MC6809::AA}, MC6809::CMPBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::CMPBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::CMPBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::CMPBi_oE}, {{MC6809::AB, MC6809::AF}, MC6809::CMPBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::CMPBi_oW},
      {{MC6809::AD, MC6809::AA}, MC6809::CMPDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::CMPDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::CMPDi_oD}, {{MC6809::AD, MC6809::AE}, MC6809::CMPDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::CMPDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::CMPDi_oW},
      {{MC6809::AE, MC6809::AA}, MC6809::CMPEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::CMPEi_oB}, {{MC6809::AE, MC6809::AD}, MC6809::CMPEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::CMPEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::CMPEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::CMPEi_oW},
      {{MC6809::AF, MC6809::AA}, MC6809::CMPFi_oA}, {{MC6809::AF, MC6809::AB}, MC6809::CMPFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::CMPFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::CMPFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::CMPFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::CMPFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::CMPWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::CMPWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::CMPWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::CMPWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::CMPWi_oF}, {{MC6809::AW, MC6809::AW}, MC6809::CMPWi_oW},
  };
  ComparePopOpcode = {
      {{MC6809::AA}, MC6809::CMPAi_Inc1},
      {{MC6809::AB}, MC6809::CMPBi_Inc1},
      {{MC6809::AE}, MC6809::CMPEi_Inc1},
      {{MC6809::AF}, MC6809::CMPFi_Inc1},
      {{MC6809::AD}, MC6809::CMPDi_Inc2},
      {{MC6809::AW}, MC6809::CMPWi_Inc2},
  };
}

unsigned MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    break;
  case MC6809::LDAi_o8:
  case MC6809::LDBi_o8:
  case MC6809::LDDi_o8:
  case MC6809::LDEi_o8:
  case MC6809::LDFi_o8:
  case MC6809::LDWi_o8:
  case MC6809::LDQi_o8:
  case MC6809::LDXi_o8:
  case MC6809::LDYi_o8:
  case MC6809::LDAi_o16:
  case MC6809::LDBi_o16:
  case MC6809::LDDi_o16:
  case MC6809::LDEi_o16:
  case MC6809::LDFi_o16:
  case MC6809::LDWi_o16:
  case MC6809::LDQi_o16:
  case MC6809::LDXi_o16:
  case MC6809::LDYi_o16:
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() && MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }

  return 0;
}

unsigned MC6809InstrInfo::isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    break;
  case MC6809::STAi_o8:
  case MC6809::STBi_o8:
  case MC6809::STDi_o8:
  case MC6809::STEi_o8:
  case MC6809::STFi_o8:
  case MC6809::STWi_o8:
  case MC6809::STQi_o8:
  case MC6809::STXi_o8:
  case MC6809::STYi_o8:
  case MC6809::STAi_o16:
  case MC6809::STBi_o16:
  case MC6809::STDi_o16:
  case MC6809::STEi_o16:
  case MC6809::STFi_o16:
  case MC6809::STWi_o16:
  case MC6809::STQi_o16:
  case MC6809::STXi_o16:
  case MC6809::STYi_o16:
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() && MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    break;
  }
  return 0;
}

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator I, Register DestReg, unsigned SubIdx, const MachineInstr &Orig, const TargetRegisterInfo &TRI) const {
  if (Orig.getOpcode() == MC6809::Load8Imm) {
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->removeOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(MC6809::Load8Imm));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig, TRI);
  }
}

MachineInstr *MC6809InstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI, unsigned Idx1, unsigned Idx2) const {
  // NOTE: This doesn't seem to actually be used anywhere.
  if (NewMI)
    report_fatal_error("NewMI is not supported");

  MachineFunction &MF = *MI.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const MC6809RegisterInfo *TRI = STI.getRegisterInfo();

  LLVM_DEBUG(dbgs() << "Commute: " << MI);

  // Determines the register class for a given virtual register constrained by a
  // target register class and all uses outside this instruction. This
  // effectively removes the constraints due to just this instruction, then
  // tries to apply the constraint for the other operand.
  const auto NewRegClass = [&](Register Reg, const TargetRegisterClass *RC) -> const TargetRegisterClass * {
    for (MachineOperand &MO : MRI.reg_nodbg_operands(Reg)) {
      MachineInstr *UseMI = MO.getParent();
      if (UseMI == &MI)
        continue;
      unsigned OpNo = &MO - &UseMI->getOperand(0);
      RC = UseMI->getRegClassConstraintEffect(OpNo, RC, this, TRI);
      if (!RC)
        return nullptr;
    }
    return RC;
  };

  const TargetRegisterClass *RegClass1 = getRegClass(MI.getDesc(), Idx1, TRI, MF);
  const TargetRegisterClass *RegClass2 = getRegClass(MI.getDesc(), Idx2, TRI, MF);
  Register Reg1 = MI.getOperand(Idx1).getReg();
  Register Reg2 = MI.getOperand(Idx2).getReg();

  // See if swapping the two operands are possible given their register classes.
  const TargetRegisterClass *Reg1Class = nullptr;
  const TargetRegisterClass *Reg2Class = nullptr;
  if (Reg1.isVirtual()) {
    Reg1Class = NewRegClass(Reg1, RegClass2);
    if (!Reg1Class)
      return nullptr;
  }
  if (Reg1.isPhysical() && !RegClass2->contains(Reg1))
    return nullptr;
  if (Reg2.isVirtual()) {
    Reg2Class = NewRegClass(Reg2, RegClass1);
    if (!Reg2Class)
      return nullptr;
  }
  if (Reg2.isPhysical() && !RegClass1->contains(Reg2))
    return nullptr;

  // If this fails, make sure to get it out of the way before rewriting reg
  // classes.
  MachineInstr *CommutedMI = TargetInstrInfo::commuteInstructionImpl(MI, NewMI, Idx1, Idx2);
  if (!CommutedMI)
    return nullptr;

  // PHI nodes keep the register classes of all their arguments. By the time the
  // two address instruction pass occurs, these phis have already been lowered
  // to copies. Changing register classes here can make those register classes
  // mismatch the new ones; to avoid this, we recompute the register classes for
  // any vregs copied into or out of a commuted vreg.
  const auto RecomputeCopyRC = [&](Register Reg) {
    for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
      if (!MI.isCopy())
        continue;
      Register Other = MI.getOperand(0).getReg() == Reg ? MI.getOperand(1).getReg() : MI.getOperand(0).getReg();
      if (!Other.isVirtual())
        continue;
      MRI.recomputeRegClass(Other);
    }
  };

  // Use the new register classes computed above, if any.
  if (Reg1Class) {
    MRI.setRegClass(Reg1, Reg1Class);
    RecomputeCopyRC(Reg1);
  }
  if (Reg2Class) {
    MRI.setRegClass(Reg2, Reg2Class);
    RecomputeCopyRC(Reg2);
  }
  return CommutedMI;
}

unsigned MC6809InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  const MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MCAsmInfo *MAI = MF->getTarget().getMCAsmInfo();

  const MCInstrDesc &MCID = MI.getDesc();

  switch (MI.getOpcode()) {
  default:
    // Return the size specified in .td file. If there's none, return 0, as we
    // can't define a default size.
    return MCID.getSize();
  case TargetOpcode::BUNDLE:
    return getInstBundleLength(MI);
  case MC6809::INLINEASM:
  case MC6809::INLINEASM_BR: {
    // If this machine instr is an inline asm, measure it.
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(), *MAI);
  }
  }
}

bool MC6809InstrInfo::isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const {
  switch (BranchOpc) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::Bbc:
    // BRA range is [-128,127] starting from the PC location after the
    // instruction, which is two bytes after the start of the instruction.
    return -126 <= BrOffset && BrOffset <= 129;
  case MC6809::LBRAlb:
  case MC6809::LBlbc:
    // LBRA, JMP, JSR range is [-32768,32767], covering the full memory range.
    return true;
  }
}

unsigned MC6809InstrInfo::getInstBundleLength(const MachineInstr &MI) const {
  unsigned Size = 0;
  MachineBasicBlock::const_instr_iterator I = MI.getIterator();
  MachineBasicBlock::const_instr_iterator E = MI.getParent()->instr_end();
  while (++I != E && I->isInsideBundle()) {
    assert(!I->isBundle() && "No nested bundle!");
    Size += getInstSizeInBytes(*I);
  }
  return Size;
}

// 6809 instructions aren't as regular as most commutable instructions, so this
// routine determines the commutable operands manually.
bool MC6809InstrInfo::findCommutedOpIndices(const MachineInstr &MI, unsigned &SrcOpIdx1, unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() && "MC6809InstrInfo::findCommutedOpIndices() can't handle bundles");

  // XXXX: FIXME: MarkM - Find and commute the 6809 instructions
  return false;
}

MachineBasicBlock *
MC6809InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::LBRAlb:
    return MI.getOperand(0).getMBB();
  case MC6809::Bbc:
  case MC6809::LBlbc:
    return MI.getOperand(1).getMBB();
  case MC6809::JumpRelative:
  case MC6809::G_BR:
    return MI.getOperand(0).getMBB();
  case MC6809::JumpConditionalRelative:
  case MC6809::G_BRCOND:
    return MI.getOperand(1).getMBB();
  case MC6809::JumpIndir:
  case MC6809::G_BRINDIRECT:
    return nullptr;
  }
}

bool MC6809InstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB, SmallVectorImpl<MachineOperand> &Cond, bool AllowModify) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MBB = "; MBB.dump(););
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : TBB = "; TBB->dump(););
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : FBB = "; FBB->dump(););
  auto I = MBB.getFirstTerminator();
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : First terminator : I = "; I->dump(););

  // Advance past any comparison terminators.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Advancing past any compares\n";);
  while (I != MBB.end() && I->isCompare()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Advancing past any compares : I = "; I->dump(););
    ++I;
  }

  // If no terminators, falls through.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : If no terminators, falls through\n";);
  if (I == MBB.end())
    return false;

  // Non-branch terminators cannot be analyzed.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Non-branch terminators cannot be analyzed\n";);
  if (!I->isBranch())
    return true;

  // Analyze first branch.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Analyze first branch\n";);
  auto FirstBR = I++;
#if 0
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : First branch : FirstBR = "; FirstBR->dump(););
  if (FirstBR->isPreISelOpcode())
    return true;
#endif /* 0 */
  // First branch always forms true edge, whether conditional or unconditional.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : First branch always forms true edge, whether conditional or unconditional\n";);
  TBB = getBranchDestBlock(*FirstBR);
  if (!TBB)
    return true;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : True BB : TBB = "; TBB->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : First branch is conditional?\n";);
  if (FirstBR->isConditionalBranch()) {
    Cond.push_back(FirstBR->getOperand(0));
    Cond.push_back(FirstBR->getOperand(1));
  }

  // If there's no second branch, done.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : If there's no second branch, done?\n";);
  if (I == MBB.end())
    return false;

  // Cannot analyze branch followed by non-branch.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Cannot analyze branch followed by non-branch?\n";);
  if (!I->isBranch())
    return true;

  auto SecondBR = I++;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Second branch : SecondBR = "; SecondBR->dump(););

  // If any instructions follow the second branch, cannot analyze.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : If any instructions follow the second branch, cannot analyze?\n";);
  if (I != MBB.end())
    return true;

  // Exactly two branches present.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exactly two branches present\n";);

  // Can only analyze conditional branch followed by unconditional branch.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Can only analyze conditional branch followed by unconditional branch?\n";);
  if (!SecondBR->isUnconditionalBranch() /* || SecondBR->isPreISelOpcode() */)
    return true;

  // Second unconditional branch forms false edge.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Second unconditional branch forms false edge\n";);
  FBB = getBranchDestBlock(*SecondBR);
  if (!FBB)
    return true;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Returning false\n";);
  return false;
}

unsigned MC6809InstrInfo::removeBranch(MachineBasicBlock &MBB, int *BytesRemoved) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MBB = "; MBB.dump(););
  // Since analyzeBranch succeeded, we know that the only terminators are
  // comparisons and branches.

  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();

  // Advance to first branch.
  while (Begin != End && Begin->isCompare())
    ++Begin;

  // Erase all remaining terminators.
  unsigned NumRemoved = std::distance(Begin, End);
  if (BytesRemoved) {
    *BytesRemoved = 0;
    for (const auto &I : make_range(Begin, End))
      *BytesRemoved += getInstSizeInBytes(I);
  }
  MBB.erase(Begin, End);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Returning " << NumRemoved << "\n";);
  return NumRemoved;
}

unsigned MC6809InstrInfo::insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MBB = "; MBB.dump(););
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MBB = "; TBB->dump(););
  // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MBB = "; FBB->dump(););
  // Since analyzeBranch succeeded and any existing branches were removed, the
  // only remaining terminators are comparisons.

  MachineIRBuilder Builder(MBB, MBB.end());
  unsigned NumAdded = 0;
  if (BytesAdded)
    *BytesAdded = 0;

  // Unconditional branch target.
  auto *UBB = TBB;

  // Conditional branch.
  if (!Cond.empty()) {
    assert(TBB);
    // The condition stores the arguments for the Bcc and LBcc instructionis.
    assert(Cond.size() == 2);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    // Add conditional branch.
    unsigned Opcode = MC6809::Bbc;
    auto BR = Builder.buildInstr(Opcode)
                  .addImm(Cond[0].getImm())
                  .addMBB(TBB);
#if 0
    for (const MachineOperand &Op : Cond)
      BR.add(Op);
#endif /* 0 */
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    // For 6809, assume BRA and relax into LBRA in insertIndirectBranch if
    // necessary.
    auto JMP = Builder.buildInstr(MC6809::Bbc)
                   .addImm(MC6809CC::RA)
                   .addMBB(UBB);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*JMP);
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Returning " << NumAdded << "\n";);
  return NumAdded;
}

void MC6809InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB, MachineBasicBlock &NewDestBB, MachineBasicBlock &RestoreBB, const DebugLoc &DL, int64_t BrOffset, RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.
  // XXXX: FIXME: MarkM - this process is a crock; LBRA should alway work.

  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  // XXXX: FIXME: MarkM - ensure this is unconditional
  Builder.buildInstr(MC6809::Bbc).addMBB(&NewDestBB);
}

void MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg, bool KillSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  copyPhysRegImpl(Builder, DestReg, SrcReg);
}

void MC6809InstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg, Register SrcReg) const {
  if (DestReg == SrcReg)
    return;

  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() && !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) {
    return IsClass(DestReg, Dest) && IsClass(SrcReg, Src);
  };

  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    if (AreClasses(MC6809::ADcRegClass, MC6809::ABcRegClass) || AreClasses(MC6809::AWcRegClass, MC6809::AFcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    if (AreClasses(MC6809::ABcRegClass, MC6809::ADcRegClass) || AreClasses(MC6809::AFcRegClass, MC6809::AWcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    const MC6809RegisterInfo *TRI = STI.getRegisterInfo();
    Register SrcReg8 = TRI->getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 = TRI->getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    assert(SrcReg8 && DestReg8 && "Single-bit Src and Dst must both be LSB of 8-bit registers"); 

    const MachineInstr &MI = *Builder.getInsertPt();
    // MC6809 defines LSB writes to write the whole 8-bit register, not just
    // part of it.
    assert(!MI.readsRegister(DestReg8));
    copyPhysRegImpl(Builder, DestReg8, SrcReg8);
  } else
    llvm_unreachable("Unexpected physical register copy.");
}

const TargetRegisterClass *
MC6809InstrInfo::canFoldCopy(const MachineInstr &MI, unsigned FoldIdx) const {
  if (!MI.getMF()->getFunction().doesNotRecurse())
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MC6809::ACC8RegClass.contains(FoldReg) || MC6809::BIT1RegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MC6809::ACC8RegClass || RC == &MC6809::BIT1RegClass)
      return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  }
  return nullptr;
}

void MC6809InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, Register SrcReg,
                                          bool isKill, int FI, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FI);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, MachineMemOperand::MOStore, MFI.getObjectSize(FI), MFI.getObjectAlign(FI));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);

  unsigned Opc = 0;
  switch (TRI->getSpillSize(*RC)) {
  case 1:
    if (MC6809::ACC8RegClass.hasSubClassEq(RC)) {
      Opc = MC6809::Store8Idx;
    }
    break;
  case 2:
    if (MC6809::ACC16RegClass.hasSubClassEq(RC))
      Opc = MC6809::Store16Idx;
    if (MC6809::INDEX16RegClass.hasSubClassEq(RC))
      Opc = MC6809::StorePtrIdx;
    break;
  case 4:
    if (MC6809::ACC32RegClass.hasSubClassEq(RC)) {
      Opc = MC6809::Store32Idx;
    }
    break;
  }
  assert(Opc && "Unknown register class");

  MFI.setStackID(FI, TargetStackID::Default);

  const MachineInstrBuilder MI = BuildMI(MBB, MBBI, DebugLoc(), get(Opc))
                                     .addReg(SrcReg, getKillRegState(isKill))
                                     .addFrameIndex(FI)
                                     .addImm(0)
                                     .addMemOperand(MMO);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

void MC6809InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, Register DestReg,
                                           int FI, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FI);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, MachineMemOperand::MOLoad, MFI.getObjectSize(FI), MFI.getObjectAlign(FI));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);

  unsigned Opc = 0;
  switch (TRI->getSpillSize(*RC)) {
  case 1:
    if (MC6809::ACC8RegClass.hasSubClassEq(RC))
      Opc = MC6809::Load8Idx;
    break;
  case 2:
    if (MC6809::ACC16RegClass.hasSubClassEq(RC))
      Opc = MC6809::Load16Idx;
    if (MC6809::INDEX16RegClass.hasSubClassEq(RC))
      Opc = MC6809::LoadPtrIdx;
    break;
  case 4:
    if (MC6809::ACC32RegClass.hasSubClassEq(RC))
      Opc = MC6809::Load32Idx;
    break;
  }
  assert(Opc && "Unknown register class");

  MFI.setStackID(FI, TargetStackID::Default);

  const MachineInstrBuilder MI = BuildMI(MBB, MBBI, DebugLoc(), get(Opc))
                                     .addReg(DestReg, getDefRegState(true))
                                     .addFrameIndex(FI)
                                     .addImm(0)
                                     .addMemOperand(MMO);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

bool MC6809InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  case MC6809::CallRelative:
    expandCallRelative(Builder, MI);
    break;
  case MC6809::LEAPtrAdd:
    expandLEAPtrAdd(Builder, MI);
    break;
  case MC6809::Load8Imm:
  case MC6809::Load16Imm:
  case MC6809::Load32Imm:
    expandLoadImm(Builder, MI);
    break;
  case MC6809::Load8Idx:
  case MC6809::Load16Idx:
  case MC6809::Load32Idx:
    expandLoadIdx(Builder, MI);
    break;
  case MC6809::Store8Idx:
  case MC6809::Store16Idx:
  case MC6809::Store32Idx:
    expandStoreIdx(Builder, MI);
    break;
  case MC6809::Add8Imm:
  case MC6809::Add16Imm:
    expandAddImm(Builder, MI);
    break;
  case MC6809::AddCarry8Imm:
  case MC6809::AddCarry16Imm:
    expandAddCarryImm(Builder, MI);
    break;
  case MC6809::Add8Idx:
  case MC6809::Add16Idx:
    expandAddIdx(Builder, MI);
    break;
  case MC6809::AddCarry8Idx:
  case MC6809::AddCarry16Idx:
    expandAddCarryIdx(Builder, MI);
    break;
  case MC6809::Add8Reg:
  case MC6809::Add16Reg:
  case MC6809::AddCarry8Reg:
  case MC6809::AddCarry16Reg:
    expandAddReg(Builder, MI);
    break;
  case MC6809::Add32Imm:
  case MC6809::AddCarry32Imm:
    expandAdd32Imm(Builder, MI);
    break;
  case MC6809::Add32Idx:
  case MC6809::AddCarry32Idx:
    expandAdd32Idx(Builder, MI);
    break;
  case MC6809::Sub8Imm:
  case MC6809::Sub16Imm:
    expandSubImm(Builder, MI);
    break;
  case MC6809::Sub8Idx:
  case MC6809::Sub16Idx:
    expandSubIdx(Builder, MI);
    break;
  case MC6809::Sub8Pop:
  case MC6809::Sub16Pop:
    expandSubPop(Builder, MI);
    break;
  case MC6809::SubBorrow8Imm:
  case MC6809::SubBorrow16Imm:
    expandSubBorrowImm(Builder, MI);
    break;
  case MC6809::SubBorrow8Idx:
  case MC6809::SubBorrow16Idx:
    expandSubBorrowIdx(Builder, MI);
    break;
  case MC6809::SubBorrow8Pop:
  case MC6809::SubBorrow16Pop:
    expandSubBorrowPop(Builder, MI);
    break;
  case MC6809::Sub8Reg:
  case MC6809::Sub16Reg:
  case MC6809::SubBorrow8Reg:
  case MC6809::SubBorrow16Reg:
    expandSubReg(Builder, MI);
    break;
  case MC6809::Sub32Imm:
  case MC6809::SubBorrow32Imm:
    expandSub32Imm(Builder, MI);
    break;
  case MC6809::Sub32Idx:
  case MC6809::SubBorrow32Idx:
    expandSub32Idx(Builder, MI);
    break;
  case MC6809::Sub32Pop:
    expandSub32Pop(Builder, MI);
    break;
  case MC6809::Compare8Imm:
  case MC6809::Compare16Imm:
    expandCompareImm(Builder, MI);
    break;
  case MC6809::Compare8Idx:
  case MC6809::Compare16Idx:
    expandCompareIdx(Builder, MI);
    break;
  case MC6809::Compare8Pop:
  case MC6809::Compare16Pop:
    expandComparePop(Builder, MI);
    break;
  case MC6809::Copy8:
  case MC6809::Copy16:
    MI.setDesc(Builder.getTII().get(MC6809::TFRp));
    break;
  case MC6809::Push8:
  case MC6809::PushOp8:
  case MC6809::Push16:
  case MC6809::PushOp16:
  case MC6809::PushPtr:
  case MC6809::PushOpPtr: {
    if (MI.getOperand(0).getReg() == MC6809::AW) {
      MI.setDesc(Builder.getTII().get(MC6809::PSHSWx));
      MI.removeOperand(0);
      break;
    }
    MI.setDesc(Builder.getTII().get(MC6809::PSHSs));
    unsigned short regList = 0;
    switch (MI.getOperand(0).getReg()) {
    case MC6809::CC:
      regList |= 1;
      break;
    case MC6809::AA:
      regList |= 2;
      break;
    case MC6809::AB:
      regList |= 4;
      break;
    case MC6809::AD:
      regList |= 6;
      break;
    case MC6809::DP:
      regList |= 8;
      break;
    case MC6809::IX:
      regList |= 16;
      break;
    case MC6809::IY:
      regList |= 32;
      break;
    case MC6809::SU:
      regList |= 64;
      break;
    case MC6809::PC:
      regList |= 128;
      break;
    }
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
  case MC6809::PushOp32: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PushOp32 : MI = "; MI.dump(););
    auto PushLo = Builder.buildInstr(MC6809::PSHSWx);
    auto PushHi = Builder.buildInstr(MC6809::PSHSs)
        .addImm(0x06); // Bits for AA and AB
    MI.eraseFromParent();
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PushOp32 : PushLo = "; PushLo->dump(););
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : PushOp32 : PushHi = "; PushHi->dump(););
    break;
  }
  case MC6809::Pull8:
  case MC6809::Pull16:
  case MC6809::PullPtr: {
    if (MI.getOperand(0).getReg() == MC6809::AW) {
      MI.setDesc(Builder.getTII().get(MC6809::PULSWx));
      MI.removeOperand(0);
      break;
    }
    MI.setDesc(Builder.getTII().get(MC6809::PULSs));
    unsigned short regList = 0;
    switch (MI.getOperand(0).getReg()) {
    case MC6809::CC:
      regList |= 1;
      break;
    case MC6809::AA:
      regList |= 2;
      break;
    case MC6809::AB:
      regList |= 4;
      break;
    case MC6809::AD:
      regList |= 6;
      break;
    case MC6809::DP:
      regList |= 8;
      break;
    case MC6809::IX:
      regList |= 16;
      break;
    case MC6809::IY:
      regList |= 32;
      break;
    case MC6809::SU:
      regList |= 64;
      break;
    case MC6809::PC:
      regList |= 128;
      break;
    }
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  return Changed;
}

//===---------------------------------------------------------------------===//
// Post RA pseudos
//===---------------------------------------------------------------------===//

int MC6809InstrInfo::offsetSizeInBits(MachineOperand &OffsetOp) {
  int64_t Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
  if (OffsetOp.isImm())
    Offset = OffsetOp.getImm();
  else if (OffsetOp.isCImm())
    Offset = OffsetOp.getCImm()->getSExtValue();
  else
    return -1;
  return (Offset == 0)                        ? 0
      : ((Offset >= -16 && Offset < 16)       ? 5
      : ((Offset >= -128 && Offset < 128)     ? 8
      : ((Offset >= -32768 && Offset < 32768) ? 16
      : 256))); // Do I need this? Maybe there is a relocation involved?
}

void MC6809InstrInfo::expandCallRelative(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MI.setDesc(Builder.getTII().get(MC6809::JSRi_o16PC));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLEAPtrAdd(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand IndexReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);
  int OffsetSize = -1;

  if (OffsetOp.isImm() || OffsetOp.isCImm())
    OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{IndexReg.getReg(), OffsetSize};
    auto OpcodePair = LEAPtrAddImmOpcode.find(Lookup);
    if (OpcodePair == LEAPtrAddImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    if (OffsetSize == 0)
      MI.removeOperand(2);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{IndexReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = LEAPtrAddRegOpcode.find(Lookup);
    if (OpcodePair == LEAPtrAddRegOpcode.end())
      llvm_unreachable("Unexpected LoadIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  } else
    llvm_unreachable("Unknown offset type for LEAPtrAdd");
  MI.removeOperand(1);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = LoadImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == LoadImmediateOpcode.end())
    llvm_unreachable("Unexpected LoadImm register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = LoadIdxImmOpcode.find(Lookup);
    if (OpcodePair == LoadIdxImmOpcode.end())
      llvm_unreachable("Unexpected LoadIdx numeric offset. Too large?");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(2);
    MI.removeOperand(1);
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = LoadIdxRegOpcode.find(Lookup);
    if (OpcodePair == LoadIdxRegOpcode.end())
      llvm_unreachable("Unexpected LoadIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(1);
  } else
    llvm_unreachable("Unknown offset type for LoadIdx");
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandStoreIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = StoreIdxImmOpcode.find(Lookup);
    if (OpcodePair == StoreIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = StoreIdxRegOpcode.find(Lookup);
    if (OpcodePair == StoreIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(2); // Offset is in an implicit register
    MI.removeOperand(0);
  } else
    llvm_unreachable("Unknown offset type for StoreIdx");
  // MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAddImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = AddImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == AddImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAddCarryImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = AddCarryImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == AddCarryImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned Opcode;
  bool WithCarry = MI.getOpcode() == MC6809::AddCarry8Reg || MI.getOpcode() == MC6809::AddCarry16Reg;
  MachineOperand DstOp = MI.getOperand(3);
  MachineOperand SrcOp = MI.getOperand(WithCarry ? 5 : 4);

  Opcode = WithCarry ? MC6809::ADCRp : MC6809::ADDRp;
  MI.setDesc(Builder.getTII().get(Opcode));
  MI.removeOperand(5);
  if (WithCarry)
    MI.removeOperand(4);
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.addOperand(SrcOp);
  MI.addOperand(DstOp);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  unsigned Opcode;
  bool WithCarry = MI.getOpcode() == MC6809::SubBorrow8Reg || MI.getOpcode() == MC6809::SubBorrow16Reg;
  MachineOperand DstOp = MI.getOperand(3);
  MachineOperand SrcOp = MI.getOperand(WithCarry ? 5 : 4);

  Opcode = WithCarry ? MC6809::ADCRp : MC6809::ADDRp;
  MI.setDesc(Builder.getTII().get(Opcode));
  MI.removeOperand(5);
  if (WithCarry)
    MI.removeOperand(4);
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.addOperand(SrcOp);
  MI.addOperand(DstOp);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAddIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(4);
  MachineOperand OffsetOp = MI.getOperand(5);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = AddIdxImmOpcode.find(Lookup);
    if (OpcodePair == AddIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    if (OffsetSize == 0)
      MI.removeOperand(5);
    MI.removeOperand(4); // Index; will be re-added after Offset (above)
    MI.removeOperand(3); // Remove all implicit registers and CC Bits
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(IndexOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = AddIdxRegOpcode.find(Lookup);
    if (OpcodePair == AddIdxRegOpcode.end())
      llvm_unreachable("Unexpected AddIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(5); // Remove offset register; it is encoded in the opcode
    MI.removeOperand(3); // Remove implicit registers and CC Bits
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
  } else
    llvm_unreachable("Unknown offset type for AddIdx");
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAddCarryIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(5);
  MachineOperand OffsetOp = MI.getOperand(6);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = AddCarryIdxImmOpcode.find(Lookup);
    if (OpcodePair == AddCarryIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    if (OffsetSize == 0)
      MI.removeOperand(6);
    MI.removeOperand(5); // Index; will be re-added after Offset (above)
    MI.removeOperand(4); // Remove implicit registers and CC Bits
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(IndexOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = AddCarryIdxRegOpcode.find(Lookup);
    if (OpcodePair == AddCarryIdxRegOpcode.end())
      llvm_unreachable("Unexpected AddCarryIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(6); // Remove offset register; it is encoded in the opcode
    MI.removeOperand(4); // Remove implicit registers and CC Bits
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
  } else
    llvm_unreachable("Unknown offset type for AddCarryIdx");
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAdd32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand ValOp = MI.getOperand(4);
  int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int64_t ValLo = Val & 0xFFFF;
  int64_t ValHi = (Val >> 16) & 0xFFFF;
  MachineInstrBuilder AddLo, AddHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  AddLo = Builder.buildInstr(MC6809::ADDWi16)
  .addImm(ValLo);
  AddHi = Builder.buildInstr(MC6809::ADCDi16)
  .addImm(ValHi);
  AddLo->addImplicitDefUseOperands(*MI.getMF());
  AddHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddLo = "; AddLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddHi = "; AddHi->dump(););
}

void MC6809InstrInfo::expandSub32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand ValOp = MI.getOperand(4);
  int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int64_t ValLo = Val & 0xFFFF;
  int64_t ValHi = (Val >> 16) & 0xFFFF;
  MachineInstrBuilder SubLo, SubHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  SubLo = Builder.buildInstr(MC6809::SUBWi16)
  .addImm(ValLo);
  SubHi = Builder.buildInstr(MC6809::SBCDi16)
  .addImm(ValHi);
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
}

void MC6809InstrInfo::expandAdd32Idx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(4);
  MachineOperand OffsetOp = MI.getOperand(5);
  MachineInstrBuilder AddLo, AddHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    int64_t Offset = (OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue()) + 2; // Low word
    RegPlusOffsetLen LookupL{MC6809::AW, OffsetSize};
    auto OpcodePairL = AddIdxImmOpcode.find(LookupL);
    if (OpcodePairL == AddIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddLo = Builder.buildInstr(OpcodePairL->getSecond())
    .addImm(Offset)
    .addUse(IndexOp.getReg());
    Offset -= 2; // High word
    RegPlusOffsetLen LookupH{MC6809::AD, OffsetSize};
    auto OpcodePairH = AddCarryIdxImmOpcode.find(LookupH);
    if (OpcodePairH == AddCarryIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddHi = Builder.buildInstr(OpcodePairH->getSecond())
    .addImm(Offset)
    .addUse(IndexOp.getReg());
  } else if (OffsetOp.isReg()) {
    RegPlusReg LookupL{MC6809::AW, OffsetOp.getReg()};
    auto OpcodePairL = AddIdxRegOpcode.find(LookupL);
    if (OpcodePairL == AddIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddLo = Builder.buildInstr(OpcodePairL->getSecond())
                   .addUse(IndexOp.getReg());
    RegPlusReg LookupH{MC6809::AD, OffsetOp.getReg()};
    auto OpcodePairH = AddCarryIdxRegOpcode.find(LookupH);
    if (OpcodePairH == AddCarryIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddHi = Builder.buildInstr(OpcodePairH->getSecond())
                   .addUse(IndexOp.getReg());
  } else
    llvm_unreachable("Unknown offset type for AddCarryIdx");
  AddLo->addImplicitDefUseOperands(*MI.getMF());
  AddHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddLo = "; AddLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddHi = "; AddHi->dump(););
}

void MC6809InstrInfo::expandSub32Idx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(4);
  MachineOperand OffsetOp = MI.getOperand(5);
  MachineInstrBuilder SubLo, SubHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit subtract must have q as the target register");
  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    int64_t Offset = (OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue()) + 2; // Low word
    RegPlusOffsetLen LookupL{MC6809::AW, OffsetSize};
    auto OpcodePairL = SubIdxImmOpcode.find(LookupL);
    if (OpcodePairL == SubIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubLo = Builder.buildInstr(OpcodePairL->getSecond())
    .addImm(Offset)
    .addUse(IndexOp.getReg());
    Offset -= 2; // High word
    RegPlusOffsetLen LookupH{MC6809::AD, OffsetSize};
    auto OpcodePairH = SubBorrowIdxImmOpcode.find(LookupH);
    if (OpcodePairH == SubBorrowIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubHi = Builder.buildInstr(OpcodePairH->getSecond())
    .addImm(Offset)
    .addUse(IndexOp.getReg());
  } else if (OffsetOp.isReg()) {
    RegPlusReg LookupL{MC6809::AW, OffsetOp.getReg()};
    auto OpcodePairL = SubIdxRegOpcode.find(LookupL);
    if (OpcodePairL == SubIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubLo = Builder.buildInstr(OpcodePairL->getSecond())
                   .addUse(IndexOp.getReg());
    RegPlusReg LookupH{MC6809::AD, OffsetOp.getReg()};
    auto OpcodePairH = SubBorrowIdxRegOpcode.find(LookupH);
    if (OpcodePairH == SubBorrowIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubHi = Builder.buildInstr(OpcodePairH->getSecond())
                   .addUse(IndexOp.getReg());
  } else
    llvm_unreachable("Unknown offset type for SubBorrowIdx");
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
}

void MC6809InstrInfo::expandSubImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = SubImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == SubImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubBorrowImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = SubBorrowImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == SubBorrowImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubPop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = SubPopOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == SubPopOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addOperand(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSub32Pop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  RegPlusOffsetLen LookupL{MC6809::AW, 5};
  auto OpcodePairL = SubIdxImmOpcode.find(LookupL);
  if (OpcodePairL == SubIdxImmOpcode.end())
    llvm_unreachable("Unexpected register (Low).");
  auto SubLo = Builder.buildInstr(OpcodePairL->getSecond())
                     .addImm(2)
                     .addUse(MC6809::SS);
  RegPlusOffsetLen LookupH{MC6809::AD, 0};
  auto OpcodePairH = SubBorrowIdxImmOpcode.find(LookupH);
  if (OpcodePairH == SubBorrowIdxImmOpcode.end())
    llvm_unreachable("Unexpected register (High).");
  auto SubHi = Builder.buildInstr(OpcodePairH->getSecond())
                     .addUse(MC6809::SS);
  auto Pop = Builder.buildInstr(MC6809::LEASi_o5)
                   .addDef(MC6809::SS)
                   .addImm(4)
                   .addUse(MC6809::SS);
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Pop = "; Pop->dump(););
}

void MC6809InstrInfo::expandSubBorrowPop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = SubBorrowPopOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == SubBorrowPopOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addOperand(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(4);
  MachineOperand OffsetOp = MI.getOperand(5);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = SubIdxImmOpcode.find(Lookup);
    if (OpcodePair == SubIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(4);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = SubIdxRegOpcode.find(Lookup);
    if (OpcodePair == SubIdxRegOpcode.end())
      llvm_unreachable("Unexpected SubIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  } else
    llvm_unreachable("Unknown offset type for SubIdx");
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubBorrowIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(5);
  MachineOperand OffsetOp = MI.getOperand(6);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg.getReg(), OffsetSize};
    auto OpcodePair = SubBorrowIdxImmOpcode.find(Lookup);
    if (OpcodePair == SubBorrowIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(5);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = SubBorrowIdxRegOpcode.find(Lookup);
    if (OpcodePair == SubBorrowIdxRegOpcode.end())
      llvm_unreachable("Unexpected SubBorrowIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  } else
    llvm_unreachable("Unknown offset type for SubBorrowIdx");
  MI.removeOperand(4);
  MI.removeOperand(3);
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandCompareImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = CompareImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == CompareImmediateOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandCompareIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  MachineOperand SrcReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(1);
  MachineOperand OffsetOp = MI.getOperand(2);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{SrcReg.getReg(), OffsetSize};
    auto OpcodePair = CompareIdxImmOpcode.find(Lookup);
    if (OpcodePair == CompareIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(1);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{SrcReg.getReg(), OffsetOp.getReg()};
    auto OpcodePair = CompareIdxRegOpcode.find(Lookup);
    if (OpcodePair == CompareIdxRegOpcode.end())
      llvm_unreachable("Unexpected CompareIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  } else
    llvm_unreachable("Unknown offset type for CompareIdx");
  MI.removeOperand(0);
  MI.addOperand(IndexOp);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandComparePop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = ComparePopOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == ComparePopOpcode.end())
    llvm_unreachable("Unexpected register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.removeOperand(0);
  MI.addOperand(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

bool MC6809InstrInfo::reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 2);
  switch (Cond[0].getImm()) {
  case MC6809CC::RA:
  case MC6809CC::RN:
    Cond[0].setImm(MC6809CC::RA);
    break;
  case MC6809CC::GT:
    Cond[0].setImm(MC6809CC::LE);
    break;
  case MC6809CC::LE:
    Cond[0].setImm(MC6809CC::GT);
    break;
  case MC6809CC::GE:
    Cond[0].setImm(MC6809CC::LT);
    break;
  case MC6809CC::LT:
    Cond[0].setImm(MC6809CC::GE);
    break;
  case MC6809CC::EQ:
    Cond[0].setImm(MC6809CC::NE);
    break;
  case MC6809CC::NE:
    Cond[0].setImm(MC6809CC::EQ);
    break;
  case MC6809CC::HI:
    Cond[0].setImm(MC6809CC::LS);
    break;
  case MC6809CC::LS:
    Cond[0].setImm(MC6809CC::HI);
    break;
  case MC6809CC::HS:
    Cond[0].setImm(MC6809CC::LO);
    break;
  case MC6809CC::LO:
    Cond[0].setImm(MC6809CC::HS);
    break;
  case MC6809CC::MI:
    Cond[0].setImm(MC6809CC::PL);
    break;
  case MC6809CC::PL:
    Cond[0].setImm(MC6809CC::MI);
    break;
  case MC6809CC::VC:
    Cond[0].setImm(MC6809CC::VS);
    break;
  case MC6809CC::VS:
    Cond[0].setImm(MC6809CC::VC);
    break;
  default:
    llvm_unreachable("Unknown CC value");
  }
  // Success.
  return false;
}

std::pair<unsigned, unsigned>
MC6809InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<unsigned, const char *>>
MC6809InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {
      {MC6809::MO_LO, "lo"},
      {MC6809::MO_HI, "hi"},
      {MC6809::MO_HI_JT, "hi-jt"}};
  return Flags;
}
