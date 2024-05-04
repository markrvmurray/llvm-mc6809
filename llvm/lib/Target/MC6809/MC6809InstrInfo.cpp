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
#include "MC6809.h"

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

void llvm::MC6809::emitFrameOffset(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, const DebugLoc &DL, unsigned DestReg, unsigned SrcReg, StackOffset Offset, const TargetInstrInfo *TII, MachineInstr::MIFlag Flag) {
  int64_t Bytes = Offset.getFixed();

  if (Bytes || (!Offset && SrcReg != DestReg))
    BuildMI(MBB, MBBI, DL, TII->get(MC6809::LEAPtrAdd_Imm), DestReg).addReg(SrcReg).addImm(Bytes);
}

MC6809InstrInfo::MC6809InstrInfo()
    : MC6809GenInstrInfo(/*CFSetupOpcode=*/MC6809::ADJCALLSTACKDOWN,
                         /*CFDestroyOpcode=*/MC6809::ADJCALLSTACKUP) {

  LEAPtrAddImmOpcode = {
      {{MC6809::IX, -1}, MC6809::LEAXi_o16}, {{MC6809::IX, 0}, MC6809::LEAXi_o0}, {{MC6809::IX, 5}, MC6809::LEAXi_o5}, {{MC6809::IX, 8}, MC6809::LEAXi_o8}, {{MC6809::IX, 16}, MC6809::LEAXi_o16},
      {{MC6809::IY, -1}, MC6809::LEAYi_o16}, {{MC6809::IY, 0}, MC6809::LEAYi_o0}, {{MC6809::IY, 5}, MC6809::LEAYi_o5}, {{MC6809::IY, 8}, MC6809::LEAYi_o8}, {{MC6809::IY, 16}, MC6809::LEAYi_o16},
      {{MC6809::SU, -1}, MC6809::LEAUi_o16}, {{MC6809::SU, 0}, MC6809::LEAUi_o0}, {{MC6809::SU, 5}, MC6809::LEAUi_o5}, {{MC6809::SU, 8}, MC6809::LEAUi_o8}, {{MC6809::SU, 16}, MC6809::LEAUi_o16},
      {{MC6809::SS, -1}, MC6809::LEASi_o16}, {{MC6809::SS, 0}, MC6809::LEASi_o0}, {{MC6809::SS, 5}, MC6809::LEASi_o5}, {{MC6809::SS, 8}, MC6809::LEASi_o8}, {{MC6809::SS, 16}, MC6809::LEASi_o16},
  };
  LEAPtrAddRegOpcode = {
      {{MC6809::IX, MC6809::AA}, MC6809::LEAXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::LEAXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::LEAXi_oD}, {{MC6809::IX, MC6809::AE}, MC6809::LEAXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::LEAXi_oF},
      {{MC6809::IX, MC6809::AW}, MC6809::LEAXi_oW}, {{MC6809::IY, MC6809::AA}, MC6809::LEAYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::LEAYi_oB}, {{MC6809::IY, MC6809::AD}, MC6809::LEAYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::LEAYi_oE},
      {{MC6809::IY, MC6809::AF}, MC6809::LEAYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::LEAYi_oW}, {{MC6809::SU, MC6809::AA}, MC6809::LEAUi_oA}, {{MC6809::SU, MC6809::AB}, MC6809::LEAUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::LEAUi_oD},
      {{MC6809::SU, MC6809::AE}, MC6809::LEAUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::LEAUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::LEAUi_oW}, {{MC6809::SS, MC6809::AA}, MC6809::LEASi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::LEASi_oB},
      {{MC6809::SS, MC6809::AD}, MC6809::LEASi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::LEASi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::LEASi_oF}, {{MC6809::SS, MC6809::AW}, MC6809::LEASi_oW},
  };
  LoadImmediateOpcode = {
      {MC6809::AA, MC6809::LDAi8},  {MC6809::AB, MC6809::LDBi8},  {MC6809::AE, MC6809::LDEi8},  {MC6809::AF, MC6809::LDFi8},  {MC6809::AD, MC6809::LDDi16}, {MC6809::AW, MC6809::LDWi16},
      {MC6809::AQ, MC6809::LDQi32}, {MC6809::IX, MC6809::LDXi16}, {MC6809::IY, MC6809::LDYi16}, {MC6809::SU, MC6809::LDUi16}, {MC6809::SS, MC6809::LDSi16},
  };
  LoadIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::LDAi_o16}, {{MC6809::AA, 0}, MC6809::LDAi_o0},   {{MC6809::AA, 5}, MC6809::LDAi_o5},   {{MC6809::AA, 8}, MC6809::LDAi_o8},   {{MC6809::AA, 16}, MC6809::LDAi_o16}, {{MC6809::AB, -1}, MC6809::LDBi_o16},
      {{MC6809::AB, 0}, MC6809::LDBi_o0},   {{MC6809::AB, 5}, MC6809::LDBi_o5},   {{MC6809::AB, 8}, MC6809::LDBi_o8},   {{MC6809::AB, 16}, MC6809::LDBi_o16}, {{MC6809::AD, -1}, MC6809::LDDi_o16}, {{MC6809::AD, 0}, MC6809::LDDi_o0},
      {{MC6809::AD, 5}, MC6809::LDDi_o5},   {{MC6809::AD, 8}, MC6809::LDDi_o8},   {{MC6809::AD, 16}, MC6809::LDDi_o16}, {{MC6809::AE, -1}, MC6809::LDEi_o16}, {{MC6809::AE, 0}, MC6809::LDEi_o0},   {{MC6809::AE, 5}, MC6809::LDEi_o5},
      {{MC6809::AE, 8}, MC6809::LDEi_o8},   {{MC6809::AE, 16}, MC6809::LDEi_o16}, {{MC6809::AF, -1}, MC6809::LDFi_o16}, {{MC6809::AF, 0}, MC6809::LDFi_o0},   {{MC6809::AF, 5}, MC6809::LDFi_o5},   {{MC6809::AF, 8}, MC6809::LDFi_o8},
      {{MC6809::AF, 16}, MC6809::LDFi_o16}, {{MC6809::AW, -1}, MC6809::LDWi_o16}, {{MC6809::AW, 0}, MC6809::LDWi_o0},   {{MC6809::AW, 5}, MC6809::LDWi_o5},   {{MC6809::AW, 8}, MC6809::LDWi_o8},   {{MC6809::AW, 16}, MC6809::LDWi_o16},
      {{MC6809::AQ, -1}, MC6809::LDQi_o16}, {{MC6809::AQ, 0}, MC6809::LDQi_o0},   {{MC6809::AQ, 5}, MC6809::LDQi_o5},   {{MC6809::AQ, 8}, MC6809::LDQi_o8},   {{MC6809::AQ, 16}, MC6809::LDQi_o16}, {{MC6809::IX, -1}, MC6809::LDXi_o16},
      {{MC6809::IX, 0}, MC6809::LDXi_o0},   {{MC6809::IX, 5}, MC6809::LDXi_o5},   {{MC6809::IX, 8}, MC6809::LDXi_o8},   {{MC6809::IX, 16}, MC6809::LDXi_o16}, {{MC6809::IY, -1}, MC6809::LDYi_o16}, {{MC6809::IY, 0}, MC6809::LDYi_o0},
      {{MC6809::IY, 5}, MC6809::LDYi_o5},   {{MC6809::IY, 8}, MC6809::LDYi_o8},   {{MC6809::IY, 16}, MC6809::LDYi_o16}, {{MC6809::SU, -1}, MC6809::LDUi_o16}, {{MC6809::SU, 0}, MC6809::LDUi_o0},   {{MC6809::SU, 5}, MC6809::LDUi_o5},
      {{MC6809::SU, 8}, MC6809::LDUi_o8},   {{MC6809::SU, 16}, MC6809::LDUi_o16}, {{MC6809::SS, -1}, MC6809::LDSi_o16}, {{MC6809::SS, 0}, MC6809::LDSi_o0},   {{MC6809::SS, 5}, MC6809::LDSi_o5},   {{MC6809::SS, 8}, MC6809::LDSi_o8},
      {{MC6809::SS, 16}, MC6809::LDSi_o16},
  };
  LoadIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::LDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::LDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::LDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::LDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::LDAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::LDAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::LDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::LDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::LDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::LDBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::LDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::LDBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::LDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::LDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::LDDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::LDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::LDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::LDDi_oW}, {{MC6809::AE, MC6809::AA}, MC6809::LDEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::LDEi_oB},
      {{MC6809::AE, MC6809::AD}, MC6809::LDEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::LDEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::LDEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::LDEi_oW}, {{MC6809::AF, MC6809::AA}, MC6809::LDFi_oA},
      {{MC6809::AF, MC6809::AB}, MC6809::LDFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::LDFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::LDFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::LDFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::LDFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::LDWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::LDWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::LDWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::LDWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::LDWi_oF},
      {{MC6809::AW, MC6809::AW}, MC6809::LDWi_oW}, {{MC6809::AQ, MC6809::AA}, MC6809::LDQi_oA}, {{MC6809::AQ, MC6809::AB}, MC6809::LDQi_oB}, {{MC6809::AQ, MC6809::AD}, MC6809::LDQi_oD}, {{MC6809::AQ, MC6809::AE}, MC6809::LDQi_oE},
      {{MC6809::AQ, MC6809::AF}, MC6809::LDQi_oF}, {{MC6809::AQ, MC6809::AW}, MC6809::LDQi_oW}, {{MC6809::IX, MC6809::AA}, MC6809::LDXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::LDXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::LDXi_oD},
      {{MC6809::IX, MC6809::AE}, MC6809::LDXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::LDXi_oF}, {{MC6809::IX, MC6809::AW}, MC6809::LDXi_oW}, {{MC6809::IY, MC6809::AA}, MC6809::LDYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::LDYi_oB},
      {{MC6809::IY, MC6809::AD}, MC6809::LDYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::LDYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::LDYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::LDYi_oW}, {{MC6809::SU, MC6809::AA}, MC6809::LDUi_oA},
      {{MC6809::SU, MC6809::AB}, MC6809::LDUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::LDUi_oD}, {{MC6809::SU, MC6809::AE}, MC6809::LDUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::LDUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::LDUi_oW},
      {{MC6809::SS, MC6809::AA}, MC6809::LDSi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::LDSi_oB}, {{MC6809::SS, MC6809::AD}, MC6809::LDSi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::LDSi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::LDSi_oF},
      {{MC6809::SS, MC6809::AW}, MC6809::LDSi_oW},
  };
  StoreIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::STAi_o16}, {{MC6809::AA, 0}, MC6809::STAi_o0},   {{MC6809::AA, 5}, MC6809::STAi_o5},   {{MC6809::AA, 8}, MC6809::STAi_o8},   {{MC6809::AA, 16}, MC6809::STAi_o16}, {{MC6809::AB, -1}, MC6809::STBi_o16},
      {{MC6809::AB, 0}, MC6809::STBi_o0},   {{MC6809::AB, 5}, MC6809::STBi_o5},   {{MC6809::AB, 8}, MC6809::STBi_o8},   {{MC6809::AB, 16}, MC6809::STBi_o16}, {{MC6809::AD, -1}, MC6809::STDi_o16}, {{MC6809::AD, 0}, MC6809::STDi_o0},
      {{MC6809::AD, 5}, MC6809::STDi_o5},   {{MC6809::AD, 8}, MC6809::STDi_o8},   {{MC6809::AD, 16}, MC6809::STDi_o16}, {{MC6809::AE, -1}, MC6809::STEi_o16}, {{MC6809::AE, 0}, MC6809::STEi_o0},   {{MC6809::AE, 5}, MC6809::STEi_o5},
      {{MC6809::AE, 8}, MC6809::STEi_o8},   {{MC6809::AE, 16}, MC6809::STEi_o16}, {{MC6809::AF, -1}, MC6809::STFi_o16}, {{MC6809::AF, 0}, MC6809::STFi_o0},   {{MC6809::AF, 5}, MC6809::STFi_o5},   {{MC6809::AF, 8}, MC6809::STFi_o8},
      {{MC6809::AF, 16}, MC6809::STFi_o16}, {{MC6809::AW, -1}, MC6809::STWi_o16}, {{MC6809::AW, 0}, MC6809::STWi_o0},   {{MC6809::AW, 5}, MC6809::STWi_o5},   {{MC6809::AW, 8}, MC6809::STWi_o8},   {{MC6809::AW, 16}, MC6809::STWi_o16},
      {{MC6809::AQ, -1}, MC6809::STQi_o16}, {{MC6809::AQ, 0}, MC6809::STQi_o0},   {{MC6809::AQ, 5}, MC6809::STQi_o5},   {{MC6809::AQ, 8}, MC6809::STQi_o8},   {{MC6809::AQ, 16}, MC6809::STQi_o16}, {{MC6809::IX, -1}, MC6809::STXi_o16},
      {{MC6809::IX, 0}, MC6809::STXi_o0},   {{MC6809::IX, 5}, MC6809::STXi_o5},   {{MC6809::IX, 8}, MC6809::STXi_o8},   {{MC6809::IX, 16}, MC6809::STXi_o16}, {{MC6809::IY, -1}, MC6809::STYi_o16}, {{MC6809::IY, 0}, MC6809::STYi_o0},
      {{MC6809::IY, 5}, MC6809::STYi_o5},   {{MC6809::IY, 8}, MC6809::STYi_o8},   {{MC6809::IY, 16}, MC6809::STYi_o16}, {{MC6809::SU, -1}, MC6809::STUi_o16}, {{MC6809::SU, 0}, MC6809::STUi_o0},   {{MC6809::SU, 5}, MC6809::STUi_o5},
      {{MC6809::SU, 8}, MC6809::STUi_o8},   {{MC6809::SU, 16}, MC6809::STUi_o16}, {{MC6809::SS, -1}, MC6809::STSi_o16}, {{MC6809::SS, 0}, MC6809::STSi_o0},   {{MC6809::SS, 5}, MC6809::STSi_o5},   {{MC6809::SS, 8}, MC6809::STSi_o8},
      {{MC6809::SS, 16}, MC6809::STSi_o16},
  };
  StoreIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::STAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::STAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::STAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::STAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::STAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::STAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::STBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::STBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::STBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::STBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::STBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::STBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::STDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::STDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::STDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::STDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::STDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::STDi_oW}, {{MC6809::AE, MC6809::AA}, MC6809::STEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::STEi_oB},
      {{MC6809::AE, MC6809::AD}, MC6809::STEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::STEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::STEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::STEi_oW}, {{MC6809::AF, MC6809::AA}, MC6809::STFi_oA},
      {{MC6809::AF, MC6809::AB}, MC6809::STFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::STFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::STFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::STFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::STFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::STWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::STWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::STWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::STWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::STWi_oF},
      {{MC6809::AW, MC6809::AW}, MC6809::STWi_oW}, {{MC6809::AQ, MC6809::AA}, MC6809::STQi_oA}, {{MC6809::AQ, MC6809::AB}, MC6809::STQi_oB}, {{MC6809::AQ, MC6809::AD}, MC6809::STQi_oD}, {{MC6809::AQ, MC6809::AE}, MC6809::STQi_oE},
      {{MC6809::AQ, MC6809::AF}, MC6809::STQi_oF}, {{MC6809::AQ, MC6809::AW}, MC6809::STQi_oW}, {{MC6809::IX, MC6809::AA}, MC6809::STXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::STXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::STXi_oD},
      {{MC6809::IX, MC6809::AE}, MC6809::STXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::STXi_oF}, {{MC6809::IX, MC6809::AW}, MC6809::STXi_oW}, {{MC6809::IY, MC6809::AA}, MC6809::STYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::STYi_oB},
      {{MC6809::IY, MC6809::AD}, MC6809::STYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::STYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::STYi_oF}, {{MC6809::IY, MC6809::AW}, MC6809::STYi_oW}, {{MC6809::SU, MC6809::AA}, MC6809::STUi_oA},
      {{MC6809::SU, MC6809::AB}, MC6809::STUi_oB}, {{MC6809::SU, MC6809::AD}, MC6809::STUi_oD}, {{MC6809::SU, MC6809::AE}, MC6809::STUi_oE}, {{MC6809::SU, MC6809::AF}, MC6809::STUi_oF}, {{MC6809::SU, MC6809::AW}, MC6809::STUi_oW},
      {{MC6809::SS, MC6809::AA}, MC6809::STSi_oA}, {{MC6809::SS, MC6809::AB}, MC6809::STSi_oB}, {{MC6809::SS, MC6809::AD}, MC6809::STSi_oD}, {{MC6809::SS, MC6809::AE}, MC6809::STSi_oE}, {{MC6809::SS, MC6809::AF}, MC6809::STSi_oF},
      {{MC6809::SS, MC6809::AW}, MC6809::STSi_oW},
  };
  AddImmediateOpcode = {
      {{MC6809::AA}, MC6809::ADDAi8}, {{MC6809::AALSB}, MC6809::ADDAi8}, {{MC6809::AB}, MC6809::ADDBi8},  {{MC6809::ABLSB}, MC6809::ADDBi8},
      {{MC6809::AE}, MC6809::ADDEi8}, {{MC6809::AF}, MC6809::ADDFi8},    {{MC6809::AD}, MC6809::ADDDi16}, {{MC6809::AW}, MC6809::ADDWi16},
  };
  AddCarryImmediateOpcode = {
      {{MC6809::AA}, MC6809::ADCAi8}, {{MC6809::AALSB}, MC6809::ADCAi8}, {{MC6809::AB}, MC6809::ADCBi8}, {{MC6809::ABLSB}, MC6809::ADCBi8}, {{MC6809::AD}, MC6809::ADCDi16},
  };
  AddIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ADDAi_o16}, {{MC6809::AA, 0}, MC6809::ADDAi_o0},   {{MC6809::AA, 5}, MC6809::ADDAi_o5},   {{MC6809::AA, 8}, MC6809::ADDAi_o8},   {{MC6809::AA, 16}, MC6809::ADDAi_o16}, {{MC6809::AB, -1}, MC6809::ADDBi_o16},
      {{MC6809::AB, 0}, MC6809::ADDBi_o0},   {{MC6809::AB, 5}, MC6809::ADDBi_o5},   {{MC6809::AB, 8}, MC6809::ADDBi_o8},   {{MC6809::AB, 16}, MC6809::ADDBi_o16}, {{MC6809::AD, -1}, MC6809::ADDDi_o16}, {{MC6809::AD, 0}, MC6809::ADDDi_o0},
      {{MC6809::AD, 5}, MC6809::ADDDi_o5},   {{MC6809::AD, 8}, MC6809::ADDDi_o8},   {{MC6809::AD, 16}, MC6809::ADDDi_o16}, {{MC6809::AE, -1}, MC6809::ADDEi_o16}, {{MC6809::AE, 0}, MC6809::ADDEi_o0},   {{MC6809::AE, 5}, MC6809::ADDEi_o5},
      {{MC6809::AE, 8}, MC6809::ADDEi_o8},   {{MC6809::AE, 16}, MC6809::ADDEi_o16}, {{MC6809::AF, -1}, MC6809::ADDFi_o16}, {{MC6809::AF, 0}, MC6809::ADDFi_o0},   {{MC6809::AF, 5}, MC6809::ADDFi_o5},   {{MC6809::AF, 8}, MC6809::ADDFi_o8},
      {{MC6809::AF, 16}, MC6809::ADDFi_o16}, {{MC6809::AW, -1}, MC6809::ADDWi_o16}, {{MC6809::AW, 0}, MC6809::ADDWi_o0},   {{MC6809::AW, 5}, MC6809::ADDWi_o5},   {{MC6809::AW, 8}, MC6809::ADDWi_o8},   {{MC6809::AW, 16}, MC6809::ADDWi_o16},
  };
  AddCarryIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ADCAi_o16}, {{MC6809::AA, 0}, MC6809::ADCAi_o0}, {{MC6809::AA, 5}, MC6809::ADCAi_o5}, {{MC6809::AA, 8}, MC6809::ADCAi_o8}, {{MC6809::AA, 16}, MC6809::ADCAi_o16},
      {{MC6809::AB, -1}, MC6809::ADCBi_o16}, {{MC6809::AB, 0}, MC6809::ADCBi_o0}, {{MC6809::AB, 5}, MC6809::ADCBi_o5}, {{MC6809::AB, 8}, MC6809::ADCBi_o8}, {{MC6809::AB, 16}, MC6809::ADCBi_o16},
      {{MC6809::AD, -1}, MC6809::ADCDi_o16}, {{MC6809::AD, 0}, MC6809::ADCDi_o0}, {{MC6809::AD, 5}, MC6809::ADCDi_o5}, {{MC6809::AD, 8}, MC6809::ADCDi_o8}, {{MC6809::AD, 16}, MC6809::ADCDi_o16},
  };
  AddIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ADDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ADDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ADDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ADDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ADDAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::ADDAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::ADDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ADDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ADDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ADDBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::ADDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ADDBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::ADDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ADDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ADDDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::ADDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ADDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ADDDi_oW}, {{MC6809::AE, MC6809::AA}, MC6809::ADDEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::ADDEi_oB},
      {{MC6809::AE, MC6809::AD}, MC6809::ADDEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::ADDEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::ADDEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::ADDEi_oW}, {{MC6809::AF, MC6809::AA}, MC6809::ADDFi_oA},
      {{MC6809::AF, MC6809::AB}, MC6809::ADDFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::ADDFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::ADDFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::ADDFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::ADDFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::ADDWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::ADDWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::ADDWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::ADDWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::ADDWi_oF},
      {{MC6809::AW, MC6809::AW}, MC6809::ADDWi_oW},
  };
  AddCarryIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ADCAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ADCAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ADCAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ADCAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ADCAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::ADCAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::ADCBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ADCBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ADCBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ADCBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::ADCBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ADCBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::ADCDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ADCDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ADCDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::ADCDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ADCDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ADCDi_oW},
  };
  AddPopOpcode = {
      {{MC6809::AA}, MC6809::ADDAi_Inc1}, {{MC6809::AB}, MC6809::ADDBi_Inc1}, {{MC6809::AE}, MC6809::ADDEi_Inc1}, {{MC6809::AF}, MC6809::ADDFi_Inc1}, {{MC6809::AD}, MC6809::ADDDi_Inc2}, {{MC6809::AW}, MC6809::ADDWi_Inc2},
  };
  SubBorrowImmediateOpcode = {
      {{MC6809::AA}, MC6809::SBCAi8},
      {{MC6809::AB}, MC6809::SBCBi8},
      {{MC6809::AD}, MC6809::SBCDi16},
  };
  SubImmediateOpcode = {
      {{MC6809::AA}, MC6809::SUBAi8}, {{MC6809::AB}, MC6809::SUBBi8}, {{MC6809::AE}, MC6809::SUBEi8}, {{MC6809::AF}, MC6809::SUBFi8}, {{MC6809::AD}, MC6809::SUBDi16}, {{MC6809::AW}, MC6809::SUBWi16},
  };
  SubIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::SUBAi_o16}, {{MC6809::AA, 0}, MC6809::SUBAi_o0},   {{MC6809::AA, 5}, MC6809::SUBAi_o5},   {{MC6809::AA, 8}, MC6809::SUBAi_o8},   {{MC6809::AA, 16}, MC6809::SUBAi_o16}, {{MC6809::AB, -1}, MC6809::SUBBi_o16},
      {{MC6809::AB, 0}, MC6809::SUBBi_o0},   {{MC6809::AB, 5}, MC6809::SUBBi_o5},   {{MC6809::AB, 8}, MC6809::SUBBi_o8},   {{MC6809::AB, 16}, MC6809::SUBBi_o16}, {{MC6809::AD, -1}, MC6809::SUBDi_o16}, {{MC6809::AD, 0}, MC6809::SUBDi_o0},
      {{MC6809::AD, 5}, MC6809::SUBDi_o5},   {{MC6809::AD, 8}, MC6809::SUBDi_o8},   {{MC6809::AD, 16}, MC6809::SUBDi_o16}, {{MC6809::AE, -1}, MC6809::SUBEi_o16}, {{MC6809::AE, 0}, MC6809::SUBEi_o0},   {{MC6809::AE, 5}, MC6809::SUBEi_o5},
      {{MC6809::AE, 8}, MC6809::SUBEi_o8},   {{MC6809::AE, 16}, MC6809::SUBEi_o16}, {{MC6809::AF, -1}, MC6809::SUBFi_o16}, {{MC6809::AF, 0}, MC6809::SUBFi_o0},   {{MC6809::AF, 5}, MC6809::SUBFi_o5},   {{MC6809::AF, 8}, MC6809::SUBFi_o8},
      {{MC6809::AF, 16}, MC6809::SUBFi_o16}, {{MC6809::AW, -1}, MC6809::SUBWi_o16}, {{MC6809::AW, 0}, MC6809::SUBWi_o0},   {{MC6809::AW, 5}, MC6809::SUBWi_o5},   {{MC6809::AW, 8}, MC6809::SUBWi_o8},   {{MC6809::AW, 16}, MC6809::SUBWi_o16},
  };
  SubIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::SUBAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::SUBAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::SUBAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::SUBAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::SUBAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::SUBAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::SUBBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::SUBBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::SUBBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::SUBBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::SUBBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::SUBBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::SUBDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::SUBDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::SUBDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::SUBDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::SUBDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::SUBDi_oW}, {{MC6809::AE, MC6809::AA}, MC6809::SUBEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::SUBEi_oB},
      {{MC6809::AE, MC6809::AD}, MC6809::SUBEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::SUBEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::SUBEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::SUBEi_oW}, {{MC6809::AF, MC6809::AA}, MC6809::SUBFi_oA},
      {{MC6809::AF, MC6809::AB}, MC6809::SUBFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::SUBFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::SUBFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::SUBFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::SUBFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::SUBWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::SUBWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::SUBWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::SUBWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::SUBWi_oF},
      {{MC6809::AW, MC6809::AW}, MC6809::SUBWi_oW},
  };
  SubPullOpcode = {
      {{MC6809::AA}, MC6809::SUBAi_Inc1}, {{MC6809::AB}, MC6809::SUBBi_Inc1}, {{MC6809::AE}, MC6809::SUBEi_Inc1}, {{MC6809::AF}, MC6809::SUBFi_Inc1}, {{MC6809::AD}, MC6809::SUBDi_Inc2}, {{MC6809::AW}, MC6809::SUBWi_Inc2},
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
      {{MC6809::AA, MC6809::AA}, MC6809::SBCAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::SBCAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::SBCAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::SBCAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::SBCAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::SBCAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::SBCBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::SBCBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::SBCBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::SBCBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::SBCBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::SBCBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::SBCDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::SBCDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::SBCDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::SBCDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::SBCDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::SBCDi_oW},
  };
#if 0
  SubBorrowPopOpcode = {
      {{MC6809::AA}, MC6809::SBCAi_Inc1},
      {{MC6809::AB}, MC6809::SBCBi_Inc1},
      {{MC6809::AD}, MC6809::SBCDi_Inc2},
  };
#endif
  CompareImmediateOpcode = {
      {{MC6809::AA}, MC6809::CMPAi8}, {{MC6809::AB}, MC6809::CMPBi8}, {{MC6809::AE}, MC6809::CMPEi8}, {{MC6809::AF}, MC6809::CMPFi8}, {{MC6809::AD}, MC6809::CMPDi16}, {{MC6809::AW}, MC6809::CMPWi16},
  };
  CompareIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::CMPAi_o16}, {{MC6809::AA, 0}, MC6809::CMPAi_o0},   {{MC6809::AA, 5}, MC6809::CMPAi_o5},   {{MC6809::AA, 8}, MC6809::CMPAi_o8},   {{MC6809::AA, 16}, MC6809::CMPAi_o16}, {{MC6809::AB, -1}, MC6809::CMPBi_o16},
      {{MC6809::AB, 0}, MC6809::CMPBi_o0},   {{MC6809::AB, 5}, MC6809::CMPBi_o5},   {{MC6809::AB, 8}, MC6809::CMPBi_o8},   {{MC6809::AB, 16}, MC6809::CMPBi_o16}, {{MC6809::AD, -1}, MC6809::CMPDi_o16}, {{MC6809::AD, 0}, MC6809::CMPDi_o0},
      {{MC6809::AD, 5}, MC6809::CMPDi_o5},   {{MC6809::AD, 8}, MC6809::CMPDi_o8},   {{MC6809::AD, 16}, MC6809::CMPDi_o16}, {{MC6809::AE, -1}, MC6809::CMPEi_o16}, {{MC6809::AE, 0}, MC6809::CMPEi_o0},   {{MC6809::AE, 5}, MC6809::CMPEi_o5},
      {{MC6809::AE, 8}, MC6809::CMPEi_o8},   {{MC6809::AE, 16}, MC6809::CMPEi_o16}, {{MC6809::AF, -1}, MC6809::CMPFi_o16}, {{MC6809::AF, 0}, MC6809::CMPFi_o0},   {{MC6809::AF, 5}, MC6809::CMPFi_o5},   {{MC6809::AF, 8}, MC6809::CMPFi_o8},
      {{MC6809::AF, 16}, MC6809::CMPFi_o16}, {{MC6809::AW, -1}, MC6809::CMPWi_o16}, {{MC6809::AW, 0}, MC6809::CMPWi_o0},   {{MC6809::AW, 5}, MC6809::CMPWi_o5},   {{MC6809::AW, 8}, MC6809::CMPWi_o8},   {{MC6809::AW, 16}, MC6809::CMPWi_o16},
  };
  CompareIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::CMPAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::CMPAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::CMPAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::CMPAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::CMPAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::CMPAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::CMPBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::CMPBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::CMPBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::CMPBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::CMPBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::CMPBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::CMPDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::CMPDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::CMPDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::CMPDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::CMPDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::CMPDi_oW}, {{MC6809::AE, MC6809::AA}, MC6809::CMPEi_oA}, {{MC6809::AE, MC6809::AB}, MC6809::CMPEi_oB},
      {{MC6809::AE, MC6809::AD}, MC6809::CMPEi_oD}, {{MC6809::AE, MC6809::AE}, MC6809::CMPEi_oE}, {{MC6809::AE, MC6809::AF}, MC6809::CMPEi_oF}, {{MC6809::AE, MC6809::AW}, MC6809::CMPEi_oW}, {{MC6809::AF, MC6809::AA}, MC6809::CMPFi_oA},
      {{MC6809::AF, MC6809::AB}, MC6809::CMPFi_oB}, {{MC6809::AF, MC6809::AD}, MC6809::CMPFi_oD}, {{MC6809::AF, MC6809::AE}, MC6809::CMPFi_oE}, {{MC6809::AF, MC6809::AF}, MC6809::CMPFi_oF}, {{MC6809::AF, MC6809::AW}, MC6809::CMPFi_oW},
      {{MC6809::AW, MC6809::AA}, MC6809::CMPWi_oA}, {{MC6809::AW, MC6809::AB}, MC6809::CMPWi_oB}, {{MC6809::AW, MC6809::AD}, MC6809::CMPWi_oD}, {{MC6809::AW, MC6809::AE}, MC6809::CMPWi_oE}, {{MC6809::AW, MC6809::AF}, MC6809::CMPWi_oF},
      {{MC6809::AW, MC6809::AW}, MC6809::CMPWi_oW},
  };
#if 0
  ComparePopOpcode = {
      {{MC6809::AA}, MC6809::CMPAi_Inc1},
      {{MC6809::AB}, MC6809::CMPBi_Inc1},
      {{MC6809::AE}, MC6809::CMPEi_Inc1},
      {{MC6809::AF}, MC6809::CMPFi_Inc1},
      {{MC6809::AD}, MC6809::CMPDi_Inc2},
      {{MC6809::AW}, MC6809::CMPWi_Inc2},
  };
#endif
  CompareImmediateOpcode = {
      {{MC6809::AA}, MC6809::CMPAi8}, {{MC6809::AB}, MC6809::CMPBi8}, {{MC6809::AE}, MC6809::CMPEi8}, {{MC6809::AF}, MC6809::CMPFi8}, {{MC6809::AD}, MC6809::CMPDi16}, {{MC6809::AW}, MC6809::CMPWi16},
  };
  TestRegOpcode = {
      {{MC6809::AA}, MC6809::TSTAa}, {{MC6809::AB}, MC6809::TSTBa}, {{MC6809::AE}, MC6809::TSTEa}, {{MC6809::AF}, MC6809::TSTFa}, {{MC6809::AD}, MC6809::TSTDa}, {{MC6809::AW}, MC6809::TSTWa},
  };
  ANDImmediateOpcode = {
      {{MC6809::AA}, MC6809::ANDAi8}, {{MC6809::AALSB}, MC6809::ANDAi8}, {{MC6809::AB}, MC6809::ANDBi8}, {{MC6809::ABLSB}, MC6809::ANDBi8}, {{MC6809::AD}, MC6809::ANDDi16},
  };
  ANDIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ANDAi_o16}, {{MC6809::AA, 0}, MC6809::ANDAi_o0}, {{MC6809::AA, 5}, MC6809::ANDAi_o5}, {{MC6809::AA, 8}, MC6809::ANDAi_o8}, {{MC6809::AA, 16}, MC6809::ANDAi_o16},
      {{MC6809::AB, -1}, MC6809::ANDBi_o16}, {{MC6809::AB, 0}, MC6809::ANDBi_o0}, {{MC6809::AB, 5}, MC6809::ANDBi_o5}, {{MC6809::AB, 8}, MC6809::ANDBi_o8}, {{MC6809::AB, 16}, MC6809::ANDBi_o16},
      {{MC6809::AD, -1}, MC6809::ANDDi_o16}, {{MC6809::AD, 0}, MC6809::ANDDi_o0}, {{MC6809::AD, 5}, MC6809::ANDDi_o5}, {{MC6809::AD, 8}, MC6809::ANDDi_o8}, {{MC6809::AD, 16}, MC6809::ANDDi_o16},
  };
  ANDIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ANDAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ANDAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ANDAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ANDAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ANDAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::ANDAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::ANDBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ANDBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ANDBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ANDBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::ANDBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ANDBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::ANDDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ANDDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ANDDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::ANDDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ANDDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ANDDi_oW},
  };
  ORImmediateOpcode = {
      {{MC6809::AA}, MC6809::ORAi8}, {{MC6809::AALSB}, MC6809::ORAi8}, {{MC6809::AB}, MC6809::ORBi8}, {{MC6809::ABLSB}, MC6809::ORBi8}, {{MC6809::AD}, MC6809::ORDi16},
  };
  ORIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::ORAi_o16}, {{MC6809::AA, 0}, MC6809::ORAi_o0}, {{MC6809::AA, 5}, MC6809::ORAi_o5}, {{MC6809::AA, 8}, MC6809::ORAi_o8}, {{MC6809::AA, 16}, MC6809::ORAi_o16},
      {{MC6809::AB, -1}, MC6809::ORBi_o16}, {{MC6809::AB, 0}, MC6809::ORBi_o0}, {{MC6809::AB, 5}, MC6809::ORBi_o5}, {{MC6809::AB, 8}, MC6809::ORBi_o8}, {{MC6809::AB, 16}, MC6809::ORBi_o16},
      {{MC6809::AD, -1}, MC6809::ORDi_o16}, {{MC6809::AD, 0}, MC6809::ORDi_o0}, {{MC6809::AD, 5}, MC6809::ORDi_o5}, {{MC6809::AD, 8}, MC6809::ORDi_o8}, {{MC6809::AD, 16}, MC6809::ORDi_o16},
  };
  ORIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::ORAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::ORAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::ORAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::ORAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::ORAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::ORAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::ORBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::ORBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::ORBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::ORBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::ORBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::ORBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::ORDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::ORDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::ORDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::ORDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::ORDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::ORDi_oW},
  };
  XORImmediateOpcode = {
      {{MC6809::AA}, MC6809::EORAi8},
      {{MC6809::AB}, MC6809::EORBi8},
      {{MC6809::AD}, MC6809::EORDi16},
  };
  XORIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::EORAi_o16}, {{MC6809::AA, 0}, MC6809::EORAi_o0}, {{MC6809::AA, 5}, MC6809::EORAi_o5}, {{MC6809::AA, 8}, MC6809::EORAi_o8}, {{MC6809::AA, 16}, MC6809::EORAi_o16},
      {{MC6809::AB, -1}, MC6809::EORBi_o16}, {{MC6809::AB, 0}, MC6809::EORBi_o0}, {{MC6809::AB, 5}, MC6809::EORBi_o5}, {{MC6809::AB, 8}, MC6809::EORBi_o8}, {{MC6809::AB, 16}, MC6809::EORBi_o16},
      {{MC6809::AD, -1}, MC6809::EORDi_o16}, {{MC6809::AD, 0}, MC6809::EORDi_o0}, {{MC6809::AD, 5}, MC6809::EORDi_o5}, {{MC6809::AD, 8}, MC6809::EORDi_o8}, {{MC6809::AD, 16}, MC6809::EORDi_o16},
  };
  XORIdxRegOpcode = {
      {{MC6809::AA, MC6809::AA}, MC6809::EORAi_oA}, {{MC6809::AA, MC6809::AB}, MC6809::EORAi_oB}, {{MC6809::AA, MC6809::AD}, MC6809::EORAi_oD}, {{MC6809::AA, MC6809::AE}, MC6809::EORAi_oE}, {{MC6809::AA, MC6809::AF}, MC6809::EORAi_oF},
      {{MC6809::AA, MC6809::AW}, MC6809::EORAi_oW}, {{MC6809::AB, MC6809::AA}, MC6809::EORBi_oA}, {{MC6809::AB, MC6809::AB}, MC6809::EORBi_oB}, {{MC6809::AB, MC6809::AD}, MC6809::EORBi_oD}, {{MC6809::AB, MC6809::AE}, MC6809::EORBi_oE},
      {{MC6809::AB, MC6809::AF}, MC6809::EORBi_oF}, {{MC6809::AB, MC6809::AW}, MC6809::EORBi_oW}, {{MC6809::AD, MC6809::AA}, MC6809::EORDi_oA}, {{MC6809::AD, MC6809::AB}, MC6809::EORDi_oB}, {{MC6809::AD, MC6809::AD}, MC6809::EORDi_oD},
      {{MC6809::AD, MC6809::AE}, MC6809::EORDi_oE}, {{MC6809::AD, MC6809::AF}, MC6809::EORDi_oF}, {{MC6809::AD, MC6809::AW}, MC6809::EORDi_oW},
  };
}

bool MC6809InstrInfo::isJumpTableBranch(const MachineBasicBlock::instr_iterator &I) const { return false; }

bool MC6809InstrInfo::isIndirBranch(const MachineBasicBlock::instr_iterator &I) const { return false; }

bool MC6809InstrInfo::isCondBranch(const MachineBasicBlock::instr_iterator &I) const {
  if (I->isBranch()) {
    if (I->getOpcode() == MC6809::Bbc || I->getOpcode() == MC6809::LBlbc)
      return I->getOperand(0).getImm() != MC6809CC::RA;
    return I->isConditionalBranch();
  }
  return false;
}

bool MC6809InstrInfo::isUnCondBranch(const MachineBasicBlock::instr_iterator &I) const {
  if (I->isBranch()) {
    if (I->getOpcode() == MC6809::Bbc || I->getOpcode() == MC6809::LBlbc)
      return I->getOperand(0).getImm() == MC6809CC::RA;
    return I->isUnconditionalBranch();
  }
  return false;
}

MachineBasicBlock *MC6809InstrInfo::getBB(const MachineBasicBlock::instr_iterator &I) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : I = "; I->dump(););
  if (I->getOpcode() == TargetOpcode::G_BR || I->getOpcode() == MC6809::BranchRelative || I->getOpcode() == MC6809::LongBranchRelative || I->getOpcode() == MC6809::JMPe || I->getOpcode() == MC6809::JMPi_o16PC)
    return I->getOperand(0).getMBB();
  if (I->getOpcode() == TargetOpcode::G_BRCOND || I->getOpcode() == MC6809::ConditionalBranchRelative || I->getOpcode() == MC6809::ConditionalLongBranchRelative || I->getOpcode() == MC6809::Bbc || I->getOpcode() == MC6809::LBlbc)
    return I->getOperand(1).getMBB();
  llvm_unreachable("Unable to handle opcode. Please fix me!");
}

Register MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : No Match, exiting\n";);
    break;
  case MC6809::LDAi_o0:
  case MC6809::LDBi_o0:
  case MC6809::LDDi_o0:
  case MC6809::LDEi_o0:
  case MC6809::LDFi_o0:
  case MC6809::LDWi_o0:
  case MC6809::LDQi_o0:
  case MC6809::LDXi_o0:
  case MC6809::LDYi_o0:
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
  case MC6809::Load_i8_Idx_Imm:
  case MC6809::Load_i8_Idx_Reg8:
  case MC6809::Load_i8_Idx_Reg16:
  case MC6809::Load_i16_Idx_Imm:
  case MC6809::Load_i16_Idx_Reg8:
  case MC6809::Load_i16_Idx_Reg16:
  case MC6809::Load_iPtr_Idx_Imm:
  case MC6809::Load_iPtr_Idx_Reg8:
  case MC6809::Load_iPtr_Idx_Reg16:
  case MC6809::Load_i32_Idx_Imm:
  case MC6809::Load_i32_Idx_Reg8:
  case MC6809::Load_i32_Idx_Reg16:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match - checking further\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(0).getSubReg() (== 0) = " << MI.getOperand(0).getSubReg() << "\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(1).isFI() = " << MI.getOperand(1).isFI() << "\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(2).isImm() = " << MI.getOperand(2).isImm() << "\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(2).getImm() (== 0) = " << MI.getOperand(2).getImm() << "\n";);
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() && MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match further\n";);
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : No Match\n";);
    break;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : 0 : MI = "; MI.dump(););
  return 0;
}

Register MC6809InstrInfo::isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : No Match, exiting\n";);
    break;
  case MC6809::STAi_o0:
  case MC6809::STBi_o0:
  case MC6809::STDi_o0:
  case MC6809::STEi_o0:
  case MC6809::STFi_o0:
  case MC6809::STWi_o0:
  case MC6809::STQi_o0:
  case MC6809::STXi_o0:
  case MC6809::STYi_o0:
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
  case MC6809::Store_i8_Idx_Imm:
  case MC6809::Store_i8_Idx_Reg8:
  case MC6809::Store_i8_Idx_Reg16:
  case MC6809::Store_i16_Idx_Imm:
  case MC6809::Store_i16_Idx_Reg8:
  case MC6809::Store_i16_Idx_Reg16:
  case MC6809::Store_iPtr_Idx_Imm:
  case MC6809::Store_iPtr_Idx_Reg8:
  case MC6809::Store_iPtr_Idx_Reg16:
  case MC6809::Store_i32_Idx_Imm:
  case MC6809::Store_i32_Idx_Reg8:
  case MC6809::Store_i32_Idx_Reg16:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match - checking further\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(0).getSubReg() (== 0) = " << MI.getOperand(0).getSubReg() << "\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(1).isFI() = " << MI.getOperand(1).isFI() << "\n";);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(2).isImm() = " << MI.getOperand(2).isImm() << "\n";);
    // LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match : MI.getOperand(2).getImm() (== 0) = " << MI.getOperand(2).getImm() << "\n";);
    if (MI.getOperand(0).getSubReg() == 0 && MI.getOperand(1).isFI() && MI.getOperand(2).isImm() && MI.getOperand(2).getImm() == 0) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Match further\n";);
      FrameIndex = MI.getOperand(1).getIndex();
      return MI.getOperand(0).getReg();
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : No Match\n";);
    break;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : 0 : MI = "; MI.dump(););
  return 0;
}

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator I, Register DestReg, unsigned SubIdx, const MachineInstr &Orig, const TargetRegisterInfo &TRI) const {
  auto opcode = Orig.getOpcode();
  if (opcode == MC6809::Load_i8_Imm || opcode == MC6809::Load_i16_Imm || opcode == MC6809::Load_i32_Imm) {
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->removeOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(opcode));
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
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

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

// XXXX FixMe: MarkM. Branch offset relaxation should cover for all sins committed, but only
// once we have lowered to non-pseudo instructions.
bool MC6809InstrInfo::isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const {
#if 0
  switch (BranchOpc) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::BranchRelative:
  case MC6809::LongBranchRelative:
  case MC6809::Bbc:
  case MC6809::JMPi_o8PC:
    // BRA range is [-128,127] starting from the PC location after the
    // instruction, which is two bytes after the start of the instruction.
    return -126 <= BrOffset && BrOffset <= 129;
  case MC6809::ConditionalBranchRelative:
  case MC6809::ConditionalLongBranchRelative:
  case MC6809::LBRAlb:
  case MC6809::LBlbc:
  case MC6809::JMPi_o16PC:
    // LBRA, JMP, JSR range is [-32768,32767], covering the full memory range.
    return true;
  }
#else
  return true;
#endif
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

MachineBasicBlock *MC6809InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  switch (MI.getOpcode()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bad branch opcode : MI = "; MI.dump(););
    llvm_unreachable("Bad branch opcode");
  case MC6809::JMPi_o8PC:
  case MC6809::JMPi_o16PC:
  case MC6809::BRAb:
  case MC6809::LBRAlb:
  case MC6809::LongBranchRelative:
  case MC6809::BranchRelative:
  case MC6809::JumpRelative:
  case MC6809::LongJumpRelative:
  case TargetOpcode::G_BR:
    return MI.getOperand(0).getMBB();
  case MC6809::Bbc:
  case MC6809::LBlbc:
  case MC6809::ConditionalBranchRelative:
  case MC6809::ConditionalLongBranchRelative:
  case TargetOpcode::G_BRCOND:
    return MI.getOperand(1).getMBB();
  case MC6809::JumpIndir:
  case TargetOpcode::G_BRINDIRECT:
    return nullptr;
  }
}

// Branch analysis.
// Cond vector output format:
//   0 elements indicates an unconditional branch.
//   1 element indicates a conditional branch; the element is
//     the condition to check.
bool MC6809InstrInfo::analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB, SmallVectorImpl<MachineOperand> &Cond, bool AllowModify) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  // Start from the bottom of the block and work up, examining the
  // terminator instructions.
  MachineBasicBlock::iterator I = MBB.end(), UnCondBrIter = I;
  while (I != MBB.begin()) {
    --I;
    if (I->isDebugValue())
      continue;

    // Working from the bottom, when we see a non-terminator instruction, we're
    // done.
    if (!isUnpredicatedTerminator(*I))
      break;

    // A terminator that isn't a branch can't easily be handled by this
    // analysis.
    if (!I->isBranch()) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Not a branch : Exit : return = true\n";);
      return true;
    }

    // Cannot handle branches that don't branch to a block.
    if (!I->getOperand(0).isMBB()) {
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Not branching to a block : Exit : return = true\n";);
      return true;
    }

    // Handle unconditional branches.
    if (I->getNumExplicitOperands() == 1) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(0).getMBB();
        continue;
      }

      // If the block has any instructions after an unconditional branch, delete them.
      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();
      Cond.clear();
      FBB = nullptr;

      // Delete the unconditional branch if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(0).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = I;
        continue;
      }

      // TBB is used to indicate the unconditional destination.
      TBB = I->getOperand(0).getMBB();
      continue;
    }

    // Handle conditional branches.
    assert(I->getNumExplicitOperands() == 2 && "Invalid conditional branch");
    MC6809CC::CondCode CC = I->getNumExplicitOperands() < 2 ? MC6809CC::INVALID : MC6809CC::CondCode(I->getOperand(1).getImm());

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(0).getMBB();
      if (CC != MC6809CC::INVALID && AllowModify && UnCondBrIter != MBB.end() && MBB.isLayoutSuccessor(TargetBB)) {
        // If we can modify the code and it ends in something like:
        //
        //     jCC L1
        //     jmp L2
        //   L1:
        //     ...
        //   L2:
        //
        // Then we can change this to:
        //
        //     jnCC L2
        //   L1:
        //     ...
        //   L2:
        //
        // Which is a bit more efficient.
        // We conditionally jump to the fall-through block.
        CC = getOppositeCondition(CC);
        MachineBasicBlock::iterator OldInst = I;

        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(MC6809::Bbc)).addImm(CC).addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(MC6809::BRAb)).addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(0).getMBB();
      Cond.push_back(MachineOperand::CreateImm(CC));
      continue;
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return = true\n";);
    return true;
  }

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return = false\n";);
  return false;
}

unsigned MC6809InstrInfo::removeBranch(MachineBasicBlock &MBB, int *BytesRemoved) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  MachineBasicBlock::iterator I = MBB.end();
  int Bytes = 0;
  unsigned Count = 0;

  while (I != MBB.begin()) {
    --I;
    if (I->isDebugValue())
      continue;
    if (!I->isBranch())
      break;
    // Remove the branch.
    Bytes += getInstSizeInBytes(*I);
    I->eraseFromParent();
    I = MBB.end();
    ++Count;
  }

  if (BytesRemoved)
    *BytesRemoved = Bytes;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return Count = " << Count << "\n";);
  return Count;
}

unsigned MC6809InstrInfo::insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  // Shouldn't be a fall through.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  assert(TBB && "InsertBranch must not be told to insert a fallthrough");
  assert(Cond.size() <= 1 && "MC6809 branch conditions have one component!");
  int Bytes = 0;
  unsigned Count = 0;

  if (Cond.empty()) {
    // Unconditional branch?
    assert(!FBB && "Unconditional branch with multiple successors!");
    Bytes += getInstSizeInBytes(*BuildMI(&MBB, DL, get(MC6809::BRAb)).addMBB(TBB));
    ++Count;
  } else {
    // Conditional branch.
    Bytes += getInstSizeInBytes(*BuildMI(&MBB, DL, get(MC6809::Bbc)).add(Cond[0]).addMBB(TBB));
    ++Count;

    // If FBB is null, it is implied to be a fall-through block.
    if (FBB) {
      // Two-way Conditional branch. Insert the second branch.
      Bytes += getInstSizeInBytes(*BuildMI(&MBB, DL, get(MC6809::BRAb)).addMBB(FBB));
      ++Count;
    }
  }

  if (BytesAdded)
    *BytesAdded = Bytes;
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : return Count = " << Count << "\n";);
  return Count;
}

void MC6809InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB, MachineBasicBlock &NewDestBB, MachineBasicBlock &RestoreBB, const DebugLoc &DL, int64_t BrOffset, RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.
  // XXXX: FIXME: MarkM - this process is a crock; LBRA should always work.
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : FixMe: MarkM - this process is a crock\n";);

  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  Builder.buildInstr(MC6809::LBRAlb).addMBB(&NewDestBB);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
  llvm_unreachable("This process is a crock - fix me!");
}

void MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, MCRegister DestReg, MCRegister SrcReg, bool KillSrc) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI->dump(););
  MachineIRBuilder Builder(MBB, MI);
  if (DestReg == SrcReg)
    return;

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 1\n";);
  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() && !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 2\n";);
  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) { return IsClass(DestReg, Dest) && IsClass(SrcReg, Src); };

  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 3\n";);
  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)\n";);
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)\n";);
    if (AreClasses(MC6809::ADcRegClass, MC6809::ABcRegClass) || AreClasses(MC6809::AWcRegClass, MC6809::AFcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)\n";);
    if (AreClasses(MC6809::ABcRegClass, MC6809::ADcRegClass) || AreClasses(MC6809::AFcRegClass, MC6809::AWcRegClass))
      return;
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__
                      << " : AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) || "
                         "AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)\n";);
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass)\n";);
    // XXXX FixMe Markm need to mask out the non-arithmetic bits?
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)\n";);
    // XXXX FixMe Markm need to mask out the non-arithmetic bits?
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)\n";);
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::CCFlagRegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::BIT1RegClass, MC6809::CCFlagRegClass)\n";);
    const TargetRegisterInfo *TRI = Builder.getMRI()->getTargetRegisterInfo();
    DestReg = TRI->getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    MachineBasicBlock::iterator B, E;
    B = Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::CC);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bundle enter\n";);
    switch (SrcReg) {
    case MC6809::C:
      E = Builder.buildInstr(MC6809::AND_i8_Imm).addDef(DestReg).addUse(DestReg).addImm(0xFE);
      break;
    case MC6809::V:
      Builder.buildInstr(MC6809::LSR_i8_Reg).addDef(DestReg).addImm(1);
      E = Builder.buildInstr(MC6809::AND_i8_Imm).addDef(DestReg).addUse(DestReg).addImm(0xFD);
      break;
    case MC6809::Z:
      Builder.buildInstr(MC6809::LSR_i8_Reg).addDef(DestReg).addImm(2);
      E = Builder.buildInstr(MC6809::AND_i8_Imm).addDef(DestReg).addUse(DestReg).addImm(0xFB);
      break;
    case MC6809::N:
      Builder.buildInstr(MC6809::LSR_i8_Reg).addDef(DestReg).addImm(3);
      E = Builder.buildInstr(MC6809::AND_i8_Imm).addDef(DestReg).addUse(DestReg).addImm(0xF7);
      break;
    default:
      llvm_unreachable("Unexpected CC bit copy out.");
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bundle exit\n";);
    auto Bundler = MIBundleBuilder(MBB, B, ++E);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bundle built\n";);
    LLVM_DEBUG(for (auto &I
                    : Bundler) {
      dbgs() << "OINQUE DEBUG " << __func__ << " : bundle : I = ";
      I.dump();
    });
  } else if (AreClasses(MC6809::CCFlagRegClass, MC6809::BIT1RegClass)) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AreClasses(MC6809::CCFlagRegClass, MC6809::BIT1RegClass)\n";);
    SrcReg = SrcReg == MC6809::AALSB ? MC6809::AA : MC6809::AB;
    switch (DestReg) {
    case MC6809::C:
      Builder.buildInstr(MC6809::ANDCC_Imm).addDef(MC6809::CC).addUse(MC6809::CC).addImm(0xFE);
      Builder.buildInstr(MC6809::Push_i8).addUse(MC6809::CC);
      Builder.buildInstr(MC6809::AND_i8_Imm).addDef(SrcReg).addUse(SrcReg).addImm(0xFE);
      Builder.buildInstr(MC6809::OR_i8_Dec_Idx).addDef(SrcReg).addUse(SrcReg).addUse(MC6809::SS);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::CC).addUse(SrcReg);
      break;
    case MC6809::V:
      Builder.buildInstr(MC6809::ANDCC_Imm).addDef(MC6809::CC).addUse(MC6809::CC).addImm(0xFD);
      Builder.buildInstr(MC6809::Push_i8).addUse(MC6809::CC);
      Builder.buildInstr(MC6809::AND_i1_Imm).addDef(SrcReg).addUse(SrcReg).addImm(0xFE);
      Builder.buildInstr(MC6809::LSL_i8_Reg).addDef(SrcReg).addUse(SrcReg).addImm(1);
      Builder.buildInstr(MC6809::OR_i8_Dec_Idx).addDef(SrcReg).addUse(SrcReg).addUse(MC6809::SS);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::CC).addUse(SrcReg);
      break;
    case MC6809::Z:
      Builder.buildInstr(MC6809::ANDCC_Imm).addDef(MC6809::CC).addUse(MC6809::CC).addImm(0xFB);
      Builder.buildInstr(MC6809::Push_i8).addUse(MC6809::CC);
      Builder.buildInstr(MC6809::AND_i1_Imm).addDef(SrcReg).addUse(SrcReg).addImm(0xFE);
      Builder.buildInstr(MC6809::LSL_i8_Reg).addDef(SrcReg).addUse(SrcReg).addImm(2);
      Builder.buildInstr(MC6809::OR_i8_Dec_Idx).addDef(SrcReg).addUse(SrcReg).addUse(MC6809::SS);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::CC).addUse(SrcReg);
      break;
    case MC6809::N:
      Builder.buildInstr(MC6809::ANDCC_Imm).addDef(MC6809::CC).addUse(MC6809::CC).addImm(0xF7);
      Builder.buildInstr(MC6809::Push_i8).addUse(MC6809::CC);
      Builder.buildInstr(MC6809::AND_i1_Imm).addDef(SrcReg).addUse(SrcReg).addImm(0xFE);
      Builder.buildInstr(MC6809::OR_i8_Dec_Idx).addDef(SrcReg).addUse(SrcReg).addUse(MC6809::SS);
      Builder.buildInstr(MC6809::LSL_i8_Reg).addDef(SrcReg).addUse(SrcReg).addImm(3);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::CC).addUse(SrcReg);
      break;
    default:
      llvm_unreachable("Unexpected CC bit copy in.");
    }
  } else
    llvm_unreachable("Unexpected physical register copy.");
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

const TargetRegisterClass *MC6809InstrInfo::canFoldCopy(const MachineInstr &MI, const TargetInstrInfo &TII, unsigned FoldIdx) const {
  if (!MI.getMF()->getFunction().doesNotRecurse())
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MC6809::ACC8RegClass.contains(FoldReg) || MC6809::BIT1RegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MC6809::ACC8RegClass || RC == &MC6809::BIT1RegClass)
      return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);
  }
  return nullptr;
}

void MC6809InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg, bool isKill, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI, Register VReg) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, TRI, /*IsLoad=*/false);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI ="; MI->dump(););
}

void MC6809InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI, Register VReg) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter\n";);
  loadStoreRegStackSlot(MBB, MI, DestReg, false, FrameIndex, RC, TRI, /*IsLoad=*/true);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI->dump(););
}

// Load or store one register from/to a location on the static stack.
static void loadStoreRegisterStaticStackSlot(MachineIRBuilder &Builder, MachineOperand MO, int FrameIndex, int64_t Offset, MachineMemOperand *MMO) {
  const MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI = *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Reg = MO.getReg();
  unsigned Size = 0;
  if (Reg.isPhysical()) {
    if (MC6809::BIT1RegClass.contains(Reg))
      Size = 1;
    else if (MC6809::CCondRegClass.contains(Reg) || MC6809::ACC8RegClass.contains(Reg))
      Size = 8;
    else if (MC6809::ACC16RegClass.contains(Reg) || MC6809::INDEX16RegClass.contains(Reg))
      Size = 16;
    else if (MC6809::ACC32RegClass.contains(Reg))
      Size = 32;
    else {
      // LLVM_DEBUG(dbgs() << "OINQUUE DEBUG : " << __func__ << " : " );
      llvm_unreachable("Unexpected physical register class");
    }
  } else {
    if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::BIT1RegClass))
      Size = 1;
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::CCondRegClass))
      Size = 8;
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC8RegClass))
      Size = 8;
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))
      Size = 16;
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::INDEX16RegClass))
      Size = 16;
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC32RegClass))
      Size = 32;
    else {
      // LLVM_DEBUG(dbgs() << "OINQUUE DEBUG : " << __func__ << " : " );
      llvm_unreachable("Unexpected virtual register class");
    }
  }
  assert(Size != 0);

  // Convert bit to byte if directly possible.
  if (Reg.isPhysical() && MC6809::ACC_LSBRegClass.contains(Reg)) {
    Reg = TRI.getMatchingSuperReg(Reg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    MO.setReg(Reg);
  }

  // Emit directly through ACC if possible.
  if ((Reg.isPhysical() && (MC6809::ACC8RegClass.contains(Reg) || MC6809::ACC16RegClass.contains(Reg) || MC6809::ACC32RegClass.contains(Reg))) ||
      (Reg.isVirtual() && (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC8RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC32RegClass)))) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Size = " << Size << "\n";);
    unsigned opcode;
    switch (Size) {
    default:
      llvm_unreachable("Unknown register size");
    case 1:
    case 8:
      opcode = MO.isDef() ? MC6809::Load_i8_Idx_Imm : MC6809::Store_i8_Idx_Imm;
      break;
    case 16:
      opcode = MO.isDef() ? MC6809::Load_i16_Idx_Imm : MC6809::Store_i16_Idx_Imm;
      break;
    case 32:
      opcode = MO.isDef() ? MC6809::Load_i32_Idx_Imm : MC6809::Store_i32_Idx_Imm;
      break;
    }
    Builder.buildInstr(opcode).add(MO).addFrameIndex(FrameIndex, Offset).addImm(0).addMemOperand(MMO);
    return;
  }

  // Emit via copy through ACC.
  bool IsBit = (Reg.isPhysical() && MC6809::BIT1RegClass.contains(Reg)) || (Reg.isVirtual() && (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::BIT1RegClass) || MO.getSubReg() == MC6809::sub_lsb));
  MachineOperand Tmp = MachineOperand::CreateReg(Builder.getMRI()->createVirtualRegister(&MC6809::ACC8RegClass), MO.isDef());
  if (Tmp.isUse()) {
    // Define the temporary register via copy from the MO.
    MachineOperand TmpDef = Tmp;
    TmpDef.setIsDef();
    if (IsBit) {
      TmpDef.setSubReg(MC6809::sub_lsb);
      TmpDef.setIsUndef();
    }
    Builder.buildInstr(MC6809::COPY).add(TmpDef).add(MO);

    loadStoreRegisterStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);
  } else {
    assert(Tmp.isDef());

    loadStoreRegisterStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);

    // Define the MO via copy from the temporary register.
    MachineOperand TmpUse = Tmp;
    TmpUse.setIsUse();
    if (IsBit)
      TmpUse.setSubReg(MC6809::sub_lsb);
    Builder.buildInstr(MC6809::COPY).add(MO).add(TmpUse);
  }
}

void MC6809InstrInfo::loadStoreRegStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg, bool IsKill, int FrameIndex, const TargetRegisterClass *RC, const TargetRegisterInfo *TRI, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  if ((Reg.isPhysical() && MC6809::ACC16RegClass.contains(Reg)) || (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))) {
    Register Tmp = Reg;
    if (!Reg.isPhysical()) {
      assert(Reg.isVirtual());
      // Live intervals for the original virtual register will already have
      // been computed by this point. Since this code introduces
      // subregisters, these must be using a new virtual register; otherwise
      // there would be no subregister live ranges for the new instructions.
      // This can cause VirtRegMap to fail.
      Tmp = MRI.createVirtualRegister(&MC6809::ACC16RegClass);
    }
    if (!IsLoad && Tmp != Reg)
      Builder.buildCopy(Tmp, Reg);
    loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(Tmp, IsLoad), FrameIndex, 0, MF.getMachineMemOperand(MMO, 0, 2));
    if (IsLoad && Tmp != Reg)
      Builder.buildCopy(Reg, Tmp);
  } else {
    loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MMO);
  }

  LLVM_DEBUG({
    dbgs() << "Inserted stack slot load/store:\n";
    for (const auto &MI : make_range(MIS.begin(), MIS.getInitial()))
      dbgs() << MI;
  });
}

bool MC6809InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  case MC6809::BranchRelative:
    MI.setDesc(Builder.getTII().get(MC6809::BRAb));
    break;
  case MC6809::LongBranchRelative:
    MI.setDesc(Builder.getTII().get(MC6809::LBRAlb));
    break;
  case MC6809::ConditionalBranchRelative:
    MI.setDesc(Builder.getTII().get(MC6809::Bbc));
    MI.removeOperand(2);
    break;
  case MC6809::ConditionalLongBranchRelative:
    MI.setDesc(Builder.getTII().get(MC6809::LBlbc));
    MI.removeOperand(2);
    break;
  case MC6809::ReturnImplicit:
    MI.setDesc(Builder.getTII().get(MC6809::RTSr));
    MI.removeOperand(0);
    break;
  case MC6809::ReturnIRQImplicit:
    MI.setDesc(Builder.getTII().get(MC6809::RTIr));
    MI.removeOperand(0);
    break;
  case MC6809::SEX16Implicit:
    MI.setDesc(Builder.getTII().get(MC6809::SEXx));
    MI.removeOperand(1);
    MI.removeOperand(0);
    break;
  case MC6809::SEX32Implicit:
    MI.setDesc(Builder.getTII().get(MC6809::SEXWx));
    MI.removeOperand(1);
    MI.removeOperand(0);
    break;
  case MC6809::ZEX16Implicit:
    MI.setDesc(Builder.getTII().get(MC6809::CLRAa));
    MI.removeOperand(1);
    MI.removeOperand(0);
    break;
  case MC6809::ZEX32Implicit:
    MI.setDesc(Builder.getTII().get(MC6809::CLRDa));
    MI.removeOperand(1);
    MI.removeOperand(0);
    break;
  case MC6809::Load_i1_Imm:
    expandLoad1Imm(Builder, MI);
    break;
  case MC6809::Negate_i8:
  case MC6809::Negate_i16:
  case MC6809::Negate_i32:
    expandNegate(Builder, MI);
    break;
  case MC6809::LSL_i8_Reg:
  case MC6809::LSL_i16_Reg:
    expandShiftLeft(Builder, MI);
    break;
  case MC6809::Mul_i8_i8:
  case MC6809::MulH_i8_i8:
    expandMul8_8(Builder, MI);
    break;
  case MC6809::Mul_i8_i16:
    expandMul8_16(Builder, MI);
    break;
  case MC6809::MulSetCarry_i16_Imm:
  case MC6809::MulHSetCarry_i16_Imm:
    expandMul16Imm(Builder, MI);
    break;
  case MC6809::MulSetCarry_i16_Idx_Imm:
    expandMul16IdxImm(Builder, MI);
    break;
  case MC6809::MulSetCarry_i16_Idx_Reg8:
  case MC6809::MulSetCarry_i16_Idx_Reg16:
    expandMul16IdxReg(Builder, MI);
    break;
  case MC6809::MulSetCarry_i16_Reg:
  case MC6809::MulHSetCarry_i16_Reg:
    expandMul16Reg(Builder, MI);
    break;
  case MC6809::BranchSubroutine:
    expandCallRelative(Builder, MI);
    break;
  case MC6809::LEAPtrAdd_Imm:
  case MC6809::LEAPtrAdd_Reg8:
  case MC6809::LEAPtrAdd_Reg16:
    expandLEAPtrAdd(Builder, MI);
    break;
  case MC6809::Load_i8_Imm:
  case MC6809::Load_i16_Imm:
  case MC6809::Load_i32_Imm:
    expandLoadImm(Builder, MI);
    break;
  case MC6809::Load_i8_Idx_Imm:
  case MC6809::Load_i8_Idx_Reg8:
  case MC6809::Load_i8_Idx_Reg16:
  case MC6809::Load_i16_Idx_Imm:
  case MC6809::Load_i16_Idx_Reg8:
  case MC6809::Load_i16_Idx_Reg16:
  case MC6809::Load_i32_Idx_Imm:
  case MC6809::Load_i32_Idx_Reg8:
  case MC6809::Load_i32_Idx_Reg16:
  case MC6809::Load_iPtr_Idx_Imm:
  case MC6809::Load_iPtr_Idx_Reg8:
  case MC6809::Load_iPtr_Idx_Reg16:
    expandLoadIdx(Builder, MI);
    break;
  case MC6809::Store_i8_Idx_Imm:
  case MC6809::Store_i8_Idx_Reg8:
  case MC6809::Store_i8_Idx_Reg16:
  case MC6809::Store_i16_Idx_Imm:
  case MC6809::Store_i16_Idx_Reg8:
  case MC6809::Store_i16_Idx_Reg16:
  case MC6809::Store_i32_Idx_Imm:
  case MC6809::Store_i32_Idx_Reg8:
  case MC6809::Store_i32_Idx_Reg16:
  case MC6809::Store_iPtr_Idx_Imm:
  case MC6809::Store_iPtr_Idx_Reg8:
  case MC6809::Store_iPtr_Idx_Reg16:
    expandStoreIdx(Builder, MI);
    break;
  case MC6809::AND_i1_Imm:
  case MC6809::AND_i8_Imm:
  case MC6809::AND_i16_Imm:
    expandImm(ANDImm, Builder, MI);
    break;
  case MC6809::AND_i8_Idx_Imm:
  case MC6809::AND_i16_Idx_Imm:
    expandIdxImm(ANDIdxImm, Builder, MI);
    break;
  case MC6809::AND_i8_Idx_Reg8:
  case MC6809::AND_i16_Idx_Reg8:
  case MC6809::AND_i8_Idx_Reg16:
  case MC6809::AND_i16_Idx_Reg16:
    expandIdxReg(ANDIdxReg, Builder, MI);
    break;
  case MC6809::AND_i8_Reg:
  case MC6809::AND_i16_Reg:
    expandANDReg(Builder, MI);
    break;
  case MC6809::OR_i8_Imm:
  case MC6809::OR_i16_Imm:
    expandImm(ORImm, Builder, MI);
    break;
  case MC6809::OR_i8_Idx_Imm:
  case MC6809::OR_i16_Idx_Imm:
    expandIdxImm(ORIdxImm, Builder, MI);
    break;
  case MC6809::OR_i8_Idx_Reg8:
  case MC6809::OR_i16_Idx_Reg8:
  case MC6809::OR_i8_Idx_Reg16:
  case MC6809::OR_i16_Idx_Reg16:
    expandIdxReg(ORIdxReg, Builder, MI);
    break;
  case MC6809::OR_i8_Reg:
  case MC6809::OR_i16_Reg:
    expandORReg(Builder, MI);
    break;
  case MC6809::XOR_i8_Imm:
  case MC6809::XOR_i16_Imm:
    expandImm(XORImm, Builder, MI);
    break;
  case MC6809::XOR_i8_Idx_Imm:
  case MC6809::XOR_i16_Idx_Imm:
    expandIdxImm(XORIdxImm, Builder, MI);
    break;
  case MC6809::XOR_i8_Idx_Reg8:
  case MC6809::XOR_i16_Idx_Reg8:
  case MC6809::XOR_i8_Idx_Reg16:
  case MC6809::XOR_i16_Idx_Reg16:
    expandIdxReg(XORIdxReg, Builder, MI);
    break;
  case MC6809::XOR_i8_Reg:
  case MC6809::XOR_i16_Reg:
    expandXORReg(Builder, MI);
    break;
  case MC6809::Add_i8_Imm:
  case MC6809::Add_i16_Imm:
  case MC6809::AddSetCarry_i8_Imm:
  case MC6809::AddSetCarry_i16_Imm:
    expandImm(AddImm, Builder, MI);
    break;
  case MC6809::Add_i32_Imm:
  case MC6809::AddSetCarry_i32_Imm:
    expandAdd32Imm(Builder, MI);
    break;
  case MC6809::Add_i8_Idx_Imm:
  case MC6809::Add_i16_Idx_Imm:
  case MC6809::AddSetCarry_i8_Idx_Imm:
  case MC6809::AddSetCarry_i16_Idx_Imm:
    expandIdxImm(AddIdxImm, Builder, MI);
    break;
  case MC6809::Add_i32_Idx_Imm:
  case MC6809::AddSetCarry_i32_Idx_Imm:
    expandAdd32IdxImm(Builder, MI);
    break;
  case MC6809::AddSetCarry_i8_Idx_Reg8:
  case MC6809::AddSetCarry_i16_Idx_Reg8:
  case MC6809::AddSetCarry_i8_Idx_Reg16:
  case MC6809::AddSetCarry_i16_Idx_Reg16:
    expandIdxReg(AddIdxReg, Builder, MI);
    break;
#if 0
  case MC6809::AddSetCarry_i32_Idx_Reg8:
  case MC6809::AddSetCarry_i32_Idx_Reg16:
    expandAdd32IdxReg(Builder, MI);
    break;
#endif
  case MC6809::Add_i8_Reg:
  case MC6809::Add_i16_Reg:
    expandAddReg(Builder, MI);
    break;
  case MC6809::AddSetCarry_i8_Reg:
  case MC6809::AddSetCarry_i16_Reg:
    expandAddSetCarryReg(Builder, MI);
    break;
  case MC6809::Sub_i8_Imm:
  case MC6809::Sub_i16_Imm:
  case MC6809::SubSetCarry_i8_Imm:
  case MC6809::SubSetCarry_i16_Imm:
    expandImm(SubImm, Builder, MI);
    break;
  case MC6809::Sub_i32_Imm:
  case MC6809::SubSetCarry_i32_Imm:
    expandSub32Imm(Builder, MI);
    break;
  case MC6809::Sub_i8_Idx_Imm:
  case MC6809::Sub_i16_Idx_Imm:
  case MC6809::SubSetCarry_i8_Idx_Imm:
  case MC6809::SubSetCarry_i16_Idx_Imm:
    expandIdxImm(SubIdxImm, Builder, MI);
    break;
  case MC6809::Sub_i32_Idx_Imm:
  case MC6809::SubSetCarry_i32_Idx_Imm:
    expandSub32IdxImm(Builder, MI);
    break;
  case MC6809::SubSetCarry_i8_Idx_Reg8:
  case MC6809::SubSetCarry_i16_Idx_Reg8:
  case MC6809::SubSetCarry_i8_Idx_Reg16:
  case MC6809::SubSetCarry_i16_Idx_Reg16:
    expandIdxReg(SubIdxReg, Builder, MI);
    break;
#if 0
  case MC6809::SubSetCarry_i32_Idx_Reg8:
  case MC6809::SubSetCarry_i32_Idx_Reg16:
    expandSub32IdxReg(Builder, MI);
    break;
#endif
  case MC6809::Sub_i8_Reg:
  case MC6809::Sub_i16_Reg:
    expandSubReg(Builder, MI);
    break;
  case MC6809::SubSetCarry_i8_Reg:
  case MC6809::SubSetCarry_i16_Reg:
    expandSubSetCarryReg(Builder, MI);
    break;
  case MC6809::Sub_i8_Pull:
  case MC6809::Sub_i16_Pull:
  case MC6809::Sub_i32_Pull:
    expandSubPull(Builder, MI);
    break;
  case MC6809::Compare_i8_Imm:
  case MC6809::Compare_i16_Imm:
    expandCompareImm(Builder, MI);
    break;
#if 0
  case MC6809::Compare_i32_Imm:
    expandCompare32Imm(Builder, MI);
    break;
#endif
  case MC6809::Compare_i8_Idx_Imm:
  case MC6809::Compare_i16_Idx_Imm:
  case MC6809::Compare_i8_Idx_Reg8:
  case MC6809::Compare_i16_Idx_Reg8:
  case MC6809::Compare_i8_Idx_Reg16:
  case MC6809::Compare_i16_Idx_Reg16:
    expandCompareIdx(Builder, MI);
    break;
  case MC6809::Test_i8_Reg:
  case MC6809::Test_i16_Reg:
    expandTestReg(Builder, MI);
    break;
  case MC6809::Test_i32_Reg:
    expandTestReg32(Builder, MI);
    break;
  case MC6809::Copy8:
  case MC6809::Copy16:
    MI.setDesc(Builder.getTII().get(MC6809::TFRp));
    break;
  case MC6809::Push_i8:
  case MC6809::Push_i16:
  case MC6809::Push_i32:
  case MC6809::Push_Ptr: {
    auto PushReg = MI.getOperand(1).getReg();
    if (PushReg == MC6809::AQ) {
      MI.setDesc(Builder.getTII().get(MC6809::PSHSWx));
      MI.removeOperand(1);
      MI.removeOperand(0);
      Builder.buildInstr(MC6809::PSHSs).addImm(0x06); // PSHS D
      break;
    } else if (PushReg == MC6809::AW) {
      MI.setDesc(Builder.getTII().get(MC6809::PSHSWx));
      MI.removeOperand(1);
      MI.removeOperand(0);
      break;
    }
    MI.setDesc(Builder.getTII().get(MC6809::PSHSs));
    unsigned short regList = 0;
    switch (PushReg) {
    default:
      llvm_unreachable("Register not recognised for Push instruction!");
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
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
  case MC6809::Pull_i8:
  case MC6809::Pull_i16:
  case MC6809::Pull_Ptr: {
    auto PullReg = MI.getOperand(1).getReg();
    if (PullReg == MC6809::AQ) {
      MI.setDesc(Builder.getTII().get(MC6809::PULSWx));
      MI.removeOperand(1);
      MI.removeOperand(0);
      Builder.buildInstr(MC6809::PULSs).addImm(0x06); // PULS D
      break;
    } else if (PullReg == MC6809::AW) {
      MI.setDesc(Builder.getTII().get(MC6809::PULSWx));
      MI.removeOperand(1);
      MI.removeOperand(0);
      break;
    }
    MI.setDesc(Builder.getTII().get(MC6809::PULSs));
    unsigned short regList = 0;
    switch (PullReg) {
    default:
      llvm_unreachable("Register not recognised for Pull instruction!");
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
    case MC6809::SS:
      regList |= 64;
      break;
    case MC6809::PC:
      regList |= 128;
      break;
    }
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    break;
  }
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Changed = " << Changed << "\n";);
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
  return (Offset == 0) ? 0 : ((Offset >= -16 && Offset < 16) ? 5 : ((Offset >= -128 && Offset < 128) ? 8 : ((Offset >= -32768 && Offset < 32768) ? 16 : 256))); // Do I need this? Maybe there is a relocation involved?
}

void MC6809InstrInfo::expandCallRelative(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MI.setDesc(Builder.getTII().get(MC6809::BSRb));
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
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandImm(ContextImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  auto DestReg = MI.getOperand(0).getReg();
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val;

  if (ValOp.isImm() || ValOp.isCImm())
    Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  auto OpcodePair = Context.Opcode->find(DestReg);
  if (OpcodePair == Context.Opcode->end())
    llvm_unreachable("Unexpected register");
  if (Val != Context.IdentityValue) {
    auto Instr = Builder.buildInstr(OpcodePair->getSecond()).addImm(Val);
    Instr->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandIdxImm(ContextIndexImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  auto DestReg = MI.getOperand(0).getReg();
  auto IndexReg = MI.getOperand(operandCount - 2).getReg();
  auto OffsetOp = MI.getOperand(operandCount - 1);
  int Offset;

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestReg, OffsetSize};
    auto OpcodePair = Context.Opcode->find(Lookup);
    if (OpcodePair == Context.Opcode->end())
      llvm_unreachable("Unexpected operand(s) in indexed instruction.");
    auto Instr = Builder.buildInstr(OpcodePair->getSecond());
    if (OffsetSize == 0)
      Instr.addReg(IndexReg);
    else {
      Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
      Instr.addImm(Offset).addReg(IndexReg);
    }
    Instr->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
  } else
    llvm_unreachable("Unknown offset type");
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandIdxReg(ContextIndexRegister Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto DestReg = MI.getOperand(0).getReg();
  auto IndexReg = MI.getOperand(2).getReg();
  auto OffsetReg = MI.getOperand(3).getReg();

  RegPlusReg Lookup{DestReg, OffsetReg};
  auto OpcodePair = Context.Opcode->find(Lookup);
  if (OpcodePair == Context.Opcode->end())
    llvm_unreachable("Unexpected register offset operand.");
  auto Instr = Builder.buildInstr(OpcodePair->getSecond()).addReg(IndexReg);
  Instr->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
}

void MC6809InstrInfo::expandNegate(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  bool has6309 = STI.has6309();
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Illegal register for Neg(1|16|32)");
  case MC6809::AA:
    MI.setDesc(Builder.getTII().get(MC6809::NEGAa));
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AB:
    MI.setDesc(Builder.getTII().get(MC6809::NEGBa));
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AD:
    if (has6309) {
      MI.setDesc(Builder.getTII().get(MC6809::NEGDa));
      MI.removeOperand(3);
      MI.removeOperand(2);
      MI.removeOperand(1);
      MI.removeOperand(0);
      MI.addImplicitDefUseOperands(*MI.getMF());
    }
    break;
  case MC6809::AE:
    Builder.buildInstr(MC6809::COMEa)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AE).addUse(MC6809::A0).addUse(MC6809::AE)->addImplicitDefUseOperands(*MI.getMF());
    MI.eraseFromParent();
    break;
  case MC6809::AF:
    Builder.buildInstr(MC6809::COMFa)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AF).addUse(MC6809::A0).addUse(MC6809::AF)->addImplicitDefUseOperands(*MI.getMF());
    MI.eraseFromParent();
    break;
  case MC6809::AW:
    Builder.buildInstr(MC6809::COMWa)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AW).addUse(MC6809::A0).addUse(MC6809::AW)->addImplicitDefUseOperands(*MI.getMF());
    MI.eraseFromParent();
    break;
  case MC6809::AQ:
    Builder.buildInstr(MC6809::COMDa)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::COMWa)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AW).addUse(MC6809::A0).addUse(MC6809::AW)->addImplicitDefUseOperands(*MI.getMF());
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AD).addUse(MC6809::A0).addUse(MC6809::AD)->addImplicitDefUseOperands(*MI.getMF());
    MI.eraseFromParent();
    break;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandShiftLeft(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Illegal register for ASL/LSL");
  case MC6809::AA:
    MI.setDesc(Builder.getTII().get(MC6809::ASLAa));
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AB:
    MI.setDesc(Builder.getTII().get(MC6809::ASLBa));
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AD:
    MI.setDesc(Builder.getTII().get(MC6809::ASLDa));
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandMul8_8(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto Opcode = MI.getOpcode();
  assert((Opcode == MC6809::Mul_i8_i8 || Opcode == MC6809::MulH_i8_i8) && "Invalid multiply opcode");
  if (Opcode == MC6809::Mul_i8_i8)
    assert((MI.getOperand(0).getReg() == MC6809::AB) && "Results must be AB");
  else
    assert((MI.getOperand(0).getReg() == MC6809::AA) && "Results must be AA");
  assert(((MI.getOperand(1).getReg() == MC6809::AA && MI.getOperand(2).getReg() == MC6809::AB) || (MI.getOperand(1).getReg() == MC6809::AB && MI.getOperand(2).getReg() == MC6809::AA)) && "Arguments must be AA and AB");
  MI.setDesc(Builder.getTII().get(MC6809::MULx));
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandMul8_16(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto Opcode = MI.getOpcode();
  assert((Opcode == MC6809::Mul_i8_i16) && "Invalid multiply opcode");
  assert(MI.getOperand(0).getReg() == MC6809::AD && "Results must be in AD");
  assert(((MI.getOperand(1).getReg() == MC6809::AA && MI.getOperand(2).getReg() == MC6809::AB) || (MI.getOperand(1).getReg() == MC6809::AB && MI.getOperand(2).getReg() == MC6809::AA)) && "Arguments must be AB and AA");
  MI.setDesc(Builder.getTII().get(MC6809::MULx));
  MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandMul16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) || (MI.getOperand(0).getReg() == MC6809::AD && MI.getOperand(1).getReg() == MC6809::AW)) && "Results must be in AW and AD");
  auto ValueOp = MI.getOperand(3);
  auto Value = ValueOp.isImm() ? ValueOp.getImm() : ValueOp.getCImm()->getSExtValue();
  auto Instr = Builder.buildInstr(MC6809::MULDi16).addImm(Value);
  Instr->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
}

void MC6809InstrInfo::expandMul16IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) && "Results must be in AW and AD");
  auto IndexReg = MI.getOperand(3).getReg();
  auto OffsetOp = MI.getOperand(4);
  int Offset;

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    unsigned Opcode;
    switch (OffsetSize) {
    case 0:
      Opcode = MC6809::MULDi_o0;
      break;
    case 5:
      Opcode = MC6809::MULDi_o5;
      break;
    case 8:
      Opcode = MC6809::MULDi_o8;
      break;
    case 16:
      Opcode = MC6809::MULDi_o16;
      break;
    }
    auto Instr = Builder.buildInstr(Opcode);
    if (OffsetSize == 0) {
      Instr.addReg(IndexReg);
    } else {
      Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
      Instr.addImm(Offset).addReg(IndexReg);
    }
    Instr->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
  } else
    llvm_unreachable("Unknown offset type");
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMul16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // auto DestReg = MI.getOperand(0).getReg();
  auto IndexReg = MI.getOperand(2).getReg();
  auto OffsetReg = MI.getOperand(3).getReg();
  unsigned Opcode;

  switch (OffsetReg) {
  default:
    llvm_unreachable("Illegal offset register");
  case MC6809::AA:
    Opcode = MC6809::MULDi_oA;
    break;
  case MC6809::AB:
    Opcode = MC6809::MULDi_oB;
    break;
  case MC6809::AD:
    Opcode = MC6809::MULDi_oD;
    break;
  case MC6809::AE:
    Opcode = MC6809::MULDi_oE;
    break;
  case MC6809::AF:
    Opcode = MC6809::MULDi_oF;
    break;
  case MC6809::AW:
    Opcode = MC6809::MULDi_oW;
    break;
  }
  auto Instr = Builder.buildInstr(Opcode).addReg(OffsetReg).addReg(IndexReg);
  Instr->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Instr = "; Instr->dump(););
}

#if 0
void MC6809InstrInfo::expandMul16Pop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert((MI.getOperand(0).getReg() == MC6809::AW) && "Results must be in AW");
  assert(MI.getOperand(1).getReg() == MC6809::AD && "Argument 1 must be in AD");
  MI.setDesc(Builder.getTII().get(MC6809::MULDi_Inc2));
  //MI.removeOperand(2);
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addOperand(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
  MI.addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}
#endif

void MC6809InstrInfo::expandMul16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) || (MI.getOperand(0).getReg() == MC6809::AD && MI.getOperand(1).getReg() == MC6809::AW)) && "Results must be in AW and AD");
  auto Reg = MI.getOperand(3).getReg();
  MachineBasicBlock::iterator B, E;
  MachineBasicBlock &MBB = Builder.getMBB();
  B = Builder.buildInstr(MC6809::Push_i16).addReg(Reg);
  E = Builder.buildInstr(MC6809::MULDi_Inc2).addReg(MC6809::SS);
  E->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bundle exit\n";);
  auto Bundler = MIBundleBuilder(MBB, B, ++E);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Bundle built\n";);
  LLVM_DEBUG(for (auto &I
                  : Bundler) {
    dbgs() << "OINQUE DEBUG " << __func__ << " : bundle : I = ";
    I.dump();
  });
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandLoad1Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  Register DestReg = MI.getOperand(0).getReg();
  int64_t Val = MI.getOperand(1).getImm();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    DestReg = Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    assert(DestReg && "Unexpected destination for LDImm1");
    assert(MC6809::ACC8RegClass.contains(DestReg));
    Opcode = MC6809::Load_i8_Imm;
    MI.getOperand(0).setReg(DestReg);
    MI.getOperand(1).setImm(!!Val);
    break;
  }
  case MC6809::N:
    if (Val) {
      Opcode = MC6809::ORCCi8;
      MI.getOperand(1).setImm(0x08);
      MI.removeOperand(0);
    } else {
      Opcode = MC6809::ANDCCi8;
      MI.getOperand(1).setImm(0xF7);
      MI.removeOperand(0);
    }
    break;
  case MC6809::Z:
    if (Val) {
      Opcode = MC6809::ORCCi8;
      MI.getOperand(1).setImm(0x04);
      MI.removeOperand(0);
    } else {
      Opcode = MC6809::ANDCCi8;
      MI.getOperand(1).setImm(0xFB);
      MI.removeOperand(0);
    }
    break;
  case MC6809::V:
    if (Val) {
      Opcode = MC6809::ORCCi8;
      MI.getOperand(1).setImm(0x02);
      MI.removeOperand(0);
    } else {
      Opcode = MC6809::ANDCCi8;
      MI.getOperand(1).setImm(0xFD);
      MI.removeOperand(0);
    }
    break;
  case MC6809::C:
    if (Val) {
      Opcode = MC6809::ORCCi8;
      MI.getOperand(1).setImm(0x01);
      MI.removeOperand(0);
    } else {
      Opcode = MC6809::ANDCCi8;
      MI.getOperand(1).setImm(0xFE);
      MI.removeOperand(0);
    }
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandLoadImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = LoadImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == LoadImmediateOpcode.end())
    llvm_unreachable("Unexpected LoadImm register.");
  MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
  MI.addImplicitDefUseOperands(*MI.getMF());
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

void MC6809InstrInfo::expandANDReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for ANDReg");

  unsigned Opcode = MC6809::ANDRp;
  auto ANDReg = Builder.buildInstr(Opcode).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  ANDReg->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : ANDReg = "; ANDReg->dump(););
}

void MC6809InstrInfo::expandORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for ORReg");

  unsigned Opcode = MC6809::ORRp;
  auto ORReg = Builder.buildInstr(Opcode).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  ORReg->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : ORReg = "; ORReg->dump(););
}

void MC6809InstrInfo::expandXORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for XORReg");

  unsigned Opcode = MC6809::EORRp;
  auto EORReg = Builder.buildInstr(Opcode).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  EORReg->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : EORReg = "; EORReg->dump(););
}

void MC6809InstrInfo::expandAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for AddReg");

  auto AddReg = Builder.buildInstr(MC6809::ADDRp).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  AddReg->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddReg = "; AddReg->dump(););
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source 2 must be same for AddSetCarryReg");

  auto AddReg = Builder.buildInstr(MC6809::ADCRp).addDef(MI.getOperand(0).getReg()).addDef(MI.getOperand(1).getReg()).addUse(MI.getOperand(3).getReg()).addUse(MI.getOperand(2).getReg());
  AddReg->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandAdd32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  MachineOperand DestReg = MI.getOperand(0);
  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  int64_t Val;
  auto ValOp = MI.getOperand(operandCount - 1);
  if (ValOp.isImm() || ValOp.isCImm())
    Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  else
    llvm_unreachable("No constant argument found for 32-bit add immediate");
  int64_t ValLo = Val & 0xFFFF;
  int64_t ValHi = (Val >> 16) & 0xFFFF;

  auto AddLo = Builder.buildInstr(MC6809::ADDWi16).addImm(ValLo);
  AddLo->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AddLo : = "; AddLo->dump(););
  auto AddHi = Builder.buildInstr(MC6809::ADCDi16).addImm(ValHi);
  AddHi->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : AddHi : = "; AddHi->dump(););
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

void MC6809InstrInfo::expandSub32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  MachineOperand DestReg = MI.getOperand(0);
  assert(DestReg.getReg() == MC6809::AQ && "32-bit subtract must have q as the target register");
  int64_t Val;
  auto ValOp = MI.getOperand(operandCount - 1);
  if (ValOp.isImm() || ValOp.isCImm())
    Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  else
    llvm_unreachable("No constant argument found for 32-bit subtract immediate");
  int64_t ValLo = Val & 0xFFFF;
  int64_t ValHi = (Val >> 16) & 0xFFFF;
  auto SubLo = Builder.buildInstr(MC6809::SUBWi16).addImm(ValLo);
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubLo : = "; SubLo->dump(););
  auto SubHi = Builder.buildInstr(MC6809::SBCDi16).addImm(ValHi);
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubHi : = "; SubHi->dump(););
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
}

#if 0
void MC6809InstrInfo::expandSub32ImmRev(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto DestReg = MI.getOperand(0);
  assert(DestReg.getReg() == MC6809::AQ && "32-bit subtract must have q as the target register");
  MachineOperand ValOp = MI.getOperand(2);
  assert((ValOp.isImm() || ValOp.isCImm()) && "The argument must be a constant");
  int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  if (Val != 0) {
    int64_t ValLo = Val & 0xFFFF;
    int64_t ValHi = (Val >> 16) & 0xFFFF;

    auto SubLo = Builder.buildInstr(MC6809::SUBWi16).addImm(ValLo);
    SubLo->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubLo : = "; SubLo->dump(););
    auto SubHi = Builder.buildInstr(MC6809::SBCDi16).addImm(ValHi);
    SubHi->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubHi : = "; SubHi->dump(););
  } else {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Subtract32 : Constant is 0, so ignoring.\n";);
  }

  auto Com1 = Builder.buildInstr(MC6809::COMDa);
  Com1->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Com1 : = "; Com1->dump(););
  auto Com2 = Builder.buildInstr(MC6809::COMWa);
  Com2->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Com2 : = "; Com2->dump(););
  auto Inc2 = Builder.buildInstr(MC6809::ADCRp)
                  .addDef(MC6809::AW)
                  .addReg(MC6809::A0)
                  .addReg(MC6809::AW);
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Inc2 : = "; Inc2->dump(););
  Inc2->addImplicitDefUseOperands(*MI.getMF());
  auto Inc1 = Builder.buildInstr(MC6809::ADCRp)
                  .addDef(MC6809::AD)
                  .addReg(MC6809::A0)
                  .addReg(MC6809::AD);
  Inc1->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Inc1 : = "; Inc1->dump(););
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n ";);
}
#endif

void MC6809InstrInfo::expandAdd32IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(operandCount - 2);
  MachineOperand OffsetOp = MI.getOperand(operandCount - 1);
  MachineInstrBuilder AddLo, AddHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    int64_t Offset = (OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue()) + 2; // Low word
    RegPlusOffsetLen LookupL{MC6809::AW, OffsetSize};
    auto OpcodePairL = AddIdxImmOpcode.find(LookupL);
    if (OpcodePairL == AddIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddLo = Builder.buildInstr(OpcodePairL->getSecond()).addImm(Offset).addUse(IndexOp.getReg());
    Offset -= 2; // High word
    RegPlusOffsetLen LookupH{MC6809::AD, OffsetSize};
    auto OpcodePairH = AddCarryIdxImmOpcode.find(LookupH);
    if (OpcodePairH == AddCarryIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    AddHi = Builder.buildInstr(OpcodePairH->getSecond()).addImm(Offset).addUse(IndexOp.getReg());
  } else
    llvm_unreachable("Unknown offset type for index");
  AddLo->addImplicitDefUseOperands(*MI.getMF());
  AddHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddLo = "; AddLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : AddHi = "; AddHi->dump(););
}

#if 0
void MC6809InstrInfo::expandAdd32IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(2);
  MachineOperand OffsetOp = MI.getOperand(3);
  MachineInstrBuilder AddLo, AddHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit add must have q as the target register");
  if (OffsetOp.isReg()) {
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
#endif

#if 0
void MC6809InstrInfo::expandSubImmRev(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto DestReg = MI.getOperand(0).getReg();
  MachineOperand ValOp = MI.getOperand(2);
  assert((ValOp.isImm() || ValOp.isCImm()) && "The argument must be a constant");
  auto SubOpcodePair = SubImmediateOpcode.find(DestReg);
  if (SubOpcodePair == SubImmediateOpcode.end())
    llvm_unreachable("Unexpected subtract register.");
  int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  if (Val != 0) {
    MachineInstrBuilder Subtract = Builder.buildInstr(SubOpcodePair->getSecond()).addImm(Val);
    Subtract->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Subtract : = "; Subtract->dump(););
  } else {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Subtract : constant is 0, so ignoring\n";);
  }

  MachineInstrBuilder Negate;
  MachineInstrBuilder Inc;
  bool NeedInc = false;

  switch (DestReg) {
  default:
    llvm_unreachable("Unexpected negate register.");
  case MC6809::AA:
    Negate = Builder.buildInstr(MC6809::NEGAa);
    break;
  case MC6809::AB:
    Negate = Builder.buildInstr(MC6809::NEGBa);
    break;
  case MC6809::AD:
    Negate = Builder.buildInstr(MC6809::NEGDa);
    break;
  case MC6809::AE:
    Negate = Builder.buildInstr(MC6809::COMEa);
    Inc = Builder.buildInstr(MC6809::ADCRp)
              .addUse(MC6809::AE)
              .addReg(MC6809::A0)
              .addReg(MC6809::AE);
    NeedInc = true;
    break;
  case MC6809::AF:
    Negate = Builder.buildInstr(MC6809::COMFa);
    Inc = Builder.buildInstr(MC6809::ADCRp)
              .addUse(MC6809::AF)
              .addReg(MC6809::A0)
              .addReg(MC6809::AF);
    NeedInc = true;
    break;
  case MC6809::AW:
    Negate = Builder.buildInstr(MC6809::COMWa);
    Inc = Builder.buildInstr(MC6809::ADCRp)
              .addUse(MC6809::AW)
              .addReg(MC6809::A0)
              .addReg(MC6809::AW);
    NeedInc = true;
    break;
  }
  Negate->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Negate : = "; Negate->dump(););
  if (NeedInc) {
    Inc->addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Inc : = "; Inc->dump(););
  }
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n ";);
}
#endif

void MC6809InstrInfo::expandSubReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for SubReg");

  auto SUBReg = Builder.buildInstr(MC6809::SUBRp).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  SUBReg->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SUBReg = "; SUBReg->dump(););
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source 2 must be same for SubSetCarryReg");

  auto SubReg = Builder.buildInstr(MC6809::SBCRp).addDef(MI.getOperand(0).getReg()).addDef(MI.getOperand(1).getReg()).addUse(MI.getOperand(3).getReg()).addUse(MI.getOperand(2).getReg());
  SubReg->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
}

void MC6809InstrInfo::expandSubPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););

  auto OpcodePair = SubPullOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == SubPullOpcode.end()) {
    if (MI.getOperand(0).getReg() == MC6809::AQ) {
      auto SUBPull1 = Builder.buildInstr(MC6809::SUBWi_Dec2).addUse(MC6809::SS);
      SUBPull1->addImplicitDefUseOperands(*MI.getMF());
      auto SUBPull2 = Builder.buildInstr(MC6809::SBCDi_Dec2).addUse(MC6809::SS);
      SUBPull2->addImplicitDefUseOperands(*MI.getMF());
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit :  SUBPull1 = "; SUBPull1->dump(););
      LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit :  SUBPull2 = "; SUBPull2->dump(););
      MI.eraseFromParent();
    } else
      llvm_unreachable("Unexpected register for SubPull.");
  } else {
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
    MI.addImplicitDefUseOperands(*MI.getMF());
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : MI = "; MI.dump(););
  }
}

void MC6809InstrInfo::expandSub32IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  auto operandCount = MI.getNumExplicitOperands();
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(operandCount - 2);
  MachineOperand OffsetOp = MI.getOperand(operandCount - 1);
  MachineInstrBuilder SubLo, SubHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit subtract must have q as the target register");
  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    int64_t Offset = (OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue()) + 2; // Low word
    RegPlusOffsetLen LookupL{MC6809::AW, OffsetSize};
    auto OpcodePairL = SubIdxImmOpcode.find(LookupL);
    if (OpcodePairL == SubIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubLo = Builder.buildInstr(OpcodePairL->getSecond()).addImm(Offset).addUse(IndexOp.getReg());
    Offset -= 2; // High word
    RegPlusOffsetLen LookupH{MC6809::AD, OffsetSize};
    auto OpcodePairH = SubBorrowIdxImmOpcode.find(LookupH);
    if (OpcodePairH == SubBorrowIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubHi = Builder.buildInstr(OpcodePairH->getSecond()).addImm(Offset).addUse(IndexOp.getReg());
  } else
    llvm_unreachable("Unknown offset type for Sub32IdxImm");
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
}

void MC6809InstrInfo::expandSub32IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  MachineOperand DestReg = MI.getOperand(0);
  MachineOperand IndexOp = MI.getOperand(2);
  MachineOperand OffsetOp = MI.getOperand(3);
  MachineInstrBuilder SubLo, SubHi;

  assert(DestReg.getReg() == MC6809::AQ && "32-bit subtract must have q as the target register");
  if (OffsetOp.isReg()) {
    RegPlusReg LookupL{MC6809::AW, OffsetOp.getReg()};
    auto OpcodePairL = SubIdxRegOpcode.find(LookupL);
    if (OpcodePairL == SubIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubLo = Builder.buildInstr(OpcodePairL->getSecond()).addUse(IndexOp.getReg());
    RegPlusReg LookupH{MC6809::AD, OffsetOp.getReg()};
    auto OpcodePairH = SubBorrowIdxRegOpcode.find(LookupH);
    if (OpcodePairH == SubBorrowIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    SubHi = Builder.buildInstr(OpcodePairH->getSecond()).addUse(IndexOp.getReg());
  } else
    llvm_unreachable("Unknown offset type for SubBorrowIdx");
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
}

#if 0
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
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  Pop->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubLo = "; SubLo->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : SubHi = "; SubHi->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Pop = "; Pop->dump(););
}
#endif

void MC6809InstrInfo::expandCompareImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of compares must be a register");
  assert((MI.getOperand(3).isImm() || MI.getOperand(3).isCImm()) && "The final operand of immediate compares must be an immediate constant");

  auto SrcReg = MI.getOperand(2).getReg();
  auto OpcodePair = CompareImmediateOpcode.find(SrcReg);
  if (OpcodePair == CompareImmediateOpcode.end())
    llvm_unreachable("Compare Immediate - unexpected register.");
  auto Opcode = OpcodePair->getSecond();
  auto Compare = Builder.buildInstr(Opcode).add(MI.getOperand(3));
  Compare->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Compare = "; Compare->dump(););
}

#if 0
void MC6809InstrInfo::expandCompare32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && MI.getOperand(2).getReg() == MC6809::AQ && "The source of i32-bit compares must be the AQ register");
  assert((MI.getOperand(3).isImm() || MI.getOperand(3).isCImm()) && "The final operand of immediate compares must be an immediate constant");

  MachineOperand ValOp = MI.getOperand(3);
  int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int64_t ValLo = Val & 0xFFFF;
  int64_t ValHi = (Val >> 16) & 0xFFFF;

  // FIXME: MarkM: This is badly broken. This instruction sequence won't work.
  errs() << "OINQUE DEBUG Warning : This 32-bit compare instruction sequence won't work!\n";
  auto SubLo = Builder.buildInstr(MC6809::SUBWi16)
                   .addImm(ValLo);
  SubLo->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubLo : = "; SubLo->dump(););
  auto SubHi = Builder.buildInstr(MC6809::SBCDi16)
                   .addImm(ValHi);
  SubHi->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SubHi : = "; SubHi->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit\n";);
  MI.eraseFromParent();
}
#endif

void MC6809InstrInfo::expandCompareIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of compares must be a register");
  assert(MI.getOperand(3).isReg() && "The index operand of indexed compares must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  auto IndexOp = MI.getOperand(3);
  auto OffsetOp = MI.getOperand(4);
  unsigned Opcode = 0;

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{SrcReg, OffsetSize};
    auto OpcodePair = CompareIdxImmOpcode.find(Lookup);
    if (OpcodePair == CompareIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s) in compare indexed/immediate.");
    Opcode = OpcodePair->getSecond();
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{SrcReg, OffsetOp.getReg()};
    auto OpcodePair = CompareIdxRegOpcode.find(Lookup);
    if (OpcodePair == CompareIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s) in compare indexed/register.");
  } else
    llvm_unreachable("Unknown offset type for CompareIdx");
  auto Compare = Builder.buildInstr(Opcode).add(OffsetOp).add(IndexOp);
  Compare->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Compare = "; Compare->dump(););
}

#if 0
void MC6809InstrInfo::expandComparePop(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of compares must be a register");

  auto OpcodePair = ComparePopOpcode.find(MI.getOperand(2).getReg());
  if (OpcodePair == ComparePopOpcode.end())
    llvm_unreachable("Unexpected register.");
  auto Opcode = OpcodePair->getSecond();
  auto Compare = Builder.buildInstr(Opcode)
                     .add(MachineOperand::CreateReg(MC6809::SS, /* isDef */ false));
  Compare->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Compare = "; Compare->dump(););
}
#endif

void MC6809InstrInfo::expandTestReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of tests must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of tests must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of register tests must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  auto OpcodePair = TestRegOpcode.find(SrcReg);
  if (OpcodePair == TestRegOpcode.end())
    llvm_unreachable("Compare Immediate - unexpected register.");
  auto Opcode = OpcodePair->getSecond();
  auto Test = Builder.buildInstr(Opcode);
  Test->addImplicitDefUseOperands(*MI.getMF());
  MI.eraseFromParent();
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Exit : Test = "; Test->dump(););
}

void MC6809InstrInfo::expandTestReg32(MachineIRBuilder &Builder, MachineInstr &MI) const {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Enter : MI = "; MI.dump(););
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of tests must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of tests must be an immediate constant");
  assert(MI.getOperand(2).isReg() && MI.getOperand(2).getReg() == MC6809::AQ && "The source of 32-bit register tests must be the Q register");

  // We do a 32-bit above stack store to set the flags like a TST instruction would.
  auto Store = Builder.buildInstr(MC6809::STQi_o5).addImm(-4).addUse(MC6809::SS);
  Store->addImplicitDefUseOperands(*MI.getMF());
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : Store : = "; Store->dump(););
  MI.eraseFromParent();
}

#if 0
MC6809CC::CondCode MC6809::GetBranchConditionForPredicate(CmpInst::Predicate Pred, bool &IsSigned) {
  IsSigned = CmpInst::isSigned(Pred);
  switch (Pred) {
  case CmpInst::FCMP_FALSE:
  case CmpInst::FCMP_UNO:
    return MC6809CC::CS;
  case CmpInst::FCMP_TRUE:
  case CmpInst::FCMP_ORD:
    return MC6809CC::CC;
  case CmpInst::FCMP_OEQ:
  case CmpInst::FCMP_UEQ:
  case CmpInst::ICMP_EQ:
    return MC6809CC::EQ;
  case CmpInst::FCMP_ONE:
  case CmpInst::FCMP_UNE:
  case CmpInst::ICMP_NE:
    return MC6809CC::NE;
  case CmpInst::ICMP_UGT:
    return MC6809CC::HI;
  case CmpInst::ICMP_ULT:
    return MC6809CC::LO;
  case CmpInst::ICMP_ULE:
    return MC6809CC::LS;
  case CmpInst::ICMP_UGE:
    return MC6809CC::HS;
  case CmpInst::FCMP_OGT:
  case CmpInst::FCMP_UGT:
  case CmpInst::ICMP_SGT:
    return MC6809CC::GT;
  case CmpInst::FCMP_OLT:
  case CmpInst::FCMP_ULT:
  case CmpInst::ICMP_SLT:
    return MC6809CC::LT;
  case CmpInst::FCMP_OLE:
  case CmpInst::FCMP_ULE:
  case CmpInst::ICMP_SLE:
    return MC6809CC::LE;
  case CmpInst::FCMP_OGE:
  case CmpInst::FCMP_UGE:
  case CmpInst::ICMP_SGE:
    return MC6809CC::GE;
  default:
    llvm_unreachable("Unknown predicate");
  }
}
#endif

bool MC6809InstrInfo::reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 1);
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

std::pair<unsigned, unsigned> MC6809InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const { return std::make_pair(TF, 0u); }

ArrayRef<std::pair<unsigned, const char *>> MC6809InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {{MC6809::MO_LO, "lo"}, {MC6809::MO_HI, "hi"}, {MC6809::MO_HI_JT, "hi-jt"}};
  return Flags;
}
