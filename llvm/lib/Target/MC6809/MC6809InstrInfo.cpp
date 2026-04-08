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
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"

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

MC6809InstrInfo::MC6809InstrInfo(const MC6809Subtarget &STI)
    : MC6809GenInstrInfo(STI, RI,
                         /*CFSetupOpcode=*/MC6809::ADJCALLSTACKDOWN,
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
  SubBorrowImmediateOpcode = {
      {{MC6809::AA}, MC6809::SBCAi8}, {{MC6809::AALSB}, MC6809::SBCAi8}, {{MC6809::AB}, MC6809::SBCBi8}, {{MC6809::ABLSB}, MC6809::SBCBi8}, {{MC6809::AD}, MC6809::SBCDi16},
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
  SubBorrowIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::SBCAi_o16}, {{MC6809::AA, 0}, MC6809::SBCAi_o0}, {{MC6809::AA, 5}, MC6809::SBCAi_o5}, {{MC6809::AA, 8}, MC6809::SBCAi_o8}, {{MC6809::AA, 16}, MC6809::SBCAi_o16},
      {{MC6809::AB, -1}, MC6809::SBCBi_o16}, {{MC6809::AB, 0}, MC6809::SBCBi_o0}, {{MC6809::AB, 5}, MC6809::SBCBi_o5}, {{MC6809::AB, 8}, MC6809::SBCBi_o8}, {{MC6809::AB, 16}, MC6809::SBCBi_o16},
      {{MC6809::AD, -1}, MC6809::SBCDi_o16}, {{MC6809::AD, 0}, MC6809::SBCDi_o0}, {{MC6809::AD, 5}, MC6809::SBCDi_o5}, {{MC6809::AD, 8}, MC6809::SBCDi_o8}, {{MC6809::AD, 16}, MC6809::SBCDi_o16},
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
  AddPullOpcode = {
      {{MC6809::AA}, MC6809::ADDAi_Inc1}, {{MC6809::AB}, MC6809::ADDBi_Inc1}, {{MC6809::AE}, MC6809::ADDEi_Inc1}, {{MC6809::AF}, MC6809::ADDFi_Inc1}, {{MC6809::AD}, MC6809::ADDDi_Inc2}, {{MC6809::AW}, MC6809::ADDWi_Inc2},
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
  CompareImmediateOpcode = {
      {{MC6809::AA}, MC6809::CMPAi8}, {{MC6809::AB}, MC6809::CMPBi8}, {{MC6809::AE}, MC6809::CMPEi8}, {{MC6809::AF}, MC6809::CMPFi8}, {{MC6809::AD}, MC6809::CMPDi16}, {{MC6809::AW}, MC6809::CMPWi16},
  };
  CompareIdxImmOpcode = {
      {{MC6809::AA, -1}, MC6809::CMPAi_o16}, {{MC6809::AA, 0}, MC6809::CMPAi_o0},   {{MC6809::AA, 5}, MC6809::CMPAi_o5},   {{MC6809::AA, 8}, MC6809::CMPAi_o8},   {{MC6809::AA, 16}, MC6809::CMPAi_o16}, {{MC6809::AB, -1}, MC6809::CMPBi_o16},
      {{MC6809::AB, 0}, MC6809::CMPBi_o0},   {{MC6809::AB, 5}, MC6809::CMPBi_o5},   {{MC6809::AB, 8}, MC6809::CMPBi_o8},   {{MC6809::AB, 16}, MC6809::CMPBi_o16}, {{MC6809::AD, -1}, MC6809::CMPDi_o16}, {{MC6809::AD, 0}, MC6809::CMPDi_o0},
      {{MC6809::AD, 5}, MC6809::CMPDi_o5},   {{MC6809::AD, 8}, MC6809::CMPDi_o8},   {{MC6809::AD, 16}, MC6809::CMPDi_o16}, {{MC6809::AE, -1}, MC6809::CMPEi_o16}, {{MC6809::AE, 0}, MC6809::CMPEi_o0},   {{MC6809::AE, 5}, MC6809::CMPEi_o5},
      {{MC6809::AE, 8}, MC6809::CMPEi_o8},   {{MC6809::AE, 16}, MC6809::CMPEi_o16}, {{MC6809::AF, -1}, MC6809::CMPFi_o16}, {{MC6809::AF, 0}, MC6809::CMPFi_o0},   {{MC6809::AF, 5}, MC6809::CMPFi_o5},   {{MC6809::AF, 8}, MC6809::CMPFi_o8},
      {{MC6809::AF, 16}, MC6809::CMPFi_o16}, {{MC6809::AW, -1}, MC6809::CMPWi_o16}, {{MC6809::AW, 0}, MC6809::CMPWi_o0},   {{MC6809::AW, 5}, MC6809::CMPWi_o5},   {{MC6809::AW, 8}, MC6809::CMPWi_o8},   {{MC6809::AW, 16}, MC6809::CMPWi_o16},
      {{MC6809::IX, -1}, MC6809::CMPXi_o16}, {{MC6809::IX, 0}, MC6809::CMPXi_o0},   {{MC6809::IX, 5}, MC6809::CMPXi_o5},   {{MC6809::IX, 8}, MC6809::CMPXi_o8},   {{MC6809::IX, 16}, MC6809::CMPXi_o16},
      {{MC6809::IY, -1}, MC6809::CMPYi_o16}, {{MC6809::IY, 0}, MC6809::CMPYi_o0},   {{MC6809::IY, 5}, MC6809::CMPYi_o5},   {{MC6809::IY, 8}, MC6809::CMPYi_o8},   {{MC6809::IY, 16}, MC6809::CMPYi_o16},
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
      {{MC6809::IX, MC6809::AA}, MC6809::CMPXi_oA}, {{MC6809::IX, MC6809::AB}, MC6809::CMPXi_oB}, {{MC6809::IX, MC6809::AD}, MC6809::CMPXi_oD}, {{MC6809::IX, MC6809::AE}, MC6809::CMPXi_oE}, {{MC6809::IX, MC6809::AF}, MC6809::CMPXi_oF},
      {{MC6809::IX, MC6809::AW}, MC6809::CMPXi_oW},
      {{MC6809::IY, MC6809::AA}, MC6809::CMPYi_oA}, {{MC6809::IY, MC6809::AB}, MC6809::CMPYi_oB}, {{MC6809::IY, MC6809::AD}, MC6809::CMPYi_oD}, {{MC6809::IY, MC6809::AE}, MC6809::CMPYi_oE}, {{MC6809::IY, MC6809::AF}, MC6809::CMPYi_oF},
      {{MC6809::IY, MC6809::AW}, MC6809::CMPYi_oW},
  };
  CompareImmediateOpcode = {
      {{MC6809::AA}, MC6809::CMPAi8}, {{MC6809::AB}, MC6809::CMPBi8}, {{MC6809::AE}, MC6809::CMPEi8}, {{MC6809::AF}, MC6809::CMPFi8}, {{MC6809::AD}, MC6809::CMPDi16}, {{MC6809::AW}, MC6809::CMPWi16},
      {{MC6809::IX}, MC6809::CMPXi16}, {{MC6809::IY}, MC6809::CMPYi16}, {{MC6809::SU}, MC6809::CMPUi16}, {{MC6809::SS}, MC6809::CMPSi16},
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
  ANDPullOpcode = {
      {{MC6809::AA}, MC6809::ANDAi_Inc1}, {{MC6809::AB}, MC6809::ANDBi_Inc1}, {{MC6809::AD}, MC6809::ANDDi_Inc2},
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
  ORPullOpcode = {
      {{MC6809::AA}, MC6809::ORAi_Inc1}, {{MC6809::AB}, MC6809::ORBi_Inc1}, {{MC6809::AD}, MC6809::ORDi_Inc2},
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
  XORPullOpcode = {
      {{MC6809::AA}, MC6809::EORAi_Inc1}, {{MC6809::AB}, MC6809::EORBi_Inc1}, {{MC6809::AD}, MC6809::EORDi_Inc2},
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
  if (I->getOpcode() == TargetOpcode::G_BR || I->getOpcode() == MC6809::BranchRelative || I->getOpcode() == MC6809::LongBranchRelative || I->getOpcode() == MC6809::JMPe || I->getOpcode() == MC6809::JMPi_o16PC)
    return I->getOperand(0).getMBB();
  if (I->getOpcode() == TargetOpcode::G_BRCOND || I->getOpcode() == MC6809::ConditionalBranchRelative || I->getOpcode() == MC6809::ConditionalLongBranchRelative || I->getOpcode() == MC6809::Bbc || I->getOpcode() == MC6809::LBlbc)
    return I->getOperand(1).getMBB();
  llvm_unreachable("Unable to handle opcode. Please fix me!");
}

Register MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  SmallVector<const MachineMemOperand *, 1> Accesses;
  if (MI.mayLoad() && hasLoadFromStackSlot(MI, Accesses) && Accesses.size() == 1) {
    FrameIndex = cast<FixedStackPseudoSourceValue>(Accesses.front()->getPseudoValue())->getFrameIndex();
    return MI.getOperand(0).getReg();
  }
  return 0;
}

Register MC6809InstrInfo::isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  SmallVector<const MachineMemOperand *, 1> Accesses;
  if (MI.mayStore() && hasStoreToStackSlot(MI, Accesses) && Accesses.size() == 1) {
    FrameIndex = cast<FixedStackPseudoSourceValue>(Accesses.front()->getPseudoValue())->getFrameIndex();
    return MI.getOperand(0).getReg();
  }
  return 0;
}

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator I, Register DestReg, unsigned SubIdx, const MachineInstr &Orig) const {
  auto opcode = Orig.getOpcode();
  if (opcode == MC6809::Load_i8_Imm || opcode == MC6809::Load_i16_Imm || opcode == MC6809::Load_i32_Imm) {
    const TargetRegisterInfo &TRI = getRegisterInfo();
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->removeOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(opcode));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig);
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

  const TargetRegisterClass *RegClass1 = getRegClass(MI.getDesc(), Idx1);
  const TargetRegisterClass *RegClass2 = getRegClass(MI.getDesc(), Idx2);
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
  return true;
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
  switch (MI.getOpcode()) {
  default:
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
  case MC6809::BranchJumpTable:
  case MC6809::JMPi_oDI:
  case MC6809::JMPi_oD:
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
      return true;
    }

    // Cannot handle branches that don't branch to a block.
    if (!I->getOperand(0).isMBB()) {
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
    return true;
  }

  return false;
}

unsigned MC6809InstrInfo::removeBranch(MachineBasicBlock &MBB, int *BytesRemoved) const {
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
  return Count;
}

unsigned MC6809InstrInfo::insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded) const {
  // Shouldn't be a fall through.
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
  return Count;
}

void MC6809InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB, MachineBasicBlock &NewDestBB, MachineBasicBlock &RestoreBB, const DebugLoc &DL, int64_t BrOffset, RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.
  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  Builder.buildInstr(MC6809::LBRAlb).addMBB(&NewDestBB);
}

/// Check if a register is a stack-backed spill pseudo-register.
static bool isSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return true;
  default:
    return false;
  }
}

/// Get the SPILL_D parent register for any spill register.
static MCPhysReg getSpillDParent(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_B0: case MC6809::SPILL_D0: return MC6809::SPILL_D0;
  case MC6809::SPILL_A1: case MC6809::SPILL_B1: case MC6809::SPILL_D1: return MC6809::SPILL_D1;
  case MC6809::SPILL_A2: case MC6809::SPILL_B2: case MC6809::SPILL_D2: return MC6809::SPILL_D2;
  case MC6809::SPILL_A3: case MC6809::SPILL_B3: case MC6809::SPILL_D3: return MC6809::SPILL_D3;
  case MC6809::SPILL_A4: case MC6809::SPILL_B4: case MC6809::SPILL_D4: return MC6809::SPILL_D4;
  case MC6809::SPILL_A5: case MC6809::SPILL_B5: case MC6809::SPILL_D5: return MC6809::SPILL_D5;
  case MC6809::SPILL_A6: case MC6809::SPILL_B6: case MC6809::SPILL_D6: return MC6809::SPILL_D6;
  case MC6809::SPILL_A7: case MC6809::SPILL_B7: case MC6809::SPILL_D7: return MC6809::SPILL_D7;
  case MC6809::SPILL_X0: return MC6809::SPILL_X0;
  case MC6809::SPILL_X1: return MC6809::SPILL_X1;
  case MC6809::SPILL_X2: return MC6809::SPILL_X2;
  case MC6809::SPILL_X3: return MC6809::SPILL_X3;
  default: llvm_unreachable("Not a spill register");
  }
}

/// Get byte offset within the 2-byte SPILL_D frame object.
/// Big-endian: A (high byte) at offset 0, B (low byte) at offset 1.
static int getSpillByteOffset(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
    return 0; // Full 16-bit, offset 0
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
    return 0; // High byte (big-endian)
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
    return 1; // Low byte (big-endian)
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return 0; // Full 16-bit, offset 0
  default: llvm_unreachable("Not a spill register");
  }
}

/// Get the corresponding real hardware register for a spill register.
static Register getRealRegForSpill(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
    return MC6809::AA;
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
    return MC6809::AB;
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
    return MC6809::AD;
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return MC6809::IX;
  default: llvm_unreachable("Not a spill register");
  }
}

/// Check if a spill register is an INDEX spill (uses LDX/STX, not LDD/STD).
static bool isIndexSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return true;
  default:
    return false;
  }
}

/// Get the size in bytes of a spill register (1 for A/B, 2 for D/X).
static unsigned getSpillRegSize(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
    return 1;
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return 2;
  default: llvm_unreachable("Not a spill register");
  }
}

/// Compute the actual stack offset for a spill register's frame slot.
/// This replicates the logic from eliminateFrameIndex so we can emit
/// concrete S-indexed instructions during post-RA expansion (after PEI
/// has already run and frame indices are no longer valid).
static int computeSpillStackOffset(MCPhysReg SpillReg, MachineFunction &MF) {
  auto &FuncInfo = *MF.getInfo<MC6809FunctionInfo>();
  // INDEX spill registers use themselves as the key (no parent register).
  MCPhysReg SpillKey = isIndexSpillReg(SpillReg) ? SpillReg : getSpillDParent(SpillReg);
  int FI = FuncInfo.SpillRegFrameIndices[SpillKey];
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  int Offset = MFI.getObjectOffset(FI);

  // Local stack objects have negative offsets; no PC skip needed.
  // (Fixed/arg objects have positive offsets and need +2 for return address,
  // but spill slots are always local.)

  // Compute callee-saved size — count ALL CSRs pushed to the hard stack.
  unsigned CalleeSavedSize = 0;
  for (const auto &CSI : MFI.getCalleeSavedInfo()) {
    const TargetRegisterInfo *TRI = MF.getRegInfo().getTargetRegisterInfo();
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(CSI.getReg());
    CalleeSavedSize += TRI->getSpillSize(*RC);
  }

  // Same formula for FP and non-FP — U is set to S after both allocation
  // and CSR pushes, so the offset from U includes both.
  Offset += MFI.getStackSize() + CalleeSavedSize;

  // Add byte offset within the 2-byte slot (0 for A/D, 1 for B).
  Offset += getSpillByteOffset(SpillReg);

  return Offset;
}

/// Pick the right indexed load opcode for a given register and offset.
static unsigned getLoadIdxOpcode(Register Reg, int Offset) {
  bool Is8 = (Offset >= -128 && Offset <= 127);
  if (Reg == MC6809::AA) return Is8 ? MC6809::LDAi_o8 : MC6809::LDAi_o16;
  if (Reg == MC6809::AB) return Is8 ? MC6809::LDBi_o8 : MC6809::LDBi_o16;
  if (Reg == MC6809::AD) return Is8 ? MC6809::LDDi_o8 : MC6809::LDDi_o16;
  if (Reg == MC6809::IX) return Is8 ? MC6809::LDXi_o8 : MC6809::LDXi_o16;
  if (Reg == MC6809::IY) return Is8 ? MC6809::LDYi_o8 : MC6809::LDYi_o16;
  llvm_unreachable("Unexpected register for spill load");
}

/// Pick the right indexed store opcode for a given register and offset.
static unsigned getStoreIdxOpcode(Register Reg, int Offset) {
  bool Is8 = (Offset >= -128 && Offset <= 127);
  if (Reg == MC6809::AA) return Is8 ? MC6809::STAi_o8 : MC6809::STAi_o16;
  if (Reg == MC6809::AB) return Is8 ? MC6809::STBi_o8 : MC6809::STBi_o16;
  if (Reg == MC6809::AD) return Is8 ? MC6809::STDi_o8 : MC6809::STDi_o16;
  if (Reg == MC6809::IX) return Is8 ? MC6809::STXi_o8 : MC6809::STXi_o16;
  if (Reg == MC6809::IY) return Is8 ? MC6809::STYi_o8 : MC6809::STYi_o16;
  llvm_unreachable("Unexpected register for spill store");
}

/// Emit a concrete U-indexed (frame pointer) load from a spill register's
/// stack slot. Uses U (not S) so PSHS/PULS don't invalidate offsets.
static MachineInstrBuilder emitSpillLoad(MachineIRBuilder &Builder,
                                         Register RealReg, MCPhysReg SpillReg,
                                         MachineFunction &MF) {
  int Offset = computeSpillStackOffset(SpillReg, MF);
  unsigned Size = getSpillRegSize(SpillReg);
  Register Reg = (Size == 2) ? Register(MC6809::AD) : RealReg;
  unsigned Opcode = getLoadIdxOpcode(Reg, Offset);
  auto MI = Builder.buildInstr(Opcode)
      .addDef(Reg, RegState::Implicit)
      .addImm(Offset)
      .addReg(MC6809::SU);  // Frame pointer — stable across PSHS/PULS
  return MI;
}

/// Emit a concrete U-indexed (frame pointer) store to a spill register's
/// stack slot. Uses U (not S) so PSHS/PULS don't invalidate offsets.
static MachineInstrBuilder emitSpillStore(MachineIRBuilder &Builder,
                                          Register RealReg, MCPhysReg SpillReg,
                                          MachineFunction &MF) {
  int Offset = computeSpillStackOffset(SpillReg, MF);
  unsigned Size = getSpillRegSize(SpillReg);
  Register Reg = (Size == 2) ? Register(MC6809::AD) : RealReg;
  unsigned Opcode = getStoreIdxOpcode(Reg, Offset);
  auto MI = Builder.buildInstr(Opcode)
      .addUse(Reg, RegState::Implicit)
      .addImm(Offset)
      .addReg(MC6809::SU);  // Frame pointer — stable across PSHS/PULS
  return MI;
}

 void  MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, Register DestReg, Register SrcReg, bool KillSrc, bool RenamableDest, bool RenamableSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  if (DestReg == SrcReg)
    return;

  // Handle copies involving stack-backed spill pseudo-registers.
  // Emit concrete S-indexed instructions (not pseudos with frame indices,
  // since PEI has already run by the time copyPhysReg is called).
  if (isSpillReg(DestReg) || isSpillReg(SrcReg)) {
    MachineFunction &MF = *MBB.getParent();
    if (isSpillReg(DestReg) && !isSpillReg(SrcReg)) {
      // Real → Spill: Store to spill slot.
      if (isIndexSpillReg(DestReg)) {
        // INDEX spill: use IY as staging (callee-saved, avoids IX conflicts).
        int Offset = computeSpillStackOffset(DestReg, MF);
        Register StageReg = MC6809::IY;
        if (SrcReg != StageReg)
          Builder.buildInstr(MC6809::TFRp).addDef(StageReg).addUse(SrcReg);
        unsigned Opcode = getStoreIdxOpcode(StageReg, Offset);
        Builder.buildInstr(Opcode)
            .addUse(StageReg, RegState::Implicit)
            .addImm(Offset)
            .addReg(MC6809::SU);
      } else if (SrcReg == MC6809::IX || SrcReg == MC6809::IY) {
        // INDEX → ACC spill: use STX/STY directly (no D clobber).
        int Offset = computeSpillStackOffset(DestReg, MF);
        unsigned Opcode = getStoreIdxOpcode(SrcReg, Offset);
        Builder.buildInstr(Opcode)
            .addUse(SrcReg, RegState::Implicit)
            .addImm(Offset)
            .addReg(MC6809::SU);
      } else {
        Register RealAcc = getRealRegForSpill(DestReg);
        if (SrcReg != RealAcc)
          Builder.buildInstr(MC6809::TFRp).addDef(RealAcc).addUse(SrcReg);
        emitSpillStore(Builder, RealAcc, DestReg, MF);
      }
    } else if (!isSpillReg(DestReg) && isSpillReg(SrcReg)) {
      // Spill → Real: Load from spill slot.
      if (isIndexSpillReg(SrcReg)) {
        // INDEX spill → Real: use IY as staging (callee-saved).
        int Offset = computeSpillStackOffset(SrcReg, MF);
        Register StageReg = MC6809::IY;
        unsigned Opcode = getLoadIdxOpcode(StageReg, Offset);
        Builder.buildInstr(Opcode)
            .addDef(StageReg, RegState::Implicit)
            .addImm(Offset)
            .addReg(MC6809::SU);
        if (DestReg != StageReg)
          Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(StageReg);
      } else if (DestReg == MC6809::IX || DestReg == MC6809::IY) {
        // ACC spill → INDEX: use LDX/LDY directly (no D clobber).
        int Offset = computeSpillStackOffset(SrcReg, MF);
        unsigned Opcode = getLoadIdxOpcode(DestReg, Offset);
        Builder.buildInstr(Opcode)
            .addDef(DestReg, RegState::Implicit)
            .addImm(Offset)
            .addReg(MC6809::SU);
      } else {
        Register RealAcc = getRealRegForSpill(SrcReg);
        emitSpillLoad(Builder, RealAcc, SrcReg, MF);
        if (DestReg != RealAcc)
          Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(RealAcc);
      }
    } else {
      // Spill → Spill: Load to real accumulator, store to dest slot.
      Register TmpReal = getRealRegForSpill(SrcReg);
      emitSpillLoad(Builder, TmpReal, SrcReg, MF);
      emitSpillStore(Builder, TmpReal, DestReg, MF);
    }
    return;
  }

  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() && !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) { return IsClass(DestReg, Dest) && IsClass(SrcReg, Src); };

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
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass) ||
             AreClasses(MC6809::STACK16RegClass, MC6809::STACK16RegClass) ||
             AreClasses(MC6809::STACK16RegClass, MC6809::INDEX16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::STACK16RegClass) ||
             AreClasses(MC6809::STACK16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::STACK16RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCFlagRegClass)) {
    // TODO: May need AND #0x0F to mask EFHI bits if callers expect only NZVC.
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::CCFlagRegClass, MC6809::ACC8RegClass)) {
    // TODO: May need AND #0x0F to mask EFHI bits if callers expect only NZVC.
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass) || AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::BIT1RegClass)) {
    // BIT1 → ACC8: the boolean value is in the LSB of an ACC8 register.
    // Copy the byte and mask to ensure only bit 0 is set.
    const TargetRegisterInfo *TRI = Builder.getMRI()->getTargetRegisterInfo();
    SrcReg = TRI->getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    if (!SrcReg) SrcReg = MI->getOperand(1).getReg(); // fallback
    if (DestReg != SrcReg)
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
    Builder.buildInstr(MC6809::AND_i8_Imm).addDef(DestReg).addUse(DestReg).addImm(1);
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::ACC8RegClass)) {
    // ACC8 → BIT1: just copy the byte (the LSB is the boolean).
    const TargetRegisterInfo *TRI = Builder.getMRI()->getTargetRegisterInfo();
    DestReg = TRI->getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    if (!DestReg) DestReg = MI->getOperand(0).getReg();
    if (DestReg != SrcReg)
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::Imag8RegClass, MC6809::ACC8RegClass)) {
    // ACC8 → Imag8: store accumulator to direct-page imaginary register.
    unsigned StoreOpc;
    if (SrcReg == MC6809::AA)
      StoreOpc = MC6809::STAd;
    else if (SrcReg == MC6809::AB)
      StoreOpc = MC6809::STBd;
    else {
      // Other ACC8 (e.g., AE, AF): TFR to AA first, then store.
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AA).addUse(SrcReg);
      StoreOpc = MC6809::STAd;
    }
    Builder.buildInstr(StoreOpc).addReg(DestReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::Imag8RegClass)) {
    // Imag8 → ACC8: load from direct-page imaginary register.
    unsigned LoadOpc;
    if (DestReg == MC6809::AA)
      LoadOpc = MC6809::LDAd;
    else if (DestReg == MC6809::AB)
      LoadOpc = MC6809::LDBd;
    else {
      // Other ACC8: load to AA, then TFR.
      Builder.buildInstr(MC6809::LDAd).addReg(SrcReg);
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::AA);
      return;
    }
    Builder.buildInstr(LoadOpc).addReg(SrcReg);
  } else if (AreClasses(MC6809::Imag16RegClass, MC6809::ACC16RegClass)) {
    // ACC16 → Imag16: store D to direct-page imaginary register.
    if (SrcReg != MC6809::AD)
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AD).addUse(SrcReg);
    Builder.buildInstr(MC6809::STDd).addReg(DestReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → ACC16: load D from direct-page imaginary register.
    Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
    if (DestReg != MC6809::AD)
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::AD);
  } else if (AreClasses(MC6809::Imag8RegClass, MC6809::Imag8RegClass)) {
    // Imag8 → Imag8: load to AA, store to dest.
    Builder.buildInstr(MC6809::LDAd).addReg(SrcReg);
    Builder.buildInstr(MC6809::STAd).addReg(DestReg);
  } else if (AreClasses(MC6809::Imag16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → Imag16: load to D, store to dest.
    Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
    Builder.buildInstr(MC6809::STDd).addReg(DestReg);
  } else if (AreClasses(MC6809::Imag16RegClass, MC6809::INDEX16RegClass) ||
             AreClasses(MC6809::Imag16RegClass, MC6809::STACK16RegClass)) {
    // INDEX16/STACK16 → Imag16: TFR to D, store D.
    Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AD).addUse(SrcReg);
    Builder.buildInstr(MC6809::STDd).addReg(DestReg);
  } else if (AreClasses(MC6809::INDEX16RegClass, MC6809::Imag16RegClass) ||
             AreClasses(MC6809::STACK16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → INDEX16/STACK16: load D, TFR to dest.
    Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::AD);
  } else
    llvm_unreachable("Unexpected physical register copy.");
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

void MC6809InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg, bool isKill, int FrameIndex, const TargetRegisterClass *RC, Register VReg, MachineInstr::MIFlag Flags) const {
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, /*IsLoad=*/false);
}

void MC6809InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg, int FrameIndex, const TargetRegisterClass *RC, Register VReg, unsigned SubReg, MachineInstr::MIFlag Flags) const {
  loadStoreRegStackSlot(MBB, MI, DestReg, false, FrameIndex, RC, /*IsLoad=*/true);
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
    else if (MC6809::CCFlagRegClass.contains(Reg) || MC6809::ACC8RegClass.contains(Reg))
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
    else if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::CCFlagRegClass))
      Size = 8;
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
  if (Reg.isPhysical() && MC6809::BIT1RegClass.contains(Reg)) {
    Reg = TRI.getMatchingSuperReg(Reg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    MO.setReg(Reg);
  }

  // Emit directly through ACC if possible.
  if ((Reg.isPhysical() && (MC6809::ACC8RegClass.contains(Reg) || MC6809::ACC16RegClass.contains(Reg) || MC6809::ACC32RegClass.contains(Reg))) ||
      (Reg.isVirtual() && (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC8RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC32RegClass)))) {
    unsigned opcode;
    switch (Size) {
    default:
      llvm_unreachable("Unknown register size");
    case 1:
    case 8:
      opcode = MO.isDef() ? MC6809::Load_i8_Mem : MC6809::Store_i8_Mem;
      break;
    case 16:
      opcode = MO.isDef() ? MC6809::Load_i16_Mem : MC6809::Store_i16_Mem;
      break;
    case 32:
      opcode = MO.isDef() ? MC6809::Load_i32_Mem : MC6809::Store_i32_Mem;
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

void MC6809InstrInfo::loadStoreRegStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg, bool IsKill, int FrameIndex, const TargetRegisterClass *RC, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  if ((Reg.isPhysical() && MC6809::INDEX16RegClass.contains(Reg)) ||
      (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::INDEX16RegClass))) {
    // INDEX16 registers (X, Y, U, S) save/restore via TFR to/from D
    // then Store/Load D. This avoids creating an intermediate ACC16
    // virtual register that the scavenger might assign to a spill
    // pseudo-register (which can't be pushed/pulled).
    if (IsLoad) {
      loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(MC6809::AD, /*isDef=*/true), FrameIndex, 0, MF.getMachineMemOperand(MMO, 0, 2));
      Builder.buildInstr(MC6809::TFRp).addDef(Reg).addUse(MC6809::AD);
    } else {
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AD).addUse(Reg);
      loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(MC6809::AD, /*isDef=*/false), FrameIndex, 0, MF.getMachineMemOperand(MMO, 0, 2));
    }
  } else if ((Reg.isPhysical() && MC6809::ACC16RegClass.contains(Reg)) || (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))) {
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
  MachineIRBuilder Builder(MI);

  if (!MI.isPseudo()) {
    return false;
  }
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
  case MC6809::ZEX8Implicit:
    unsigned Opcode;
    switch (MI.getOperand(1).getReg()) {
    case MC6809::AALSB:
      Opcode = MC6809::ANDAi8;
      break;
    case MC6809::ABLSB:
      Opcode = MC6809::ANDBi8;
      break;
    }
    MI.setDesc(Builder.getTII().get(Opcode));
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(1));
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
    expandNegate(Builder, MI);
    break;
  case MC6809::LSL_i8_Reg:
  case MC6809::LSL_i16_Reg:
    expandShiftLeft(Builder, MI);
    break;
  case MC6809::LSR_i8_Reg:
  case MC6809::LSR_i16_Reg:
    expandShiftRight(Builder, MI, /*Arithmetic=*/false);
    break;
  case MC6809::ASR_i8_Reg:
  case MC6809::ASR_i16_Reg:
    expandShiftRight(Builder, MI, /*Arithmetic=*/true);
    break;
  case MC6809::ROL_i8_Reg:
    expandRotate(Builder, MI, /*Left=*/true);
    break;
  case MC6809::ROR_i8_Reg:
    expandRotate(Builder, MI, /*Left=*/false);
    break;
  case MC6809::MUL_D:
    expandMulD(Builder, MI);
    break;
  case MC6809::Multiply_i16_Imm:
    expandMul16Imm(Builder, MI);
    break;
  case MC6809::MultiplyHigh_i16_Imm:
    expandMulH16Imm(Builder, MI);
    break;
  case MC6809::Multiply_i16_Mem:
    expandMul16IdxImm(Builder, MI);
    break;
  case MC6809::MultiplyHigh_i16_Mem:
    expandMulH16IdxImm(Builder, MI);
    break;
  case MC6809::Multiply_i16_Reg:
    expandMul16Reg(Builder, MI);
    break;
  case MC6809::MultiplyHigh_i16_Reg:
    expandMulH16Reg(Builder, MI);
    break;
  case MC6809::BranchSubroutine:
    expandCallRelative(Builder, MI);
    break;
  case MC6809::BranchJumpTable:
    // Keep as pseudo — expanded in MCInstLower to avoid branch relaxation
    // seeing the concrete JMPi_oDI (which has isBranch and crashes verify).
    break;
  case MC6809::LEAPtrAdd_Imm:
  case MC6809::LEA_Ptr_Imm:
  case MC6809::LEAPtrAdd_Reg8:
  case MC6809::LEAPtrAdd_Reg16:
    expandLEAPtrAdd(Builder, MI);
    break;
  case MC6809::Load_i8_Imm:
  case MC6809::Load_i16_Imm:
  case MC6809::Load_iPtr_Imm:
  case MC6809::Load_i32_Imm:
    expandLoadImm(Builder, MI);
    break;
  case MC6809::Load_i8_Mem:
  case MC6809::Load_i16_Mem:
  case MC6809::Load_i32_Mem:
  case MC6809::Load_iPtr_Mem:
    expandLoadIdx(Builder, MI);
    break;
  case MC6809::Store_i8_Mem:
  case MC6809::Store_i16_Mem:
  case MC6809::Store_i32_Mem:
  case MC6809::Store_iPtr_Mem:
    expandStoreIdx(Builder, MI);
    break;
  case MC6809::AND_i1_Imm:
  case MC6809::AND_i8_Imm:
    expandImm(ANDImm, Builder, MI);
    break;
  case MC6809::AND_i16_Imm:
    expandBitwiseImm16(ANDImm, MC6809::ANDAi8, MC6809::ANDBi8, Builder, MI);
    break;
  case MC6809::AND_i8_Mem:
    expandIdxImm(ANDIdxImm, Builder, MI);
    break;
  case MC6809::AND_i16_Mem:
    expandBitwiseMem16(ANDIdxImm, Builder, MI);
    break;
  case MC6809::AND_i8_Reg:
  case MC6809::AND_i16_Reg:
    expandANDReg(Builder, MI);
    break;
  case MC6809::AND_i8_Pull:
  case MC6809::AND_i16_Pull:
    expandANDPull(Builder, MI);
    break;
  case MC6809::OR_i8_Imm:
    expandImm(ORImm, Builder, MI);
    break;
  case MC6809::OR_i16_Imm:
    expandBitwiseImm16(ORImm, MC6809::ORAi8, MC6809::ORBi8, Builder, MI);
    break;
  case MC6809::OR_i8_Mem:
    expandIdxImm(ORIdxImm, Builder, MI);
    break;
  case MC6809::OR_i16_Mem:
    expandBitwiseMem16(ORIdxImm, Builder, MI);
    break;
  case MC6809::OR_i8_Reg:
  case MC6809::OR_i16_Reg:
    expandORReg(Builder, MI);
    break;
  case MC6809::OR_i8_Pull:
  case MC6809::OR_i16_Pull:
    expandORPull(Builder, MI);
    break;
  case MC6809::XOR_i8_Imm:
    expandImm(XORImm, Builder, MI);
    break;
  case MC6809::XOR_i16_Imm:
    expandBitwiseImm16(XORImm, MC6809::EORAi8, MC6809::EORBi8, Builder, MI);
    break;
  case MC6809::XOR_i8_Mem:
    expandIdxImm(XORIdxImm, Builder, MI);
    break;
  case MC6809::XOR_i16_Mem:
    expandBitwiseMem16(XORIdxImm, Builder, MI);
    break;
  case MC6809::XOR_i8_Reg:
  case MC6809::XOR_i16_Reg:
    expandXORReg(Builder, MI);
    break;
  case MC6809::XOR_i8_Pull:
  case MC6809::XOR_i16_Pull:
    expandXORPull(Builder, MI);
    break;
  case MC6809::Add_i8_Imm:
  case MC6809::Add_i16_Imm:
  case MC6809::AddSetCarry_i8_Imm:
  case MC6809::AddSetCarry_i16_Imm:
    expandImm(AddImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i8_Imm:
    expandImm(AddCarryImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i16_Imm:
    expandCarryImm16(true, Builder, MI);
    break;
  case MC6809::Add_i8_Mem:
  case MC6809::Add_i16_Mem:
  case MC6809::AddSetCarry_i8_Mem:
  case MC6809::AddSetCarry_i16_Mem:
    expandIdxImm(AddIdxImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i8_Mem:
    expandIdxImm(AddCarryIdxImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i16_Mem:
    expandCarryMem16(true, Builder, MI);
    break;
  case MC6809::Add_i8_Reg:
  case MC6809::Add_i16_Reg:
    expandAddReg(Builder, MI);
    break;
  case MC6809::AddSetCarry_i8_Reg:
  case MC6809::AddSetCarry_i16_Reg:
    expandAddSetCarryReg(Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i8_Reg:
  case MC6809::AddSetCarryUse_i16_Reg:
    expandAddSetCarryUseReg(Builder, MI);
    break;
  case MC6809::Sub_i8_Imm:
  case MC6809::Sub_i16_Imm:
  case MC6809::SubSetCarry_i8_Imm:
  case MC6809::SubSetCarry_i16_Imm:
    expandImm(SubImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Imm:
    expandImm(SubBorrowImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i16_Imm:
    expandCarryImm16(false, Builder, MI);
    break;
  case MC6809::Sub_i8_Mem:
  case MC6809::Sub_i16_Mem:
  case MC6809::SubSetCarry_i8_Mem:
  case MC6809::SubSetCarry_i16_Mem:
    expandIdxImm(SubIdxImm, Builder, MI);
    break;
  case MC6809::Sub_i8_Reg:
  case MC6809::Sub_i16_Reg:
    expandSubReg(Builder, MI);
    break;
  case MC6809::SubSetCarry_i8_Reg:
  case MC6809::SubSetCarry_i16_Reg:
    expandSubSetCarryReg(Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Mem:
    expandIdxImm(SubBorrowIdxImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i16_Mem:
    expandCarryMem16(false, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Reg:
  case MC6809::SubSetCarryUse_i16_Reg:
    expandSubSetCarryUseReg(Builder, MI);
    break;
  case MC6809::Add_i8_Pull:
  case MC6809::Add_i16_Pull:
    expandAddPull(Builder, MI);
    break;
  case MC6809::Sub_i8_Pull:
  case MC6809::Sub_i16_Pull:
    expandSubPull(Builder, MI);
    break;
  case MC6809::Compare_i8_Imm:
  case MC6809::Compare_i16_Imm:
  case MC6809::Compare_ptr_Imm:
    expandCompareImm(Builder, MI);
    break;
  case MC6809::Compare_i8_Mem:
  case MC6809::Compare_i16_Mem:
  case MC6809::Compare_ptr_Mem:
    expandCompareIdx(Builder, MI);
    break;
  case MC6809::Compare_i8_Pull:
  case MC6809::Compare_i16_Pull:
  case MC6809::Compare_ptr_Pull:
    expandComparePull(Builder, MI);
    break;
  case MC6809::Compare_i8_Reg:
  case MC6809::Compare_i16_Reg:
  case MC6809::Compare_ptr_Reg:
    expandCompareReg(Builder, MI);
    break;
  case MC6809::Test_i8_Reg:
  case MC6809::Test_i16_Reg:
    expandTestReg(Builder, MI);
    break;
  case MC6809::Copy8:
  case MC6809::Copy16:
    MI.setDesc(Builder.getTII().get(MC6809::TFRp));
    break;
  case MC6809::Push_i8:
  case MC6809::Push_i16:
  case MC6809::Push_Ptr: {
    // Operand 0 = stack register (def), operand 1 = value to push (use).
    Register PushReg = MI.getOperand(1).getReg();
    // If pushing a spill register, load into an INDEX register and push that
    // to avoid clobbering D (which may hold a live value).
    if (isSpillReg(PushReg) && getSpillRegSize(PushReg) == 2) {
      MachineFunction &MF = *MI.getMF();
      int Offset = computeSpillStackOffset(PushReg, MF);
      unsigned LoadOpc = getLoadIdxOpcode(MC6809::IX, Offset);
      MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
      PreBuilder.buildInstr(LoadOpc)
          .addDef(MC6809::IX, RegState::Implicit)
          .addImm(Offset).addReg(MC6809::SU);
      MI.getOperand(1).setReg(MC6809::IX);
    } else if (isSpillReg(PushReg)) {
      Register RealReg = getRealRegForSpill(PushReg);
      MachineFunction &MF = *MI.getMF();
      MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
      emitSpillLoad(PreBuilder, RealReg, PushReg, MF);
      MI.getOperand(1).setReg(RealReg);
    }
    MI.setDesc(Builder.getTII().get(MC6809::PSHSs));
    MI.getOperand(0).setReg(MC6809::SS);
    MI.getOperand(0).setImplicit();
    break;
  }
  case MC6809::Pull_i8:
  case MC6809::Pull_i16:
  case MC6809::Pull_Ptr: {
    auto PullReg = MI.getOperand(1).getReg();
    // If pulling into a spill register, pull into real accumulator then store.
    bool PullToSpill = isSpillReg(PullReg);
    Register OrigPullReg = PullReg;
    if (PullToSpill) {
      PullReg = getRealRegForSpill(PullReg);
      MI.getOperand(0).setReg(PullReg);
    }
    if (PullReg == MC6809::AQ) {
      MI.setDesc(Builder.getTII().get(MC6809::PULSWx));
      MI.removeOperand(0);
      Builder.buildInstr(MC6809::PULSs).addImm(0x06); // PULS D
      break;
    } else if (PullReg == MC6809::AW) {
      MI.setDesc(Builder.getTII().get(MC6809::PULSWx));
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
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(regList));
    // If the pull target was a spill register, store from real reg to spill slot.
    if (PullToSpill) {
      MachineFunction &MF = *MI.getMF();
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      emitSpillStore(PostBuilder, PullReg, OrigPullReg, MF);
    }
    break;
  }
  }
  return Changed;
}

//===---------------------------------------------------------------------===//
// Post RA pseudos
//===---------------------------------------------------------------------===//

static int offsetSizeInBitsForValue(int64_t Offset) {
  return (Offset == 0) ? 0 : ((Offset >= -16 && Offset < 16) ? 5 : ((Offset >= -128 && Offset < 128) ? 8 : ((Offset >= -32768 && Offset < 32768) ? 16 : 256)));
}

int MC6809InstrInfo::offsetSizeInBits(MachineOperand &OffsetOp) {
  int64_t Offset;
  if (OffsetOp.isImm())
    Offset = OffsetOp.getImm();
  else if (OffsetOp.isCImm())
    Offset = OffsetOp.getCImm()->getSExtValue();
  else
    return -1;
  return offsetSizeInBitsForValue(Offset);
}

void MC6809InstrInfo::expandCallRelative(MachineIRBuilder &Builder, MachineInstr &MI) const {
  MI.setDesc(Builder.getTII().get(MC6809::BSRb));
}

void MC6809InstrInfo::expandLEAPtrAdd(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto IndexReg = MI.getOperand(0);
  auto IndexOp = MI.getOperand(1);
  auto OffsetOp = MI.getOperand(2);
  MI.removeOperand(2);
  MI.removeOperand(1);

  // If the RESULT register is an INDEX spill, use IY as staging and store back.
  // Only pre-load from spill if the BASE (operand 1) is the SAME spill register
  // (modify-in-place). For pure definitions (base is frame index or different
  // register), don't pre-load — the spill slot isn't initialized yet.
  Register OrigSpillReg;
  if (isIndexSpillReg(IndexReg.getReg())) {
    OrigSpillReg = IndexReg.getReg();
    Register StageReg = MC6809::IY;
    bool SameSpillBase = (IndexOp.isReg() && IndexOp.getReg() == OrigSpillReg);
    if (SameSpillBase) {
      // Modify-in-place: load current spill value into IY.
      MachineFunction &MF = *MI.getMF();
      int SpillOff = computeSpillStackOffset(OrigSpillReg, MF);
      unsigned LoadOpc = getLoadIdxOpcode(StageReg, SpillOff);
      Builder.buildInstr(LoadOpc)
          .addDef(StageReg, RegState::Implicit)
          .addImm(SpillOff).addReg(MC6809::SU);
      // Rewrite base to IY.
      IndexOp = MachineOperand::CreateReg(StageReg, false);
    }
    // Rewrite result to IY (for both modify-in-place and pure definition).
    IndexReg = MachineOperand::CreateReg(StageReg, true);
    MI.getOperand(0).setReg(StageReg);
  }

  // Check register offset first (LEAPtrAdd_Reg8/Reg16) — offsetSizeInBits
  // crashes on register operands.
  if (OffsetOp.isReg()) {
    Register OffsetReg = OffsetOp.getReg();
    // If the offset is a spill register, load into real accumulator first.
    if (isSpillReg(OffsetReg)) {
      Register RealReg = getRealRegForSpill(OffsetReg);
      MachineFunction &MF = *MI.getMF();
      MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
      emitSpillLoad(PreBuilder, RealReg, OffsetReg, MF);
      OffsetOp = MachineOperand::CreateReg(RealReg, false);
      OffsetReg = RealReg;
    }
    RegPlusReg Lookup{IndexReg.getReg(), OffsetReg};
    auto OpcodePair = LEAPtrAddRegOpcode.find(Lookup);
    if (OpcodePair == LEAPtrAddRegOpcode.end())
      llvm_unreachable("Unexpected LEAPtrAdd register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    // The concrete indexed-offset-reg instruction (e.g. LEAXi_oD) has:
    //   - Result register as implicit def (from Defs list)
    //   - Offset register as implicit use (from Uses list)
    //   - One explicit operand: $ireg (the base/index register)
    MI.getOperand(0).setImplicit();  // result becomes implicit
    MI.addOperand(IndexOp);          // base register as sole explicit operand
    // Don't return — fall through to the post-store for SPILL_X.
  } else {

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{IndexReg.getReg(), OffsetSize};
    auto OpcodePair = LEAPtrAddImmOpcode.find(Lookup);
    if (OpcodePair == LEAPtrAddImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexOp);
  } else
    llvm_unreachable("Unknown offset type for LEAPtrAdd");
  }

  // If the original register was a spill, store staging reg back to spill slot.
  if (OrigSpillReg.isValid()) {
    Register StageReg = MC6809::IY;  // Must match the staging register above
    MachineFunction &MF = *MI.getMF();
    int SpillOff = computeSpillStackOffset(OrigSpillReg, MF);
    MachineBasicBlock::iterator After = std::next(MI.getIterator());
    MachineIRBuilder PostBuilder(*MI.getParent(), After);
    unsigned StoreOpc = getStoreIdxOpcode(StageReg, SpillOff);
    PostBuilder.buildInstr(StoreOpc)
        .addUse(StageReg, RegState::Implicit)
        .addImm(SpillOff).addReg(MC6809::SU);
  }
}

void MC6809InstrInfo::expandImm(ContextImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto operandCount = MI.getNumExplicitOperands();
  auto DestReg = MI.getOperand(0).getReg();
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    MI.getOperand(0).setReg(RealReg);
    if (operandCount >= 3 && MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == DestReg)
      MI.getOperand(1).setReg(RealReg);
    DestReg = RealReg;
  }
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val;

  if (ValOp.isImm() || ValOp.isCImm())
    Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  else
    llvm_unreachable("Unable to determine immediate value");
  if (Val != Context.IdentityValue) {
    auto OpcodePair = Context.Opcode->find(DestReg);
    if (OpcodePair == Context.Opcode->end()) {
      if (DestReg == MC6809::AW) {
        // Now we cheat!
        OpcodePair = Context.Opcode->find(MC6809::AD);
        assert((OpcodePair != Context.Opcode->end()) && "This should not be reached! We have the D register available.");
        MachineBasicBlock &MBB = *MI.getParent();
        MachineBasicBlock::iterator B, E;
        B = Builder.buildInstr(MC6809::EXGp).addDef(MC6809::AD).addDef(DestReg).addUse(MC6809::AD).addUse(DestReg);
        Builder.buildInstr(OpcodePair->getSecond()).addDef(MC6809::AD, RegState::Implicit).addImm(Val);
        E = Builder.buildInstr(MC6809::EXGp).addDef(DestReg).addDef(MC6809::AD).addUse(DestReg).addUse(MC6809::AD);
        auto Bundler = MIBundleBuilder(MBB, B, ++E);
        finalizeBundle(MBB, Bundler.begin(), Bundler.end());
        LLVM_DEBUG(for (auto &I : Bundler) {
          I.dump();
        });
      } else
        llvm_unreachable("Cannot find machine instruction with this immediate operand");
    } else {
      Builder.buildInstr(OpcodePair->getSecond()).addDef(DestReg, RegState::Implicit).addImm(Val);
    }
  }
  // Store result back to spill slot BEFORE erasing MI.
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock &MBB = *MI.getParent();
    auto NextIt = std::next(MachineBasicBlock::iterator(MI));
    MachineIRBuilder StoreBuilder(MBB, NextIt);
    emitSpillStore(StoreBuilder, DestReg, OrigSpillReg, MF);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandIdxImm(ContextIndexImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto operandCount = MI.getNumExplicitOperands();
  auto DestReg = MI.getOperand(0).getReg();
  // If destination is a spill register, load into real accumulator, operate,
  // then store back.
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    MI.getOperand(0).setReg(RealReg);
    // Also fix the tied source operand (operand 1 for most arith pseudos).
    if (operandCount >= 3 && MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == DestReg)
      MI.getOperand(1).setReg(RealReg);
    DestReg = RealReg;
  }
  auto IndexReg = MI.getOperand(operandCount - 2).getReg();
  auto OffsetOp = MI.getOperand(operandCount - 1);
  assert((OffsetOp.isImm() || OffsetOp.isCImm()) && "This offset must be an immediate");

  int OffsetSize = offsetSizeInBits(OffsetOp);
  assert((OffsetSize >= 0) && "Unknown immediate offset size");
  auto Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
  RegPlusOffsetLen Lookup{DestReg, OffsetSize};
  auto OpcodePair = Context.Opcode->find(Lookup);
  if (OpcodePair == Context.Opcode->end()) {
    if (DestReg == MC6809::AW) {
      // Now we cheat!
      RegPlusOffsetLen Lookup1{MC6809::AD, OffsetSize};
      OpcodePair = Context.Opcode->find(Lookup1);
      assert((OpcodePair != Context.Opcode->end()) && "This should not be reached! We have the D register available.");
      MachineBasicBlock &MBB = *MI.getParent();
      MachineBasicBlock::iterator B, E;
      B = Builder.buildInstr(MC6809::EXGp).addDef(MC6809::AD).addDef(DestReg).addUse(MC6809::AD).addUse(DestReg);
      auto Instr = Builder.buildInstr(OpcodePair->getSecond()).addDef(MC6809::AD, RegState::Implicit);
      if (OffsetSize == 0)
        Instr.addReg(IndexReg);
      else
        Instr.addImm(Offset).addReg(IndexReg);
      E = Builder.buildInstr(MC6809::EXGp).addDef(DestReg).addDef(MC6809::AD).addUse(DestReg).addUse(MC6809::AD);
      auto Bundler = MIBundleBuilder(MBB, B, ++E);
      finalizeBundle(MBB, Bundler.begin(), Bundler.end());
      LLVM_DEBUG(for (auto &I : Bundler) {
        I.dump();
      });
    } else
      llvm_unreachable("Cannot find machine instruction with these immediate indexed operands");
  } else {
    auto Instr = Builder.buildInstr(OpcodePair->getSecond()).addDef(DestReg, RegState::Implicit);
    if (OffsetSize == 0)
      Instr.addReg(IndexReg);
    else
      Instr.addImm(Offset).addReg(IndexReg);
  }
  // Store result back to spill slot BEFORE erasing MI.
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock &MBB = *MI.getParent();
    // Insert after MI (which is about to be erased, but is still in the MBB).
    auto NextIt = std::next(MachineBasicBlock::iterator(MI));
    MachineIRBuilder StoreBuilder(MBB, NextIt);
    emitSpillStore(StoreBuilder, DestReg, OrigSpillReg, MF);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCarryImm16(bool IsAdd, MachineIRBuilder &Builder,
                                       MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    // 6309 has ADCD/SBCD — use the standard expand path.
    expandImm(IsAdd ? AddCarryImm : SubBorrowImm, Builder, MI);
    return;
  }
  // 6809: split 16-bit carry immediate into two 8-bit operations.
  // ADCB #lo / ADCA #hi  or  SBCB #lo / SBCA #hi
  auto DestReg = MI.getOperand(0).getReg();
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    DestReg = RealReg;
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int Lo = Val & 0xFF;
  int Hi = (Val >> 8) & 0xFF;
  unsigned AdcbOpc = IsAdd ? MC6809::ADCBi8 : MC6809::SBCBi8;
  unsigned AdcaOpc = IsAdd ? MC6809::ADCAi8 : MC6809::SBCAi8;
  Builder.buildInstr(AdcbOpc).addDef(MC6809::AB, RegState::Implicit).addImm(Lo);
  Builder.buildInstr(AdcaOpc).addDef(MC6809::AA, RegState::Implicit).addImm(Hi);
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    emitSpillStore(Builder, DestReg, OrigSpillReg, MF);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCarryMem16(bool IsAdd, MachineIRBuilder &Builder,
                                       MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    // 6309 has ADCD/SBCD — use the standard expand path.
    expandIdxImm(IsAdd ? AddCarryIdxImm : SubBorrowIdxImm, Builder, MI);
    return;
  }
  // 6809: split 16-bit carry indexed into two 8-bit operations.
  // ADCB offset+1,base / ADCA offset,base  or  SBCB/SBCA
  auto DestReg = MI.getOperand(0).getReg();
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    DestReg = RealReg;
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto IndexReg = MI.getOperand(operandCount - 2).getReg();
  auto OffsetOp = MI.getOperand(operandCount - 1);
  auto Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
  // Low byte is at offset+1 (big-endian), high byte at offset.
  int OffsetLo = Offset + 1;
  int OffsetHi = Offset;
  int OffsetLoSize = offsetSizeInBitsForValue(OffsetLo);
  int OffsetHiSize = offsetSizeInBitsForValue(OffsetHi);
  // Look up the 8-bit opcodes for B and A.
  RegPlusOffsetLen LookupB{MC6809::AB, OffsetLoSize};
  RegPlusOffsetLen LookupA{MC6809::AA, OffsetHiSize};
  auto &CarryMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
  auto OpcB = CarryMap.find(LookupB);
  auto OpcA = CarryMap.find(LookupA);
  assert(OpcB != CarryMap.end() && OpcA != CarryMap.end());
  auto InstrB = Builder.buildInstr(OpcB->getSecond()).addDef(MC6809::AB, RegState::Implicit);
  if (OffsetLoSize == 0)
    InstrB.addReg(IndexReg);
  else
    InstrB.addImm(OffsetLo).addReg(IndexReg);
  auto InstrA = Builder.buildInstr(OpcA->getSecond()).addDef(MC6809::AA, RegState::Implicit);
  if (OffsetHiSize == 0)
    InstrA.addReg(IndexReg);
  else
    InstrA.addImm(OffsetHi).addReg(IndexReg);
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    emitSpillStore(Builder, DestReg, OrigSpillReg, MF);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandBitwiseImm16(ContextImmediate Context, unsigned OpcA,
                                         unsigned OpcB, MachineIRBuilder &Builder,
                                         MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    // 6309 has ANDD/ORD/EORD — use the standard expand path.
    expandImm(Context, Builder, MI);
    return;
  }
  // 6809: split 16-bit immediate into two 8-bit byte operations.
  // OpcA #hi / OpcB #lo (big-endian: A=high byte, B=low byte)
  auto DestReg = MI.getOperand(0).getReg();
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    // Save D — it may hold a live value while we operate on the spill.
    Builder.buildInstr(MC6809::PSHSs, {}, {Register(MC6809::AD)});
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    DestReg = RealReg;
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int Lo = Val & 0xFF;
  int Hi = (Val >> 8) & 0xFF;
  if (Lo != Context.IdentityValue)
    Builder.buildInstr(OpcB).addDef(MC6809::AB, RegState::Implicit).addImm(Lo);
  if (Hi != Context.IdentityValue)
    Builder.buildInstr(OpcA).addDef(MC6809::AA, RegState::Implicit).addImm(Hi);
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    emitSpillStore(Builder, DestReg, OrigSpillReg, MF);
    Builder.buildInstr(MC6809::PULSs, {}, {Register(MC6809::AD)});
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandBitwiseMem16(ContextIndexImmediate Context,
                                         MachineIRBuilder &Builder,
                                         MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    // 6309 has ANDD/ORD/EORD indexed — use the standard expand path.
    expandIdxImm(Context, Builder, MI);
    return;
  }
  // 6809: split 16-bit indexed into two 8-bit byte operations.
  // OpB offset+1,base / OpA offset,base (big-endian: A=high, B=low)
  auto DestReg = MI.getOperand(0).getReg();
  bool DestIsSpill = isSpillReg(DestReg);
  Register OrigSpillReg = DestReg;
  if (DestIsSpill) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    // Save D — it may hold a live value while we operate on the spill.
    // Uses S stack; U-relative spill offsets are unaffected.
    Builder.buildInstr(MC6809::PSHSs, {}, {Register(MC6809::AD)});
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    DestReg = RealReg;
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto IndexReg = MI.getOperand(operandCount - 2).getReg();
  auto OffsetOp = MI.getOperand(operandCount - 1);
  auto Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
  // Low byte at offset+1, high byte at offset (big-endian).
  int OffsetLo = Offset + 1;
  int OffsetHi = Offset;
  int OffsetLoSize = offsetSizeInBitsForValue(OffsetLo);
  int OffsetHiSize = offsetSizeInBitsForValue(OffsetHi);
  // Look up the 8-bit opcodes for B (low) and A (high).
  RegPlusOffsetLen LookupB{MC6809::AB, OffsetLoSize};
  RegPlusOffsetLen LookupA{MC6809::AA, OffsetHiSize};
  auto OpcB = Context.Opcode->find(LookupB);
  auto OpcA = Context.Opcode->find(LookupA);
  assert(OpcB != Context.Opcode->end() && OpcA != Context.Opcode->end());
  auto InstrB = Builder.buildInstr(OpcB->getSecond()).addDef(MC6809::AB, RegState::Implicit);
  if (OffsetLoSize == 0)
    InstrB.addReg(IndexReg);
  else
    InstrB.addImm(OffsetLo).addReg(IndexReg);
  auto InstrA = Builder.buildInstr(OpcA->getSecond()).addDef(MC6809::AA, RegState::Implicit);
  if (OffsetHiSize == 0)
    InstrA.addReg(IndexReg);
  else
    InstrA.addImm(OffsetHi).addReg(IndexReg);
  if (DestIsSpill) {
    MachineFunction &MF = *MI.getMF();
    emitSpillStore(Builder, DestReg, OrigSpillReg, MF);
    // Restore D's original value.
    Builder.buildInstr(MC6809::PULSs, {}, {Register(MC6809::AD)});
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandIdxReg(ContextIndexRegister Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto IndexReg = MI.getOperand(2).getReg();
  auto OffsetReg = MI.getOperand(3).getReg();

  RegPlusReg Lookup{DestReg, OffsetReg};
  auto OpcodePair = Context.Opcode->find(Lookup);
  if (OpcodePair == Context.Opcode->end())
    llvm_unreachable("Cannot find machine instruction with these register indexed operands");
  Builder.buildInstr(OpcodePair->getSecond()).addDef(DestReg, RegState::Implicit).addReg(IndexReg);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandNegate(MachineIRBuilder &Builder, MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  bool has6309 = STI.has6309();
  Register Reg = MI.getOperand(0).getReg();

  // Handle spill registers: load to real register, negate, store back.
  if (isSpillReg(Reg)) {
    Register OrigSpillReg = Reg;
    Register RealReg = getRealRegForSpill(Reg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, OrigSpillReg, MF);
    // Rewrite operands to the real register.
    MI.getOperand(0).setReg(RealReg);
    if (MI.getNumOperands() > 1 && MI.getOperand(1).isReg() &&
        MI.getOperand(1).getReg() == OrigSpillReg)
      MI.getOperand(1).setReg(RealReg);
    // Recursively expand with the real register.
    expandNegate(Builder, MI);
    // Store back to spill slot.
    emitSpillStore(Builder, RealReg, OrigSpillReg, MF);
    return;
  }

  switch (Reg) {
  default:
    llvm_unreachable("Illegal register for Neg(1|16)");
  case MC6809::AA:
    MI.setDesc(Builder.getTII().get(MC6809::NEGAa));
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    // MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AB:
    MI.setDesc(Builder.getTII().get(MC6809::NEGBa));
    MI.removeOperand(3);
    MI.removeOperand(2);
    MI.removeOperand(1);
    MI.removeOperand(0);
    // MI.addImplicitDefUseOperands(*MI.getMF());
    break;
  case MC6809::AD:
    if (has6309) {
      MI.setDesc(Builder.getTII().get(MC6809::NEGDa));
      MI.removeOperand(3);
      MI.removeOperand(2);
      MI.removeOperand(1);
      MI.removeOperand(0);
    } else {
      // Standard 6809: no NEGD. Use COMA; COMB; ADDD #1 (two's complement).
      Builder.buildInstr(MC6809::COMAa);
      Builder.buildInstr(MC6809::COMBa);
      Builder.buildInstr(MC6809::ADDDi16).addDef(MC6809::AD, RegState::Implicit).addImm(1);
      MI.eraseFromParent();
    }
    break;
  case MC6809::AE:
    Builder.buildInstr(MC6809::COMEa);
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AE).addUse(MC6809::A0).addUse(MC6809::AE);
    MI.eraseFromParent();
    break;
  case MC6809::AF:
    Builder.buildInstr(MC6809::COMFa);
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AF).addUse(MC6809::A0).addUse(MC6809::AF);
    MI.eraseFromParent();
    break;
  case MC6809::AW:
    Builder.buildInstr(MC6809::COMWa);
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AW).addUse(MC6809::A0).addUse(MC6809::AW);
    MI.eraseFromParent();
    break;
  case MC6809::AQ:
    Builder.buildInstr(MC6809::COMDa);
    Builder.buildInstr(MC6809::COMWa);
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AW).addUse(MC6809::A0).addUse(MC6809::AW);
    Builder.buildInstr(MC6809::ADCRp).addDef(MC6809::AD).addUse(MC6809::A0).addUse(MC6809::AD);
    MI.eraseFromParent();
    break;
  }
}

void MC6809InstrInfo::expandShiftLeft(MachineIRBuilder &Builder, MachineInstr &MI) const {
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
  case MC6809::AD: {
    const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
    if (STI.has6309()) {
      MI.setDesc(Builder.getTII().get(MC6809::ASLDa));
      MI.removeOperand(1);
      MI.removeOperand(0);
      MI.addImplicitDefUseOperands(*MI.getMF());
    } else {
      // 6809: ASLB then ROLA (carry from B propagates to A)
      MachineFunction &MF = *MI.getMF();
      Builder.buildInstr(MC6809::ASLBa)->addImplicitDefUseOperands(MF);
      Builder.buildInstr(MC6809::ROLAa)->addImplicitDefUseOperands(MF);
      MI.eraseFromParent();
    }
    break;
  }
  }
}

void MC6809InstrInfo::expandShiftRight(MachineIRBuilder &Builder, MachineInstr &MI, bool Arithmetic) const {
  Register Reg = MI.getOperand(0).getReg();
  unsigned Opcode;
  switch (Reg) {
  default:
    llvm_unreachable("Illegal register for LSR/ASR");
  case MC6809::AA:
    Opcode = Arithmetic ? MC6809::ASRAa : MC6809::LSRAa;
    break;
  case MC6809::AB:
    Opcode = Arithmetic ? MC6809::ASRBa : MC6809::LSRBa;
    break;
  case MC6809::AD: {
    const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
    if (STI.has6309()) {
      Opcode = Arithmetic ? MC6809::ASRDa : MC6809::LSRDa;
    } else {
      // 6809: shift A right first, then ROR B (carry from A to B)
      // LSHR: LSRA + RORB  (logical: 0 into A bit 7, carry into B bit 7)
      // ASHR: ASRA + RORB  (arithmetic: sign preserved in A, carry into B)
      unsigned FirstOpc = Arithmetic ? MC6809::ASRAa : MC6809::LSRAa;
      MachineFunction &MF = *MI.getMF();
      Builder.buildInstr(FirstOpc)->addImplicitDefUseOperands(MF);
      Builder.buildInstr(MC6809::RORBa)->addImplicitDefUseOperands(MF);
      MI.eraseFromParent();
      return;
    }
    break;
  }
  }
  MI.setDesc(Builder.getTII().get(Opcode));
  MI.removeOperand(1); // remove immediate
  MI.removeOperand(0); // remove register (now implicit)
  MI.addImplicitDefUseOperands(*MI.getMF());
}

void MC6809InstrInfo::expandRotate(MachineIRBuilder &Builder, MachineInstr &MI, bool Left) const {
  Register Reg = MI.getOperand(0).getReg();
  unsigned Opcode;
  switch (Reg) {
  default:
    llvm_unreachable("Illegal register for ROL/ROR");
  case MC6809::AA:
    Opcode = Left ? MC6809::ROLAa : MC6809::RORAa;
    break;
  case MC6809::AB:
    Opcode = Left ? MC6809::ROLBa : MC6809::RORBa;
    break;
  }
  MI.setDesc(Builder.getTII().get(Opcode));
  MI.removeOperand(1);
  MI.removeOperand(0);
  MI.addImplicitDefUseOperands(*MI.getMF());
}

void MC6809InstrInfo::expandMulD(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOpcode() == MC6809::MUL_D && "Invalid multiply opcode");
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();

  if (DstReg == MC6809::AD && SrcReg == MC6809::AD) {
    // Direct: operands already in real D register.
    MI.setDesc(Builder.getTII().get(MC6809::MULx));
    MI.getOperand(0).setImplicit();
    MI.getOperand(1).setImplicit();
    return;
  }

  // Spill register operand: load from spill slot → D, MUL, store D → spill slot.
  // Emit concrete S-indexed instructions (PEI has already resolved frame indices).
  MachineFunction &MF = *MI.getMF();
  MachineBasicBlock &MBB = *MI.getParent();

  // Load spill slot into real D before the MUL.
  if (SrcReg != MC6809::AD) {
    assert(isSpillReg(SrcReg) && "MUL_D source must be AD or spill register");
    MachineIRBuilder PreBuilder(MBB, MI.getIterator());
    emitSpillLoad(PreBuilder, MC6809::AD, SrcReg, MF);
  }

  // The MUL instruction itself.
  MI.setDesc(Builder.getTII().get(MC6809::MULx));
  MI.getOperand(0).setReg(MC6809::AD);
  MI.getOperand(0).setImplicit();
  MI.getOperand(1).setReg(MC6809::AD);
  MI.getOperand(1).setImplicit();

  // Store result from D to spill slot after the MUL.
  if (DstReg != MC6809::AD) {
    assert(isSpillReg(DstReg) && "MUL_D dest must be AD or spill register");
    MachineBasicBlock::iterator InsertPt = MI.getIterator();
    ++InsertPt;
    MachineIRBuilder PostBuilder(MBB, InsertPt);
    emitSpillStore(PostBuilder, MC6809::AD, DstReg, MF);
  }
}

void MC6809InstrInfo::expandMul16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) && "Result must be in AW and AD must be the source");
  auto ValueOp = MI.getOperand(2);
  auto Value = ValueOp.isImm() ? ValueOp.getImm() : ValueOp.getCImm()->getSExtValue();
  Builder.buildInstr(MC6809::MULDi16)
      .addDef(MI.getOperand(0).getReg(), RegState::ImplicitDefine)
      .addUse(MI.getOperand(1).getReg(), RegState::Implicit)
      .addImm(Value);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert((MI.getOperand(0).getReg() == MC6809::AD && MI.getOperand(1).getReg() == MC6809::AD) && "Result must be in AD and AD must be the source");
  auto ValueOp = MI.getOperand(2);
  auto Value = ValueOp.isImm() ? ValueOp.getImm() : ValueOp.getCImm()->getSExtValue();
  Builder.buildInstr(MC6809::MULDi16)
      .addDef(MI.getOperand(0).getReg(), RegState::ImplicitDefine)
      .addUse(MI.getOperand(1).getReg(), RegState::Implicit)
      .addImm(Value);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMul16IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) && "Result must be in AW and AD must be the source");
  auto IndexReg = MI.getOperand(2).getReg();
  auto OffsetOp = MI.getOperand(3);
  unsigned Opcode;
  int Offset;

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
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
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg(), RegState::ImplicitDefine)
                     .addUse(MI.getOperand(1).getReg(), RegState::Implicit);
    if (OffsetSize == 0) {
      Instr.addReg(IndexReg);
    } else {
      Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
      Instr.addImm(Offset).addReg(IndexReg);
    }
  } else
    llvm_unreachable("Unknown offset type");
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert((MI.getOperand(0).getReg() == MC6809::AD && MI.getOperand(1).getReg() == MC6809::AD) && "Result must be in AD and AD must be the source");
  auto IndexReg = MI.getOperand(2).getReg();
  auto OffsetOp = MI.getOperand(3);
  unsigned Opcode;
  int Offset;

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
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
    auto Instr = Builder.buildInstr(Opcode)
                     .addDef(MI.getOperand(0).getReg(), RegState::ImplicitDefine)
                     .addUse(MI.getOperand(1).getReg(), RegState::Implicit);
    if (OffsetSize == 0) {
      Instr.addReg(IndexReg);
    } else {
      Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
      Instr.addImm(Offset).addReg(IndexReg);
    }
  } else
    llvm_unreachable("Unknown offset type");
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMul16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
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
  Builder.buildInstr(Opcode)
      .addDef(MI.getOperand(0).getReg(), RegState::ImplicitDefine)
      .addDef(MI.getOperand(1).getReg(), RegState::ImplicitDefine)
      .addReg(OffsetReg, RegState::Implicit).addReg(IndexReg);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  llvm_unreachable("Write me!");
}

void MC6809InstrInfo::expandMul16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(((MI.getOperand(0).getReg() == MC6809::AW && MI.getOperand(1).getReg() == MC6809::AD) || (MI.getOperand(0).getReg() == MC6809::AD && MI.getOperand(1).getReg() == MC6809::AW)) && "Results must be in AW and AD");
  auto Reg = MI.getOperand(3).getReg();
  MachineBasicBlock::iterator B, E;
  MachineBasicBlock &MBB = Builder.getMBB();
  B = Builder.buildInstr(MC6809::Push_i16).addReg(Reg);
  E = Builder.buildInstr(MC6809::MULDi_Inc2).addReg(MC6809::SS);
  auto Bundler = MIBundleBuilder(MBB, B, ++E);
  LLVM_DEBUG(for (auto &I
                  : Bundler) {
    I.dump();
  });
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  llvm_unreachable("Write me!");
}

void MC6809InstrInfo::expandLoad1Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
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
}

void MC6809InstrInfo::expandLoadImm(MachineIRBuilder &Builder, MachineInstr &MI) const {

  auto DestRegOp = MI.getOperand(0);

  // If the destination is a spill register, save the real accumulator,
  // load immediate into it, store to spill slot (U-relative), then restore.
  // Don't recurse into expandLoadImm — it removes MI and invalidates Builder.
  if (isSpillReg(DestRegOp.getReg())) {
    Register RealReg = isIndexSpillReg(DestRegOp.getReg()) ? MC6809::IY
                                                           : getRealRegForSpill(DestRegOp.getReg());
    MachineFunction &MF = Builder.getMF();
    auto ValOp = MI.getOperand(1);
    // Load immediate into staging register, then store to spill slot.
    auto OpcodePair = LoadImmediateOpcode.find(RealReg);
    assert(OpcodePair != LoadImmediateOpcode.end());
    auto NewMI = Builder.buildInstr(OpcodePair->getSecond()).addDef(RealReg, RegState::Implicit);
    if (ValOp.isGlobal())
      NewMI.addGlobalAddress(ValOp.getGlobal(), ValOp.getOffset(), ValOp.getTargetFlags());
    else {
      int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
      NewMI.addImm(Val);
    }
    // Store to spill slot (use STY for INDEX spills, STD for ACC spills).
    if (isIndexSpillReg(DestRegOp.getReg())) {
      int SpillOff = computeSpillStackOffset(DestRegOp.getReg(), MF);
      // RealReg is IX from getRealRegForSpill, but we use IY for INDEX spills
      unsigned StoreOpc = getStoreIdxOpcode(MC6809::IY, SpillOff);
      Builder.buildInstr(StoreOpc)
          .addUse(MC6809::IY, RegState::Implicit)
          .addImm(SpillOff).addReg(MC6809::SU);
    } else {
      emitSpillStore(Builder, RealReg, DestRegOp.getReg(), MF);
    }
    MI.removeFromParent();
    return;
  }

  auto ValOp = MI.getOperand(1);
  auto OpcodePair = LoadImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == LoadImmediateOpcode.end())
    llvm_unreachable("Unexpected LoadImm register.");
  auto NewMI = Builder.buildInstr(OpcodePair->getSecond()).addDef(DestRegOp.getReg(), RegState::Implicit);
  // Preserve the operand type: integer immediate or global address reference.
  if (ValOp.isGlobal())
    NewMI.addGlobalAddress(ValOp.getGlobal(), ValOp.getOffset(), ValOp.getTargetFlags());
  else {
    int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
    NewMI.addImm(Val);
  }
  MI.removeFromParent();
}

void MC6809InstrInfo::expandLoadIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {

  auto DestRegOp = MI.getOperand(0);

  // If the destination is a spill register, load into D then store to the
  // spill slot. The SpillDSaveRestore pass (addPrePEI) handles saving and
  // restoring D around this operation when D is live — we don't need to
  // do emergency save/restore here. (The old emergency mechanism created
  // frame slots after PEI with unreliable offsets.)
  if (isSpillReg(DestRegOp.getReg())) {
    MachineFunction &MF = *MI.getMF();
    if (isIndexSpillReg(DestRegOp.getReg())) {
      // INDEX spill dest: load into IY (staging), then store to spill slot.
      Register StageReg = MC6809::IY;
      MI.getOperand(0).setReg(StageReg);
      expandLoadIdx(Builder, MI);
      int SpillOff = computeSpillStackOffset(DestRegOp.getReg(), MF);
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      unsigned StoreOpc = getStoreIdxOpcode(StageReg, SpillOff);
      PostBuilder.buildInstr(StoreOpc)
          .addUse(StageReg, RegState::Implicit)
          .addImm(SpillOff).addReg(MC6809::SU);
    } else {
      Register RealReg = getRealRegForSpill(DestRegOp.getReg());
      // Load from memory into the real accumulator.
      MI.getOperand(0).setReg(RealReg);
      expandLoadIdx(Builder, MI);
      // Store the real accumulator to the spill slot (U-relative).
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      emitSpillStore(PostBuilder, RealReg, DestRegOp.getReg(), MF);
    }
    return;
  }

  // If the index register is an INDEX spill, load into a staging index reg.
  // Use IY if the dest operand is IX (avoid conflict), otherwise IX.
  if (isIndexSpillReg(MI.getOperand(1).getReg())) {
    Register SpillReg = MI.getOperand(1).getReg();
    Register DestReg = MI.getOperand(0).getReg();
    Register StageReg = MC6809::IY;  // Prefer IY (callee-saved)
    MachineFunction &MF = *MI.getMF();
    int SpillOff = computeSpillStackOffset(SpillReg, MF);
    unsigned LoadOpc = getLoadIdxOpcode(StageReg, SpillOff);
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    PreBuilder.buildInstr(LoadOpc)
        .addDef(StageReg, RegState::Implicit)
        .addImm(SpillOff).addReg(MC6809::SU);
    MI.getOperand(1).setReg(StageReg);
  }

  auto IndexRegOp = MI.getOperand(1);
  auto OffsetOp = MI.getOperand(2);
  MI.removeOperand(2);
  MI.removeOperand(1);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{DestRegOp.getReg(), OffsetSize};
    auto OpcodePair = LoadIdxImmOpcode.find(Lookup);
    if (OpcodePair == LoadIdxImmOpcode.end())
      llvm_unreachable("Unexpected LoadIdx numeric offset. Too large?");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexRegOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{DestRegOp.getReg(), OffsetOp.getReg()};
    auto OpcodePair = LoadIdxRegOpcode.find(Lookup);
    if (OpcodePair == LoadIdxRegOpcode.end())
      llvm_unreachable("Unexpected LoadIdx register offset operand.");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    MI.addOperand(OffsetOp);
    MI.getOperand(1).setImplicit();
    MI.addOperand(IndexRegOp);
  } else
    llvm_unreachable("Unknown offset type for LoadIdx");
}

void MC6809InstrInfo::expandStoreIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {

  // If the source is a spill register, load from the spill slot.
  // INDEX spills use IY (avoids clobbering IX which may be the index base).
  // ACC spills use D with emergency save/restore.
  bool NeedRestore = false;
  int EmergencyOffset = 0;
  if (isIndexSpillReg(MI.getOperand(0).getReg())) {
    Register SpillReg = MI.getOperand(0).getReg();
    MachineFunction &MF = *MI.getMF();
    int SpillOff = computeSpillStackOffset(SpillReg, MF);
    MachineIRBuilder LoadBuilder(*MI.getParent(), MI.getIterator());
    unsigned LoadOpc = getLoadIdxOpcode(MC6809::IY, SpillOff);
    LoadBuilder.buildInstr(LoadOpc)
        .addDef(MC6809::IY, RegState::Implicit)
        .addImm(SpillOff).addReg(MC6809::SU);
    MI.getOperand(0).setReg(MC6809::IY);
  } else if (isSpillReg(MI.getOperand(0).getReg())) {
    Register SpillReg = MI.getOperand(0).getReg();
    Register RealReg = getRealRegForSpill(SpillReg);
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder LoadBuilder(*MI.getParent(), MI.getIterator());

    // Save $ad to emergency spill slot
    int EmergencyFI = MF.getFrameInfo().CreateStackObject(2, Align(1), true);
    EmergencyOffset = MF.getFrameInfo().getObjectOffset(EmergencyFI);
    LoadBuilder.buildInstr(MC6809::STDi_o8)
        .addUse(MC6809::AD, RegState::Implicit)
        .addImm(EmergencyOffset)
        .addReg(MC6809::SU);

    emitSpillLoad(LoadBuilder, RealReg, SpillReg, MF);
    MI.getOperand(0).setReg(RealReg);
    NeedRestore = true;
  }

  // If the index register is an INDEX spill, load into a staging index reg.
  // Use IY if the source operand is IX (avoid conflict), otherwise IX.
  if (isIndexSpillReg(MI.getOperand(1).getReg())) {
    Register SpillReg = MI.getOperand(1).getReg();
    Register SrcReg = MI.getOperand(0).getReg();
    Register StageReg = MC6809::IY;  // Prefer IY (callee-saved)
    MachineFunction &MF = *MI.getMF();
    int SpillOff = computeSpillStackOffset(SpillReg, MF);
    unsigned LoadOpc = getLoadIdxOpcode(StageReg, SpillOff);
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    PreBuilder.buildInstr(LoadOpc)
        .addDef(StageReg, RegState::Implicit)
        .addImm(SpillOff).addReg(MC6809::SU);
    MI.getOperand(1).setReg(StageReg);
  }

  auto SrcRegOp = MI.getOperand(0); // re-read AFTER spill fix
  auto IndexRegOp = MI.getOperand(1);
  auto OffsetOp = MI.getOperand(2);
  MI.removeOperand(2);
  MI.removeOperand(1);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{SrcRegOp.getReg(), OffsetSize};
    auto OpcodePair = StoreIdxImmOpcode.find(Lookup);
    if (OpcodePair == StoreIdxImmOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexRegOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{SrcRegOp.getReg(), OffsetOp.getReg()};
    auto OpcodePair = StoreIdxRegOpcode.find(Lookup);
    if (OpcodePair == StoreIdxRegOpcode.end())
      llvm_unreachable("Unexpected operand(s).");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    MI.getOperand(2).setImplicit();
  } else
    llvm_unreachable("Unknown offset type for StoreIdx");

  // Restore $ad from emergency spill slot
  if (NeedRestore) {
    MachineBasicBlock::iterator RestorePt = MI.getIterator();
    ++RestorePt;
    MachineIRBuilder RestoreBuilder(*MI.getParent(), RestorePt);
    RestoreBuilder.buildInstr(MC6809::LDDi_o8)
        .addDef(MC6809::AD, RegState::Implicit)
        .addImm(EmergencyOffset)
        .addReg(MC6809::SU);
  }
}

// Forward declarations for 6809 register-to-memory helpers.
static void emit6809RegByteFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned Opc_o8, unsigned Opc_o5);
static void emit6809RegPairFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned OpcB_o8, unsigned OpcA_o8,
                                   unsigned OpcB_o5, unsigned OpcA_o0);

void MC6809InstrInfo::expandANDReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for ANDReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::ANDRp)
        .addDef(MI.getOperand(0).getReg())
        .addUse(MI.getOperand(2).getReg())
        .addUse(MI.getOperand(1).getReg());
  } else if (MI.getOpcode() == MC6809::AND_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ANDBi_o8, MC6809::ANDAi_o8,
                           MC6809::ANDBi_o5, MC6809::ANDAi_o0);
  } else {
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ANDBi_o8, MC6809::ANDBi_o5);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandANDPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto SrcReg = MI.getOperand(1).getReg();
  // Always use SS — Push expands to PSHS (S stack), U is reserved.
  if (!MI.getMF()->getSubtarget<MC6809Subtarget>().has6309() && DestReg == MC6809::AD) {
    // 6809: Push_i16 put 2 bytes on S stack. Big-endian: S+0=hi, S+1=lo.
    Builder.buildInstr(MC6809::ANDBi_o5).addDef(MC6809::AB, RegState::Implicit).addImm(1).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::ANDAi_o0).addDef(MC6809::AA, RegState::Implicit).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5).addImm(2).addReg(MC6809::SS);
  } else {
    auto OpcodePair = ANDPullOpcode.find(DestReg);
    Builder.buildInstr(OpcodePair->getSecond())
        .addDef(DestReg, RegState::Implicit)
        .addUse(SrcReg, RegState::Implicit)
        .addUse(MC6809::SS);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandORPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto SrcReg = MI.getOperand(1).getReg();
  if (!MI.getMF()->getSubtarget<MC6809Subtarget>().has6309() && DestReg == MC6809::AD) {
    // 6809: Push_i16 put 2 bytes on S stack. Big-endian: S+0=hi, S+1=lo.
    Builder.buildInstr(MC6809::ORBi_o5).addDef(MC6809::AB, RegState::Implicit).addImm(1).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::ORAi_o0).addDef(MC6809::AA, RegState::Implicit).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5).addImm(2).addReg(MC6809::SS);
  } else {
    auto OpcodePair = ORPullOpcode.find(DestReg);
    Builder.buildInstr(OpcodePair->getSecond())
        .addDef(DestReg, RegState::Implicit)
        .addUse(SrcReg, RegState::Implicit)
        .addUse(MC6809::SS);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandXORPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto SrcReg = MI.getOperand(1).getReg();
  if (!MI.getMF()->getSubtarget<MC6809Subtarget>().has6309() && DestReg == MC6809::AD) {
    // 6809: Push_i16 put 2 bytes on S stack. Big-endian: S+0=hi, S+1=lo.
    Builder.buildInstr(MC6809::EORBi_o5).addDef(MC6809::AB, RegState::Implicit).addImm(1).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::EORAi_o0).addDef(MC6809::AA, RegState::Implicit).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5).addImm(2).addReg(MC6809::SS);
  } else {
    auto OpcodePair = XORPullOpcode.find(DestReg);
    Builder.buildInstr(OpcodePair->getSecond())
        .addDef(DestReg, RegState::Implicit)
        .addUse(SrcReg, RegState::Implicit)
        .addUse(MC6809::SS);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for ORReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::ORRp).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  } else if (MI.getOpcode() == MC6809::OR_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ORBi_o8, MC6809::ORAi_o8,
                           MC6809::ORBi_o5, MC6809::ORAi_o0);
  } else {
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ORBi_o8, MC6809::ORBi_o5);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandXORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for XORReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::EORRp).addDef(MI.getOperand(0).getReg()).addUse(MI.getOperand(2).getReg()).addUse(MI.getOperand(1).getReg());
  } else if (MI.getOpcode() == MC6809::XOR_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::EORBi_o8, MC6809::EORAi_o8,
                           MC6809::EORBi_o5, MC6809::EORAi_o0);
  } else {
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::EORBi_o8, MC6809::EORBi_o5);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for AddReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::ADDRp)
        .addDef(MI.getOperand(0).getReg())
        .addUse(MI.getOperand(2).getReg())
        .addUse(MI.getOperand(1).getReg());
  } else if (MI.getOpcode() == MC6809::Add_i8_Reg) {
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ADDBi_o8, MC6809::ADDBi_o5);
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ADDBi_o8, MC6809::ADDAi_o8,
                           MC6809::ADDBi_o5, MC6809::ADDAi_o0);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source 2 must be same for AddSetCarryReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::ADDRp)
        .addDef(MI.getOperand(0).getReg())
        .addDef(MI.getOperand(1).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(2).getReg())
        .addUse(MI.getOperand(3).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(4).getReg());
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(4).getReg(),
                           MC6809::ADDBi_o8, MC6809::ADDAi_o8,
                           MC6809::ADDBi_o5, MC6809::ADDAi_o0);
  }
  MI.eraseFromParent();
}

/// Emit 6809 single-byte add/sub from register. Both operands may be spill
/// registers. Load LHS into real accumulator, operate from RHS's spill slot
/// (U-relative) or push RHS to stack, then store back.
static void emit6809RegByteFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned Opc_o8, unsigned Opc_o5) {
  MachineFunction &MF = Builder.getMF();
  Register RealLHS = isSpillReg(LHS) ? getRealRegForSpill(LHS) : LHS;
  // Determine which accumulator half (A or B) we're operating on.
  Register AccReg = (RealLHS == MC6809::AA || RealLHS == MC6809::AALSB)
                        ? MC6809::AA : MC6809::AB;
  if (isSpillReg(LHS))
    emitSpillLoad(Builder, RealLHS, LHS, MF);
  if (isSpillReg(RHS)) {
    int Offset = computeSpillStackOffset(RHS, MF);
    // Spill_B registers are the low byte of 16-bit spill slots (offset+1).
    int ByteOffset = Offset + 1;
    Builder.buildInstr(Opc_o8)
        .addDef(AccReg, RegState::Implicit)
        .addImm(ByteOffset).addReg(MC6809::SU);
  } else {
    // RHS is a real register — push 1 byte to stack and operate.
    Builder.buildInstr(MC6809::PSHSs)
        .addDef(MC6809::SS)
        .addUse(RHS, RegState::Implicit)
        .addUse(MC6809::SS);
    Builder.buildInstr(Opc_o5)
        .addDef(AccReg, RegState::Implicit)
        .addImm(0).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5)
        .addImm(1).addReg(MC6809::SS);
  }
  if (isSpillReg(LHS))
    emitSpillStore(Builder, RealLHS, LHS, MF);
}

/// Emit 6809 two-byte carry/sub from register: load LHS into D if spill,
/// use RHS from its spill slot (U-relative) or push to stack (S-relative),
/// then do 8-bit B op + A op. OpcB_imm/OpcA_imm are the i8 immediate opcodes
/// (used only for selecting the right indexed variants).
static void emit6809RegPairFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned OpcB_o8, unsigned OpcA_o8,
                                   unsigned OpcB_o5, unsigned OpcA_o0) {
  MachineFunction &MF = Builder.getMF();
  // Load LHS into real D if it's a spill register.
  if (isSpillReg(LHS))
    emitSpillLoad(Builder, getRealRegForSpill(LHS), LHS, MF);
  // RHS: if it's a spill register, use its U-relative stack slot directly.
  if (isSpillReg(RHS)) {
    int Offset = computeSpillStackOffset(RHS, MF);
    // Big-endian: high byte at Offset, low byte at Offset+1.
    Builder.buildInstr(OpcB_o8)
        .addDef(MC6809::AB, RegState::Implicit)
        .addImm(Offset + 1).addReg(MC6809::SU);
    Builder.buildInstr(OpcA_o8)
        .addDef(MC6809::AA, RegState::Implicit)
        .addImm(Offset).addReg(MC6809::SU);
  } else {
    // RHS is a real register — push to S-stack and operate.
    Builder.buildInstr(MC6809::PSHSs)
        .addDef(MC6809::SS)
        .addUse(RHS, RegState::Implicit)
        .addUse(MC6809::SS);
    // S-relative: high byte at 0,s, low byte at 1,s.
    Builder.buildInstr(OpcB_o5)
        .addDef(MC6809::AB, RegState::Implicit)
        .addImm(1).addReg(MC6809::SS);
    Builder.buildInstr(OpcA_o0)
        .addDef(MC6809::AA, RegState::Implicit)
        .addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5)
        .addDef(MC6809::SS)
        .addImm(2).addReg(MC6809::SS);
  }
  // Store result back if LHS was a spill register.
  if (isSpillReg(LHS))
    emitSpillStore(Builder, getRealRegForSpill(LHS), LHS, MF);
}

void MC6809InstrInfo::expandAddSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source 2 must be same for AddSetCarryUseReg");
  assert(MI.getOperand(1).getReg() == MI.getOperand(3).getReg() && "Carry and Carry_in must be same for AddSetCarryUseReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::ADCRp)
        .addDef(MI.getOperand(0).getReg())
        .addDef(MI.getOperand(1).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(2).getReg())
        .addUse(MI.getOperand(3).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(4).getReg());
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(4).getReg(),
                           MC6809::ADCBi_o8, MC6809::ADCAi_o8,
                           MC6809::ADCBi_o5, MC6809::ADCAi_o0);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for SubReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::SUBRp)
        .addDef(MI.getOperand(0).getReg())
        .addUse(MI.getOperand(2).getReg())
        .addUse(MI.getOperand(1).getReg());
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::SUBBi_o8, MC6809::SBCAi_o8,
                           MC6809::SUBBi_o5, MC6809::SBCAi_o0);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source must be same for SubSetCarryReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::SUBRp)
        .addDef(MI.getOperand(0).getReg())
        .addDef(MI.getOperand(1).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(4).getReg())
        .addUse(MI.getOperand(3).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(2).getReg());
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(4).getReg(),
                           MC6809::SUBBi_o8, MC6809::SBCAi_o8,
                           MC6809::SUBBi_o5, MC6809::SBCAi_o0);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(2).getReg() && "Dest and Source must be same for SubSetCarryUseReg");
  assert(MI.getOperand(1).getReg() == MI.getOperand(3).getReg() && "Carry and Carry_in must be same for SubSetCarryUseReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::SBCRp)
        .addDef(MI.getOperand(0).getReg())
        .addDef(MI.getOperand(1).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(4).getReg())
        .addUse(MI.getOperand(3).getReg(), RegState::Implicit)
        .addUse(MI.getOperand(2).getReg());
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(4).getReg(),
                           MC6809::SBCBi_o8, MC6809::SBCAi_o8,
                           MC6809::SBCBi_o5, MC6809::SBCAi_o0);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto SrcReg = MI.getOperand(1).getReg();

  // If the destination is a spill register, load it into the real accumulator,
  // do the add-pull, then store back.
  if (isSpillReg(DestReg)) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    auto OpcodePair = AddPullOpcode.find(RealReg);
    Builder.buildInstr(OpcodePair->getSecond())
        .addDef(RealReg, RegState::Implicit)
        .addUse(SrcReg, RegState::Implicit)
        .addUse(MC6809::SS);
    emitSpillStore(Builder, RealReg, DestReg, MF);
    MI.eraseFromParent();
    return;
  }

  // Always use SS — Push_i8 expands to PSHS (S stack), U is reserved.
  auto OpcodePair = AddPullOpcode.find(DestReg);
  Builder.buildInstr(OpcodePair->getSecond())
                   .addDef(DestReg, RegState::Implicit)
                   .addUse(SrcReg, RegState::Implicit)
                   .addUse(MC6809::SS);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubPull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto DestReg = MI.getOperand(0).getReg();
  auto SrcReg = MI.getOperand(1).getReg();

  // If the destination is a spill register, load into real accumulator,
  // do the sub-pull, then store back.
  if (isSpillReg(DestReg)) {
    Register RealReg = getRealRegForSpill(DestReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, DestReg, MF);
    auto OpcodePair = SubPullOpcode.find(RealReg);
    Builder.buildInstr(OpcodePair->getSecond())
        .addDef(RealReg, RegState::Implicit)
        .addUse(SrcReg, RegState::Implicit)
        .addUse(MC6809::SS);
    emitSpillStore(Builder, RealReg, DestReg, MF);
    MI.eraseFromParent();
    return;
  }

  // Always use SS for pull operations.
  auto OpcodePair = SubPullOpcode.find(DestReg);
  Builder.buildInstr(OpcodePair->getSecond())
                   .addDef(DestReg, RegState::Implicit)
                   .addUse(SrcReg, RegState::Implicit)
                   .addUse(MC6809::SS);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCompareImm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  // assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of compares must be a register");
  assert((MI.getOperand(3).isImm() || MI.getOperand(3).isCImm()) && "The final operand of immediate compares must be an immediate constant");

  auto SrcReg = MI.getOperand(2).getReg();
  if (isSpillReg(SrcReg)) {
    Register RealReg = getRealRegForSpill(SrcReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, SrcReg, MF);
    SrcReg = RealReg;
  }
  auto OpcodePair = CompareImmediateOpcode.find(SrcReg);
  if (OpcodePair == CompareImmediateOpcode.end())
    llvm_unreachable("Compare Immediate - unexpected register.");
  auto Opcode = OpcodePair->getSecond();
  Builder.buildInstr(Opcode).add(MI.getOperand(3));
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCompareIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  // assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of compares must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of compares must be a register");
  assert(MI.getOperand(3).isReg() && "The index operand of indexed compares must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  if (isSpillReg(SrcReg)) {
    MachineFunction &MF = *MI.getMF();
    // Optimization: if the spill was stored from an INDEX register (STX/STY)
    // and that register hasn't been redefined, use CMPX/CMPY directly.
    // This avoids clobbering D which may hold a live value (bug #30).
    int SpillOffset = computeSpillStackOffset(SrcReg, MF);
    Register IndexSrc = Register();
    MachineBasicBlock &MBB = *MI.getParent();
    for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
         It != MBB.rend(); ++It) {
      unsigned Opc = It->getOpcode();
      // Check for STX/STY to the same spill offset via U.
      // Operand layout: (offset_imm, index_reg, implicit src_reg)
      if ((Opc == MC6809::STXi_o5 || Opc == MC6809::STXi_o8 || Opc == MC6809::STXi_o16) &&
          It->getOperand(0).isImm() &&
          It->getOperand(0).getImm() == SpillOffset) {
        IndexSrc = MC6809::IX;
        break;
      }
      if ((Opc == MC6809::STYi_o5 || Opc == MC6809::STYi_o8 || Opc == MC6809::STYi_o16) &&
          It->getOperand(0).isImm() &&
          It->getOperand(0).getImm() == SpillOffset) {
        IndexSrc = MC6809::IY;
        break;
      }
      // Skip D save/restore instructions (STD/LDD to frame slots) that may
      // have been inserted by SpillDSaveRestore between the STX and here.
      if (It->mayStore() && !It->definesRegister(MC6809::IX, /*TRI=*/nullptr) &&
          !It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
        continue;
      // If IX or IY is redefined before we find the store, stop.
      if (It->definesRegister(MC6809::IX, /*TRI=*/nullptr) ||
          It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
        break;
    }
    if (IndexSrc.isValid()) {
      SrcReg = IndexSrc;  // Use CMPX/CMPY directly, preserving D.
    } else {
      Register RealReg = getRealRegForSpill(SrcReg);
      emitSpillLoad(Builder, RealReg, SrcReg, MF);
      SrcReg = RealReg;
    }
  }
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
  Builder.buildInstr(Opcode)
      .add(OffsetOp).add(IndexOp);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCompareReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the 2nd source register for the comparison
  // Operand 3 is the 1st source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of compares must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The 2nd source of register tests must be a register");
  assert(MI.getOperand(3).isReg() && "The 1st source of register tests must be a register");

  Builder.buildInstr(MC6809::CMPRp)
      .addUse(MI.getOperand(3).getReg())
      .addUse(MI.getOperand(2).getReg());
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandComparePull(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Compare_*_Pull: compare register with value pushed on S stack.
  // Push_i16 already pushed the second operand. We compare the first
  // operand (in an accumulator) with [S], then clean up the stack.
  // Operand 0 = CC (def), 1 = condition code, 2 = src register, 3 = stack reg
  assert(MI.getOperand(2).isReg() && "The source of pull compares must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  if (isSpillReg(SrcReg)) {
    Register RealReg = getRealRegForSpill(SrcReg);
    MachineFunction &MF = *MI.getMF();
    emitSpillLoad(Builder, RealReg, SrcReg, MF);
    SrcReg = RealReg;
  }
  int Size = (SrcReg == MC6809::AA || SrcReg == MC6809::AB ||
              SrcReg == MC6809::AE || SrcReg == MC6809::AF) ? 1 : 2;
  // Compare with [S+0] (zero offset from stack pointer).
  RegPlusOffsetLen Lookup{SrcReg, 0};
  auto OpcodePair = CompareIdxImmOpcode.find(Lookup);
  if (OpcodePair == CompareIdxImmOpcode.end())
    llvm_unreachable("Unexpected register in Compare_Pull expansion.");
  Builder.buildInstr(OpcodePair->getSecond()).addReg(MC6809::SS);
  // Clean up the pushed value.
  Builder.buildInstr(MC6809::LEASi_o5).addImm(Size).addReg(MC6809::SS);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandTestReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Operand 0 is the CC register
  // Operand 1 is the 4-bit field that Bcc and LBcc use as the condition.
  // Operand 2 is the source register for the comparison
  // This is needed to model the G_ICMP/G_BRCOND behaviour.
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of tests must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of tests must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of register tests must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  if (isSpillReg(SrcReg)) {
    // Optimization: if the spill was stored from an INDEX register (STX/STY),
    // use CMPX/CMPY #0 directly. This avoids clobbering D which may hold a
    // live value (e.g. loop sum while testing loop counter — bug #31).
    MachineFunction &MF = *MI.getMF();
    int SpillOffset = computeSpillStackOffset(SrcReg, MF);
    Register IndexSrc = Register();
    MachineBasicBlock &MBB = *MI.getParent();
    for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
         It != MBB.rend(); ++It) {
      unsigned Opc = It->getOpcode();
      if ((Opc == MC6809::STXi_o5 || Opc == MC6809::STXi_o8 || Opc == MC6809::STXi_o16) &&
          It->getOperand(0).isImm() &&
          It->getOperand(0).getImm() == SpillOffset) {
        IndexSrc = MC6809::IX;
        break;
      }
      if ((Opc == MC6809::STYi_o5 || Opc == MC6809::STYi_o8 || Opc == MC6809::STYi_o16) &&
          It->getOperand(0).isImm() &&
          It->getOperand(0).getImm() == SpillOffset) {
        IndexSrc = MC6809::IY;
        break;
      }
      // Skip stores that don't affect IX/IY (e.g., D save/restore from
      // SpillDSaveRestore inserted between STX and this test).
      if (It->mayStore() && !It->definesRegister(MC6809::IX, /*TRI=*/nullptr) &&
          !It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
        continue;
      if (It->definesRegister(MC6809::IX, /*TRI=*/nullptr) ||
          It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
        break;
    }
    if (IndexSrc.isValid()) {
      // Use CMPX/CMPY #0 directly — preserves D.
      auto OpcodePair = CompareImmediateOpcode.find(IndexSrc);
      assert(OpcodePair != CompareImmediateOpcode.end());
      Builder.buildInstr(OpcodePair->getSecond()).addImm(0);
      MI.eraseFromParent();
      return;
    }
    // Fallback: load spill into D and test.
    Register RealReg = getRealRegForSpill(SrcReg);
    emitSpillLoad(Builder, RealReg, SrcReg, MF);
    SrcReg = RealReg;
  }
  // TSTD/TSTW are HD6309-only. On 6809, use CMPD/CMPW #0 instead.
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (!STI.has6309() && (SrcReg == MC6809::AD || SrcReg == MC6809::AW)) {
    unsigned CmpOpc = (SrcReg == MC6809::AD) ? MC6809::CMPDi16 : MC6809::CMPWi16;
    Builder.buildInstr(CmpOpc).addImm(0);
  } else {
    auto OpcodePair = TestRegOpcode.find(SrcReg);
    if (OpcodePair == TestRegOpcode.end())
      llvm_unreachable("Test register - unexpected register.");
    Builder.buildInstr(OpcodePair->getSecond());
  }
  MI.eraseFromParent();
}

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
