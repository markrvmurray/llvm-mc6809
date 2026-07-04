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
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/VirtRegMap.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/Support/CommandLine.h"
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
      {{MC6809::AA}, MC6809::ADDAi8}, {{MC6809::AB}, MC6809::ADDBi8},
      {{MC6809::AE}, MC6809::ADDEi8}, {{MC6809::AF}, MC6809::ADDFi8},    {{MC6809::AD}, MC6809::ADDDi16}, {{MC6809::AW}, MC6809::ADDWi16},
  };
  AddCarryImmediateOpcode = {
      {{MC6809::AA}, MC6809::ADCAi8}, {{MC6809::AB}, MC6809::ADCBi8}, {{MC6809::AD}, MC6809::ADCDi16},
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
      {{MC6809::AA}, MC6809::SBCAi8}, {{MC6809::AB}, MC6809::SBCBi8}, {{MC6809::AD}, MC6809::SBCDi16},
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
      {{MC6809::AA}, MC6809::ANDAi8}, {{MC6809::AB}, MC6809::ANDBi8}, {{MC6809::AD}, MC6809::ANDDi16},
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
      {{MC6809::AA}, MC6809::ORAi8}, {{MC6809::AB}, MC6809::ORBi8}, {{MC6809::AD}, MC6809::ORDi16},
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

/// Check if this is a fused compare-and-branch pseudo.
static bool isFusedCompareBranch(unsigned Opc) {
  switch (Opc) {
  case MC6809::TestBranch_i8_Reg:  case MC6809::TestBranch_i16_Reg:
  case MC6809::TestBranch_i8_Mem:  case MC6809::TestBranch_i16_Mem:
  case MC6809::CompareBranch_i8_Imm:  case MC6809::CompareBranch_i16_Imm:
  case MC6809::CompareBranch_i8_Reg:  case MC6809::CompareBranch_i16_Reg:
  case MC6809::CompareBranch_i8_Mem:  case MC6809::CompareBranch_i16_Mem:
    return true;
  default:
    return false;
  }
}

// Bug #206 + #271 cat-1: select the verifier-friendly Bbc/LBlbc variant.
// The _NoC variants are encoding-equivalent codegen-only opcodes that
// declare `Uses = [N, Z, V]` (no C); used when the cc immediate doesn't
// read C at runtime. The _OnlyC variants declare `Uses = [C]`; used
// when only C matters (cc=HS/CC=4, LO/CS=5). The canonical Bbc/LBlbc
// keeps the union `Uses = [N, Z, V, C]` for cc values that read more
// than one bit (HI/LS, GE/LT, GT/LE etc.). Predicates live in
// MC6809.h:MC6809CC::doesNotReadCarry / doesOnlyReadCarry so the
// same logic is reused by the GISel selector.
static unsigned pickBbcVariant(int64_t CC) {
  if (MC6809CC::doesNotReadCarry(CC))
    return MC6809::Bbc_NoC;
  if (MC6809CC::doesOnlyReadCarry(CC))
    return MC6809::Bbc_OnlyC;
  return MC6809::Bbc;
}

static unsigned pickLBlbcVariant(int64_t CC) {
  if (MC6809CC::doesNotReadCarry(CC))
    return MC6809::LBlbc_NoC;
  if (MC6809CC::doesOnlyReadCarry(CC))
    return MC6809::LBlbc_OnlyC;
  return MC6809::LBlbc;
}

// Bug #206 + #271 cat-1: classify Bbc / Bbc_NoC / Bbc_OnlyC / LBlbc /
// LBlbc_NoC / LBlbc_OnlyC uniformly. All six are the same hardware
// instruction with different LLVM-side metadata (the _NoC / _OnlyC
// variants declare a tighter Uses set so the verifier doesn't
// false-positive on TST + branch / CarrySet + Store + branch pairs).
static bool isBbcOrLBlbc(unsigned Opc) {
  return Opc == MC6809::Bbc || Opc == MC6809::Bbc_NoC ||
         Opc == MC6809::Bbc_OnlyC ||
         Opc == MC6809::LBlbc || Opc == MC6809::LBlbc_NoC ||
         Opc == MC6809::LBlbc_OnlyC;
}

bool MC6809InstrInfo::isCondBranch(const MachineBasicBlock::instr_iterator &I) const {
  if (I->isBranch()) {
    if (isBbcOrLBlbc(I->getOpcode()))
      return I->getOperand(0).getImm() != MC6809CC::RA;
    if (isFusedCompareBranch(I->getOpcode()))
      return true;
    return I->isConditionalBranch();
  }
  return false;
}

bool MC6809InstrInfo::isUnCondBranch(const MachineBasicBlock::instr_iterator &I) const {
  if (I->isBranch()) {
    if (isBbcOrLBlbc(I->getOpcode()))
      return I->getOperand(0).getImm() == MC6809CC::RA;
    return I->isUnconditionalBranch();
  }
  return false;
}

MachineBasicBlock *MC6809InstrInfo::getBB(const MachineBasicBlock::instr_iterator &I) const {
  if (I->getOpcode() == TargetOpcode::G_BR || I->getOpcode() == MC6809::BranchRelative || I->getOpcode() == MC6809::LongBranchRelative || I->getOpcode() == MC6809::JMPe || I->getOpcode() == MC6809::JMPi_o16PC)
    return I->getOperand(0).getMBB();
  if (I->getOpcode() == TargetOpcode::G_BRCOND || I->getOpcode() == MC6809::ConditionalBranchRelative || I->getOpcode() == MC6809::ConditionalLongBranchRelative || isBbcOrLBlbc(I->getOpcode()))
    return I->getOperand(1).getMBB();
  // Fused compare-and-branch: target MBB is the last operand.
  if (isFusedCompareBranch(I->getOpcode()))
    return I->getOperand(I->getNumOperands() - 1).getMBB();
  llvm_unreachable("Unable to handle opcode. Please fix me!");
}

Register MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const {
  SmallVector<const MachineMemOperand *, 1> Accesses;
  // A compare/branch may read memory (e.g. Compare_ptr_Mem / CompareBranch_ptr_Mem,
  // a pointer compared against a frame slot) but does not load a value into
  // operand 0 — its operand 0 is the CCond def or the condition immediate, not
  // the loaded register. Only a genuine load defines its loaded value there.
  if (MI.mayLoad() && !MI.isCompare() && !MI.isBranch() &&
      MI.getNumOperands() > 0 && MI.getOperand(0).isReg() &&
      hasLoadFromStackSlot(MI, Accesses) && Accesses.size() == 1) {
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

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator I, Register DestReg, unsigned SubIdx, const MachineInstr &Orig, LaneBitmask UsedLanes) const {
  auto opcode = Orig.getOpcode();
  if (opcode == MC6809::Load_i8_Imm || opcode == MC6809::Load_i16_Imm || opcode == MC6809::Load_i32_Imm) {
    const TargetRegisterInfo &TRI = getRegisterInfo();
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->removeOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(opcode));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig, UsedLanes);
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
  // Meta instructions (BUNDLE handled separately, KILL, IMPLICIT_DEF,
  // DBG_VALUE, DBG_LABEL, etc.) emit no bytes.
  if (MI.isMetaInstruction() && MI.getOpcode() != TargetOpcode::BUNDLE)
    return 0;

  const MachineBasicBlock &MBB = *MI.getParent();
  const MachineFunction *MF = MBB.getParent();
  const MCAsmInfo *MAI = MF->getTarget().getMCAsmInfo();

  const MCInstrDesc &MCID = MI.getDesc();

  switch (MI.getOpcode()) {
  default: {
    // Default branch: TableGen-declared `let Size = N` is authoritative.
    // PseudoInstExpansion-based pseudos (BranchSubroutine, JumpAbsolute,
    // ReturnImplicit, etc.) carry explicit `let Size` matching their
    // post-expansion concrete form (bug #183 commit 1).
    //
    // Fallback: if a pseudo ships without `let Size`, return MaxInstLength
    // (5) so BranchRelaxation conservatively OVERESTIMATES the block size
    // and widens borderline branches that might otherwise overflow. The
    // alternative — silently returning 0 — caused bug #174's PCRel8
    // fixup overflow in __file_wstr_get when LongBranchSubroutine pseudos
    // contributed 0 to BR's accounting (commit 1 fixed those specifically;
    // this fallback ensures the next undeclared pseudo fails safe instead
    // of repeating that class of bug).
    unsigned Size = MCID.getSize();
    if (!Size)
      Size = MAI->getMaxInstLength(&MF->getSubtarget());
    return Size;
  }
  case TargetOpcode::BUNDLE:
    return getInstBundleLength(MI);
  case MC6809::INLINEASM:
  case MC6809::INLINEASM_BR:
    // If this machine instr is an inline asm, measure it.
    return getInlineAsmLength(MI.getOperand(0).getSymbolName(), *MAI);
  }
}

// XXXX FixMe: MarkM. Branch offset relaxation should cover for all sins committed, but only
// once we have lowered to non-pseudo instructions.
bool MC6809InstrInfo::isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const {
  // Branch ranges (signed offset from PC after the instruction):
  //   Short branches (BRA, BCC, BCS, etc.):    8-bit signed: -128..+127
  //   Long branches (LBRA, LBCC, LBCS, etc.): 16-bit signed: -32768..+32767
  //
  // The pseudos BranchRelative / ConditionalBranchRelative get expanded into
  // BRAb / Bbc respectively. If isBranchOffsetInRange returns false for
  // these, the BranchRelaxation pass replaces them with their long-branch
  // counterparts (LongBranchRelative / ConditionalLongBranchRelative).
  //
  // Returning true unconditionally was a stub — it disabled relaxation and
  // caused short branches with out-of-range offsets to silently truncate
  // their offset bytes, jumping to wrong addresses (was bug #58).
  //
  // Bug #182 fix: BR's BrOffset is the raw start-to-start distance
  // (per `BranchRelaxation::isBlockInRange` => `DestOffset - BrOffset`).
  // The encoded displacement byte is `target - (PC after instruction)`
  // = `BrOffset - <instruction size>`. Subtract the size here so the
  // `isInt<N>` check evaluates the value that will actually be encoded.
  // Without the subtraction the short forms boundary-fail at raw disp
  // ∈ [-128..-126] (backward) or [127..129] (forward): BR concludes
  // "in range" but the assembler emits a -130/+130-style fixup, which
  // the AsmBackend PCRel8 guard rejects.
  switch (BranchOpc) {
  case MC6809::BranchRelative:
  case MC6809::ConditionalBranchRelative:
  case MC6809::JumpRelative:
  case MC6809::BRAb:
  case MC6809::Bbc:
  case MC6809::Bbc_NoC:    // bug #206: encoding-equivalent variant
  case MC6809::Bbc_OnlyC:  // bug #271 cat-1: encoding-equivalent variant
    return isInt<8>(BrOffset - 2);   // 2-byte short forms
  case MC6809::LongBranchRelative:
  case MC6809::LongJumpRelative:
  case MC6809::LBRAlb:
    return isInt<16>(BrOffset - 3);  // 3-byte page-1 long
  case MC6809::ConditionalLongBranchRelative:
  case MC6809::LBlbc:
  case MC6809::LBlbc_NoC:    // bug #206: encoding-equivalent variant
  case MC6809::LBlbc_OnlyC:  // bug #271 cat-1: encoding-equivalent variant
    return isInt<16>(BrOffset - 4);  // 4-byte page-2 long
  default:
    // Unknown branch opcode — be conservative and say it's in range so we
    // don't break anything that doesn't follow the BRA/LBRA pattern.
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

// Bug #196: findCommutedOpIndices is no longer overridden — the
// base-class TargetInstrInfo::findCommutedOpIndices uses the
// TableGen-generated `isCommutable` flag to recognise commutable
// pseudos and returns the standard {1, 2} operand pair. Per the
// AArch64 reference (which also doesn't override), this is the
// simplest design. Add `let isCommutable = 1` to TableGen multiclass
// bodies (e.g. MC6809Bitwise._Reg in MC6809InstrFamilies.td) to
// enrol new commute targets.

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
  case MC6809::Bbc_NoC:    // bug #206
  case MC6809::Bbc_OnlyC:  // bug #271 cat-1
  case MC6809::LBlbc:
  case MC6809::LBlbc_NoC:    // bug #206
  case MC6809::LBlbc_OnlyC:  // bug #271 cat-1
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

    // Fused compare-and-branch pseudos: opaque to branch analysis.
    if (isFusedCompareBranch(I->getOpcode()))
      return true;

    // Operand layout differs by branch kind:
    //   Unconditional (BRAb, LBRAlb, BranchRelative, ...):  (MBB)
    //   Conditional   (Bbc, LBlbc, ConditionalBranchRelative, ...): (CondImm, MBB)
    // Map operand count → MBB operand index. Anything else is opaque.
    unsigned NumOps = I->getNumExplicitOperands();
    unsigned MBBOpIdx;
    if (NumOps == 1)
      MBBOpIdx = 0;
    else if (NumOps == 2)
      MBBOpIdx = 1;
    else
      return true;

    // Cannot handle branches that don't branch to a block.
    if (!I->getOperand(MBBOpIdx).isMBB()) {
      return true;
    }

    // Handle unconditional branches.
    if (NumOps == 1) {
      UnCondBrIter = I;

      if (!AllowModify) {
        TBB = I->getOperand(MBBOpIdx).getMBB();
        continue;
      }

      // If the block has any instructions after an unconditional branch, delete them.
      while (std::next(I) != MBB.end())
        std::next(I)->eraseFromParent();
      Cond.clear();
      FBB = nullptr;

      // Delete the unconditional branch if it's equivalent to a fall-through.
      if (MBB.isLayoutSuccessor(I->getOperand(MBBOpIdx).getMBB())) {
        TBB = nullptr;
        I->eraseFromParent();
        I = MBB.end();
        UnCondBrIter = I;
        continue;
      }

      // TBB is used to indicate the unconditional destination.
      TBB = I->getOperand(MBBOpIdx).getMBB();
      continue;
    }

    // Handle conditional branches: operand 0 is the condition code immediate,
    // operand 1 is the target MBB.
    assert(NumOps == 2 && "Invalid conditional branch");
    // Guard against generic MIR instructions (G_BRCOND) where operand 0
    // is a virtual register, not an immediate condition code. These appear
    // before instruction selection and are opaque to branch analysis.
    if (!I->getOperand(0).isImm())
      return true;
    MC6809CC::CondCode CC = MC6809CC::CondCode(I->getOperand(0).getImm());

    // Working from the bottom, handle the first conditional branch.
    if (Cond.empty()) {
      MachineBasicBlock *TargetBB = I->getOperand(MBBOpIdx).getMBB();
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

        // Emit short by default; standard LLVM BranchRelaxation widens
        // via CFG-split + insertIndirectBranch when the displacement is
        // out of int8 range (bug #174). BuildMI auto-attaches the
        // implicit Uses declared by the chosen Bbc/Bbc_NoC MCInstrDesc
        // (bug #206 picker — _NoC drops C from Uses for cc that doesn't
        // consume it).
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(pickBbcVariant(CC))).addImm(CC).addMBB(UnCondBrIter->getOperand(0).getMBB());
        BuildMI(MBB, UnCondBrIter, MBB.findDebugLoc(I), get(MC6809::BRAb)).addMBB(TargetBB);

        OldInst->eraseFromParent();
        UnCondBrIter->eraseFromParent();

        // Restart the analysis.
        UnCondBrIter = MBB.end();
        I = MBB.end();
        continue;
      }

      FBB = TBB;
      TBB = I->getOperand(MBBOpIdx).getMBB();
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
    if (isFusedCompareBranch(I->getOpcode()))
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
    // Unconditional branch — emit short by default. Standard LLVM
    // BranchRelaxation widens via insertIndirectBranch when out of int8
    // range (bug #174).
    assert(!FBB && "Unconditional branch with multiple successors!");
    Bytes += getInstSizeInBytes(*BuildMI(&MBB, DL, get(MC6809::BRAb)).addMBB(TBB));
    ++Count;
  } else {
    // Conditional branch — emit short by default. BranchRelaxation
    // widens via CFG-split + insertIndirectBranch as needed (bug #174).
    // Bug #206: pickBbcVariant selects Bbc_NoC for cc that doesn't
    // consume C — verifier-friendly, encoding-equivalent.
    int64_t CC = Cond[0].getImm();
    Bytes += getInstSizeInBytes(*BuildMI(&MBB, DL, get(pickBbcVariant(CC))).add(Cond[0]).addMBB(TBB));
    ++Count;

    // If FBB is null, it is implied to be a fall-through block.
    if (FBB) {
      // Two-way Conditional branch. Insert the second branch (short).
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
/// Phase A 2026-05-16: SPILL_Q[0..31] enum values are consecutive
/// (verified via MC6809GenRegisterInfoEnums.inc: SPILL_Q0 = 381,
/// SPILL_Q31 = 412), so range checks replace the 32-entry case lists
/// that were previously hand-spelled across 5 sites in this file.
/// Same for SPILL_Q*HI (697..728) and SPILL_Q*LO (729..760).
static bool isInSpillQ(Register Reg) {
  return Reg >= MC6809::SPILL_Q0 && Reg <= MC6809::SPILL_Q31;
}

static bool isSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return true;
  default:
    return isInSpillQ(Reg);
  }
}

/// Get the SPILL_D parent register for any spill register.  SPILL_Q*N
/// is its own parent (Phase A consolidation; see isInSpillQ).
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
  default:
    if (isInSpillQ(Reg)) return Reg;
    llvm_unreachable("Not a spill register");
  }
}

/// Get byte offset within the SPILL_D / SPILL_X / SPILL_Q frame object.
/// Big-endian: A (high byte) at offset 0, B (low byte) at offset 1.
/// SPILL_Q*N is at offset 0 (full 32-bit) — Phase A consolidation.
static int getSpillByteOffset(Register Reg) {
  if (isInSpillQ(Reg)) return 0;  // Full 32-bit
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
/// SPILL_Q*N → AQ (Phase A consolidation; see isInSpillQ).
static Register getRealRegForSpill(Register Reg) {
  if (isInSpillQ(Reg)) return MC6809::AQ;
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

/// Bug #161 round 14: Q (32-bit) spill register predicate. Used to route
/// emitSpillLoad / emitSpillStore through LDQ / STQ rather than the
/// AD-staged 16-bit path.  Phase A consolidation 2026-05-16: just a
/// thin wrapper around isInSpillQ.
static bool isQSpillReg(Register Reg) {
  return isInSpillQ(Reg);
}

/// Bug #161 round 14: SPILL_Q half-word sub-register predicate. The
/// VirtRegMap rewriter substitutes these when a REG_SEQUENCE-built ACC32
/// vreg lands in SPILL_Q* and a sub-reg consumer (sub_lo_word / sub_hi_word)
/// is rewritten. Returns true and outputs the parent SPILL_Q + which
/// half (true = LO = stack offset +2, false = HI = offset +0,
/// big-endian Q layout).
///
/// Phase A consolidation 2026-05-16: SPILL_Q*HI[N] and SPILL_Q*LO[N]
/// are consecutive in the enum (HI: 697..728, LO: 729..760, per
/// MC6809GenRegisterInfoEnums.inc) and parallel to SPILL_Q[N] (381..412).
/// Two range checks replace the 64-entry case list.
static bool isQSpillHalfReg(Register Reg, MCPhysReg &Parent, bool &IsLo) {
  if (Reg >= MC6809::SPILL_Q0HI && Reg <= MC6809::SPILL_Q31HI) {
    Parent = MC6809::SPILL_Q0 + (Reg - MC6809::SPILL_Q0HI);
    IsLo = false;
    return true;
  }
  if (Reg >= MC6809::SPILL_Q0LO && Reg <= MC6809::SPILL_Q31LO) {
    Parent = MC6809::SPILL_Q0 + (Reg - MC6809::SPILL_Q0LO);
    IsLo = true;
    return true;
  }
  return false;
}

/// Bug #301 Phase C Path C (2026-05-16): SPILL_Q byte sub-register predicate.
/// Each of the 32 SPILL_Q slots has 4 byte sub-registers in enum order:
/// SPILL_QnHIHI (slot+0), SPILL_QnHILO (slot+1), SPILL_QnLOHI (slot+2),
/// SPILL_QnLOLO (slot+3).  Returns true and outputs the parent SPILL_Q +
/// the byte offset within the 4-byte slot.
static bool isQSpillByteReg(Register Reg, MCPhysReg &Parent,
                             unsigned &ByteOffset) {
  if (Reg >= MC6809::SPILL_Q0HIHI && Reg <= MC6809::SPILL_Q31LOLO) {
    unsigned Idx = Reg - MC6809::SPILL_Q0HIHI;
    Parent = MC6809::SPILL_Q0 + (Idx / 4);
    ByteOffset = Idx % 4;
    return true;
  }
  return false;
}

/// Get the size in bytes of a spill register (1 for A/B, 2 for D/X, 4 for Q).
static unsigned getSpillRegSize(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3: case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3: case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
    return 1;
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3: case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
  case MC6809::SPILL_X0: case MC6809::SPILL_X1: case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return 2;
  case MC6809::SPILL_Q0:  case MC6809::SPILL_Q1:  case MC6809::SPILL_Q2:  case MC6809::SPILL_Q3:
  case MC6809::SPILL_Q4:  case MC6809::SPILL_Q5:  case MC6809::SPILL_Q6:  case MC6809::SPILL_Q7:
  case MC6809::SPILL_Q8:  case MC6809::SPILL_Q9:  case MC6809::SPILL_Q10: case MC6809::SPILL_Q11:
  case MC6809::SPILL_Q12: case MC6809::SPILL_Q13: case MC6809::SPILL_Q14: case MC6809::SPILL_Q15:
  case MC6809::SPILL_Q16: case MC6809::SPILL_Q17: case MC6809::SPILL_Q18: case MC6809::SPILL_Q19:
  case MC6809::SPILL_Q20: case MC6809::SPILL_Q21: case MC6809::SPILL_Q22: case MC6809::SPILL_Q23:
  case MC6809::SPILL_Q24: case MC6809::SPILL_Q25: case MC6809::SPILL_Q26: case MC6809::SPILL_Q27:
  case MC6809::SPILL_Q28: case MC6809::SPILL_Q29: case MC6809::SPILL_Q30: case MC6809::SPILL_Q31:
    return 4;
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

/// For the backward-scan spill peephole in expandTestReg / expandCompareIdx:
/// returns true if MI is a store that could overwrite bytes in the range
/// [SpillOffset, SpillOffset + SpillSize) of the S-frame. Conservative —
/// returns true for any mayStore whose target bytes can't be proven disjoint
/// (unknown opcode, non-immediate offset).
///
/// The peephole's premise is "the matching STX/STY wrote this spill slot and
/// nothing has clobbered it since, so IX/IY still mirrors the slot". Byte-level
/// rewrites of the slot (e.g. SubSetCarry_i8_Reg on the sub-byte spill alias)
/// break that premise without redefining IX/IY, so the scan must notice them.
static bool storeOverlapsSpillSlot(const MachineInstr &MI, int SpillOffset,
                                   unsigned SpillSize) {
  if (!MI.mayStore()) return false;
  unsigned StoreBytes = 0;
  int StoreOff = 0;
  bool HasExplicitOffset = true;
  switch (MI.getOpcode()) {
  case MC6809::STAi_o0: case MC6809::STBi_o0: case MC6809::STEi_o0:
  case MC6809::STFi_o0:
    StoreBytes = 1; HasExplicitOffset = false; break;
  case MC6809::STAi_o5: case MC6809::STAi_o8: case MC6809::STAi_o16:
  case MC6809::STBi_o5: case MC6809::STBi_o8: case MC6809::STBi_o16:
  case MC6809::STEi_o5: case MC6809::STEi_o8: case MC6809::STEi_o16:
  case MC6809::STFi_o5: case MC6809::STFi_o8: case MC6809::STFi_o16:
    StoreBytes = 1; break;
  case MC6809::STDi_o0: case MC6809::STXi_o0: case MC6809::STYi_o0:
  case MC6809::STWi_o0: case MC6809::STUi_o0: case MC6809::STSi_o0:
    StoreBytes = 2; HasExplicitOffset = false; break;
  case MC6809::STDi_o5: case MC6809::STDi_o8: case MC6809::STDi_o16:
  case MC6809::STXi_o5: case MC6809::STXi_o8: case MC6809::STXi_o16:
  case MC6809::STYi_o5: case MC6809::STYi_o8: case MC6809::STYi_o16:
  case MC6809::STWi_o5: case MC6809::STWi_o8: case MC6809::STWi_o16:
  case MC6809::STUi_o5: case MC6809::STUi_o8: case MC6809::STUi_o16:
  case MC6809::STSi_o5: case MC6809::STSi_o8: case MC6809::STSi_o16:
    StoreBytes = 2; break;
  case MC6809::STQi_o0:
    StoreBytes = 4; HasExplicitOffset = false; break;
  case MC6809::STQi_o5: case MC6809::STQi_o8: case MC6809::STQi_o16:
    StoreBytes = 4; break;
  default:
    // Unknown store (e.g. register-offset indexed store or a memcpy pseudo).
    // Can't prove it misses the spill slot, so bail conservatively.
    return true;
  }
  // Index of the base-register operand: operand after the offset imm for
  // non-o0 forms, or operand 0 for the o0 forms.
  unsigned BaseOpIdx = 0;
  if (HasExplicitOffset) {
    if (!MI.getOperand(0).isImm()) return true;
    StoreOff = MI.getOperand(0).getImm();
    BaseOpIdx = 1;
  }
  // Only stores on the frame base ($su) can hit our spill slot. Non-SU base
  // stores (e.g. via IX, IY) address unrelated memory — ignore them.
  if (MI.getNumOperands() <= BaseOpIdx || !MI.getOperand(BaseOpIdx).isReg() ||
      MI.getOperand(BaseOpIdx).getReg() != MC6809::SU)
    return false;
  int SpillEnd = SpillOffset + (int)SpillSize;
  int StoreEnd = StoreOff + (int)StoreBytes;
  return StoreOff < SpillEnd && StoreEnd > SpillOffset;
}

/// Pick the right indexed load opcode for a given register and offset.
static unsigned getLoadIdxOpcode(Register Reg, int Offset) {
  bool Is8 = (Offset >= -128 && Offset <= 127);
  if (Reg == MC6809::AA) return Is8 ? MC6809::LDAi_o8 : MC6809::LDAi_o16;
  if (Reg == MC6809::AB) return Is8 ? MC6809::LDBi_o8 : MC6809::LDBi_o16;
  if (Reg == MC6809::AD) return Is8 ? MC6809::LDDi_o8 : MC6809::LDDi_o16;
  if (Reg == MC6809::AE) return Is8 ? MC6809::LDEi_o8 : MC6809::LDEi_o16;
  if (Reg == MC6809::AF) return Is8 ? MC6809::LDFi_o8 : MC6809::LDFi_o16;
  if (Reg == MC6809::AW) return Is8 ? MC6809::LDWi_o8 : MC6809::LDWi_o16;
  if (Reg == MC6809::IX) return Is8 ? MC6809::LDXi_o8 : MC6809::LDXi_o16;
  if (Reg == MC6809::IY) return Is8 ? MC6809::LDYi_o8 : MC6809::LDYi_o16;
  if (Reg == MC6809::AQ) return Is8 ? MC6809::LDQi_o8 : MC6809::LDQi_o16;
  llvm_unreachable("Unexpected register for spill load");
}

/// Pick the right indexed store opcode for a given register and offset.
static unsigned getStoreIdxOpcode(Register Reg, int Offset) {
  bool Is8 = (Offset >= -128 && Offset <= 127);
  if (Reg == MC6809::AA) return Is8 ? MC6809::STAi_o8 : MC6809::STAi_o16;
  if (Reg == MC6809::AB) return Is8 ? MC6809::STBi_o8 : MC6809::STBi_o16;
  if (Reg == MC6809::AD) return Is8 ? MC6809::STDi_o8 : MC6809::STDi_o16;
  if (Reg == MC6809::AE) return Is8 ? MC6809::STEi_o8 : MC6809::STEi_o16;
  if (Reg == MC6809::AF) return Is8 ? MC6809::STFi_o8 : MC6809::STFi_o16;
  if (Reg == MC6809::AW) return Is8 ? MC6809::STWi_o8 : MC6809::STWi_o16;
  if (Reg == MC6809::IX) return Is8 ? MC6809::STXi_o8 : MC6809::STXi_o16;
  if (Reg == MC6809::IY) return Is8 ? MC6809::STYi_o8 : MC6809::STYi_o16;
  if (Reg == MC6809::AQ) return Is8 ? MC6809::STQi_o8 : MC6809::STQi_o16;
  llvm_unreachable("Unexpected register for spill store");
}

// Bug #387: the static-stack support functions. getSymLoadOpcode/Store are
// defined later in this file (the global-symbol expander); forward-declare them
// so the spill expanders can emit an extended/absolute access to a static slot.
static unsigned getSymLoadOpcode(Register Reg, bool IsDP, bool IsPIC);
static unsigned getSymStoreOpcode(Register Reg, bool IsDP, bool IsPIC);
static unsigned getStaticStackOpcode(unsigned IdxOpc, bool IsPIC);

// The frame index backing a spill register's slot (mirrors the key logic in
// computeSpillStackOffset).
static int spillSlotFI(MCPhysReg SpillReg, MachineFunction &MF) {
  auto &FuncInfo = *MF.getInfo<MC6809FunctionInfo>();
  MCPhysReg SpillKey =
      isIndexSpillReg(SpillReg) ? SpillReg : getSpillDParent(SpillReg);
  return FuncInfo.SpillRegFrameIndices[SpillKey];
}

// True when a spill register's slot was moved to the static stack (Bug #387).
// Such a slot is NOT at any U-relative address — computeSpillStackOffset's
// result is meaningless for it; it must be reached by an extended access.
static bool isStaticSpillSlot(MCPhysReg SpillReg, MachineFunction &MF) {
  return MF.getFrameInfo().getStackID(spillSlotFI(SpillReg, MF)) ==
         TargetStackID::Mc6809Static;
}

// The per-function static-stack byte offset of a spill register (the slot's
// static offset plus the sub-slot byte for B). MC6809StaticStackAlloc resolves
// the carried TI_STATIC_STACK target index + this offset to the real global.
static int staticSpillOffset(MCPhysReg SpillReg, MachineFunction &MF) {
  return MF.getFrameInfo().getObjectOffset(spillSlotFI(SpillReg, MF)) +
         getSpillByteOffset(SpillReg);
}

/// Emit a concrete U-indexed (frame pointer) load from a spill register's
/// stack slot. Uses U (not S) so PSHS/PULS don't invalidate offsets.
static MachineInstrBuilder emitSpillLoad(MachineIRBuilder &Builder,
                                         Register RealReg, MCPhysReg SpillReg,
                                         MachineFunction &MF) {
  unsigned Size = getSpillRegSize(SpillReg);
  // INDEX spills (SPILL_X0..X3) use LDX/LDY directly — no D clobber.
  // Q spills (SPILL_Q0..Q3) use LDQ — operate directly on AQ (HD6309).
  // ACC spills (SPILL_D0..D7) route through D as before.
  Register Reg = (Size == 2 && !isIndexSpillReg(SpillReg))
                     ? Register(MC6809::AD)
                     : RealReg;
  // Bug #387: a static-stack slot is addressed absolutely, not via U.
  if (isStaticSpillSlot(SpillReg, MF)) {
    unsigned Opc = getSymLoadOpcode(Reg, /*IsDP=*/false,
                                    MF.getTarget().isPositionIndependent());
    return Builder.buildInstr(Opc)
        .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(SpillReg, MF))
        .addDef(Reg, RegState::Implicit);
  }
  int Offset = computeSpillStackOffset(SpillReg, MF);
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
  unsigned Size = getSpillRegSize(SpillReg);
  // INDEX spills (SPILL_X0..X3) use STX/STY directly — no D clobber.
  // Q spills (SPILL_Q0..Q3) use STQ — operate directly on AQ (HD6309).
  // ACC spills (SPILL_D0..D7) route through D as before.
  Register Reg = (Size == 2 && !isIndexSpillReg(SpillReg))
                     ? Register(MC6809::AD)
                     : RealReg;
  // Bug #387: a static-stack slot is addressed absolutely, not via U. Keep the
  // SpillReg implicit-def (Bug #285) so the pseudo's destination stays defined.
  if (isStaticSpillSlot(SpillReg, MF)) {
    unsigned Opc = getSymStoreOpcode(Reg, /*IsDP=*/false,
                                     MF.getTarget().isPositionIndependent());
    return Builder.buildInstr(Opc)
        .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(SpillReg, MF))
        .addUse(Reg, RegState::Implicit)
        .addReg(Register(SpillReg), RegState::ImplicitDefine);
  }
  int Offset = computeSpillStackOffset(SpillReg, MF);
  unsigned Opcode = getStoreIdxOpcode(Reg, Offset);
  // Bug #285: this store fills the imaginary spill slot, so it models
  // a write of the SpillReg pseudo-register. Without an explicit
  // implicit-def, callers that erase the originating pseudo (e.g.
  // ZEX32Implicit, SEX32Implicit, *_i32_Mem spill helpers) leave the
  // SPILL_* destination with no visible def in the final MIR.
  // -fextend-lifetimes' FAKE_USE references (-Og) then fail with
  // "Using an undefined physical register" at the post-postrapseudos
  // verifier. Manifest at Og-hd6309-mame on hash_buf.c __get_buf
  // after Bug #284's wider ZEX32Implicit Defs caused regalloc to
  // prefer $spill_q0 over $aq.
  auto MI = Builder.buildInstr(Opcode)
      .addUse(Reg, RegState::Implicit)
      .addImm(Offset)
      .addReg(MC6809::SU)  // Frame pointer — stable across PSHS/PULS
      .addReg(Register(SpillReg), RegState::ImplicitDefine);
  return MI;
}

/// Emit an accumulator arithmetic/compare op that reads its operand
/// directly from a spill register's slot (e.g. `addb n,u`). This is the single
/// choke point for the ~40 arith-with-spill sites: for an ordinary dynamic slot
/// it emits the U-indexed form (picking o8 vs o16 by the offset), and for a slot
/// the static-stack allocator moved into the static_stack global it emits the
/// extended (absolute, or PC-relative under PIC) form carrying a TI_STATIC_STACK
/// target index that MC6809StaticStackAlloc later resolves. AccReg is the
/// accumulator the op reads-and-writes (added as an implicit def, matching the
/// existing sites). Opc_o16 may be 0 when the op has no 16-bit-offset form.
static MachineInstrBuilder emitSpillArith(MachineIRBuilder &Builder,
                                          unsigned Opc_o8, unsigned Opc_o16,
                                          Register AccReg, MCPhysReg SpillReg,
                                          MachineFunction &MF) {
  if (isStaticSpillSlot(SpillReg, MF)) {
    unsigned Opc = getStaticStackOpcode(
        Opc_o8, MF.getTarget().isPositionIndependent());
    return Builder.buildInstr(Opc)
        .addDef(AccReg, RegState::Implicit)
        .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(SpillReg, MF));
  }
  int Offset = computeSpillStackOffset(SpillReg, MF);
  bool Fits8 = (Offset >= -128 && Offset <= 127);
  unsigned Opc = (Fits8 || !Opc_o16) ? Opc_o8 : Opc_o16;
  return Builder.buildInstr(Opc)
      .addDef(AccReg, RegState::Implicit)
      .addImm(Offset)
      .addReg(MC6809::SU);
}

/// Load DestReg directly from a spill register's slot (plus an optional
/// sub-slot ExtraOffset, e.g. +2 for a hi-half). Emits the U-indexed form for a
/// dynamic slot or the extended TI_STATIC_STACK form for a static-stack one.
/// DestReg is the concrete register to load (AA/AB/AD/AW/AE/AF/AQ/IX/IY);
/// getLoadIdxOpcode / getSymLoadOpcode pick the right load opcode for it.
static MachineInstrBuilder emitSpillLoadInto(MachineIRBuilder &Builder,
                                             Register DestReg,
                                             MCPhysReg SpillReg, int ExtraOffset,
                                             MachineFunction &MF) {
  if (isStaticSpillSlot(SpillReg, MF)) {
    unsigned Opc = getSymLoadOpcode(DestReg, /*IsDP=*/false,
                                    MF.getTarget().isPositionIndependent());
    return Builder.buildInstr(Opc)
        .addDef(DestReg, RegState::Implicit)
        .addTargetIndex(MC6809::TI_STATIC_STACK,
                        staticSpillOffset(SpillReg, MF) + ExtraOffset);
  }
  int Offset = computeSpillStackOffset(SpillReg, MF) + ExtraOffset;
  return Builder.buildInstr(getLoadIdxOpcode(DestReg, Offset))
      .addDef(DestReg, RegState::Implicit)
      .addImm(Offset)
      .addReg(MC6809::SU);
}

/// Store SrcReg directly into a spill register's slot (plus an optional sub-slot
/// ExtraOffset). Dynamic -> U-indexed; static -> extended TI_STATIC_STACK. When
/// DefsSpill is set the store also carries an implicit-def of SpillReg (Bug #285:
/// it fills the spill pseudo, keeping the destination visibly defined).
static MachineInstrBuilder emitSpillStoreFrom(MachineIRBuilder &Builder,
                                              Register SrcReg,
                                              MCPhysReg SpillReg, int ExtraOffset,
                                              MachineFunction &MF,
                                              bool DefsSpill = false) {
  if (isStaticSpillSlot(SpillReg, MF)) {
    unsigned Opc = getSymStoreOpcode(SrcReg, /*IsDP=*/false,
                                     MF.getTarget().isPositionIndependent());
    auto MIB = Builder.buildInstr(Opc)
                   .addUse(SrcReg, RegState::Implicit)
                   .addTargetIndex(MC6809::TI_STATIC_STACK,
                                   staticSpillOffset(SpillReg, MF) + ExtraOffset);
    if (DefsSpill)
      MIB.addReg(Register(SpillReg), RegState::ImplicitDefine);
    return MIB;
  }
  int Offset = computeSpillStackOffset(SpillReg, MF) + ExtraOffset;
  auto MIB = Builder.buildInstr(getStoreIdxOpcode(SrcReg, Offset))
                 .addUse(SrcReg, RegState::Implicit)
                 .addImm(Offset)
                 .addReg(MC6809::SU);
  if (DefsSpill)
    MIB.addReg(Register(SpillReg), RegState::ImplicitDefine);
  return MIB;
}

/// Bug #298 / #300 (consolidated Phase A 2026-05-16): bracket a body
/// of post-RA MIs with a 4-byte hard-stack scratch slot that holds
/// $aq's pre-body value.  Two variants:
///
///  - `emitAQOnHardStackScratch` saves $aq into the scratch and
///    releases the slot after body, but DOES NOT restore $aq.  Use
///    when body destroys $aq deliberately (e.g. body IS the i32
///    arithmetic whose result becomes the new $aq).  Body can read
///    the saved $aq via $ss + 0..3 indexing.  Origin:
///    `expandAddSub_i32_Reg` (Bug #298), where SRC2 is in $aq and is
///    read via $ss+0..3 while DST $aq is overwritten with ADDW+ADCD's
///    result.
///
///  - `emitAQPreservedOverHardStackScratch` saves $aq into the
///    scratch, runs body (which may transiently clobber $aq), then
///    restores $aq from the scratch and releases.  Use when body's
///    side-effect on $aq must be hidden from the caller.  Origin:
///    `expandLoadImm` SPILL_Q*N branch (Bug #300), where LDQ #imm
///    transiently stages through $aq before STQ-to-spill-slot.
///
/// Both helpers emit the same `LEAS -4,$ss; STQ ,$ss; <body>;
/// [LDQ ,$ss;] LEAS 4,$ss` shape.  Per
/// `feedback_leas_save_restore_not_for_common_pseudos.md`, only the
/// existing call sites should use these — adding to common pseudos
/// (e.g. Add/Sub_i32_Mem) blows past the 64KB cart limit.
static void emitAQOnHardStackScratch(MachineIRBuilder &Builder,
                                      llvm::function_ref<void()> Body) {
  Builder.buildInstr(MC6809::LEASi_o5).addImm(-4).addReg(MC6809::SS);
  Builder.buildInstr(MC6809::STQi_o0)
      .addUse(MC6809::AQ, RegState::Implicit)
      .addReg(MC6809::SS);
  Body();
  Builder.buildInstr(MC6809::LEASi_o5).addImm(4).addReg(MC6809::SS);
}

static void emitAQPreservedOverHardStackScratch(
    MachineIRBuilder &Builder, llvm::function_ref<void()> Body) {
  Builder.buildInstr(MC6809::LEASi_o5).addImm(-4).addReg(MC6809::SS);
  // Undef-marked read: the save/restore is the identity for whatever is
  // in $aq, defined or not. The #305 liveness gate keeps the bracket only
  // when some member of the family is live, but the live member can be a
  // lone byte half (a partially-redefined $aq the verifier would reject
  // as an undefined full-register read).
  Builder.buildInstr(MC6809::STQi_o0)
      .addUse(MC6809::AQ, RegState::Implicit | RegState::Undef)
      .addReg(MC6809::SS);
  Body();
  Builder.buildInstr(MC6809::LDQi_o0)
      .addDef(MC6809::AQ, RegState::Implicit)
      .addReg(MC6809::SS);
  Builder.buildInstr(MC6809::LEASi_o5).addImm(4).addReg(MC6809::SS);
}

// Forward declarations needed by emitTwoLDDSlotCopy.
static unsigned getLoadIdxOpcode(Register Reg, int Offset);
static unsigned getStoreIdxOpcode(Register Reg, int Offset);

/// Bug #221 / #274 (consolidated Phase A 2026-05-16): copy a 4-byte
/// i32 from one indexed address to another using two LDD/STD pairs,
/// avoiding LDQ/STQ.  LDQ/STQ would clobber AQ's sub-registers
/// (AW, AD, AA, AB, AE, AF) and risk corrupting regalloc-assigned
/// values living in those sub-registers — `$spill_q*` is an
/// imaginary reg distinct from physical $aq, so the regalloc treats
/// them independently and may legitimately place an unrelated vreg
/// in $aw / $ad across the Q-spill access.
///
/// Big-endian Q layout: byte 0..1 = HI word, byte 2..3 = LO word.
/// Emits HI half first (offset 0), then LO half (offset 2).
/// Only $ad is clobbered.
///
/// If `MarkerReg` is non-null, the LAST STD gets `implicit-def
/// <MarkerReg>` — used by `expandLoadIdx` SPILL_Q*N dst (Bug #274)
/// so downstream FAKE_USE / opaque consumers see the slot defined
/// for `-verify-machineinstrs`.
static void emitTwoLDDSlotCopy(MachineIRBuilder &Builder,
                                int SrcOff, Register SrcBase,
                                int DstOff, Register DstBase,
                                Register MarkerReg = Register()) {
  for (int H : {0, 2}) {
    bool LastHalf = (H == 2);
    unsigned LdOpc = getLoadIdxOpcode(MC6809::AD, SrcOff + H);
    Builder.buildInstr(LdOpc)
        .addDef(MC6809::AD, RegState::Implicit)
        .addImm(SrcOff + H).addReg(SrcBase);
    unsigned StOpc = getStoreIdxOpcode(MC6809::AD, DstOff + H);
    auto St = Builder.buildInstr(StOpc)
        .addUse(MC6809::AD, RegState::Implicit)
        .addImm(DstOff + H).addReg(DstBase);
    if (LastHalf && MarkerReg.isValid())
      St.addDef(MarkerReg, RegState::Implicit);
  }
}

/// Check if a register needs materialization (spill or imaginary — not a real
/// hardware register that can be used directly in instructions).
/// Check if Reg is a byte sub-register of an RS imaginary 16-bit register.
/// These are RS0HI..RS3HI (high bytes) and RS0LO..RS3LO (low bytes), added
/// to AAc/ABc so that TableGen-synthesised intersection classes include
/// RS0..RS3 in ACC16_with_sub_hi_byte_in_AAc etc.
static bool isImag16ByteSubReg(Register Reg) {
  switch (Reg.id()) {
  case MC6809::RS0HI: case MC6809::RS0LO:
  case MC6809::RS1HI: case MC6809::RS1LO:
  case MC6809::RS2HI: case MC6809::RS2LO:
  case MC6809::RS3HI: case MC6809::RS3LO:
    return true;
  default: return false;
  }
}

/// True if Reg is the HI (high) byte of its parent RS register.
static bool isImag16HiByte(Register Reg) {
  switch (Reg.id()) {
  case MC6809::RS0HI: case MC6809::RS1HI:
  case MC6809::RS2HI: case MC6809::RS3HI:
    return true;
  default: return false;
  }
}

/// Emit code that places SrcReg's byte value into the real accumulator half
/// Target (MC6809::AA or MC6809::AB), WITHOUT touching the other real half.
/// Used by the byte-merge/byte-extract pseudo expansions (bug #118 Layer 1,
/// approach b) to stage values into AD with strict separation between the
/// two halves.
///
/// SrcReg must be a member of the ACC8 class: real AA/AB/AE/AF, a SPILL_A*/
/// SPILL_B* byte spill, an Imag8 (RC*) direct-page byte, or (post-step-5) an
/// RS byte half (RS*HI/RS*LO).
static void loadByteInto(MachineIRBuilder &Builder, MCPhysReg Target,
                         Register SrcReg, MachineFunction &MF) {
  assert((Target == MC6809::AA || Target == MC6809::AB) &&
         "loadByteInto target must be AA or AB");
  if (SrcReg == Target)
    return;
  // Real byte registers — direct TFR, no staging through the other half.
  if (SrcReg == MC6809::AA || SrcReg == MC6809::AB ||
      SrcReg == MC6809::AE || SrcReg == MC6809::AF) {
    Builder.buildInstr(MC6809::TFRp).addDef(Target).addUse(SrcReg);
    return;
  }
  // Byte spill slots — LDA/LDB at the spill offset, Target chooses A vs B.
  if (isSpillReg(SrcReg)) {
    emitSpillLoad(Builder, Target, SrcReg, MF);
    return;
  }
  // Direct-page bytes (Imag8 RC* or RS byte halves) — LDAd/LDBd by Target.
  if (MC6809::Imag8RegClass.contains(SrcReg) || isImag16ByteSubReg(SrcReg)) {
    unsigned Opc = (Target == MC6809::AA) ? MC6809::LDAd : MC6809::LDBd;
    Builder.buildInstr(Opc).addReg(SrcReg);
    return;
  }
  llvm_unreachable("loadByteInto: unexpected source register class");
}

static bool needsMaterialization(Register Reg) {
  if (isSpillReg(Reg)) return true;
  if (isImag16ByteSubReg(Reg)) return true;
  return Reg.isPhysical() &&
         (MC6809::Imag8RegClass.contains(Reg) ||
          MC6809::Imag16RegClass.contains(Reg));
}

/// Get the physical hardware register that a non-physical register maps to.
static Register getPhysRegFor(Register Reg) {
  if (isSpillReg(Reg))
    return getRealRegForSpill(Reg);
  if (isImag16ByteSubReg(Reg))
    return isImag16HiByte(Reg) ? MC6809::AA : MC6809::AB;
  if (Reg.isPhysical() && MC6809::Imag8RegClass.contains(Reg))
    return MC6809::AB;
  if (Reg.isPhysical() && MC6809::Imag16RegClass.contains(Reg))
    return MC6809::AD;
  return Reg; // Already a real hardware register.
}

/// Materialize: if Reg is a spill or imaginary register, emit a load into the
/// corresponding hardware register and return it. If already physical, no-op.
static Register materializeReg(MachineIRBuilder &Builder, Register Reg,
                                MachineFunction &MF) {
  if (!needsMaterialization(Reg))
    return Reg;
  Register PhysReg = getPhysRegFor(Reg);
  if (isSpillReg(Reg)) {
    emitSpillLoad(Builder, PhysReg, Reg, MF);
  } else if (isImag16ByteSubReg(Reg)) {
    // Load a single byte from the RS slot using the sub-reg's own symbol.
    // Big-endian: HI byte symbol resolves to the base address (__rsNhi),
    // LO byte symbol resolves to base+1 (__rsNlo). We must NOT use LDD
    // of the parent — that clobbers the other half of D.
    unsigned Opc = isImag16HiByte(Reg) ? MC6809::LDAd : MC6809::LDBd;
    Builder.buildInstr(Opc).addReg(Reg);
    // After LDD: A=hi byte, B=lo byte. PhysReg is AA or AB as appropriate.
  } else if (MC6809::Imag8RegClass.contains(Reg)) {
    unsigned Opc = (PhysReg == MC6809::AA) ? MC6809::LDAd : MC6809::LDBd;
    Builder.buildInstr(Opc).addReg(Reg);
  } else if (MC6809::Imag16RegClass.contains(Reg)) {
    Builder.buildInstr(MC6809::LDDd).addReg(Reg);
  }
  return PhysReg;
}

/// Dematerialize: if OrigReg is a spill or imaginary register, store PhysReg
/// back to it. If OrigReg is already physical, no-op.
static void dematerializeReg(MachineIRBuilder &Builder, Register PhysReg,
                              Register OrigReg, MachineFunction &MF) {
  if (!needsMaterialization(OrigReg))
    return;
  if (isSpillReg(OrigReg)) {
    emitSpillStore(Builder, PhysReg, OrigReg, MF);
  } else if (isImag16ByteSubReg(OrigReg)) {
    // Store a single byte back to the RS slot using the sub-reg's own symbol.
    unsigned Opc = isImag16HiByte(OrigReg) ? MC6809::STAd : MC6809::STBd;
    Builder.buildInstr(Opc)
        .addReg(OrigReg, RegState::Undef)
        .addUse(isImag16HiByte(OrigReg) ? MC6809::AA : MC6809::AB,
                RegState::Implicit)
        .addReg(OrigReg, RegState::ImplicitDefine);
  } else if (MC6809::Imag8RegClass.contains(OrigReg)) {
    unsigned Opc = (PhysReg == MC6809::AA) ? MC6809::STAd : MC6809::STBd;
    Builder.buildInstr(Opc)
        .addReg(OrigReg, RegState::Undef)
        .addUse(PhysReg, RegState::Implicit)
        .addReg(OrigReg, RegState::ImplicitDefine);
  } else if (MC6809::Imag16RegClass.contains(OrigReg)) {
    // Declare the read of the accumulator being stored (the hardware
    // stores D) -- without it the verifier loses the staging value's
    // liveness and downstream kill flags go stale.
    Builder.buildInstr(MC6809::STDd)
        .addReg(OrigReg, RegState::Undef)
        .addUse(MC6809::AD, RegState::Implicit)
        .addReg(OrigReg, RegState::ImplicitDefine);
  }
}

/// Preserve a staging hardware register around a materialize/dematerialize
/// window. Post-RA, the surrounding code may hold a live value in the real
/// accumulator precisely BECAUSE regalloc parked this operand in an
/// imaginary/spill home; the staging load would silently clobber it (the
/// pseudo declares no accumulator Defs -- a fixed dead-def would veto
/// allocating the register at all). The push is undef-marked and
/// unconditional: no local liveness probe can decide "holds a value someone
/// reads" once sub-register halves are defined and consumed around the
/// site, and pushing then popping garbage is the identity when the register
/// was in fact dead. NOTE: any pre-existing S-relative memory operand
/// inside the window must be displacement-compensated (+size of the push).
static void pushStagingReg(MachineIRBuilder &Builder, Register RealReg) {
  if (RealReg == MC6809::AQ) {
    // AQ = D:W -- no page-1 push encodes it. PSHSW then PSHS D; the pull
    // mirrors in reverse. (HD6309-only staging register.)
    Builder.buildInstr(MC6809::PSHSWx);
    Builder.buildInstr(MC6809::PSHSs).addUse(MC6809::AD, RegState::Undef);
    return;
  }
  if (RealReg == MC6809::AW || RealReg == MC6809::AE ||
      RealReg == MC6809::AF) {
    Builder.buildInstr(MC6809::PSHSWx);
    return;
  }
  Builder.buildInstr(MC6809::PSHSs).addUse(RealReg, RegState::Undef);
}
static void pullStagingReg(MachineIRBuilder &Builder, Register RealReg) {
  if (RealReg == MC6809::AQ) {
    Builder.buildInstr(MC6809::PULSs).addDef(MC6809::AD);
    Builder.buildInstr(MC6809::PULSWx);
    return;
  }
  if (RealReg == MC6809::AW || RealReg == MC6809::AE ||
      RealReg == MC6809::AF) {
    Builder.buildInstr(MC6809::PULSWx);
    return;
  }
  Builder.buildInstr(MC6809::PULSs).addDef(RealReg);
}

/// Bytes a pushStagingReg push moves S down by.
static unsigned stagingPushSize(Register RealReg) {
  if (RealReg == MC6809::AQ)
    return 4;
  if (RealReg == MC6809::AD || RealReg == MC6809::AW ||
      RealReg == MC6809::AE || RealReg == MC6809::AF)
    return 2;
  return 1;
}

/// After pushing staging registers, any pre-existing S-relative operand on
/// the still-unexpanded pseudo is stale by the pushed byte count -- bump it.
static void compensateSSOperands(MachineInstr &MI, unsigned Bytes) {
  if (!Bytes)
    return;
  unsigned N = MI.getNumExplicitOperands();
  for (unsigned I = 0; I + 1 < N; ++I) {
    if (!MI.getOperand(I).isReg() || MI.getOperand(I).getReg() != MC6809::SS)
      continue;
    MachineOperand &Off = MI.getOperand(I + 1);
    // Offsets arrive both as plain immediates and as CImm (i16 ...).
    if (Off.isImm())
      Off.setImm(Off.getImm() + Bytes);
    else if (Off.isCImm())
      Off.ChangeToImmediate(Off.getCImm()->getSExtValue() + Bytes);
  }
}

/// Wrap a CC-producing expansion (compare/test, incl. the fused branch
/// forms) whose source operands may be imaginary/spill: push the staging
/// reals, expand, pull them back before any trailing terminator the
/// expansion emitted. PULS does not touch CC, so the restored flags reach
/// the consumer intact.
static void wrapStagedCCSources(MachineInstr &MI,
                                function_ref<void()> Expand) {
  SmallVector<Register, 2> Staged;
  for (const MachineOperand &MO : MI.explicit_operands())
    if (MO.isReg() && MO.isUse() && needsMaterialization(MO.getReg())) {
      Register R = getPhysRegFor(MO.getReg());
      if (!llvm::is_contained(Staged, R))
        Staged.push_back(R);
    }
  if (Staged.empty()) {
    Expand();
    return;
  }
  MachineBasicBlock &MBB = *MI.getParent();
  MachineBasicBlock::iterator NextPt = std::next(MI.getIterator());
  unsigned Bytes = 0;
  {
    MachineIRBuilder B(MBB, MI.getIterator());
    for (Register R : Staged) {
      pushStagingReg(B, R);
      Bytes += stagingPushSize(R);
    }
  }
  compensateSSOperands(MI, Bytes);
  Expand();
  MachineBasicBlock::iterator I = NextPt;
  while (I != MBB.begin() && std::prev(I)->isTerminator())
    --I;
  MachineIRBuilder B(MBB, I);
  for (Register R : llvm::reverse(Staged))
    pullStagingReg(B, R);
}

// Bug #161 round 17: materialize spill / imaginary operands into real
// hardware registers before emitting any HD6309 page-3 register-pair
// instruction (ADDR / ADCR / SUBR / SBCR / ANDR / ORR / EORR / etc).
// These opcodes encode 4-bit hardware register codes in their postbyte
// — a raw SPILL_* (DwarfRegNum 0x2000+) operand encodes as garbage that
// MAME / real silicon decodes as a different (or invalid) instruction.
// The bug surfaced as test-dp-loop FAILing with MAME "Unexpected
// exception" — the loop body's `eorr spill_b1, b` emitted bytes
// `10 36 a9` which round-trips through the disassembler as <unknown>.
// Returns true if the HD6309 page-3 reg-reg expansion was emitted.
// Returns false when Src1 and Src2 would land in the same hardware
// byte half of D — in that case Src2's load would clobber Src1's
// value, and the caller MUST fall back to the 6809 page-1 path
// (which reads RHS directly from the U-relative spill slot, so no
// second-half register is needed at all). The page-1 path is
// strictly bigger but safe — collision is an exceptional case.
static bool emitHD6309RegRegOp(MachineIRBuilder &Builder, MachineInstr &MI,
                                unsigned Opcode) {
  MachineFunction &MF = *MI.getMF();
  Register Dst = MI.getOperand(0).getReg();
  Register Src1 = MI.getOperand(1).getReg();
  Register Src2 = MI.getOperand(2).getReg();
  Register OrigDst = Dst;

  // Bug #161 round 18: detect Src1/Src2 same-half collision and bail.
  // Two flavours both produce a degenerate `SUBR B,B`-style postbyte:
  //
  // (a) Bug-#63 "skip second ACC spill" — Src1 has already been
  //     materialized to AB by MaterializeSpills and Src2 is still a
  //     SPILL_B* that this helper would materialize here, so the
  //     helper's LDB would overwrite the pre-MI LDB.
  //
  // (b) Regalloc collapsed both operands to the same physical AB/AA
  //     (e.g. coalesced into one byte-sized live range), so Src1Phys
  //     == Src2Phys with NO materialization on either side. The
  //     resulting `SUBR B,B` is `B = B - B = 0`, garbage data.
  //
  // We can't safely TFR or use the alt half because MaterializeSpills
  // may have set up the alt half with an unrelated value live across
  // this MI. Bail in both cases and let the 6809 page-1 fallback
  // (which reads RHS from the U-relative spill slot directly) handle
  // it. The fallback is bigger but correct.
  auto effectivePhys = [](Register R) -> Register {
    return needsMaterialization(R) ? getPhysRegFor(R) : R;
  };
  Register Src1Phys = effectivePhys(Src1);
  Register Src2Phys = effectivePhys(Src2);
  // Same-physreg collision applies to every register the page-3
  // reg-reg ops can name: byte halves AA/AB (8-bit ops), AD (16-bit
  // ops), IX/IY/SU/SS/PC and HD6309 W/V/E/F (16- or 8-bit). The
  // postbyte folds them into reg1==reg2 → degenerate `op X,X`.
  if (Src1Phys == Src2Phys &&
      (Src1Phys == MC6809::AA || Src1Phys == MC6809::AB ||
       Src1Phys == MC6809::AD || Src1Phys == MC6809::AW ||
       Src1Phys == MC6809::AE || Src1Phys == MC6809::AF ||
       Src1Phys == MC6809::IX || Src1Phys == MC6809::IY ||
       Src1Phys == MC6809::SU || Src1Phys == MC6809::SS))
    return false;

  // Preserve every distinct real register the staging below clobbers
  // (the pseudo only declares its imaginary/spill operands).
  SmallVector<Register, 3> Staged;
  for (Register R : {Src1, Src2, Dst})
    if (needsMaterialization(R) && !llvm::is_contained(Staged, getPhysRegFor(R)))
      Staged.push_back(getPhysRegFor(R));
  for (Register R : Staged)
    pushStagingReg(Builder, R);
  if (needsMaterialization(Src1)) Src1 = materializeReg(Builder, Src1, MF);
  if (needsMaterialization(Src2)) Src2 = materializeReg(Builder, Src2, MF);
  if (needsMaterialization(Dst))  Dst  = materializeReg(Builder, Dst,  MF);
  Builder.buildInstr(Opcode).addDef(Dst).addUse(Src2).addUse(Src1);
  if (needsMaterialization(OrigDst))
    dematerializeReg(Builder, Dst, OrigDst, MF);
  for (Register R : llvm::reverse(Staged))
    pullStagingReg(Builder, R);
  return true;
}

// Bug #357: report a TFRp as a copy so generic MachineCopyPropagation can
// forward/eliminate the pervasive register moves TFRp produces — but ONLY for
// the ATOMIC 16-bit registers IX/IY/SU/SS.
//
// The accumulator family (AA/AB/AD/AW/AE/AF) is deliberately EXCLUDED. Its
// byte ops carry a partial-subregister def: `ldb N,u` is `implicit-def $ad`
// (D's composite changed because its low byte B did, even though the high byte
// A is hardware-preserved — the Bug #184 / #362 shape). MachineCopyPropagation
// reads that as "$ad (hence AA) fully redefined", so it wrongly concludes a
// prior `tfr b,a` def of A is dead and deletes it — losing A's value that a
// later `std` reads as the high half of D. (This regressed test-getopt et al.
// when ACC was admitted.) The same partial-def idiom is why #362 excluded the
// accumulator from dead-load DCE. IX/IY/SU/SS are atomic (no allocatable
// sub-registers, no partial-def idiom), so copy-propagation on them is sound.
//
// Atomic 16-bit also subsumes the earlier guards for free: it admits only
// same-width transfers and never DP / CC / PC / spill / imaginary / mixed-size
// (HD6309 byte-replicate) TFR.
std::optional<DestSourcePair>
MC6809InstrInfo::isCopyInstrImpl(const MachineInstr &MI) const {
  if (MI.getOpcode() != MC6809::TFRp)
    return std::nullopt;
  // TFRp operand layout: op0 = DEST ($reg2), op1 = SRC ($reg1).
  Register Dst = MI.getOperand(0).getReg();
  Register Src = MI.getOperand(1).getReg();
  auto IsAtomic16 = [](Register R) {
    return MC6809::INDEX16RegClass.contains(R);
  };
  if (!IsAtomic16(Dst) || !IsAtomic16(Src))
    return std::nullopt;
  return DestSourcePair{MI.getOperand(0), MI.getOperand(1)};
}

 void  MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, Register DestReg, Register SrcReg, bool KillSrc, bool RenamableDest, bool RenamableSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  if (DestReg == SrcReg)
    return;

  // Handle copies involving spill pseudo-registers or imaginary registers.
  // Emit concrete S-indexed instructions (not pseudos with frame indices,
  // since PEI has already run by the time copyPhysReg is called).
  if (needsMaterialization(DestReg) || needsMaterialization(SrcReg)) {
    MachineFunction &MF = *MBB.getParent();
    if (needsMaterialization(DestReg) && !needsMaterialization(SrcReg)) {
      // Real → Spill/Imaginary: Store to spill slot or imaginary.
      if (isIndexSpillReg(DestReg)) {
        // INDEX spill: use IY as staging (callee-saved, avoids IX conflicts).
        Register StageReg = MC6809::IY;
        if (SrcReg != StageReg)
          Builder.buildInstr(MC6809::TFRp).addDef(StageReg).addUse(SrcReg);
        emitSpillStoreFrom(Builder, StageReg, DestReg, /*ExtraOffset=*/0, MF);
      } else if (isSpillReg(DestReg) && (SrcReg == MC6809::IX || SrcReg == MC6809::IY)) {
        // INDEX → ACC spill: use STX/STY directly (no D clobber).
        emitSpillStoreFrom(Builder, SrcReg, DestReg, /*ExtraOffset=*/0, MF);
      } else if (MC6809::Imag16RegClass.contains(DestReg) &&
                 (SrcReg == MC6809::IX || SrcReg == MC6809::IY ||
                  SrcReg == MC6809::SU || SrcReg == MC6809::SS)) {
        // INDEX/STACK → Imag16: store the source register directly to the
        // direct-page slot using STX/STY/STU/STS. Avoids the TFR-to-D + STD
        // sequence that would clobber AA/AB and silently corrupt anything
        // live there (e.g. an outgoing call argument being prepared by
        // CallLowering for an indirect call via __rs7).
        unsigned Opc;
        switch (SrcReg) {
        case MC6809::IX: Opc = MC6809::STXd; break;
        case MC6809::IY: Opc = MC6809::STYd; break;
        case MC6809::SU: Opc = MC6809::STUd; break;
        case MC6809::SS: Opc = MC6809::STSd; break;
        default: llvm_unreachable("unreachable");
        }
        Builder.buildInstr(Opc).addReg(DestReg, RegState::Undef).addUse(SrcReg, RegState::Implicit).addReg(DestReg, RegState::ImplicitDefine);
      } else {
        Register RealAcc = getPhysRegFor(DestReg);
        if (SrcReg != RealAcc) {
          // The TFR staging clobbers RealAcc, which may hold a live value
          // (the COPY only declares DestReg) -- preserve it.
          pushStagingReg(Builder, RealAcc);
          Builder.buildInstr(MC6809::TFRp).addDef(RealAcc).addUse(SrcReg);
          dematerializeReg(Builder, RealAcc, DestReg, MF);
          pullStagingReg(Builder, RealAcc);
        } else {
          dematerializeReg(Builder, RealAcc, DestReg, MF);
        }
      }
    } else if (!needsMaterialization(DestReg) && needsMaterialization(SrcReg)) {
      // Spill/Imaginary → Real: Load from spill slot or imaginary.
      if (isIndexSpillReg(SrcReg)) {
        // INDEX spill → Real: use IY as staging (callee-saved).
        Register StageReg = MC6809::IY;
        emitSpillLoadInto(Builder, StageReg, SrcReg, /*ExtraOffset=*/0, MF);
        if (DestReg != StageReg)
          Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(StageReg);
      } else if (isSpillReg(SrcReg) && (DestReg == MC6809::IX || DestReg == MC6809::IY)) {
        // ACC spill → INDEX: use LDX/LDY directly (no D clobber).
        emitSpillLoadInto(Builder, DestReg, SrcReg, /*ExtraOffset=*/0, MF);
      } else if (MC6809::Imag16RegClass.contains(SrcReg) &&
                 (DestReg == MC6809::IX || DestReg == MC6809::IY ||
                  DestReg == MC6809::SU || DestReg == MC6809::SS)) {
        // Imag16 → INDEX/STACK: load directly with LDX/LDY/LDU/LDS.
        // Avoids the LDD + TFR sequence that would clobber AA/AB.
        unsigned Opc;
        switch (DestReg) {
        case MC6809::IX: Opc = MC6809::LDXd; break;
        case MC6809::IY: Opc = MC6809::LDYd; break;
        case MC6809::SU: Opc = MC6809::LDUd; break;
        case MC6809::SS: Opc = MC6809::LDSd; break;
        default: llvm_unreachable("unreachable");
        }
        Builder.buildInstr(Opc).addDef(DestReg, RegState::Implicit).addReg(SrcReg);
      } else {
        Register RealAcc = getPhysRegFor(SrcReg);
        if (DestReg != RealAcc) {
          // Staging through RealAcc clobbers it while the COPY only
          // declares DestReg -- preserve it around the window.
          pushStagingReg(Builder, RealAcc);
          materializeReg(Builder, SrcReg, MF);
          Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(RealAcc);
          pullStagingReg(Builder, RealAcc);
        } else {
          materializeReg(Builder, SrcReg, MF);
        }
      }
    } else {
      // Spill/Imaginary → Spill/Imaginary: materialize src, dematerialize
      // dest. The staging register is pure scratch here -- preserve it.
      Register TmpReal = getPhysRegFor(SrcReg);
      pushStagingReg(Builder, TmpReal);
      materializeReg(Builder, SrcReg, MF);
      dematerializeReg(Builder, TmpReal, DestReg, MF);
      pullStagingReg(Builder, TmpReal);
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
    // Bug #161 round 18 follow-up #2: HD6309's TFR with size mismatch
    // (8-bit src → 16-bit dst) BYTE-REPLICATES the source into both
    // halves of the destination — it does NOT zero-extend. So the
    // bare TFRp emitted here was producing `tfr b,w` ⇒ W=B:B (e.g.
    // 0x2A2A for value 42) instead of W=0:B. Caught via the rc=1
    // `_s` test cluster: vfprintf's signed-i16→i32 sign-extend path
    // relied on this COPY to zero-extend each half, and the resulting
    // i32 had the LO byte duplicated where the HI byte should have
    // been — printing "Test 257 Passed" instead of "Test 1 Passed".
    //
    // Emit an explicit zero-extend: 8-bit transfer to the matching
    // half-byte of the 16-bit dest, then clear the unused half.
    if (DestReg == MC6809::AD) {
      // AD = AA(hi):AB(lo). Move src into AB, clear AA.
      if (SrcReg != MC6809::AB)
        Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(SrcReg);
      Builder.buildInstr(MC6809::CLRAa);
    } else if (DestReg == MC6809::AW) {
      // AW = AE(hi):AF(lo). Move src into AF, clear AE.
      if (SrcReg != MC6809::AF)
        Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AF).addUse(SrcReg);
      Builder.buildInstr(MC6809::CLREa);
    } else {
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
    }
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    if (AreClasses(MC6809::ABcRegClass, MC6809::ADcRegClass) || AreClasses(MC6809::AFcRegClass, MC6809::AWcRegClass))
      return;
    // Symmetric to the 8→16 case above: HD6309's TFR with 16-bit src
    // and 8-bit dst takes the LOW byte of the source (E or B), not
    // the HIGH byte. For copies that don't share a sub-reg (e.g.
    // AD → AF, AW → AB), emit an 8-bit transfer from the source's
    // low byte to the destination.
    Register SrcLo;
    if      (SrcReg == MC6809::AD) SrcLo = MC6809::AB;
    else if (SrcReg == MC6809::AW) SrcLo = MC6809::AF;
    else                            SrcLo = SrcReg;
    if (DestReg != SrcLo)
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcLo);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass) || AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass) || AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass) ||
             AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCFlagRegClass)) {
    // TODO: May need AND #0x0F to mask EFHI bits if callers expect only NZVC.
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::CCFlagRegClass, MC6809::ACC8RegClass)) {
    // TODO: May need AND #0x0F to mask EFHI bits if callers expect only NZVC.
    Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
  } else if (AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass) || AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)) {
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
    Builder.buildInstr(MC6809::STDd).addReg(DestReg, RegState::Undef).addReg(DestReg, RegState::ImplicitDefine);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → ACC16: load D from direct-page imaginary register.
    Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
    if (DestReg != MC6809::AD)
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::AD);
  } else if (AreClasses(MC6809::Imag8RegClass, MC6809::Imag8RegClass)) {
    // Imag8 → Imag8: load to AA, store to dest.
    Builder.buildInstr(MC6809::LDAd).addReg(SrcReg);
    Builder.buildInstr(MC6809::STAd).addReg(DestReg, RegState::Undef).addReg(DestReg, RegState::ImplicitDefine);
  } else if (AreClasses(MC6809::Imag16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → Imag16: load to D, store to dest.
    Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
    Builder.buildInstr(MC6809::STDd).addReg(DestReg, RegState::Undef).addReg(DestReg, RegState::ImplicitDefine);
  } else if (AreClasses(MC6809::Imag16RegClass, MC6809::INDEX16RegClass)) {
    // INDEX16 → Imag16: store the source register directly to the
    // direct-page slot. Each 16-bit hardware register has its own STxd
    // (direct-page) opcode, so we don't need to route through D — that
    // would clobber AA/AB and silently corrupt anything live there
    // (e.g. an outgoing call argument being prepared by CallLowering).
    unsigned Opc;
    switch (SrcReg) {
    case MC6809::IX: Opc = MC6809::STXd; break;
    case MC6809::IY: Opc = MC6809::STYd; break;
    case MC6809::SU: Opc = MC6809::STUd; break;
    case MC6809::SS: Opc = MC6809::STSd; break;
    default:
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AD).addUse(SrcReg);
      Builder.buildInstr(MC6809::STDd).addReg(DestReg, RegState::Undef).addReg(DestReg, RegState::ImplicitDefine);
      return;
    }
    Builder.buildInstr(Opc).addReg(DestReg, RegState::Undef).addUse(SrcReg, RegState::Implicit).addReg(DestReg, RegState::ImplicitDefine);
  } else if (AreClasses(MC6809::INDEX16RegClass, MC6809::Imag16RegClass)) {
    // Imag16 → INDEX16: load directly into the destination.
    unsigned Opc;
    switch (DestReg) {
    case MC6809::IX: Opc = MC6809::LDXd; break;
    case MC6809::IY: Opc = MC6809::LDYd; break;
    case MC6809::SU: Opc = MC6809::LDUd; break;
    case MC6809::SS: Opc = MC6809::LDSd; break;
    default:
      Builder.buildInstr(MC6809::LDDd).addReg(SrcReg);
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(MC6809::AD);
      return;
    }
    Builder.buildInstr(Opc).addDef(DestReg, RegState::Implicit).addReg(SrcReg);
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC32RegClass)) {
    // Bug #266 root cause (2026-05-10): ACC32 → ACC16 sub-register
    // extraction. AQ = AD:AW (sub_hi_word=AD, sub_lo_word=AW). The
    // hardware Q register physically aliases AD + AW — extracting
    // either half is a NO-OP because the bits already live in the
    // destination physreg. A COPY without an explicit sub-reg index
    // here is the regalloc's way of renaming the value; no instruction
    // emission needed.
    //
    // For other ACC16 destinations (RS imag regs, SPILL_D*), we route
    // through AD by first claiming AD as the "HI half" of AQ (no
    // instruction), then dematerialising AD to the actual destination
    // via the existing imaginary/spill machinery (TFR or STD as
    // appropriate).
    //
    // Discovered via MAME debugger session on test-ffs at
    // -Og hd6309 mame: the previous COPY_CC_PLACEHOLDER fallthrough
    // emitted SWI3 here, which trapped to an uninitialised vector at
    // $FFF2 and effectively turned the test into an infinite loop.
    if (DestReg == MC6809::AD || DestReg == MC6809::AW)
      return;
    // Other ACC16 destination (Imag16, SPILL_D*): route through AD then
    // store/transfer to the real destination via the existing AD-source
    // copyPhysReg path.
    if (SrcReg != MC6809::AQ)
      return; // unexpected ACC32 src
    // Recurse with src=AD to reuse the AD→ImagXX/spill paths above.
    copyPhysReg(MBB, MI, DL, DestReg, MC6809::AD, KillSrc, RenamableDest, RenamableSrc);
  } else {
    // Bug #186 v5 PLACEHOLDER (2026-04-27): no proper sequence yet for
    // this physreg pair. Drop a COPY_CC_PLACEHOLDER pseudo so the
    // backend stays compile-clean. AsmPrinter expands it to a comment +
    // SWI3 trap so runtime hits surface noisily and asm-grep finds the
    // offending sites. Once we know which pairs are actually exercised,
    // replace specific cases above (or retire this pseudo entirely).
    Builder.buildInstr(MC6809::COPY_CC_PLACEHOLDER)
        .addReg(DestReg, RegState::Define)
        .addReg(SrcReg);
  }
}

const TargetRegisterClass *MC6809InstrInfo::canFoldCopy(const MachineInstr &MI, const TargetInstrInfo &TII, unsigned FoldIdx) const {
  if (!MI.getMF()->getFunction().doesNotRecurse())
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MC6809::ACC8RegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, TII, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MC6809::ACC8RegClass)
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

  Register Reg = MO.getReg();
  unsigned Size = 0;
  if (Reg.isPhysical()) {
    // PHANTOM_CARRY is allocatable and may be spilled under pressure.
    // Per MC6809Reg1Class's RegInfo ("1-bit wide, but takes 8 bits to
    // spill"), spill width is 8 bits.
    if (MC6809::PHANTOM_CARRYRegClass.contains(Reg))
      Size = 8;
    else if (MC6809::CCFlagRegClass.contains(Reg) || MC6809::ACC8RegClass.contains(Reg))
      Size = 8;
    else if (MC6809::ACC16RegClass.contains(Reg) || MC6809::INDEX16RegClass.contains(Reg))
      Size = 16;
    else if (MC6809::ACC32RegClass.contains(Reg))
      Size = 32;
    else {
      llvm_unreachable("Unexpected physical register class");
    }
  } else {
    // Bug #307: virtual phantom_carry vregs hit this path during
    // pressure-driven spill.
    if (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::PHANTOM_CARRYRegClass))
      Size = 8;
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
      llvm_unreachable("Unexpected virtual register class");
    }
  }
  assert(Size != 0);

  // Emit directly through ACC or INDEX if possible.
  // INDEX16 uses Store/Load_iPtr_Mem (STX/LDX/STY/LDY) — no D clobber.
  // ACC16 uses Store/Load_i16_Mem (STD/LDD).
  bool IsIdx = (Reg.isPhysical() && MC6809::INDEX16RegClass.contains(Reg)) || (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::INDEX16RegClass));
  if ((Reg.isPhysical() && (MC6809::ACC8RegClass.contains(Reg) || MC6809::ACC16RegClass.contains(Reg) || MC6809::ACC32RegClass.contains(Reg) || MC6809::INDEX16RegClass.contains(Reg))) ||
      (Reg.isVirtual() && (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC8RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC32RegClass) || MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::INDEX16RegClass)))) {
    unsigned opcode;
    switch (Size) {
    default:
      llvm_unreachable("Unknown register size");
    case 1:
    case 8:
      opcode = MO.isDef() ? MC6809::Load_i8_Mem : MC6809::Store_i8_Mem;
      break;
    case 16:
      if (IsIdx)
        opcode = MO.isDef() ? MC6809::Load_iPtr_Mem : MC6809::Store_iPtr_Mem;
      else
        opcode = MO.isDef() ? MC6809::Load_i16_Mem : MC6809::Store_i16_Mem;
      break;
    case 32:
      // Bug #271 cat-3: use SpillLoad_i32_Mem / SpillStore_i32_Mem,
      // whose operand class is ACC32 (= AQ + SPILL_Q0..3). The plain
      // Load_i32_Mem / Store_i32_Mem are AQc-constrained for Bug #208
      // round 4 sub-reg-aliasing reasons in user codegen, but the
      // spill framework passes the original ACC32-class vreg directly
      // — a class mismatch against AQc that -verify-machineinstrs
      // would flag. The Spill variants share their post-RA expander,
      // so codegen is unchanged.
      opcode = MO.isDef() ? MC6809::SpillLoad_i32_Mem : MC6809::SpillStore_i32_Mem;
      break;
    }
    Builder.buildInstr(opcode).add(MO).addFrameIndex(FrameIndex, Offset).addImm(0).addMemOperand(MMO);
    return;
  }

  // Emit via copy through ACC.
  // Bug #311: the IsBit / sub_lsb branches that used to live here are
  // gone; ACC8 carries all values that need this fallback path.
  MachineOperand Tmp = MachineOperand::CreateReg(Builder.getMRI()->createVirtualRegister(&MC6809::ACC8RegClass), MO.isDef());
  if (Tmp.isUse()) {
    // Define the temporary register via copy from the MO.
    MachineOperand TmpDef = Tmp;
    TmpDef.setIsDef();
    Builder.buildInstr(MC6809::COPY).add(TmpDef).add(MO);

    loadStoreRegisterStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);
  } else {
    assert(Tmp.isDef());

    loadStoreRegisterStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);

    // Define the MO via copy from the temporary register.
    MachineOperand TmpUse = Tmp;
    TmpUse.setIsUse();
    Builder.buildInstr(MC6809::COPY).add(MO).add(TmpUse);
  }
}

void MC6809InstrInfo::loadStoreRegStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg, bool IsKill, int FrameIndex, const TargetRegisterClass *RC, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  // UsesSpillRegisters is detected by determineCalleeSaves (which scans all
  // instructions before PEI runs hasFP). SU is unconditionally reserved in
  // getReservedRegs, so we no longer need to coordinate flag-setting here.

  // Imaginary registers (RC0..RC255, RS0..RS127) are direct-page memory
  // locations — they don't use frame indices. Emit direct-page LDA/STA
  // (or LDD/STD for 16-bit) with the register as the address operand.
  // MCInstLower converts imaginary register operands to symbol references.
  if (Reg.isPhysical() &&
      (MC6809::Imag8RegClass.contains(Reg) || MC6809::Imag16RegClass.contains(Reg))) {
    MachineIRBuilder Builder(MBB, MI);
    if (MC6809::Imag16RegClass.contains(Reg)) {
      if (IsLoad) {
        Builder.buildInstr(MC6809::LDDd).addReg(Reg);
      } else {
        Builder.buildInstr(MC6809::STDd).addReg(Reg, RegState::Undef).addReg(Reg, RegState::ImplicitDefine);
      }
    } else {
      if (IsLoad) {
        Builder.buildInstr(MC6809::LDAd).addReg(Reg);
      } else {
        Builder.buildInstr(MC6809::STAd).addReg(Reg, RegState::Undef).addReg(Reg, RegState::ImplicitDefine);
      }
    }
    return;
  }

  MachinePointerInfo PtrInfo = MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore, MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  if ((Reg.isPhysical() && MC6809::INDEX16RegClass.contains(Reg)) ||
      (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::INDEX16RegClass))) {
    unsigned Opcode = IsLoad ? MC6809::Load_iPtr_Mem : MC6809::Store_iPtr_Mem;
    auto MIB = Builder.buildInstr(Opcode);
    MIB.addReg(Reg, getDefRegState(IsLoad) | getKillRegState(IsKill && !IsLoad));
    MIB.addFrameIndex(FrameIndex).addImm(0).addMemOperand(MMO);
  } else if ((Reg.isPhysical() && MC6809::ACC16RegClass.contains(Reg)) || (Reg.isVirtual() && MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))) {
    // Bug #271 cat-2: pass Reg directly to Store_i16_Mem / Load_i16_Mem.
    // The historical fresh-vreg + COPY pattern (whose comment claimed
    // "this code introduces subregisters") was leaving 96 verifier hits
    // at -Og hd6309 mame: the Store's operand was the fresh vreg but
    // greedy's spill bookkeeping recorded the live segment as belonging
    // to the original Reg, so the verifier flagged "Instruction ending
    // live segment doesn't read the register" on every 16-bit spill.
    // Reg's class is already a subclass of ACC16 — the operand class
    // of Store_i16_Mem / Load_i16_Mem — so going direct is also
    // verifier-clean. No sub-reg index is involved (the load/store
    // operates on the full 16-bit value).
    loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MF.getMachineMemOperand(MMO, 0, 2));
  } else {
    loadStoreRegisterStaticStackSlot(Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MMO);
  }

  LLVM_DEBUG({
    dbgs() << "Inserted stack slot load/store:\n";
    for (const auto &MI : make_range(MIS.begin(), MIS.getInitial()))
      dbgs() << MI;
  });
}

// The _Mem sibling of a two-source _Reg pseudo, for folding a spilled
// second-source reload directly into the operation's memory operand
// (`addb <slot>,u` instead of reload + register op). FoldIdx is the explicit
// operand index of the non-tied second source — the only operand that can
// become a memory operand (the tied dst/src accumulator cannot).
//
// This fold is what lets the stock inline spiller handle the single-register
// accumulator classes (ABc = {AB}, AAc = {AA}): a byte _Reg op has TWO
// simultaneously-live uses in a one-register class, which is unallocatable
// unless one of them can be read from its spill slot in place. The byte
// _Reg forms are HD6309-only (base 6809 selects the Push/Pull shape), so
// this is the HD6309 escape hatch the SPILL_* pseudo-registers used to
// provide via MaterializeSpills' second-spill-skip path.
// Diagnostic escape hatch for bisecting spill-fold-related miscompiles:
// -mc6809-disable-mem-fold reverts to plain reload+register-op spilling for
// the two-source _Reg pseudos (the EXTRACT byte fold is unaffected).
static cl::opt<bool> DisableMemFold(
    "mc6809-disable-mem-fold", cl::Hidden, cl::init(false),
    cl::desc("Disable folding spilled second sources into _Mem forms"));

// Finer-grained diagnostic knob: disable only the index-domain pointer
// compare folds while keeping the (allocability-critical) byte folds.
static cl::opt<bool> DisablePtrFold(
    "mc6809-disable-ptr-fold", cl::Hidden, cl::init(false),
    cl::desc("Disable folding spilled pointer-compare second sources"));

// Diagnostic knob for the CMPD->CMPX/CMPY compare substitution in
// expandCompareImm (the "keep a reloaded PHI value comparable" heuristic).
static cl::opt<bool> DisableCmpSubst(
    "mc6809-disable-cmp-subst", cl::Hidden, cl::init(false),
    cl::desc("Disable the CMPD->CMPX/CMPY reload-substitution heuristic"));

struct MC6809MemFoldInfo {
  unsigned MemOpc;
  unsigned FoldIdx;
};
static std::optional<MC6809MemFoldInfo> memFoldSibling(unsigned Opc) {
  switch (Opc) {
  // Byte arithmetic (tied dst/src at 0/1, second source at 2).
  case MC6809::Add_i8_Reg:
  case MC6809::Add_i8_RegA:
    return MC6809MemFoldInfo{MC6809::Add_i8_Mem, 2};
  case MC6809::Sub_i8_Reg:
  case MC6809::Sub_i8_RegA:
    return MC6809MemFoldInfo{MC6809::Sub_i8_Mem, 2};
  case MC6809::AddSetCarry_i8_Reg:
  case MC6809::AddSetCarry_i8_RegA:
    return MC6809MemFoldInfo{MC6809::AddSetCarry_i8_Mem, 2};
  case MC6809::SubSetCarry_i8_Reg:
  case MC6809::SubSetCarry_i8_RegA:
    return MC6809MemFoldInfo{MC6809::SubSetCarry_i8_Mem, 2};
  case MC6809::AddSetOverflow_i8_Reg:
  case MC6809::AddSetOverflow_i8_RegA:
    return MC6809MemFoldInfo{MC6809::AddSetOverflow_i8_Mem, 2};
  case MC6809::SubSetOverflow_i8_Reg:
  case MC6809::SubSetOverflow_i8_RegA:
    return MC6809MemFoldInfo{MC6809::SubSetOverflow_i8_Mem, 2};
  case MC6809::AddSetCarryUse_i8_Reg:
  case MC6809::AddSetCarryUse_i8_RegA:
    return MC6809MemFoldInfo{MC6809::AddSetCarryUse_i8_Mem, 2};
  case MC6809::SubSetCarryUse_i8_Reg:
  case MC6809::SubSetCarryUse_i8_RegA:
    return MC6809MemFoldInfo{MC6809::SubSetCarryUse_i8_Mem, 2};
  case MC6809::AddSetOverflowUse_i8_Reg:
  case MC6809::AddSetOverflowUse_i8_RegA:
    return MC6809MemFoldInfo{MC6809::AddSetOverflowUse_i8_Mem, 2};
  case MC6809::SubSetOverflowUse_i8_Reg:
  case MC6809::SubSetOverflowUse_i8_RegA:
    return MC6809MemFoldInfo{MC6809::SubSetOverflowUse_i8_Mem, 2};
  // Byte bitwise (same shape; ACC8-classed, so this is a code-quality fold,
  // not an allocability requirement).
  case MC6809::AND_i8_Reg:
    return MC6809MemFoldInfo{MC6809::AND_i8_Mem, 2};
  case MC6809::OR_i8_Reg:
    return MC6809MemFoldInfo{MC6809::OR_i8_Mem, 2};
  case MC6809::XOR_i8_Reg:
    return MC6809MemFoldInfo{MC6809::XOR_i8_Mem, 2};
  // i16 arithmetic/carry/bitwise (ACC16-classed — multi-register, so like
  // the byte bitwise fold this is code quality: a spilled second source
  // reads straight from its slot, e.g. ADDD off,u, instead of reloading).
  case MC6809::Add_i16_Reg:
    return MC6809MemFoldInfo{MC6809::Add_i16_Mem, 2};
  case MC6809::Sub_i16_Reg:
    return MC6809MemFoldInfo{MC6809::Sub_i16_Mem, 2};
  case MC6809::AddSetCarry_i16_Reg:
    return MC6809MemFoldInfo{MC6809::AddSetCarry_i16_Mem, 2};
  case MC6809::SubSetCarry_i16_Reg:
    return MC6809MemFoldInfo{MC6809::SubSetCarry_i16_Mem, 2};
  case MC6809::AddSetCarryUse_i16_Reg:
    return MC6809MemFoldInfo{MC6809::AddSetCarryUse_i16_Mem, 2};
  case MC6809::SubSetCarryUse_i16_Reg:
    return MC6809MemFoldInfo{MC6809::SubSetCarryUse_i16_Mem, 2};
  case MC6809::AddSetOverflow_i16_Reg:
    return MC6809MemFoldInfo{MC6809::AddSetOverflow_i16_Mem, 2};
  case MC6809::SubSetOverflow_i16_Reg:
    return MC6809MemFoldInfo{MC6809::SubSetOverflow_i16_Mem, 2};
  case MC6809::AddSetOverflowUse_i16_Reg:
    return MC6809MemFoldInfo{MC6809::AddSetOverflowUse_i16_Mem, 2};
  case MC6809::SubSetOverflowUse_i16_Reg:
    return MC6809MemFoldInfo{MC6809::SubSetOverflowUse_i16_Mem, 2};
  case MC6809::AND_i16_Reg:
    return MC6809MemFoldInfo{MC6809::AND_i16_Mem, 2};
  case MC6809::OR_i16_Reg:
    return MC6809MemFoldInfo{MC6809::OR_i16_Mem, 2};
  case MC6809::XOR_i16_Reg:
    return MC6809MemFoldInfo{MC6809::XOR_i16_Mem, 2};
  // i16 compares (same operand layout as the byte compares).
  case MC6809::Compare_i16_Reg:
    return MC6809MemFoldInfo{MC6809::Compare_i16_Mem, 3};
  case MC6809::CompareBranch_i16_Reg:
    return MC6809MemFoldInfo{MC6809::CompareBranch_i16_Mem, 2};
  // Byte compares. Compare: (outs CCond)(ins cc, src, src2) — src2 at 3.
  // CompareBranch: (outs)(ins cc, src, src2, label) — src2 at 2, and the
  // label operand is copied after the folded memory pair.
  case MC6809::Compare_i8_Reg:
    return MC6809MemFoldInfo{MC6809::Compare_i8_Mem, 3};
  case MC6809::CompareBranch_i8_Reg:
    return MC6809MemFoldInfo{MC6809::CompareBranch_i8_Mem, 2};
  // Index-domain pointer compares: with the SPILL_X escape registers gone,
  // INDEX16 is effectively {IX, IY}; a spilled second pointer of a reg-reg
  // compare reads straight from its slot (CMPX/CMPY off,r) instead of
  // reloading through the other index register.
  case MC6809::Compare_ptr_Reg:
    if (DisablePtrFold)
      return std::nullopt;
    return MC6809MemFoldInfo{MC6809::Compare_ptr_Mem, 3};
  case MC6809::CompareBranch_ptr_Reg:
    if (DisablePtrFold)
      return std::nullopt;
    return MC6809MemFoldInfo{MC6809::CompareBranch_ptr_Mem, 2};
  default:
    return std::nullopt;
  }
}

MachineInstr *MC6809InstrInfo::foldMemoryOperandImpl(
    MachineFunction &MF, MachineInstr &MI, ArrayRef<unsigned> Ops,
    MachineBasicBlock::iterator InsertPt, int FrameIndex,
    MachineInstr *& /*CopyMI*/, LiveIntervals * /*LIS*/,
    VirtRegMap *VRM) const {
  // Bug #118 Layer 1, approach (b): fold a reload of the 16-bit source of
  // EXTRACT_LO_i16 / EXTRACT_HI_i16 into a direct one-byte frame load.
  //
  // Without this fold, when regalloc spills the 16-bit source vreg to a
  // frame slot, the spiller inserts `LDD FrameIndex,u` (defs AA+AB) right
  // before the pseudo's expansion. That clobbers the AB output of any
  // *other* EXTRACT_LO_i16 that happens to share the destination byte
  // register — which is exactly what the earlier ABc/AAc out-class
  // constraint fix permits (both extracts legally landing in AB). The
  // miscompile was observed in test-strchr at -Oz, where `ptr - haystack`
  // compiled as `ptr - ptr` because haystack's low byte was destroyed
  // between its extract and the subtraction.
  //
  // A narrow LDA/LDB of just the wanted byte eliminates the AD clobber on
  // the spill-to-frame path. DP-backed SPILL_D / RS paths are unaffected
  // and remain handled by post-RA materialisation.
  unsigned Opc = MI.getOpcode();

  // A FAKE_USE exists only to extend values' visible lifetimes for debug
  // quality (-Og). A spilled operand needs no reload -- the value is
  // observable in its stack slot -- so drop the operand instead of
  // forcing an INF-weight reload interval (a FAKE_USE can carry more
  // simultaneously-live byte operands than the two-register byte class
  // can hold, which deadlocked greedy once the byte spill
  // pseudo-registers were retired).
  if (Opc == TargetOpcode::FAKE_USE || Opc == MC6809::FakeUse_Mem) {
    // FakeUse_Mem (not FAKE_USE itself) as the result: the generic
    // foldMemoryOperand wrapper attaches a load memoperand and asserts
    // mayLoad on whatever we return.
    MachineBasicBlock &MBB = *MI.getParent();
    auto MIB =
        BuildMI(MBB, InsertPt, MI.getDebugLoc(), get(MC6809::FakeUse_Mem));
    for (unsigned I = 0, E = MI.getNumOperands(); I != E; ++I)
      if (!llvm::is_contained(Ops, I))
        MIB.add(MI.getOperand(I));
    return MIB;
  }

  if (Opc == MC6809::EXTRACT_LO_i16 || Opc == MC6809::EXTRACT_HI_i16) {
    // Only fold the source operand (index 1). Folding a spilled def would
    // need a separate extract-into-scratch-then-store, which is not a
    // single memory-operand fold; let the spiller handle that path.
    if (Ops.size() != 1 || Ops[0] != 1)
      return nullptr;

    // MC6809 is big-endian within the D pair: AA is the high byte (offset 0)
    // and AB is the low byte (offset +1) of a 2-byte frame slot.
    int ByteOffset = (Opc == MC6809::EXTRACT_LO_i16) ? 1 : 0;

    MachineBasicBlock &MBB = *MI.getParent();
    const MachineOperand &Dst = MI.getOperand(0);
    MachineInstr *NewMI = BuildMI(MBB, InsertPt, MI.getDebugLoc(),
                                  get(MC6809::Load_i8_Mem))
                              .add(Dst)
                              .addFrameIndex(FrameIndex, ByteOffset)
                              .addImm(0);
    return NewMI;
  }

  // Fold a spilled byte half of MERGE_LOHI_i16 into its _MemLo/_MemHi
  // sibling: the expansion reads that half straight from the slot, so the
  // spiller needs no register for the reload. This was the last remaining
  // greedy deadlock once the byte spill pseudo-registers were retired --
  // the reload's INF-weight single-byte interval could find both AA and
  // AB blocked by physical interference at an fp-heavy merge point.
  if (Opc == MC6809::MERGE_LOHI_i16 && Ops.size() == 1 &&
      (Ops[0] == 1 || Ops[0] == 2)) {
    bool FoldLo = (Ops[0] == 1);
    const MachineOperand &Kept = MI.getOperand(FoldLo ? 2 : 1);
    MachineBasicBlock &MBB = *MI.getParent();
    MachineInstr *NewMI =
        BuildMI(MBB, InsertPt, MI.getDebugLoc(),
                get(FoldLo ? MC6809::MERGE_LOHI_i16_MemLo
                           : MC6809::MERGE_LOHI_i16_MemHi))
            .add(MI.getOperand(0))
            .add(Kept)
            .addFrameIndex(FrameIndex, 0)
            .addImm(0);
    return NewMI;
  }
  // The remaining register half of an already half-folded merge also
  // spilled: fold it too (both halves then read straight from their
  // slots).
  if ((Opc == MC6809::MERGE_LOHI_i16_MemLo ||
       Opc == MC6809::MERGE_LOHI_i16_MemHi) &&
      Ops.size() == 1 && Ops[0] == 1) {
    bool HadMemLo = (Opc == MC6809::MERGE_LOHI_i16_MemLo);
    MachineBasicBlock &MBB = *MI.getParent();
    auto MIB = BuildMI(MBB, InsertPt, MI.getDebugLoc(),
                       get(MC6809::MERGE_LOHI_i16_Mem2))
                   .add(MI.getOperand(0));
    if (HadMemLo) {
      // Existing mem operand is the LO slot; the folded reg was HI.
      MIB.add(MI.getOperand(2)).add(MI.getOperand(3));
      MIB.addFrameIndex(FrameIndex, 0).addImm(0);
    } else {
      // Folded reg was LO; existing mem operand is the HI slot.
      MIB.addFrameIndex(FrameIndex, 0).addImm(0);
      MIB.add(MI.getOperand(2)).add(MI.getOperand(3));
    }
    return MIB;
  }

  // Fold a spilled second source of a two-source _Reg pseudo into its _Mem
  // sibling (see memFoldSibling above). The rebuilt instruction reads that
  // operand from the frame slot in place; eliminateFrameIndex later supplies
  // the U/S base + offset (or leaves the slot dynamic in a static-stack
  // function — the conservative whole-frame marking skips slots reached by
  // pseudos without a _Sym lowering, so this fold needs no static-stack
  // special case).
  if (DisableMemFold)
    return nullptr;
  auto Fold = memFoldSibling(Opc);
  if (!Fold)
    return nullptr;
  if (Ops.size() != 1 || Ops[0] != Fold->FoldIdx)
    return nullptr;

  const MCInstrDesc &MemDesc = get(Fold->MemOpc);
  MachineRegisterInfo &MRI = MF.getRegInfo();
  unsigned NumExplicit = MI.getNumExplicitOperands();

  // The kept register operands must satisfy the _Mem desc's operand classes
  // (e.g. the bitwise _Reg accumulator is ACC8 but the _Mem form is
  // ACC8_AB_SP — no E/F indexed encoding). Constrain each; if any vreg
  // cannot be constrained, the fold is not possible.
  unsigned NewOpIdx = 0;
  for (unsigned I = 0; I < NumExplicit; ++I) {
    if (I == Fold->FoldIdx) {
      NewOpIdx += 2; // frame index + offset immediate
      continue;
    }
    const MachineOperand &MO = MI.getOperand(I);
    if (MO.isReg() && MO.getReg().isVirtual() &&
        NewOpIdx < MemDesc.getNumOperands()) {
      const TargetRegisterClass *RC = getRegClass(MemDesc, NewOpIdx);
      if (RC) {
        // A vreg an earlier regalloc round already assigned cannot be
        // re-constrained: the assignment is not revisited, so a class
        // shrink would let an out-of-class register (e.g. AE/AF on the
        // indexed byte forms, which have no E/F encoding) survive to the
        // verifier. Accept the fold only if the existing assignment
        // already satisfies the _Mem class.
        if (VRM && VRM->hasPhys(MO.getReg())) {
          if (!RC->contains(VRM->getPhys(MO.getReg())))
            return nullptr;
          // The existing assignment satisfies the _Mem class; also narrow
          // the vreg's class so the verifier's per-operand class check on
          // the rebuilt instruction passes (a narrower class still
          // satisfies every wider requirement elsewhere).
          if (!MRI.constrainRegClass(MO.getReg(), RC))
            return nullptr;
        } else if (!MRI.constrainRegClass(MO.getReg(), RC)) {
          return nullptr;
        }
      }
    }
    ++NewOpIdx;
  }

  MachineBasicBlock &MBB = *MI.getParent();
  MachineInstrBuilder MIB =
      BuildMI(MBB, InsertPt, MI.getDebugLoc(), MemDesc);
  for (unsigned I = 0; I < NumExplicit; ++I) {
    if (I == Fold->FoldIdx) {
      MIB.addFrameIndex(FrameIndex, 0);
      MIB.addImm(0);
      continue;
    }
    MIB.add(MI.getOperand(I));
  }
  // Carry over selector-appended virtual implicit operands (the phantom
  // carry-out def on the SetCarry/SetOverflow family). Physical implicit
  // operands (NZ/V/C defs) are re-added from the _Mem desc by BuildMI.
  for (unsigned I = NumExplicit, E = MI.getNumOperands(); I != E; ++I) {
    const MachineOperand &MO = MI.getOperand(I);
    if (MO.isReg() && MO.getReg().isVirtual())
      MIB.add(MO);
  }
  return MIB;
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
    // Emit short by default; standard LLVM BranchRelaxation widens to
    // LBRA via insertIndirectBranch when the displacement is out of
    // int8 range (bug #174).
    MI.setDesc(Builder.getTII().get(MC6809::BRAb));
    break;
  case MC6809::LongBranchRelative:
    // Explicit long pseudo — escape hatch for callers that want to
    // force a long branch regardless of distance.
    MI.setDesc(Builder.getTII().get(MC6809::LBRAlb));
    break;
  case MC6809::ConditionalBranchRelative: {
    // Emit short by default. Strip the pseudo's CCond:$bits operand —
    // the assembler encoding has no register field — and replace it
    // with the implicit Uses on N/Z/V(/C) declared by the chosen
    // Bbc / Bbc_NoC / Bbc_OnlyC variant (bug #137, bug #206,
    // bug #271 cat-1). Without these implicit uses the post-RA
    // MachineInstr would lack any CC dependency at all and the
    // scheduler would freely insert flag-clobbering instructions
    // between the cmp and this branch. BranchRelaxation widens to
    // LBlbc via CFG-split + insertBranch + insertIndirectBranch when
    // the displacement is out of int8 range (bug #174).
    int64_t CC = MI.getOperand(0).getImm();
    MI.setDesc(Builder.getTII().get(pickBbcVariant(CC)));
    MI.removeOperand(2);
    if (MC6809CC::doesOnlyReadCarry(CC)) {
      // OnlyC variant: just $c.
      MI.addOperand(MachineOperand::CreateReg(MC6809::C, /*isDef=*/false, /*isImp=*/true));
    } else {
      // Bbc and Bbc_NoC both read N/Z/V; Bbc additionally reads C.
      MI.addOperand(MachineOperand::CreateReg(MC6809::N, /*isDef=*/false, /*isImp=*/true));
      MI.addOperand(MachineOperand::CreateReg(MC6809::Z, /*isDef=*/false, /*isImp=*/true));
      MI.addOperand(MachineOperand::CreateReg(MC6809::V, /*isDef=*/false, /*isImp=*/true));
      if (!MC6809CC::doesNotReadCarry(CC))
        MI.addOperand(MachineOperand::CreateReg(MC6809::C, /*isDef=*/false, /*isImp=*/true));
    }
    break;
  }
  case MC6809::ConditionalLongBranchRelative: {
    // Explicit long conditional pseudo — escape hatch. Same operand
    // shape transformation as the short form above (bug #206 picker
    // + bug #271 cat-1 OnlyC variant).
    int64_t CC = MI.getOperand(0).getImm();
    MI.setDesc(Builder.getTII().get(pickLBlbcVariant(CC)));
    MI.removeOperand(2);
    if (MC6809CC::doesOnlyReadCarry(CC)) {
      MI.addOperand(MachineOperand::CreateReg(MC6809::C, /*isDef=*/false, /*isImp=*/true));
    } else {
      MI.addOperand(MachineOperand::CreateReg(MC6809::N, /*isDef=*/false, /*isImp=*/true));
      MI.addOperand(MachineOperand::CreateReg(MC6809::Z, /*isDef=*/false, /*isImp=*/true));
      MI.addOperand(MachineOperand::CreateReg(MC6809::V, /*isDef=*/false, /*isImp=*/true));
      if (!MC6809CC::doesNotReadCarry(CC))
        MI.addOperand(MachineOperand::CreateReg(MC6809::C, /*isDef=*/false, /*isImp=*/true));
    }
    break;
  }
  case MC6809::ReturnImplicit:
    MI.setDesc(Builder.getTII().get(MC6809::RTSr));
    MI.removeOperand(0);
    break;
  case MC6809::ReturnIRQImplicit:
    MI.setDesc(Builder.getTII().get(MC6809::RTIr));
    MI.removeOperand(0);
    break;
  case MC6809::EXTRACT_LO_i16:
  case MC6809::EXTRACT_HI_i16: {
    // Byte-extract pseudo (bug #118 Layer 1). After regalloc, resolve the
    // 16-bit source to its physical register, pick the relevant byte
    // sub-physreg (AB/AA for AD, SPILL_B*/SPILL_A* for SPILL_D*, RS*LO/HI
    // for RS*), and route that byte to the 8-bit destination via
    // copyPhysReg. All routing — materialise, dematerialise, TFR cross-half
    // — is already handled inside copyPhysReg.
    MachineFunction &MF = *MI.getMF();
    const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    unsigned SubIdx = (MI.getOpcode() == MC6809::EXTRACT_LO_i16)
                        ? MC6809::sub_lo_byte
                        : MC6809::sub_hi_byte;
    Register SrcByte = TRI->getSubReg(SrcReg, SubIdx);
    // Bug #118 Layer 1 (approach b): MaterializeSpills may have pre-routed
    // the src to its byte staging register (e.g. rewrote SPILL_D0 src → $ab
    // for EXTRACT_LO, loading just the relevant byte from the spill slot to
    // avoid the LDD clobber). In that case SrcReg is already the 8-bit
    // staging byte and getSubReg returns 0. Fall back to SrcReg itself.
    if (!SrcByte)
      SrcByte = SrcReg;
    // Bug #301 Phase C Path C (2026-05-16): SrcByte may be a SPILL_Q byte
    // sub-register (SPILL_QnHIHI/HILO/LOHI/LOLO).  copyPhysReg can't route
    // a stack-slot byte to an accumulator byte — emit LDB at slot+offset
    // directly here.  needsMaterialization/dematerialize on DstReg are
    // handled the same way as the COPY-from-SPILL_QnHI/LO case below.
    MCPhysReg QParent = 0;
    unsigned ByteOffset = 0;
    if (isQSpillByteReg(SrcByte, QParent, ByteOffset)) {
      Register OrigDst = DstReg;
      // Pure def: stage via the real register, preserved (see Extract16).
      bool StageDst = needsMaterialization(DstReg);
      Register RealDst = StageDst ? getPhysRegFor(DstReg) : DstReg;
      if (StageDst)
        pushStagingReg(Builder, RealDst);
      emitSpillLoadInto(Builder, RealDst, QParent, ByteOffset, MF);
      if (StageDst) {
        dematerializeReg(Builder, RealDst, OrigDst, MF);
        pullStagingReg(Builder, RealDst);
      }
      MI.eraseFromParent();
      return true;
    }
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                DstReg, SrcByte, /*KillSrc=*/false);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::EXTRACT_LO_word_i32:
  case MC6809::EXTRACT_HI_word_i32:
  case MC6809::Extract16_i32_lo:
  case MC6809::Extract16_i32_hi: {
    // Bug #161 round 14: extract a 16-bit half from an ACC32 source.
    // - AQ source: AQ = D:W (D high, W low). sub_lo_word = AW; sub_hi_word
    //   = AD. Use copyPhysReg AD ← AW (TFR W,D) for LO; for HI it's
    //   AD ← AD (no-op) so just elide.
    // - SPILL_Q source: 4-byte stack slot at U+offset, big-endian:
    //   bytes [0..1] = D (hi word), bytes [2..3] = W (lo word). LO
    //   needs LDD slot+2,U; HI needs LDD slot+0,U.
    //
    // Bug #302 redesign Phase 1 (2026-05-17): Extract16_i32_lo/hi
    // are the redesign replacements for EXTRACT_LO/HI_word_i32 —
    // identical post-RA semantics for the moment.  Phase 3 will
    // rewrite this case to use literal AD/AW physregs without
    // depending on the AQ sub-register hierarchy.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    unsigned Opcode = MI.getOpcode();
    bool IsLo = (Opcode == MC6809::EXTRACT_LO_word_i32 ||
                 Opcode == MC6809::Extract16_i32_lo);
    Register OrigDst = DstReg;
    // Pure def: never materialize-load the old destination value (it may
    // be undefined). Stage via the corresponding real register, preserved
    // around the window.
    bool StageDst = needsMaterialization(DstReg);
    Register RealDst = StageDst ? getPhysRegFor(DstReg) : DstReg;
    if (StageDst)
      pushStagingReg(Builder, RealDst);
    if (isQSpillReg(SrcReg)) {
      // Read the right 16-bit half directly from the stack slot.
      // Big-endian: D at +0, W at +2.
      emitSpillLoadInto(Builder, RealDst, SrcReg, /*ExtraOffset=*/IsLo ? 2 : 0, MF);
    } else {
      // SrcReg is AQ. The relevant half lives in AW (LO) or AD (HI).
      Register SrcWord = IsLo ? Register(MC6809::AW) : Register(MC6809::AD);
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  RealDst, SrcWord, /*KillSrc=*/false);
    }
    if (StageDst) {
      dematerializeReg(Builder, RealDst, OrigDst, MF);
      pullStagingReg(Builder, RealDst);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::Build32_i16i16: {
    // Bug #302 redesign Phase 1 (2026-05-17): assemble a 32-bit
    // value in ACC32 from two 16-bit halves.  Opaque ACC32
    // destination — the result vreg has no sub-word constraints in
    // its def chain (the structural fix for the Bug #302
    // REG_SEQUENCE-via-sub_lo_word/sub_hi_word destination-assembly
    // intersection-class leak).
    //
    // Operand layout: outs ACC32:$dst; ins ADc:$lo, ADc:$hi.
    //
    // Post-RA expansion:
    // - AQ destination: AQ = D:W (D high, W low).  Marshal hi → AD
    //   and lo → AW via copyPhysReg.  When the inputs are already
    //   in the right physregs the copyPhysReg becomes a no-op (or
    //   TFR-elision).  When inputs collide (e.g. lo allocated to
    //   AD), stage through a scratch — copyPhysReg handles that.
    // - SPILL_Q*N destination: write lo and hi to the right 16-bit
    //   slots in the 4-byte stack slot.  Big-endian: D at +0,
    //   W at +2.  STD $lo, slot+2,$su ; STD $hi, slot+0,$su.
    //   Inputs need to be moved into AD before STD if they're not
    //   already there (only AD has a memory-store form via STD).
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register LoReg = MI.getOperand(1).getReg();
    Register HiReg = MI.getOperand(2).getReg();

    if (isQSpillReg(DstReg)) {
      // Stack-slot destination: STD each half.  STD only writes from
      // AD, so move each input through AD first if not already there.
      //
      // Bug #301 (2026-05-22) follow-up to Bug #311: collision-aware
      // ordering.  Mirrors Bug #311's AQ-dst case but for the SPILL_Q
      // destination.  Key insight: STORE first while $ad still holds
      // the correct value, BEFORE any LDD-from-spill clobbers it.
      //
      // Four cases (LoIsSpill, HiIsSpill):
      //  1. Both SPILL_D: LDD lo→AD; STD slot+2; LDD hi→AD; STD slot+0.
      //  2. Only HI is SPILL_D, LoReg = $ad: STD slot+2 FIRST (while $ad
      //     has LO); LDD hi→AD; STD slot+0.  This is the failing case
      //     pre-fix — the old code did `LDD hi→AD` first (assuming "lo
      //     already in $ad, just store"), but the lo→AD copy at the start
      //     was a no-op so the next LDD clobbered the LO value before
      //     it had been stored.
      //  3. Only LO is SPILL_D, HiReg = $ad: STD slot+0 FIRST (while $ad
      //     has HI); LDD lo→AD; STD slot+2.
      //  4. Neither spilled: existing sequential copyPhysReg + STD.
      //
      // No cross-conflict case (Bug #311 case 1) — LoReg/HiReg are ADc
      // here, no $aw involved.
      int Offset = computeSpillStackOffset(DstReg, MF);
      bool Fits8 = ((Offset + 2) >= -128 && (Offset + 2) <= 127);
      unsigned Opc = Fits8 ? MC6809::STDi_o8 : MC6809::STDi_o16;

      bool LoIsSpill = isSpillReg(LoReg) && !isQSpillReg(LoReg);
      bool HiIsSpill = isSpillReg(HiReg) && !isQSpillReg(HiReg);

      auto LoadSpillIntoAD = [&](Register SpillReg) {
        emitSpillLoadInto(Builder, MC6809::AD, SpillReg, /*ExtraOffset=*/0, MF);
      };

      auto StoreToSlot = [&](int SlotOff, bool IsLast) {
        // Bug #305 part 2 cluster A (2026-05-18): the source MI's
        // explicit OutOperand ($spill_q*) is the only visible def of
        // that imaginary register.  Attach implicit-def $spill_q* to
        // the LAST STD so downstream FAKE_USE $spill_q* (from
        // -fextend-lifetimes at -Og) doesn't read an undefined reg.
        // Keep the dynamic path's single uniform Opc (chosen from Offset+2 for
        // both slot writes); a static-stack dest uses the extended store, with
        // the sub-slot offset (SlotOff - Offset) added to the static base.
        MachineInstrBuilder MIB;
        if (isStaticSpillSlot(DstReg, MF)) {
          unsigned SymOpc = getSymStoreOpcode(
              MC6809::AD, /*IsDP=*/false, MF.getTarget().isPositionIndependent());
          MIB = Builder.buildInstr(SymOpc)
                    .addUse(MC6809::AD, RegState::Implicit)
                    .addTargetIndex(MC6809::TI_STATIC_STACK,
                                    staticSpillOffset(DstReg, MF) + (SlotOff - Offset));
        } else {
          MIB = Builder.buildInstr(Opc)
                    .addUse(MC6809::AD, RegState::Implicit)
                    .addImm(SlotOff).addReg(MC6809::SU);
        }
        if (IsLast)
          MIB.addReg(DstReg, RegState::ImplicitDefine);
      };

      if (LoIsSpill && HiIsSpill) {
        // Case 1: both spilled.
        LoadSpillIntoAD(LoReg);
        StoreToSlot(Offset + 2, /*IsLast=*/false);
        LoadSpillIntoAD(HiReg);
        StoreToSlot(Offset, /*IsLast=*/true);
      } else if (HiIsSpill && LoReg == MC6809::AD) {
        // Case 2: HI spilled, LO already in $ad.  STORE LO FIRST.
        StoreToSlot(Offset + 2, /*IsLast=*/false);
        LoadSpillIntoAD(HiReg);
        StoreToSlot(Offset, /*IsLast=*/true);
      } else if (LoIsSpill && HiReg == MC6809::AD) {
        // Case 3: LO spilled, HI already in $ad.  STORE HI FIRST.
        StoreToSlot(Offset, /*IsLast=*/false);
        LoadSpillIntoAD(LoReg);
        StoreToSlot(Offset + 2, /*IsLast=*/true);
      } else if (HiReg == MC6809::AD && LoReg != MC6809::AD) {
        // Physreg collision (ACC16-widened inputs): HI already sits in
        // $ad, so the lo→$ad copy of the sequential order would destroy
        // it. Store HI first, then stage LO.
        StoreToSlot(Offset, /*IsLast=*/false);
        copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                    MC6809::AD, LoReg, /*KillSrc=*/false);
        StoreToSlot(Offset + 2, /*IsLast=*/true);
      } else {
        // Case 4: neither spilled (or one is in some non-$ad physreg).
        // Existing sequential behavior: move LO→$ad, STD slot+2,
        // move HI→$ad, STD slot+0.
        if (LoReg != MC6809::AD) {
          copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                      MC6809::AD, LoReg, /*KillSrc=*/false);
        }
        StoreToSlot(Offset + 2, /*IsLast=*/false);
        if (HiReg != MC6809::AD) {
          copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                      MC6809::AD, HiReg, /*KillSrc=*/false);
        }
        StoreToSlot(Offset, /*IsLast=*/true);
      }
    } else {
      // AQ destination: marshal hi into AD, lo into AW.
      //
      // Bug #302 redesign Phase 2 fixup (2026-05-17): the Build32 MI
      // declares an explicit `$aq` def (its OutOperandList is ACC32).
      // When we erase the MI, the def goes with it — and downstream
      // sub-register accesses ($ab, $aa, etc.) see $aq as undefined
      // even though our copyPhysReg stages the data into $ad and $aw
      // correctly.  Hit on labs / fseek / fseeko / imaxdiv / lldiv /
      // arc4random / vfprintf_s / hash_func at -Og-hd6309-mame in the
      // Phase 2 attempt (`Using an undefined physical register: $ab`
      // on a downstream Store_i8_Mem).
      //
      // Fix: after staging, attach `implicit-def $aq` to the last
      // emitted instruction so the verifier sees a continuous chain
      // of $aq defs.  If we emitted nothing (both inputs already in
      // place), insert an explicit COPY $aq, $aq as the marker —
      // CopyPropagation eliminates it but the verifier accepts it.
      //
      // Cross-conflict (hi=$aw, lo=$ad) isn't handled here yet — if
      // regalloc picks that shape, the second copyPhysReg's TFR W,D
      // would clobber $ad after the first copy, producing wrong
      // data.  Phase 3 rewrites this expansion to be sub-reg-
      // independent and the collision-handling will be revisited
      // there.
      // Bug #311 (2026-05-22): collision-aware expansion.  Five cases:
      //
      //   1. Cross-conflict (hi=$aw, lo=$ad): EXG D,W in one cycle.
      //   2. Both SPILL_D: load lo→AD; TFR D,W; load hi→AD.
      //   3. Only hi is SPILL_D: move lo→AW (preserve from LDD-
      //      clobber); load hi→AD.  Common case after MaterializeSpills'
      //      PhysCollision detection skips the $hi=$spill_d0 case
      //      where $lo=$ad.
      //   4. Only lo is SPILL_D AND hi is already in $ad: use LDW
      //      (HD6309) to load lo into AW directly without touching AD.
      //   5. Neither spilled (and not cross-conflict): existing
      //      sequential copyPhysReg.
      MachineBasicBlock &MBB = *MI.getParent();
      MachineBasicBlock::iterator InsertBefore = MI;
      MachineBasicBlock::iterator BeforeFirstCopy = MBB.end();
      if (InsertBefore != MBB.begin())
        BeforeFirstCopy = std::prev(InsertBefore);

      bool LoIsSpill = isSpillReg(LoReg) && !isQSpillReg(LoReg);
      bool HiIsSpill = isSpillReg(HiReg) && !isQSpillReg(HiReg);

      auto LoadSpillIntoAD = [&](Register SpillReg) {
        emitSpillLoadInto(Builder, MC6809::AD, SpillReg, /*ExtraOffset=*/0, MF);
      };
      auto LoadSpillIntoAW = [&](Register SpillReg) {
        // HD6309-only: LDW writes AW directly without touching AD.
        emitSpillLoadInto(Builder, MC6809::AW, SpillReg, /*ExtraOffset=*/0, MF);
      };

      if (HiReg == MC6809::AW && LoReg == MC6809::AD) {
        // Case 1: cross-conflict — EXG D,W swaps in one cycle.
        Builder.buildInstr(MC6809::EXGp)
            .addDef(MC6809::AD).addDef(MC6809::AW)
            .addUse(MC6809::AD).addUse(MC6809::AW);
      } else if (LoReg == MC6809::AD && !HiIsSpill) {
        // Physreg collision (ACC16-widened inputs): LO already sits in
        // $ad and HI is in some other physreg (AW handled by case 1, RS
        // imaginaries here). The sequential order would clobber LO with
        // the hi→$ad copy, so move LO to $aw first.
        copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                    MC6809::AW, MC6809::AD, /*KillSrc=*/false);
        if (HiReg != MC6809::AD)
          copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                      MC6809::AD, HiReg, /*KillSrc=*/false);
      } else if (LoIsSpill && HiIsSpill) {
        // Case 2: both spilled.  LO first (LDD writes AD, then TFR
        // moves it to AW), THEN HI into AD.
        LoadSpillIntoAD(LoReg);
        copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                    MC6809::AW, MC6809::AD, /*KillSrc=*/false);
        LoadSpillIntoAD(HiReg);
      } else if (HiIsSpill) {
        // Case 3: HI spilled.  Move LO to AW first (preserve before
        // LDD clobbers AD), then load HI into AD.
        if (LoReg != MC6809::AW) {
          copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                      MC6809::AW, LoReg, /*KillSrc=*/false);
        }
        LoadSpillIntoAD(HiReg);
      } else if (LoIsSpill) {
        // Case 4: LO spilled.  If HI already in AD, use LDW to load
        // LO into AW without touching AD.  Otherwise normal sequence.
        if (HiReg == MC6809::AD) {
          LoadSpillIntoAW(LoReg);
        } else {
          LoadSpillIntoAD(LoReg);
          copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                      MC6809::AW, MC6809::AD, /*KillSrc=*/false);
          if (HiReg != MC6809::AD) {
            copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                        MC6809::AD, HiReg, /*KillSrc=*/false);
          }
        }
      } else {
        // Case 5: neither spilled.  Existing sequential behavior.
        if (HiReg != MC6809::AD) {
          copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                      MC6809::AD, HiReg, /*KillSrc=*/false);
        }
        if (LoReg != MC6809::AW) {
          copyPhysReg(MBB, InsertBefore, MI.getDebugLoc(),
                      MC6809::AW, LoReg, /*KillSrc=*/false);
        }
      }

      // Find the last instruction we emitted (immediately before MI).
      MachineBasicBlock::iterator LastEmitted = std::prev(InsertBefore);
      bool EmittedSomething = (LastEmitted != BeforeFirstCopy);
      if (EmittedSomething) {
        // Add implicit-def $aq to the last emitted MI so the
        // verifier sees $aq freshly defined here.
        LastEmitted->addOperand(MachineOperand::CreateReg(
            MC6809::AQ, /*isDef=*/true, /*isImp=*/true));
      } else {
        // Both inputs already in place; emit a self-COPY as the
        // verifier marker.  CopyPropagation will eliminate it.
        Builder.buildInstr(TargetOpcode::COPY)
            .addDef(MC6809::AQ)
            .addUse(MC6809::AQ);
      }
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::AnyExt32_i16: {
    // Bug #302 redesign Phase 2 (2026-05-17): i16 → i32 anyext.
    // Move the i16 source into the low half of the i32 destination
    // (= AW for AQ-dest, = slot+2 for SPILL_Q*N-dest).  High half
    // is undefined — the IR's anyext semantics make the upper bits
    // don't-care.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (isQSpillReg(DstReg)) {
      // SPILL_Q*N destination: STD <src>,slot+2,$su.  Move src
      // through AD if not already there (STD only writes from AD).
      if (SrcReg != MC6809::AD) {
        copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                    MC6809::AD, SrcReg, /*KillSrc=*/false);
      }
      emitSpillStoreFrom(Builder, MC6809::AD, DstReg, /*ExtraOffset=*/2, MF);
    } else {
      // AQ destination: AW (AQ.sub_lo_word) ← src.  copyPhysReg
      // elides when src is already AW.
      if (SrcReg != MC6809::AW) {
        copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                    MC6809::AW, SrcReg, /*KillSrc=*/false);
      }
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::SignTest_i32: {
    // HD6309 native i32 sign test (Bug #301b).
    // Dst is ACC8; Src is ACC32 (AQ or SPILL_Q*N).
    //
    // Strategy: get the MSByte (= sign byte, big-endian byte 0 of i32)
    // into AA, ASLA to shift bit 7 into CC.C, then LDA #0 ; ADCA #0
    // to materialise C as a 0/1 byte in AA.  COPY AA to Dst.
    //
    // Bug #304 followup (2026-05-21): work in AA, not AB.  For AQ-source
    // the MSByte already IS AA (AQ.sub_hi_word.sub_hi_byte), so the
    // initial TFR A,B that brought it into AB was pure overhead — 2
    // bytes / 4 cycles per call.  Working in A directly saves that
    // instruction.  Both A and B are part of AQ so the AQ clobber is
    // unchanged (Defs already lists AQ — see Bug #304 fix).
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();

    // Step 1: ensure MSByte is in AA.
    if (SrcReg == MC6809::AQ) {
      // AQ.MSByte = AA — already where we need it.  No instruction.
    } else if (isQSpillReg(SrcReg)) {
      // SPILL_Q*N: load MSByte from slot+0 into AA.
      emitSpillLoadInto(Builder, MC6809::AA, SrcReg, /*ExtraOffset=*/0, MF);
    } else {
      llvm_unreachable("SignTest_i32 src must be AQ or SPILL_Q*N");
    }

    // Step 2: ASLA shifts AA left, putting bit 7 (= sign bit) into CC.C
    // and clobbering AA's contents.  We'll overwrite AA in step 3.
    Builder.buildInstr(MC6809::ASLAa)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit);

    // Step 3: LDA #0 ; ADCA #0 — AA = 0 + 0 + CC.C = sign bit (0 or 1).
    Builder.buildInstr(MC6809::LDAi8)
        .addDef(MC6809::AA, RegState::Implicit)
        .addImm(0);
    Builder.buildInstr(MC6809::ADCAi8)
        .addDef(MC6809::AA, RegState::Implicit)
        .addImm(0);

    // Step 4: COPY AA to Dst (ACC8).  Dst lands in AA / AB / AE / AF
    // or a SPILL_A* / SPILL_B* slot (materialise/dematerialise handles
    // those via copyPhysReg).
    if (DstReg != MC6809::AA) {
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  DstReg, MC6809::AA, /*KillSrc=*/true);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::EqZero_i32: {
    // Bug #301 (2026-05-16): native HD6309 i32 equal-to-zero.
    // Strategy: OR all 4 source bytes into AB, then extract CC.Z into AB.
    // Result i1 = 1 if i32 was zero, 0 otherwise.
    //
    // CAVEAT: this expansion CLOBBERS $aq's AB byte (AQ-source path) or
    // leaves $aq untouched but clobbers $ab (SPILL_Q*N-source path).
    // The legalizer/regalloc must treat the LHS i32 vreg as dead post-
    // EqZero_i32 — same effective constraint as the __cmpsi2 libcall this
    // replaces (which clobbers all caller-save regs via the call).
    //
    // AQ source (HD6309 has reg-to-reg OR via ORRp):
    //   ORR A,B    ; B = B | A
    //   ORR E,B    ; B = B | E
    //   ORR F,B    ; B = B | F  — CC.Z set iff all 4 bytes were 0
    //
    // SPILL_Q*N source (4 bytes at slot+0..3 in big-endian Q layout):
    //   LDB <slot+0>,$su
    //   ORB <slot+1>,$su
    //   ORB <slot+2>,$su
    //   ORB <slot+3>,$su   ; CC.Z set iff all bytes were 0
    //
    // Then CC.Z → bit 0 of AB:
    //   TFR CC,A     ; A = CC
    //   ANDA #4      ; A = Z bit (0x04 or 0)
    //   LSRA         ; A = 0x02 or 0
    //   LSRA         ; A = 0x01 or 0
    //   TFR A,B      ; B = 0x01 or 0
    //
    // Finally COPY B to Dst's parent byte (coalescer-tolerant dispatch
    // mirrors SignTest_i32).
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();

    // Step 1: OR all 4 bytes into AA.
    //
    // Bug #304 followup (2026-05-21): work in AA, not AB.  Avoids the
    // final TFR A,B that brought the extracted Z bit into AB for
    // copying to Dst — Dst may be AA directly, in which case the TFR
    // was pure overhead.  Both A and B are part of AQ so the AQ
    // clobber declared in EqZero_i32's Defs covers either choice.
    if (SrcReg == MC6809::AQ) {
      // ORR B,A; ORR E,A; ORR F,A  → A = A|B|E|F, CC.Z set if all 0.
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AE).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AF).addUse(MC6809::AA);
    } else if (isQSpillReg(SrcReg) && isStaticSpillSlot(SrcReg, MF)) {
      // Static-stack: LDA static+0; 3× ORA static+N (extended, PC-rel under PIC).
      bool IsPIC = MF.getTarget().isPositionIndependent();
      int Base = staticSpillOffset(SrcReg, MF);
      Builder.buildInstr(getSymLoadOpcode(MC6809::AA, /*IsDP=*/false, IsPIC))
          .addDef(MC6809::AA, RegState::Implicit)
          .addTargetIndex(MC6809::TI_STATIC_STACK, Base);
      for (int N = 1; N <= 3; ++N)
        Builder.buildInstr(getStaticStackOpcode(MC6809::ORAi_o8, IsPIC))
            .addDef(MC6809::AA, RegState::Implicit)
            .addUse(MC6809::AA, RegState::Implicit)
            .addTargetIndex(MC6809::TI_STATIC_STACK, Base + N);
    } else if (isQSpillReg(SrcReg)) {
      int Offset = computeSpillStackOffset(SrcReg, MF);
      auto pickLD = [](int Off) {
        return (Off >= -128 && Off <= 127) ? MC6809::LDAi_o8 : MC6809::LDAi_o16;
      };
      auto pickOR = [](int Off) {
        return (Off >= -128 && Off <= 127) ? MC6809::ORAi_o8 : MC6809::ORAi_o16;
      };
      // LDA slot+0,$su
      Builder.buildInstr(pickLD(Offset))
          .addDef(MC6809::AA, RegState::Implicit)
          .addImm(Offset).addReg(MC6809::SU);
      // 3× ORA slot+N,$su
      for (int N = 1; N <= 3; ++N) {
        Builder.buildInstr(pickOR(Offset + N))
            .addDef(MC6809::AA, RegState::Implicit)
            .addUse(MC6809::AA, RegState::Implicit)
            .addImm(Offset + N).addReg(MC6809::SU);
      }
    } else {
      llvm_unreachable("EqZero_i32 src must be AQ or SPILL_Q*N");
    }

    // Step 2: extract CC.Z into bit 0 of AA.
    //   TFR CC,A; ANDA #4; LSRA; LSRA
    Builder.buildInstr(MC6809::TFRp)
        .addDef(MC6809::AA).addUse(MC6809::CC);
    Builder.buildInstr(MC6809::ANDAi8)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit)
        .addImm(4);
    Builder.buildInstr(MC6809::LSRAa)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit);
    Builder.buildInstr(MC6809::LSRAa)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit);

    // Step 3: COPY AA to Dst's actual byte register.
    if (DstReg != MC6809::AA) {
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  DstReg, MC6809::AA, /*KillSrc=*/true);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::EqConst_i32: {
    // Bug #301 (2026-05-16): native HD6309 i32 equal-to-
    // constant test.  Strategy: compute X - K via SUBW+SBCD, which sets
    // CC.Z if X == K, then extract Z as a 0/1 byte (same as EqZero_i32).
    //
    // AQ source: $aq already holds X.
    //   SUBW #low16(K)   ; AW = AW - low16(K), sets C
    //   SBCD #high16(K)  ; AD = AD - high16(K) - C, sets NZ
    //                    ; CC.Z = 1 iff X == K
    //
    // SPILL_Q*N source: load Q first (clobbers $aq — assumed safe since
    // regalloc spilled the i32 to a slot, meaning $aq is free here).
    //   LDQ <slot>,$su
    //   SUBW #low16(K)
    //   SBCD #high16(K)
    //
    // CC.Z → bit 0 of AB (same idiom as EqZero_i32):
    //   TFR CC,A; ANDA #4; LSRA; LSRA; TFR A,B
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    int64_t K = MI.getOperand(2).getImm();
    uint16_t KLo = static_cast<uint16_t>(K & 0xFFFF);
    uint16_t KHi = static_cast<uint16_t>((K >> 16) & 0xFFFF);

    // Step 1: ensure X is in $aq.
    if (isQSpillReg(SrcReg)) {
      emitSpillLoadInto(Builder, MC6809::AQ, SrcReg, /*ExtraOffset=*/0, MF);
    } else if (SrcReg != MC6809::AQ) {
      llvm_unreachable("EqConst_i32 src must be AQ or SPILL_Q*N");
    }

    // Step 2: SUBW #KLo; SBCD #KHi — sets CC for X - K.
    Builder.buildInstr(MC6809::SUBWi16).addImm(KLo);
    Builder.buildInstr(MC6809::SBCDi16).addImm(KHi);

    // Bug #346: SUBW/SBCD leave CC.Z reflecting only the high 16 bits, so
    // X==K would be falsely reported whenever the halves differ only in the
    // low word with no borrow. STQ-to-scratch sets N/Z from the full 32-bit
    // Q (and leaves C), giving a correct CC.Z = (X == K) before extraction.
    Builder.buildInstr(MC6809::LEASi_o5).addImm(-4).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::STQi_o0)
        .addUse(MC6809::AQ, RegState::Implicit)
        .addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5).addImm(4).addReg(MC6809::SS);

    // Step 3: extract CC.Z into bit 0 of AA.
    // Bug #304 followup (2026-05-21): work in AA, not AB — avoids the
    // final TFR A,B that's pure overhead when Dst happens to be AA.
    Builder.buildInstr(MC6809::TFRp)
        .addDef(MC6809::AA).addUse(MC6809::CC);
    Builder.buildInstr(MC6809::ANDAi8)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit)
        .addImm(4);
    Builder.buildInstr(MC6809::LSRAa)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit);
    Builder.buildInstr(MC6809::LSRAa)
        .addDef(MC6809::AA, RegState::Implicit)
        .addUse(MC6809::AA, RegState::Implicit);

    // Step 4: COPY AA to Dst (ACC8).
    if (DstReg != MC6809::AA) {
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  DstReg, MC6809::AA, /*KillSrc=*/true);
    }
    MI.eraseFromParent();
    return true;
  }
  case TargetOpcode::COPY: {
    // Bug #161 round 14: handle a COPY whose source is one of the SPILL_Q
    // half-word sub-registers (SPILL_QnLO / SPILL_QnHI). The VirtRegMap
    // rewriter substitutes these when a REG_SEQUENCE-built ACC32 vreg
    // lands in SPILL_Q* and a downstream sub-reg COPY (sub_lo_word /
    // sub_hi_word) is rewritten. Read the right 16-bit half from the
    // parent SPILL_Q's stack slot. Big-endian: HI at +0, LO at +2.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    MCPhysReg Parent = 0; bool IsLo = false;
    if (DstReg.isPhysical() && SrcReg.isPhysical() &&
        isQSpillHalfReg(SrcReg, Parent, IsLo)) {
      Register OrigDst = DstReg;
      // Pure def: stage via the real register, preserved (see Extract16).
      bool StageDst = needsMaterialization(DstReg);
      Register RealDst = StageDst ? getPhysRegFor(DstReg) : DstReg;
      if (StageDst)
        pushStagingReg(Builder, RealDst);
      emitSpillLoadInto(Builder, RealDst, Parent, /*ExtraOffset=*/IsLo ? 2 : 0, MF);
      if (StageDst) {
        dematerializeReg(Builder, RealDst, OrigDst, MF);
        pullStagingReg(Builder, RealDst);
      }
      MI.eraseFromParent();
      return true;
    }
    // Symmetric: COPY whose DST is a SPILL_Q half-word — happens when
    // a REG_SEQUENCE source byte gets rewritten into SPILL_Q half slots.
    if (DstReg.isPhysical() && SrcReg.isPhysical() &&
        isQSpillHalfReg(DstReg, Parent, IsLo)) {
      // Move src into AD first if it's not already there, then store.
      if (SrcReg != MC6809::AD) {
        copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                    MC6809::AD, SrcReg, /*KillSrc=*/false);
      }
      emitSpillStoreFrom(Builder, MC6809::AD, Parent, /*ExtraOffset=*/IsLo ? 2 : 0, MF);
      MI.eraseFromParent();
      return true;
    }
    return false;  // fall through to default COPY handling
  }
  case MC6809::MaterializeCC_C_to_byte:
  case MC6809::MaterializeCC_V_to_byte:
  case MC6809::MaterializeCC_Z_to_byte:
  case MC6809::MaterializeCC_N_to_byte: {
    // Read a single CC flag bit and deposit it as a 0/1 byte in $ab.
    // The CC bit is the source of truth (no explicit input operand).
    //
    // CC bit layout (msb→lsb): E F H I N Z V C.  Extraction:
    //   C: LDB #0 ; ADCB #0       (cleanest — uses ADC's carry-in)
    //   V: TFR CC,B ; LSRB ; ANDB #1
    //   Z: TFR CC,B ; LSRB ; LSRB ; ANDB #1
    //   N: TFR CC,B ; LSRB×3 ; ANDB #1
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register RealDst = DstReg;
    Register OrigDst = DstReg;
    bool StageDst = needsMaterialization(DstReg);
    if (StageDst) {
      // Pure def: stage via the real register, preserved (see Extract16).
      RealDst = getPhysRegFor(DstReg);
      pushStagingReg(Builder, RealDst);
    }
    assert((RealDst == MC6809::AB || RealDst == MC6809::AA) &&
           "MaterializeCC_*_to_byte expects an A/B-allocated destination");
    // Either half works: pick the opcode family from the assigned register
    // (a hard ABc pin would propagate through the consuming byte chains and
    // oversubscribe the single-register class).
    bool IsA = (RealDst == MC6809::AA);
    unsigned Opcode = MI.getOpcode();
    if (Opcode == MC6809::MaterializeCC_C_to_byte) {
      // LD #0 ; ADC #0 — fastest path for the C bit.
      Builder.buildInstr(IsA ? MC6809::LDAi8 : MC6809::LDBi8)
          .addDef(RealDst, RegState::Implicit).addImm(0);
      Builder.buildInstr(IsA ? MC6809::ADCAi8 : MC6809::ADCBi8)
          .addDef(RealDst, RegState::Implicit).addImm(0);
    } else {
      // TFR CC,r ; LSR×N ; AND #1.
      unsigned NShifts = 0;
      switch (Opcode) {
      case MC6809::MaterializeCC_V_to_byte: NShifts = 1; break;
      case MC6809::MaterializeCC_Z_to_byte: NShifts = 2; break;
      case MC6809::MaterializeCC_N_to_byte: NShifts = 3; break;
      default: llvm_unreachable("unreachable");
      }
      Builder.buildInstr(MC6809::TFRp)
          .addDef(RealDst)
          .addUse(MC6809::CC);
      for (unsigned I = 0; I < NShifts; ++I) {
        Builder.buildInstr(IsA ? MC6809::LSRAa : MC6809::LSRBa)
            .addDef(RealDst, RegState::Implicit);
      }
      Builder.buildInstr(IsA ? MC6809::ANDAi8 : MC6809::ANDBi8)
          .addDef(RealDst, RegState::Implicit).addImm(1);
    }

    if (StageDst) {
      MachineBasicBlock &MBB = *MI.getParent();
      auto NextIt = std::next(MachineBasicBlock::iterator(MI));
      MachineIRBuilder StoreBuilder(MBB, NextIt);
      dematerializeReg(StoreBuilder, RealDst, OrigDst, MF);
      pullStagingReg(StoreBuilder, RealDst);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::MaterializeByteToCarry_i8: {
    // Bug #184 inverse of MaterializeCarryToByte_i8: restore CC.C from a
    // 0/1 byte that was previously frozen via MaterializeCarryToByte_i8.
    // Used to preserve a SubSetCarry chain's borrow across a CC-clobbering
    // region. Single LSRB shifts bit 0 of B into CC.C; B is dead after.
    //
    // $src constrained to ABc, so after regalloc $src is always $ab or
    // an AB-spill; the spill path materialises into $ab via the usual
    // mechanism, then LSRB.
    MachineFunction &MF = *MI.getMF();
    Register SrcReg = MI.getOperand(0).getReg();
    Register RealSrc = SrcReg;
    bool StagedSrc = needsMaterialization(SrcReg);
    if (StagedSrc) {
      // Preserve the staging real; PULS leaves CC (incl. the C this
      // pseudo produces) intact.
      pushStagingReg(Builder, getPhysRegFor(SrcReg));
      RealSrc = materializeReg(Builder, SrcReg, MF);
    }
    assert((RealSrc == MC6809::AB || RealSrc == MC6809::AA) &&
           "MaterializeByteToCarry_i8 expects an A/B-allocated source");
    // LSR sets CC.C = (original bit 0) and clears N, V; sets Z if the
    // shifted result == 0. We only care about CC.C; downstream sub uses it.
    // Pick the half from the assigned register.
    Builder.buildInstr(RealSrc == MC6809::AA ? MC6809::LSRAa : MC6809::LSRBa)
        .addDef(RealSrc, RegState::Implicit);
    if (StagedSrc)
      pullStagingReg(Builder, RealSrc);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::MaterializeByteToOverflow_i8: {
    // Bug #186 follow-up Phase 5 (2026-04-28): V-flag mirror of
    // MaterializeByteToCarry_i8. Restore CC.V from a 0/1 byte that was
    // previously frozen via MaterializeOverflowToByte_i8. Used to
    // preserve a SubSetOverflowUse chain's V across a CC-clobbering
    // region (cross-BB or same-BB with intervening CMP/TST/BIT).
    //
    // $src constrained to ABc; after regalloc $src is always $ab or
    // an AB-spill; spill path materialises into $ab as usual.
    //
    // Expansion:
    //   ANDB #1     ; ensure B ∈ {0,1} (defensive; producer guarantees
    //                 this but spilling-through-byte-then-restoring may
    //                 in theory reload garbage, so be safe)
    //   ADDB #0x7F  ; signed-overflow if B = 1 (since 0x7F + 0x01 =
    //                 0x80 — pos+pos→neg). CC.V = (B was 1).
    // Other CC bits clobbered, but the immediately-following
    // SubSetOverflowUse will overwrite them.
    MachineFunction &MF = *MI.getMF();
    Register SrcReg = MI.getOperand(0).getReg();
    Register RealSrc = SrcReg;
    bool StagedSrc = needsMaterialization(SrcReg);
    if (StagedSrc) {
      // Preserve the staging real; PULS leaves CC (incl. V) intact.
      pushStagingReg(Builder, getPhysRegFor(SrcReg));
      RealSrc = materializeReg(Builder, SrcReg, MF);
    }
    assert((RealSrc == MC6809::AB || RealSrc == MC6809::AA) &&
           "MaterializeByteToOverflow_i8 expects an A/B-allocated source");
    // Pick the half from the assigned register.
    bool SrcIsA = (RealSrc == MC6809::AA);
    Builder.buildInstr(SrcIsA ? MC6809::ANDAi8 : MC6809::ANDBi8)
        .addDef(RealSrc, RegState::Implicit)
        .addImm(1);
    Builder.buildInstr(SrcIsA ? MC6809::ADDAi8 : MC6809::ADDBi8)
        .addDef(RealSrc, RegState::Implicit)
        .addImm(0x7F);
    if (StagedSrc)
      pullStagingReg(Builder, RealSrc);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::ANYEXT_i8_to_i16: {
    // i8→i16 anyext pseudo. Only the low byte is defined; high byte is
    // don't-care. Emit the low-byte copy (dst's lo-byte sub-physreg ←
    // SrcReg); leave the high byte whatever regalloc/prior code left
    // there.
    MachineFunction &MF = *MI.getMF();
    const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    Register DstLo = TRI->getSubReg(DstReg, MC6809::sub_lo_byte);
    assert(DstLo && "16-bit dest must have byte sub-registers");
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                DstLo, SrcReg, /*KillSrc=*/false);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::MERGE_LOHI_i16: {
    // Merge two i8s into an i16 (bug #118 Layer 1). Strategy: stage HiReg
    // into AA and LoReg into AB so AD now holds the merged value, then
    // dematerialise AD into DstReg (single STD if dst is imag/spill, no-op
    // if dst is AD itself, TFR if dst is another Imag16).
    //
    // The two byte loads use loadByteInto, which guarantees that loading
    // a byte into AA never touches AB (and vice versa). The only remaining
    // hazard is when loading HiReg→AA destroys LoReg's value because
    // LoReg == AA. Symmetric considerations cover HiReg == AB.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register LoReg  = MI.getOperand(1).getReg();
    Register HiReg  = MI.getOperand(2).getReg();

    // Bug #306: when LoReg == AB and HiReg == AA and DstReg == AD, the
    // standard expansion below collapses to zero machine instructions
    // (both loadByteInto calls early-return on SrcReg == Target, and
    // the trailing dematerializeReg(AD, AD) is a no-op).  With no MI
    // explicitly defining $ad in the merge's place, post-RA DCE can
    // kill an upstream byte-def of $aa / $ab if that def is itself
    // DCE-vulnerable (a load-immediate or clear with dead NZ side
    // effects — typically `LDAi8 #0` zeroing the high byte for an
    // i1→i16 zext, or its `CLRAa` peephole form).  The Store_i16_Mem
    // $ad consumer then reads garbage in the killed half.
    //
    // Scoped fix: when in the zero-instruction shape AND the
    // immediately upstream def of $aa or $ab is one of the DCE-
    // vulnerable opcodes, emit a `$ad = TFRp $ad` self-transfer.  The
    // explicit Def operand is honoured by LivePhysRegs::stepForward;
    // a KILL with implicit-def is NOT.  Cost: 2 bytes / 6 cycles, but
    // only when the bug shape is actually present (i1 / i8 zext to
    // i16) — non-load-immediate upstreams (TFRp, ADDB, ANDB, etc.)
    // are already kept alive by their own NZ side effects or their
    // operand semantics, so the anchor isn't needed there.
    if (LoReg == MC6809::AB && HiReg == MC6809::AA && DstReg == MC6809::AD) {
      auto IsDCEVulnerable = [](unsigned Opc) {
        switch (Opc) {
        case MC6809::LDAi8:
        case MC6809::LDBi8:
        case MC6809::CLRAa:
        case MC6809::CLRBa:
        case MC6809::Load_i8_Imm:
          return true;
        default:
          return false;
        }
      };
      // Walk back from the MERGE.  The first instruction that touches
      // $aa / $ab / $ad tells us whether DCE could kill the upstream:
      //
      //   - If it READS any of those regs, that read already anchors
      //     the def above it — no anchor needed at MERGE.
      //   - If it DEFINES $aa or $ab without reading, and the opcode
      //     is one of the DCE-vulnerable patterns, the def has no
      //     other consumer between itself and the (zero-MI) MERGE,
      //     so DCE will kill it — anchor needed.
      bool NeedsAnchor = false;
      MachineBasicBlock &MBB = *MI.getParent();
      for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
           It != MBB.rend(); ++It) {
        if (It->readsRegister(MC6809::AA, /*TRI=*/nullptr) ||
            It->readsRegister(MC6809::AB, /*TRI=*/nullptr) ||
            It->readsRegister(MC6809::AD, /*TRI=*/nullptr)) {
          NeedsAnchor = false;
          break;
        }
        if (It->modifiesRegister(MC6809::AA, /*TRI=*/nullptr) ||
            It->modifiesRegister(MC6809::AB, /*TRI=*/nullptr)) {
          NeedsAnchor = IsDCEVulnerable(It->getOpcode());
          break;
        }
      }
      if (NeedsAnchor) {
        Builder.buildInstr(MC6809::TFRp)
            .addDef(MC6809::AD)
            .addUse(MC6809::AD);
      }
      MI.eraseFromParent();
      return true;
    }

    // When the destination leaves the register file (imag/spill), the
    // pseudo defines neither $aa nor $ab -- yet the arranging below (EXG /
    // TFR) mutates them, and regalloc may keep the SOURCE bytes live past
    // the merge (crossed halves fed a swapped EXG that silently corrupted
    // both live sources). Preserve D around the staging window.
    bool StageDst = needsMaterialization(DstReg);
    if (StageDst)
      pushStagingReg(Builder, MC6809::AD);
    if (LoReg == MC6809::AA && HiReg == MC6809::AB) {
      // Full swap of the real pair. EXG A,B is the cheapest route.
      Builder.buildInstr(MC6809::EXGp)
          .addDef(MC6809::AA).addDef(MC6809::AB)
          .addUse(MC6809::AA).addUse(MC6809::AB);
    } else if (LoReg == MC6809::AA) {
      // Loading HiReg→AA would clobber LoReg's value first. Do the lo
      // copy first (TFR A,B preserves AA), then load HiReg into AA.
      loadByteInto(Builder, MC6809::AB, LoReg, MF);
      loadByteInto(Builder, MC6809::AA, HiReg, MF);
    } else {
      // Default order — hi then lo. Safe because loadByteInto(AA, HiReg)
      // never touches AB; the subsequent loadByteInto(AB, LoReg) reads
      // LoReg (which by the above branch is not AA) and writes only AB.
      // Edge case HiReg == AB: TFR B,A reads AB without modifying it,
      // then loadByteInto(AB, LoReg) overwrites AB with LoReg — correct.
      loadByteInto(Builder, MC6809::AA, HiReg, MF);
      loadByteInto(Builder, MC6809::AB, LoReg, MF);
    }
    // AD now holds {hi:AA, lo:AB}. Route to DstReg.
    dematerializeReg(Builder, MC6809::AD, DstReg, MF);
    if (StageDst)
      pullStagingReg(Builder, MC6809::AD);
    if (DstReg != MC6809::AD && !needsMaterialization(DstReg)) {
      // DstReg is a real Imag16-class register that isn't AD itself
      // (e.g. AW on HD6309 paths in future) — needs an explicit TFR.
      // Today ADc only contains AD + spills + RS, so this branch is
      // unreachable in MC6809; kept for safety if ADc grows.
      Builder.buildInstr(MC6809::TFRp).addDef(DstReg).addUse(MC6809::AD);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::FakeUse_Mem:
    // Debug-lifetime marker with a folded (spilled) operand; served its
    // purpose through regalloc -- nothing to emit.
    MI.eraseFromParent();
    return true;
  case MC6809::MERGE_LOHI_i16_Mem2: {
    // Both merge halves read straight from their frame slots.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    auto ReadOff = [&](unsigned I) -> int64_t {
      const MachineOperand &MO = MI.getOperand(I);
      return MO.isImm() ? MO.getImm() : MO.getCImm()->getSExtValue();
    };
    Register LBase = MI.getOperand(1).getReg();
    int64_t LOff = ReadOff(2);
    Register HBase = MI.getOperand(3).getReg();
    int64_t HOff = ReadOff(4);
    Register OrigDst = DstReg;
    bool StageDst = needsMaterialization(DstReg);
    if (StageDst) {
      pushStagingReg(Builder, MC6809::AD);
      if (LBase == MC6809::SS)
        LOff += 2;
      if (HBase == MC6809::SS)
        HOff += 2;
    }
    auto Emit = [&](bool IsLo, Register Base, int64_t Off) {
      bool Fits8 = (Off >= -128 && Off <= 127);
      unsigned LdOpc = IsLo ? (Fits8 ? MC6809::LDBi_o8 : MC6809::LDBi_o16)
                            : (Fits8 ? MC6809::LDAi_o8 : MC6809::LDAi_o16);
      Builder.buildInstr(LdOpc)
          .addDef(IsLo ? MC6809::AB : MC6809::AA, RegState::Implicit)
          .addImm(Off)
          .addReg(Base);
    };
    Emit(/*IsLo=*/true, LBase, LOff);
    Emit(/*IsLo=*/false, HBase, HOff);
    dematerializeReg(Builder, MC6809::AD, OrigDst, MF);
    if (StageDst)
      pullStagingReg(Builder, MC6809::AD);
    // Real non-AD destination (AW now that the merge dst class is ACC16):
    // dematerializeReg no-ops for it, so route the AD-staged result
    // explicitly -- losing this write-back left a loop index stale in W
    // (memchr hang).
    if (OrigDst != MC6809::AD && !needsMaterialization(OrigDst))
      Builder.buildInstr(MC6809::TFRp).addDef(OrigDst).addUse(MC6809::AD);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::MERGE_LOHI_i16_MemLo:
  case MC6809::MERGE_LOHI_i16_MemHi: {
    // Merge where one byte half was spilled and folded into a frame read
    // (see foldMemoryOperandImpl). Stage the register half into its AD
    // half, then load the spilled half straight from the slot. The slot
    // operand was rewritten to (base, offset) by eliminateFrameIndex.
    MachineFunction &MF = *MI.getMF();
    bool MemIsLo = (MI.getOpcode() == MC6809::MERGE_LOHI_i16_MemLo);
    Register DstReg = MI.getOperand(0).getReg();
    Register KeptReg = MI.getOperand(1).getReg();
    Register BaseReg = MI.getOperand(2).getReg();
    const MachineOperand &OffOp = MI.getOperand(3);
    int64_t Off = OffOp.isImm() ? OffOp.getImm()
                                : OffOp.getCImm()->getSExtValue();
    Register OrigDst = DstReg;
    bool StageDst = needsMaterialization(DstReg);
    if (StageDst)
      pushStagingReg(Builder, MC6809::AD);
    // If the push shifted S and the slot is S-relative, compensate.
    if (StageDst && BaseReg == MC6809::SS)
      Off += 2;
    // Register half first (its source may be AA/AB and would otherwise be
    // clobbered by the memory load into the other half).
    loadByteInto(Builder, MemIsLo ? MC6809::AA : MC6809::AB, KeptReg, MF);
    bool Fits8 = (Off >= -128 && Off <= 127);
    unsigned LdOpc = MemIsLo ? (Fits8 ? MC6809::LDBi_o8 : MC6809::LDBi_o16)
                             : (Fits8 ? MC6809::LDAi_o8 : MC6809::LDAi_o16);
    Builder.buildInstr(LdOpc)
        .addDef(MemIsLo ? MC6809::AB : MC6809::AA, RegState::Implicit)
        .addImm(Off)
        .addReg(BaseReg);
    dematerializeReg(Builder, MC6809::AD, OrigDst, MF);
    if (StageDst)
      pullStagingReg(Builder, MC6809::AD);
    // Route to a real non-AD destination -- see MERGE_LOHI_i16_Mem2.
    if (OrigDst != MC6809::AD && !needsMaterialization(OrigDst))
      Builder.buildInstr(MC6809::TFRp).addDef(OrigDst).addUse(MC6809::AD);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::SEX8Implicit: {
    // Sign-extend the LSB of an 8-bit register ($aalsb/$ablsb) to 8 bits.
    // Bug #90: previously missing from expandPostRAPseudo, causing llc to
    // crash with "Pseudoinstruction was never lowered" for any function
    // that selected a G_SEXT from s1 to s8 at -O0. Two expansion paths:
    //
    //   same-half (src byte == dst byte):
    //     AND[AB] #1   ; mask off the bit
    //     NEG[AB]      ; 0 → 0, 1 → -1 (= 0xFF)
    //
    //   cross-half (src byte != dst byte):
    //     TFR src_byte, dst_byte
    //     AND[AB] #1
    //     NEG[AB]
    //
    // ANDA/ANDB clear V and set NZ; NEGA/NEGB set C from the input. No
    // carry-chain ordering constraint — SEX8Implicit isn't used inside a
    // multi-byte arithmetic chain today.
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();

    // Pure def: stage an imaginary/spill destination via the real
    // register, preserved (see Extract16).
    Register OrigDst = DstReg;
    bool StageDst = needsMaterialization(DstReg);
    if (StageDst) {
      DstReg = getPhysRegFor(DstReg);
      pushStagingReg(Builder, DstReg);
    }

    // SrcReg is the full byte (i1 lives in an ACC8; the byte's LSB
    // carries the boolean).
    Register SrcByte = SrcReg;

    // Resolve which page-1 byte register actually does the AND+NEG. The
    // dest may be E/F under HD6309, but AND/NEG only exist for A/B, so
    // pick the closest A or B target.
    Register WorkReg = DstReg;
    if (DstReg == MC6809::AE) WorkReg = MC6809::AA;
    else if (DstReg == MC6809::AF) WorkReg = MC6809::AB;

    // If src and work-reg live in different halves of D (or the src is
    // an HD6309 byte), copy the byte first. TFR preserves the LSB we
    // care about.
    if (SrcByte != WorkReg)
      Builder.buildInstr(MC6809::TFRp).addDef(WorkReg).addUse(SrcByte);

    // Extract the LSB and sign-extend via two's-complement negation.
    bool DstIsA = (WorkReg == MC6809::AA);
    unsigned AndOpc = DstIsA ? MC6809::ANDAi8 : MC6809::ANDBi8;
    unsigned NegOpc = DstIsA ? MC6809::NEGAa  : MC6809::NEGBa;
    Builder.buildInstr(AndOpc).addImm(1);
    Builder.buildInstr(NegOpc);

    // If the original dest was an HD6309 byte register, copy the
    // sign-extended result back from WorkReg to DstReg.
    if (WorkReg != DstReg)
      Builder.buildInstr(MC6809::TFRp).addDef(DstReg).addUse(WorkReg);

    if (StageDst) {
      dematerializeReg(Builder, DstReg, OrigDst, MF);
      pullStagingReg(Builder, DstReg);
    }

    MI.eraseFromParent();
    return true;
  }
  case MC6809::SEX16Implicit: {
    // Physreg form (2026-06-28): produces the result in physical AD; selection
    // appended `%dst = COPY $ad`, so dst placement (incl. spill -> STD) is the
    // COPY's job, not ours. We just emit the hardware sign-extend.
    Builder.buildInstr(MC6809::SEXx);
    MI.eraseFromParent();
    break;
  }
  case MC6809::SEX32Implicit: {
    // Same SPILL_Q* concern as ZEX32Implicit (bug #208 round 2):
    // SEXWx sign-extends AW into AD, leaving AQ correct, but a
    // SPILL_Q* dst needs an explicit STQ.
    //
    // Bug #274: source may be AD, AW, or any other ACC16 member that
    // survives MaterializeSpills (notably the imaginary direct-page
    // RS0..RS3, or AD when fed from a MERGE_LOHI_i16). After expansion
    // AQ must hold (AD=sign(src) | AW=src), which is what SEXWx does
    // — but only when src is already in AW. Route every non-AW source
    // through copyPhysReg before SEXWx (mirrors ZEX32Implicit's pre-
    // CLRDa src→AW copy).
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (SrcReg != MC6809::AW)
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  MC6809::AW, SrcReg, /*KillSrc=*/true);
    Builder.buildInstr(MC6809::SEXWx);
    if (DstReg != MC6809::AQ)
      dematerializeReg(Builder, MC6809::AQ, DstReg, *MI.getMF());
    MI.eraseFromParent();
    break;
  }
  case MC6809::ZEX8Implicit: {
    Register SrcReg = MI.getOperand(1).getReg();
    unsigned Opcode;
    // Bug #311 Phase 1 step 1.5 (2026-05-20): post-1.2 ZEX8Implicit
    // input is ACC8; *LSB / SPILL_*LSB sources are no longer possible.
    // ACC8 byte halves on HD6309 are AA, AB, AE, AF; non-A half routes
    // to the B-side encoding (ANDB).
    Register ByteHalf;
    if (SrcReg == MC6809::AA || SrcReg == MC6809::AE)
      ByteHalf = MC6809::AA;
    else
      ByteHalf = MC6809::AB;
    Opcode = (ByteHalf == MC6809::AA) ? MC6809::ANDAi8 : MC6809::ANDBi8;
    MI.setDesc(Builder.getTII().get(Opcode));
    MI.removeOperand(1);
    MI.removeOperand(0);
    MI.addOperand(MachineOperand::CreateImm(1));
    break;
  }
  case MC6809::ZEX16Implicit: {
    // Same SPILL_D* concern as ZEX32Implicit (bug #208 round 2): CLRAa
    // zeros AA leaving AB = src, so AD holds the correct (0:src) i16
    // — but if regalloc allocated the dst to a SPILL_D* slot the
    // result must be STD'd to it before downstream consumers read.
    //
    // Bug #209 HD6309 leg / bug #211: on HD6309 the ACC16 register
    // class includes BOTH AD and AW. If the regalloc places the dst
    // in AW (the HD6309 alternative 16-bit accumulator), CLRAa still
    // computes the value in AD — but AW is left holding stale data
    // (whatever the prior code put in E:F). A downstream STW reads
    // the stale W and writes garbage to memory. Fix: when the dst is
    // AW (or any non-AD architectural ACC16), copy AD → DstReg via
    // copyPhysReg so the correct (0:src) value lands where the
    // regalloc expects it. Mirrors ZEX32Implicit's pre-CLRDa src→AW
    // copy.
    // Physreg form (2026-06-28): produces the result in physical AD (CLRA zeros
    // AA; AB=src preserved). Selection appended `%dst = COPY $ad`, so dst
    // placement (AD coalesce / AW copy / spill STD) is the COPY's job.
    Builder.buildInstr(MC6809::CLRAa);
    MI.eraseFromParent();
    break;
  }
  case MC6809::ZEX32Implicit: {
    // Source may be AD, AW, or any other ACC16 member that survives
    // MaterializeSpills (notably the imaginary direct-page RS0..RS3).
    // Destination may be AQ or a SPILL_Q* spill slot (regalloc picks
    // freely from the ACC32 class). After expansion the dst register
    // must hold (D=0 | W=src). Steps:
    //   1. If src != AW, copy src → AW (covers AD via TFR D,W,
    //      RSn via LDD<rsN+TFR, etc. — copyPhysReg knows the
    //      sequences; AD's intermediate clobber is safe since CLRDa
    //      will overwrite it next).
    //   2. CLRDa zeros AD. AQ now holds (AD=0):(AW=src).
    //   3. If dst is a SPILL_Q*, store AQ to the spill slot via
    //      dematerializeReg — without this, the spill slot is never
    //      written and downstream EXTRACT_HI_word_i32 / byte-add
    //      consumers read uninitialised stack memory (bug #208 round
    //      2: ZEX32 dst landed in $spill_q0 and the missing STQ
    //      let strtol's `acc * base + digit` accumulator read garbage
    //      for the digit's high bytes).
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (SrcReg != MC6809::AW)
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  MC6809::AW, SrcReg, /*KillSrc=*/true);
    Builder.buildInstr(MC6809::CLRDa);
    if (DstReg != MC6809::AQ)
      dematerializeReg(Builder, MC6809::AQ, DstReg, *MI.getMF());
    MI.eraseFromParent();
    break;
  }
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
  case MC6809::BranchJumpTable: {
    // Keep as pseudo — expanded in MCInstLower to avoid branch relaxation
    // seeing the concrete JMPi_oDI (which has isBranch and crashes verify).
    // An imaginary-homed index must first be staged into $ad (the
    // printer's shift-D dispatch contract). No preserve bracket: this is
    // a terminator — control leaves the block — and the index operand
    // has always claimed the accumulator here (the retired spill
    // machinery loaded spilled indices into $ad the same way).
    Register Idx = MI.getOperand(0).getReg();
    if (needsMaterialization(Idx)) {
      materializeReg(Builder, Idx, *MI.getMF());
      MI.getOperand(0).setReg(MC6809::AD);
      MI.getOperand(0).setIsKill(false);
    }
    break;
  }
  case MC6809::LEAPtrAdd_Imm:
  case MC6809::LEA_Ptr_Imm:
  case MC6809::LEAPtrAdd_Reg8:
  case MC6809::LEAPtrAdd_Reg16:
    expandLEAPtrAdd(Builder, MI);
    break;
  case MC6809::Load_i8_PostInc:
  case MC6809::Load_i16_PostInc:
  case MC6809::Load_iPtr_PostInc:
    expandLoadPostMod(Builder, MI, /*IsInc=*/true);
    break;
  case MC6809::Load_i8_PreDec:
  case MC6809::Load_i16_PreDec:
  case MC6809::Load_iPtr_PreDec:
    expandLoadPostMod(Builder, MI, /*IsInc=*/false);
    break;
  case MC6809::Store_i8_PostInc:
  case MC6809::Store_i16_PostInc:
  case MC6809::Store_iPtr_PostInc:
    expandStorePostMod(Builder, MI, /*IsInc=*/true);
    break;
  case MC6809::Store_i8_PreDec:
  case MC6809::Store_i16_PreDec:
  case MC6809::Store_iPtr_PreDec:
    expandStorePostMod(Builder, MI, /*IsInc=*/false);
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
  case MC6809::SpillLoad_i32_Mem:
  case MC6809::Load_iPtr_Mem:
  case MC6809::Load_i8_MemIndirect:
  case MC6809::Load_i16_MemIndirect:
  case MC6809::Load_i32_MemIndirect:
  case MC6809::Load_iPtr_MemIndirect:
    expandLoadIdx(Builder, MI);
    break;
  case MC6809::JSR_iPtr_Mem:
    expandJsrIdx(Builder, MI);
    break;
  case MC6809::Load_i8_Sym:
  case MC6809::Load_i16_Sym:
  case MC6809::Load_i32_Sym:
  case MC6809::Load_iPtr_Sym:
    expandLoadSym(Builder, MI);
    break;
  case MC6809::Lea_iPtr_Sym:
    expandLeaSym(Builder, MI);
    break;
  case MC6809::Store_i8_Sym:
  case MC6809::Store_i16_Sym:
  case MC6809::Store_i32_Sym:
  case MC6809::Store_iPtr_Sym:
    expandStoreSym(Builder, MI);
    break;
  case MC6809::Store_i8_Mem:
  case MC6809::Store_i16_Mem:
  case MC6809::Store_i32_Mem:
  case MC6809::SpillStore_i32_Mem:
  case MC6809::Store_iPtr_Mem:
  case MC6809::Store_i8_MemIndirect:
  case MC6809::Store_i16_MemIndirect:
  case MC6809::Store_i32_MemIndirect:
  case MC6809::Store_iPtr_MemIndirect:
    expandStoreIdx(Builder, MI);
    break;
  case MC6809::AND_i8_Imm:
    // Also handles AND on i1 values (1-bounded byte source).
    expandImm(ANDImm, Builder, MI);
    break;
  case MC6809::AND_i16_Imm:
    expandBitwiseImm16(ANDImm, MC6809::ANDAi8, MC6809::ANDBi8, Builder, MI);
    break;
  case MC6809::AND_i8_MemIndirect:
  case MC6809::AND_i8_Mem:
    expandIdxImm(ANDIdxImm, Builder, MI);
    break;
  case MC6809::AND_i16_MemIndirect:
  case MC6809::AND_i16_Mem:
    expandBitwiseMem16(ANDIdxImm, Builder, MI);
    break;
  case MC6809::AND_i8_Reg:
  case MC6809::AND_i16_Reg:
    expandANDReg(Builder, MI);
    break;
  case MC6809::OR_i8_Imm:
    expandImm(ORImm, Builder, MI);
    break;
  case MC6809::OR_i16_Imm:
    expandBitwiseImm16(ORImm, MC6809::ORAi8, MC6809::ORBi8, Builder, MI);
    break;
  case MC6809::OR_i8_MemIndirect:
  case MC6809::OR_i8_Mem:
    expandIdxImm(ORIdxImm, Builder, MI);
    break;
  case MC6809::OR_i16_MemIndirect:
  case MC6809::OR_i16_Mem:
    expandBitwiseMem16(ORIdxImm, Builder, MI);
    break;
  case MC6809::OR_i8_Reg:
  case MC6809::OR_i16_Reg:
    expandORReg(Builder, MI);
    break;
  case MC6809::XOR_i8_Imm:
    expandImm(XORImm, Builder, MI);
    break;
  case MC6809::XOR_i16_Imm:
    expandBitwiseImm16(XORImm, MC6809::EORAi8, MC6809::EORBi8, Builder, MI);
    break;
  case MC6809::XOR_i8_MemIndirect:
  case MC6809::XOR_i8_Mem:
    expandIdxImm(XORIdxImm, Builder, MI);
    break;
  case MC6809::XOR_i16_MemIndirect:
  case MC6809::XOR_i16_Mem:
    expandBitwiseMem16(XORIdxImm, Builder, MI);
    break;
  case MC6809::XOR_i8_Reg:
  case MC6809::XOR_i16_Reg:
    expandXORReg(Builder, MI);
    break;
  case MC6809::Add_i8_Imm:
  case MC6809::Add_i16_Imm:
    expandImm(AddImm, Builder, MI);
    break;
  case MC6809::AddSetCarry_i8_Imm:
  case MC6809::AddSetCarry_i16_Imm:
  // Bug #147: AddSetOverflow_* shares the AddSetCarry expansion (same
  // ADDA/ADDB/ADDD instructions). The pseudo opcode is preserved only as
  // metadata for the selector's G_BRCOND-case V-vs-C dispatch; here we just lower to
  // the same MC instructions.
  case MC6809::AddSetOverflow_i8_Imm:
  case MC6809::AddSetOverflow_i16_Imm:
    expandImm(AddSetCarryImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i8_Imm:
  case MC6809::AddSetOverflowUse_i8_Imm:    // bug #147
    expandImm(AddCarryImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i16_Imm:
  case MC6809::AddSetOverflowUse_i16_Imm:   // bug #147
    expandCarryImm16(true, Builder, MI);
    break;
  case MC6809::Add_i8_MemIndirect:
  case MC6809::Add_i16_MemIndirect:
  case MC6809::Add_i8_Mem:
  case MC6809::Add_i16_Mem:
  case MC6809::AddSetCarry_i8_Mem:
  case MC6809::AddSetCarry_i16_Mem:
  case MC6809::AddSetOverflow_i8_Mem:    // bug #147
  case MC6809::AddSetOverflow_i16_Mem:
    expandIdxImm(AddIdxImm, Builder, MI);
    break;
  // Bug #297: i32 ADD _Mem family — emit ADDW <off+2>; ADCD <off+0>.
  // AddSetCarry_i32_Mem and AddSetOverflow_i32_Mem share the emission
  // with Add_i32_Mem (same hardware instructions; the pseudo opcode is
  // preserved only as metadata for the CC.C / CC.V flag-chain tracker
  // — see the long comment at MC6809InstrFamilies.td:336 for the bug
  // #147 / #186 rationale).
  case MC6809::Add_i32_Mem:
  case MC6809::AddSetCarry_i32_Mem:
  case MC6809::AddSetOverflow_i32_Mem:
    expandAddSub_i32_Mem(Builder, MI, /*IsAdd=*/true);
    break;
  // Static-stack sibling: memory operand is a TI_STATIC_STACK target index.
  case MC6809::Add_i32_Sym:
    expandAddSub_i32_Sym(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::AddSetCarryUse_i8_Mem:
  case MC6809::AddSetOverflowUse_i8_Mem:    // bug #147
    expandIdxImm(AddCarryIdxImm, Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i16_Mem:
  case MC6809::AddSetOverflowUse_i16_Mem:   // bug #147
    expandCarryMem16(true, Builder, MI);
    break;
  case MC6809::Add_i8_Reg:
  // Bug #221 Phase A: parallel A-half pseudo routes through the same
  // helper. getByteOpcodes() picks A-half opcodes when RealLHS == AA
  // (which is enforced by the _RegA pseudo's AAc dst class),
  // so no helper change needed.
  case MC6809::Add_i8_RegA:
  case MC6809::Add_i16_Reg:
    expandAddReg(Builder, MI);
    break;
  case MC6809::AddSetCarry_i8_Reg:
  case MC6809::AddSetCarry_i8_RegA:        // bug #221 Phase A
  case MC6809::AddSetOverflow_i8_Reg:      // bug #147
  case MC6809::AddSetOverflow_i8_RegA:     // bug #221 Phase A
    expandAddSetCarryByteReg(Builder, MI);
    break;
  case MC6809::AddSetCarry_i16_Reg:
  case MC6809::AddSetOverflow_i16_Reg:   // bug #147
    expandAddSetCarryReg(Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i8_Reg:
  case MC6809::AddSetCarryUse_i8_RegA:     // bug #221 Phase A
  case MC6809::AddSetOverflowUse_i8_Reg:   // bug #147
  case MC6809::AddSetOverflowUse_i8_RegA:  // bug #221 Phase A
    expandAddSetCarryUseByteReg(Builder, MI);
    break;
  case MC6809::AddSetCarryUse_i16_Reg:
  case MC6809::AddSetOverflowUse_i16_Reg:   // bug #147
    expandAddSetCarryUseReg(Builder, MI);
    break;
  case MC6809::Sub_i8_Imm:
  case MC6809::Sub_i16_Imm:
    expandImm(SubImm, Builder, MI);
    break;
  case MC6809::SubSetCarry_i8_Imm:
  case MC6809::SubSetCarry_i16_Imm:
  case MC6809::SubSetOverflow_i8_Imm:    // bug #147
  case MC6809::SubSetOverflow_i16_Imm:
    expandImm(SubSetCarryImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Imm:
  case MC6809::SubSetOverflowUse_i8_Imm:    // bug #147
    expandImm(SubBorrowImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i16_Imm:
  case MC6809::SubSetOverflowUse_i16_Imm:   // bug #147
    expandCarryImm16(false, Builder, MI);
    break;
  case MC6809::Sub_i8_MemIndirect:
  case MC6809::Sub_i16_MemIndirect:
  case MC6809::Sub_i8_Mem:
  case MC6809::Sub_i16_Mem:
  case MC6809::SubSetCarry_i8_Mem:
  case MC6809::SubSetCarry_i16_Mem:
  case MC6809::SubSetOverflow_i8_Mem:    // bug #147
  case MC6809::SubSetOverflow_i16_Mem:
    expandIdxImm(SubIdxImm, Builder, MI);
    break;
  // Bug #297: i32 SUB _Mem family — emit SUBW <off+2>; SBCD <off+0>.
  case MC6809::Sub_i32_Mem:
  case MC6809::SubSetCarry_i32_Mem:
  case MC6809::SubSetOverflow_i32_Mem:
    expandAddSub_i32_Mem(Builder, MI, /*IsAdd=*/false);
    break;
  // Static-stack sibling: memory operand is a TI_STATIC_STACK target index.
  case MC6809::Sub_i32_Sym:
    expandAddSub_i32_Sym(Builder, MI, /*IsAdd=*/false);
    break;
  case MC6809::Sub_i8_Reg:
  case MC6809::Sub_i8_RegA:                // bug #221 Phase A
    expandSubByteReg(Builder, MI);
    break;
  case MC6809::Sub_i16_Reg:
    expandSubReg(Builder, MI);
    break;
  case MC6809::SubSetCarry_i8_Reg:
  case MC6809::SubSetCarry_i8_RegA:        // bug #221 Phase A
  case MC6809::SubSetOverflow_i8_Reg:      // bug #147
  case MC6809::SubSetOverflow_i8_RegA:     // bug #221 Phase A
    expandSubSetCarryByteReg(Builder, MI);
    break;
  case MC6809::SubSetCarry_i16_Reg:
  case MC6809::SubSetOverflow_i16_Reg:   // bug #147
    expandSubSetCarryReg(Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Mem:
  case MC6809::SubSetOverflowUse_i8_Mem:    // bug #147
    expandIdxImm(SubBorrowIdxImm, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i16_Mem:
  case MC6809::SubSetOverflowUse_i16_Mem:   // bug #147
    expandCarryMem16(false, Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i8_Reg:
  case MC6809::SubSetCarryUse_i8_RegA:     // bug #221 Phase A
  case MC6809::SubSetOverflowUse_i8_Reg:   // bug #147
  case MC6809::SubSetOverflowUse_i8_RegA:  // bug #221 Phase A
    expandSubSetCarryUseByteReg(Builder, MI);
    break;
  case MC6809::SubSetCarryUse_i16_Reg:
  case MC6809::SubSetOverflowUse_i16_Reg:   // bug #147
    expandSubSetCarryUseReg(Builder, MI);
    break;
  // Bug #297: i32 pseudos that are dormant pre-legalizer-flip (commit 5).
  // Defined in MC6809InstrFamilies.td (commit 1) for completeness so the
  // multiclass instantiations are well-formed, but no MI emits them yet
  // and the corresponding expanders are stubs.  An llvm_unreachable here
  // catches a future selector arm that accidentally emits a variant
  // before its expander is filled in.
  // Bug #297 commit 2.1: i32 ADD/SUB _Imm and _Reg now have expanders.
  case MC6809::Add_i32_Imm:
  case MC6809::AddSetCarry_i32_Imm:
  case MC6809::AddSetOverflow_i32_Imm:
    expandAddSub_i32_Imm(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::Sub_i32_Imm:
  case MC6809::SubSetCarry_i32_Imm:
  case MC6809::SubSetOverflow_i32_Imm:
    expandAddSub_i32_Imm(Builder, MI, /*IsAdd=*/false);
    break;
  case MC6809::Add_i32_Reg:
  case MC6809::AddSetCarry_i32_Reg:
  case MC6809::AddSetOverflow_i32_Reg:
    expandAddSub_i32_Reg(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::Sub_i32_Reg:
  case MC6809::SubSetCarry_i32_Reg:
  case MC6809::SubSetOverflow_i32_Reg:
    expandAddSub_i32_Reg(Builder, MI, /*IsAdd=*/false);
    break;
  // Bug #311 Phase 2 (2026-05-21): native HD6309 i32 ADD/SUB with
  // carry-IN.  Used by the i32 G_*ADDE / G_*SUBE selector arms (see
  // selectAddE / selectSubE) once the legalizer rule for s32
  // carry-out chains is widened.
  case MC6809::AddSetCarryUse_i32_Imm:
  case MC6809::AddSetOverflowUse_i32_Imm:
    expandAddSubCarryUse_i32_Imm(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::SubSetCarryUse_i32_Imm:
  case MC6809::SubSetOverflowUse_i32_Imm:
    expandAddSubCarryUse_i32_Imm(Builder, MI, /*IsAdd=*/false);
    break;
  case MC6809::AddSetCarryUse_i32_Mem:
  case MC6809::AddSetOverflowUse_i32_Mem:
    expandAddSubCarryUse_i32_Mem(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::SubSetCarryUse_i32_Mem:
  case MC6809::SubSetOverflowUse_i32_Mem:
    expandAddSubCarryUse_i32_Mem(Builder, MI, /*IsAdd=*/false);
    break;
  case MC6809::AddSetCarryUse_i32_Reg:
  case MC6809::AddSetOverflowUse_i32_Reg:
    expandAddSubCarryUse_i32_Reg(Builder, MI, /*IsAdd=*/true);
    break;
  case MC6809::SubSetCarryUse_i32_Reg:
  case MC6809::SubSetOverflowUse_i32_Reg:
    expandAddSubCarryUse_i32_Reg(Builder, MI, /*IsAdd=*/false);
    break;
  case MC6809::Compare_i8_Imm:
  case MC6809::Compare_i16_Imm:
  case MC6809::Compare_ptr_Imm:
    wrapStagedCCSources(MI, [&] { expandCompareImm(Builder, MI); });
    break;
  case MC6809::Compare_i8_Mem:
  case MC6809::Compare_i16_Mem:
  case MC6809::Compare_ptr_Mem:
  case MC6809::Compare_i8_MemIndirect:
  case MC6809::Compare_i16_MemIndirect:
    wrapStagedCCSources(MI, [&] { expandCompareIdx(Builder, MI); });
    break;
  case MC6809::Compare_i8_Reg:
  case MC6809::Compare_i16_Reg:
  case MC6809::Compare_ptr_Reg:
    expandCompareReg(Builder, MI);
    break;
  case MC6809::Test_i8_Reg:
  case MC6809::Test_i16_Reg:
    wrapStagedCCSources(MI, [&] { expandTestReg(Builder, MI); });
    break;
  case MC6809::Test_i8_Mem:
  case MC6809::Test_i16_Mem:
    wrapStagedCCSources(MI, [&] { expandTestMem(Builder, MI); });
    break;
  // Fused test-and-branch: split into Test + ConditionalLongBranchRelative.
  case MC6809::TestBranch_i8_Reg:
  case MC6809::TestBranch_i16_Reg:
  case MC6809::TestBranch_i8_Mem:
  case MC6809::TestBranch_i16_Mem:
  // Fused compare-and-branch: split into Compare + ConditionalLongBranchRelative.
  case MC6809::CompareBranch_i8_Imm:
  case MC6809::CompareBranch_i16_Imm:
  case MC6809::CompareBranch_i8_Reg:
  case MC6809::CompareBranch_i16_Reg:
  case MC6809::CompareBranch_i8_Mem:
  case MC6809::CompareBranch_i16_Mem:
  case MC6809::CompareBranch_i8_MemIndirect:
  case MC6809::CompareBranch_i16_MemIndirect:
  case MC6809::CompareBranch_ptr_Imm: // Bug #359: index-domain pointer compare.
  case MC6809::CompareBranch_ptr_Reg: // index-domain reg-reg pointer compare.
  case MC6809::CompareBranch_ptr_Mem: // index-domain pointer-vs-memory compare.
    wrapStagedCCSources(MI, [&] { expandFusedCompareBranch(Builder, MI); });
    break;
  case MC6809::CompareBranch_i32_Imm: {
    // Bug #301 (2026-05-16): native HD6309 i32 fused compare-and-branch.
    // Operand layout (per MC6809CompareBranchBase):
    //   op 0 = i8imm:$cc  (condcode)
    //   op 1 = ACC32:$src (LHS, in AQ or SPILL_Q*N post-RA)
    //   op 2 = i32imm:$imm (RHS)
    //   op 3 = label:$tgt (branch target)
    //
    // Expansion: ensure $aq holds X, then SUBW #lo16 + SBCD #hi16 +
    // LB<cc> $tgt.  Single contiguous expansion — no scheduler can
    // wedge a flag-clobber between the SBCD and the LB<cc> because
    // they are emitted adjacent here and the BB has no other
    // post-RA-expanded MIs scheduled between them.
    MachineFunction &MF = *MI.getMF();
    unsigned CC = MI.getOperand(0).getImm();
    Register SrcReg = MI.getOperand(1).getReg();
    int64_t K = MI.getOperand(2).getImm();
    MachineBasicBlock *TgtMBB = MI.getOperand(3).getMBB();

    // Bug #346: signed GT / LE consume BOTH Z and V (GT = Z==0 && N==V;
    // LE = Z==1 || N!=V). The STQ Z-fix below sets Z from the full 32-bit
    // Q, but a store also CLEARS V — which would corrupt the N==V test the
    // signed conditions rely on. So GT/LE can't use that fix. Instead
    // rewrite them to the equivalent GE/LT against K+1, which depend only
    // on N==V (set correctly by SBCD's combined-borrow result) and need no
    // Z fix at all:  X >s K <=> X >=s K+1 ;  X <=s K <=> X <s K+1.
    // SUBW/SBCD leave Z reflecting only the high half, but GE/LT never read
    // Z, so that staleness is harmless.
    if (CC == MC6809CC::GT || CC == MC6809CC::LE) {
      if (K == 0x7fffffff) {
        // X >s INT32_MAX is always false; X <=s INT32_MAX is always true.
        if (CC == MC6809CC::LE)
          Builder.buildInstr(MC6809::LBRAlb).addMBB(TgtMBB);
        MI.eraseFromParent();
        return true;
      }
      K += 1;
      CC = (CC == MC6809CC::GT) ? MC6809CC::GE : MC6809CC::LT;
    }

    uint16_t KLo = static_cast<uint16_t>(K & 0xFFFF);
    uint16_t KHi = static_cast<uint16_t>((K >> 16) & 0xFFFF);

    // Step 1: ensure LHS is in $aq.
    if (isQSpillReg(SrcReg)) {
      emitSpillLoadInto(Builder, MC6809::AQ, SrcReg, /*ExtraOffset=*/0, MF);
    } else if (SrcReg != MC6809::AQ) {
      llvm_unreachable("CompareBranch_i32_Imm src must be AQ or SPILL_Q*N");
    }

    // Step 2: SUBW #KLo; SBCD #KHi — sets CC.{N,Z,V,C} for X - K.
    Builder.buildInstr(MC6809::SUBWi16).addImm(KLo);
    Builder.buildInstr(MC6809::SBCDi16).addImm(KHi);

    // Step 2b: Bug #317 fix (2026-05-21).  SUBW+SBCD on i32 leaves
    // CC.C correctly reflecting the combined borrow, but CC.Z
    // reflects ONLY SBCD's result (i.e., the high half of Q being
    // zero), not the full 32-bit Q being zero.  For unsigned compare
    // conditions that consume Z (HI / LS / EQ / NE), this corrupts
    // the branch target.
    //
    // Manifest: picolibc's __ultoa_invert(40) returns wrong digit
    // because `if (r > 9)` (where r is loaded into Q's low half =
    // 10) compiles to SUBW #9 + SBCD #0 + BLS — and BLS reads
    // (C | Z).  After SBCD #0 on D=0: Z=1 (D went from 0 to 0).
    // BLS branches incorrectly, skipping the q++ adjustment.
    //
    // Fix: do a STQ-to-scratch (no LDQ needed).  STQ sets CC.N and
    // CC.Z based on the full 32-bit Q (per HD6309 datasheet) and
    // leaves CC.C unchanged.  We discard the stored bytes by
    // releasing the scratch immediately after.  Net cost: 3
    // instructions (LEAS −4, STQ, LEAS +4), no register pressure.
    //
    // Done only when the condcode actually consumes Z — HI / LS /
    // EQ / NE.  Pure-C conditions (HS / LO / etc.) work fine off
    // SBCD's C alone and don't need the fix.
    bool NeedsZFix = (CC == MC6809CC::HI || CC == MC6809CC::LS ||
                      CC == MC6809CC::EQ || CC == MC6809CC::NE);
    if (NeedsZFix) {
      Builder.buildInstr(MC6809::LEASi_o5).addImm(-4).addReg(MC6809::SS);
      Builder.buildInstr(MC6809::STQi_o0)
          .addUse(MC6809::AQ, RegState::Implicit)
          .addReg(MC6809::SS);
      Builder.buildInstr(MC6809::LEASi_o5).addImm(4).addReg(MC6809::SS);
    }

    // Step 3: LB<cc> $tgt — long conditional branch on the i32 result.
    // pickLBlbcVariant chooses LBlbc / LBlbc_NoC / LBlbc_OnlyC based
    // on which CC flags the condcode actually consumes (Bug #206).
    Builder.buildInstr(pickLBlbcVariant(CC))
        .addImm(CC)
        .addMBB(TgtMBB);
    MI.eraseFromParent();
    return true;
  }
  case MC6809::CompareSet_i8_i32_Imm: {
    // Bug #301 (2026-05-16): native HD6309 i32 compare-and-set.
    // Operand layout (per pseudo def in MC6809InstrFamilies.td):
    //   op 0 = ACC8:$dst (the 0/1 byte result)
    //   op 1 = condcode:$cc
    //   op 2 = ACC32:$src (LHS, in AQ or SPILL_Q*N post-RA)
    //   op 3 = i32imm:$imm (RHS)
    //
    // Expansion: ensure $aq holds X, then SUBW #lo16 + SBCD #hi16,
    // then per-condcode branch-free CC bit extraction to AB, then
    // COPY AB to Dst (Dst is ACC8 — AA/AB/AE/AF or SPILL_A*/SPILL_B*).
    MachineFunction &MF = *MI.getMF();
    Register DstReg = MI.getOperand(0).getReg();
    unsigned CC = MI.getOperand(1).getImm();
    Register SrcReg = MI.getOperand(2).getReg();
    int64_t K = MI.getOperand(3).getImm();

    // Bug #346: signed GT/LE consume both Z and V, but the STQ Z-fix below
    // clears V. Rewrite them to GE/LT against K+1 (V/N-only, no Z fix),
    // mirroring CompareBranch_i32_Imm. INT32_MAX edge: K+1 would wrap, so
    // materialise the constant result directly (x >s MAX = 0; x <=s MAX = 1).
    if (CC == MC6809CC::GT || CC == MC6809CC::LE) {
      if (K == 0x7fffffff) {
        uint8_t Val = (CC == MC6809CC::LE) ? 1 : 0;
        Builder.buildInstr(MC6809::LDAi8)
            .addDef(MC6809::AA, RegState::Implicit)
            .addImm(Val);
        Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
        if (DstReg != MC6809::AB)
          copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(), DstReg,
                      MC6809::AB, /*KillSrc=*/true);
        MI.eraseFromParent();
        return true;
      }
      K += 1;
      CC = (CC == MC6809CC::GT) ? MC6809CC::GE : MC6809CC::LT;
    }

    uint16_t KLo = static_cast<uint16_t>(K & 0xFFFF);
    uint16_t KHi = static_cast<uint16_t>((K >> 16) & 0xFFFF);

    // Step 1: ensure LHS is in $aq.
    if (isQSpillReg(SrcReg)) {
      emitSpillLoadInto(Builder, MC6809::AQ, SrcReg, /*ExtraOffset=*/0, MF);
    } else if (SrcReg != MC6809::AQ) {
      llvm_unreachable("CompareSet_i8_i32_Imm src must be AQ or SPILL_Q*N");
    }

    // Step 2: SUBW #KLo; SBCD #KHi — sets CC.{N,Z,V,C} for X - K.
    Builder.buildInstr(MC6809::SUBWi16).addImm(KLo);
    Builder.buildInstr(MC6809::SBCDi16).addImm(KHi);

    // Bug #346: SUBW/SBCD leave CC.Z reflecting only the high 16 bits.
    // Predicates that read Z (EQ/NE/LS/HI — GT/LE were rewritten above to
    // GE/LT, which read only N/V) need a correct full-32 Z. STQ-to-scratch
    // sets N/Z from the full 32-bit Q and leaves C; V is cleared but these
    // predicates don't use V. Same fix as CompareBranch_i32_Imm.
    if (CC == MC6809CC::EQ || CC == MC6809CC::NE || CC == MC6809CC::LS ||
        CC == MC6809CC::HI) {
      Builder.buildInstr(MC6809::LEASi_o5).addImm(-4).addReg(MC6809::SS);
      Builder.buildInstr(MC6809::STQi_o0)
          .addUse(MC6809::AQ, RegState::Implicit)
          .addReg(MC6809::SS);
      Builder.buildInstr(MC6809::LEASi_o5).addImm(4).addReg(MC6809::SS);
    }

    // Step 3: per-condcode branch-free CC bit extraction into AB.
    // CC byte layout: bit 7=E, 6=F, 5=H, 4=I, 3=N, 2=Z, 1=V, 0=C.
    //
    // For each predicate we build a sequence that ends with AB = 0 or
    // 1 reflecting the truth of the predicate.  AA is used as scratch
    // (TFR CC,A; ... ; TFR A,B at the end).
    //
    // Simple single-flag predicates (EQ/NE → Z, UGE/ULT → C):
    //   TFR CC,A
    //   ANDA #<mask>            ; isolate the flag bit
    //   LSRA × N                ; shift to bit 0
    //   [EORA #1]                ; optional invert for NE / UGE
    //   TFR A,B
    //
    // Multi-flag predicates (ULE = C|Z; UGT = ~(C|Z); SLT = N^V;
    //   SGE = ~(N^V); SLE = Z|(N^V); SGT = ~(Z|(N^V))):
    //   need additional masking + combination logic.
    Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AA).addUse(MC6809::CC);
    auto andai = [&](uint8_t m) {
      Builder.buildInstr(MC6809::ANDAi8)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit)
          .addImm(m);
    };
    auto lsraN = [&](int n) {
      for (int i = 0; i < n; ++i)
        Builder.buildInstr(MC6809::LSRAa)
            .addDef(MC6809::AA, RegState::Implicit)
            .addUse(MC6809::AA, RegState::Implicit);
    };
    auto eorai = [&](uint8_t m) {
      Builder.buildInstr(MC6809::EORAi8)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit)
          .addImm(m);
    };
    switch (CC) {
    case MC6809CC::EQ:  // Z=1
      andai(0x04); lsraN(2); break;
    case MC6809CC::NE:  // Z=0
      andai(0x04); lsraN(2); eorai(0x01); break;
    case MC6809CC::CS:  // C=1 (also ULO)
      andai(0x01); break;
    case MC6809CC::CC:  // C=0 (also UHS)
      andai(0x01); eorai(0x01); break;
    case MC6809CC::LS: {
      // C | Z — OR bit 0 (C) with bit 2 (Z>>2).  Build via:
      //   B := A;  B>>=2;  B|=A;  B&=1
      // (HD6309 ORR is reg-to-reg.)
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01);
      break;
    }
    case MC6809CC::HI: {
      // NOT (C | Z) — same as LS then EOR #1
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01); eorai(0x01);
      break;
    }
    case MC6809CC::LT: {
      // N XOR V — N is bit 3, V is bit 1.  Mask both, shift V to bit
      // 2 alignment, XOR, shift to bit 0:
      //   AA = CC; AA &= 0x0A;  (keeps N at bit 3, V at bit 1)
      //   B := A; B>>=2;  (B now has N at bit 1, V at bit -1 = lost)
      //   Hmm — different approach: align both to bit 0 first.
      //   A := CC; A &= 0x0A;  A>>=1;  (V now at bit 0, N at bit 2)
      //   B := A; B>>=2;  (B now has N at bit 0)
      //   A ^= B;  A &= 1;  (bit 0 = V^N)
      andai(0x0A); lsraN(1);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::EORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01);
      break;
    }
    case MC6809CC::GE: {
      // NOT (N XOR V) — same as LT then EOR #1
      andai(0x0A); lsraN(1);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::EORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01); eorai(0x01);
      break;
    }
    case MC6809CC::LE: {
      // Z | (N XOR V) — combine LT logic with Z bit.
      //   AA = CC; B := CC;
      //   AA &= 0x0A;  AA>>=1;   ; A: bit2=N, bit0=V
      //   tmp := A; tmp>>=2;     ; tmp bit0=N
      //   A ^= tmp;  A &= 1;     ; A bit0 = N^V
      //   B &= 0x04; B>>=2;      ; B bit0 = Z
      //   A |= B;  A &= 1;
      // Using TFRp to copy CC into both A and B at start:
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::CC);
      andai(0x0A); lsraN(1);
      // tmp via stack (no third byte reg accessible without clobbering).
      // Simpler: re-use B by saving it before.  Save B to AE temporarily?
      // AE only exists on HD6309 — fine.
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AE).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRAa)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRAa)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit);
      // AA bit 0 = N (shifted from bit 2 of the AND-masked CC>>1).
      // Use EORR to XOR AE (bit 0=V) into AA.
      Builder.buildInstr(MC6809::EORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AE).addUse(MC6809::AA);
      andai(0x01);
      // Now AA bit 0 = N^V.  AB still has CC.  Mask Z out of B and shift.
      Builder.buildInstr(MC6809::ANDBi8)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit)
          .addImm(0x04);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01);
      break;
    }
    case MC6809CC::GT: {
      // NOT (Z | (N XOR V)) — same as LE then EOR #1
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::CC);
      andai(0x0A); lsraN(1);
      Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AE).addUse(MC6809::AA);
      Builder.buildInstr(MC6809::LSRAa)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRAa)
          .addDef(MC6809::AA, RegState::Implicit)
          .addUse(MC6809::AA, RegState::Implicit);
      Builder.buildInstr(MC6809::EORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AE).addUse(MC6809::AA);
      andai(0x01);
      Builder.buildInstr(MC6809::ANDBi8)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit)
          .addImm(0x04);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::LSRBa)
          .addDef(MC6809::AB, RegState::Implicit)
          .addUse(MC6809::AB, RegState::Implicit);
      Builder.buildInstr(MC6809::ORRp)
          .addDef(MC6809::AA)
          .addUse(MC6809::AB).addUse(MC6809::AA);
      andai(0x01); eorai(0x01);
      break;
    }
    default:
      llvm_unreachable("CompareSet_i8_i32_Imm: unhandled condcode");
    }

    // Step 4: TFR A,B then COPY B to Dst.
    Builder.buildInstr(MC6809::TFRp).addDef(MC6809::AB).addUse(MC6809::AA);
    if (DstReg != MC6809::AB) {
      copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(),
                  DstReg, MC6809::AB, /*KillSrc=*/true);
    }
    MI.eraseFromParent();
    return true;
  }
  case MC6809::Copy8:
  case MC6809::Copy16:
    MI.setDesc(Builder.getTII().get(MC6809::TFRp));
    break;
  case MC6809::BlockCopy_Inc_Inc:
  case MC6809::BlockCopy_Dec_Dec:
  case MC6809::BlockCopy_Inc_Stay:
  case MC6809::BlockCopy_Stay_Inc: {
    // Expand the legalizer's BlockCopy_* pseudo into the real HD6309 TFM
    // instruction it represents. Operand layout (set by tryTFMBlockCopy):
    // src_wb(def), dst_wb(def), src(use, REGTFM), dst(use, REGTFM), with
    // implicit-def/use AW. The two write-back defs are tied to the pointer uses
    // (the TFM clobbers the registers via post-increment) and are dead here;
    // read the tied uses at operands 2/3. The byte count was already loaded
    // into AW by the legalizer, so the only work is to emit the TFM reading the
    // *allocated* src/dst registers — whichever IX/IY assignment regalloc
    // chose. The mode picks the increment behaviour: Inc_Inc/TFM0pp = ascending
    // memcpy, Dec_Dec/TFM1pp = descending memmove, Stay_Inc/TFM3pp = memset
    // fill (src holds the byte and stays put while dst sweeps the range).
    Register Src = MI.getOperand(2).getReg();
    Register Dst = MI.getOperand(3).getReg();
    unsigned TFMOpc;
    switch (MI.getOpcode()) {
    case MC6809::BlockCopy_Inc_Inc:  TFMOpc = MC6809::TFM0pp; break;
    case MC6809::BlockCopy_Dec_Dec:  TFMOpc = MC6809::TFM1pp; break;
    case MC6809::BlockCopy_Inc_Stay: TFMOpc = MC6809::TFM2pp; break;
    case MC6809::BlockCopy_Stay_Inc: TFMOpc = MC6809::TFM3pp; break;
    default: llvm_unreachable("unreachable");
    }
    // TFM auto-modifies the incremented/decremented pointer register(s);
    // declare those writes as implicit defs or post-expansion passes assume
    // the pointers survive the transfer (PostRASpillOpt's slot-mirror then
    // deletes the reload after the TFM — the memchr stale-IY miscompile).
    // The "stay" register of TFM2/TFM3 really is left unchanged.
    auto TFMI = Builder.buildInstr(TFMOpc).addUse(Src).addUse(Dst);
    if (TFMOpc != MC6809::TFM3pp) // src stays put only in `tfm r,r+`
      TFMI.addDef(Src, RegState::Implicit);
    if (TFMOpc != MC6809::TFM2pp) // dst stays put only in `tfm r+,r`
      TFMI.addDef(Dst, RegState::Implicit);
    TFMI.cloneMemRefs(MI);
    MI.eraseFromParent();
    return true;
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

// Bug #149 Phase 1: shrink an indexed-immediate opcode to its narrowest
// valid form for the given offset. Iterates the eight (Reg, OffsetLen)
// → Opcode tables; for any table containing CurrentOpcode at some
// (Reg, CurLen), tries CandidateLen in {0, 5, 8} (each only if it both
// fits the offset value and is strictly smaller than CurLen) and returns
// the first hit. Cost is O(N) per call across ~640 entries, but the call
// site (MC6809FinalLowering::relaxOffsets) only triggers on indexed MIs.
std::pair<unsigned, int>
MC6809InstrInfo::getRelaxedIdxOpcode(unsigned CurrentOpcode,
                                     int64_t Offset) const {
  // The eight tables that share the {Reg, OffsetLen} -> Opcode shape.
  // Order matches MC6809InstrInfo's constructor.
  const DenseMap<RegPlusOffsetLen, unsigned> *Tables[] = {
      &LEAPtrAddImmOpcode,    &LoadIdxImmOpcode,       &StoreIdxImmOpcode,
      &AddIdxImmOpcode,       &AddCarryIdxImmOpcode,   &SubIdxImmOpcode,
      &SubBorrowIdxImmOpcode, &CompareIdxImmOpcode,
  };

  // Required size to encode the offset (0 / 5 / 8 / 16 / 256-too-big).
  int Need = offsetSizeInBitsForValue(Offset);
  if (Need < 0 || Need > 16)
    return {0, -1};

  for (const auto *Table : Tables) {
    // Find which (Reg, CurLen) maps to CurrentOpcode in this table.
    Register Reg;
    int CurLen = -2;
    for (const auto &KV : *Table) {
      if (KV.second == CurrentOpcode) {
        Reg = KV.first.Reg;
        CurLen = KV.first.OffsetLen;
        break;
      }
    }
    if (CurLen == -2)
      continue;

    // The "-1" key in these tables is a fallback alias for the _o16
    // entry (see MC6809InstrInfo ctor). Treat it as 16 for comparison
    // so we don't try to "relax" downward FROM the fallback.
    int EffCur = (CurLen < 0) ? 16 : CurLen;

    // Try strictly smaller candidates that fit the offset.
    static const int Candidates[] = {0, 5, 8};
    for (int Cand : Candidates) {
      if (Cand >= EffCur)
        break;
      if (Need > Cand)
        continue;
      auto It = Table->find({Reg, Cand});
      if (It != Table->end())
        return {It->second, Cand};
    }
    // Found the table but no smaller form available: nothing to do.
    return {0, -1};
  }
  return {0, -1};
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
      emitSpillLoadInto(Builder, StageReg, OrigSpillReg, /*ExtraOffset=*/0, MF);
      // Rewrite base to IY.
      IndexOp = MachineOperand::CreateReg(StageReg, false);
    }
    // Rewrite result to IY (for both modify-in-place and pure definition).
    IndexReg = MachineOperand::CreateReg(StageReg, true);
    MI.getOperand(0).setReg(StageReg);
  }

  // Check register offset first (LEAPtrAdd_Reg8/Reg16) — offsetSizeInBits
  // crashes on register operands.
  Register StagedOffReal;
  if (OffsetOp.isReg()) {
    Register OffsetReg = OffsetOp.getReg();
    // If the offset is a spill or imaginary register, materialize first,
    // preserving the staging real around the LEA (see pushStagingReg) --
    // the pseudo only declares the pointer def, so regalloc may have a
    // live value in the accumulator the materialization loads into.
    if (needsMaterialization(OffsetReg)) {
      MachineFunction &MF = *MI.getMF();
      MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
      StagedOffReal = getPhysRegFor(OffsetReg);
      pushStagingReg(PreBuilder, StagedOffReal);
      OffsetReg = materializeReg(PreBuilder, OffsetReg, MF);
      OffsetOp = MachineOperand::CreateReg(OffsetReg, false);
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

  // Bug #365: setDesc swapped the opcode but does NOT materialise the new
  // opcode's implicit operands, so the implicit-def $z that LEAX/LEAY declare
  // (Defs = [I?, Z]) was dropped — leaving flag-liveness blind to the hardware
  // Z-clobber. Re-add it. LEAS/LEAU declare no Z in their Defs, so gating on
  // the concrete descriptor preserves the hardware asymmetry automatically.
  // Not marked dead: deadness isn't known here, a non-dead def is always sound
  // for liveness, and it lets elideCompareZero reuse a LEAX's Z for a BEQ/BNE.
  if (MI.getDesc().hasImplicitDefOfPhysReg(MC6809::Z))
    MI.addOperand(
        MachineOperand::CreateReg(MC6809::Z, /*isDef=*/true, /*isImp=*/true));

  // If the original register was a spill, store staging reg back to spill slot.
  if (OrigSpillReg.isValid()) {
    Register StageReg = MC6809::IY;  // Must match the staging register above
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock::iterator After = std::next(MI.getIterator());
    MachineIRBuilder PostBuilder(*MI.getParent(), After);
    emitSpillStoreFrom(PostBuilder, StageReg, OrigSpillReg, /*ExtraOffset=*/0, MF);
  }

  // Restore the offset's staging real (PULS does not touch the Z the LEA
  // just defined for elideCompareZero).
  if (StagedOffReal.isValid()) {
    MachineBasicBlock::iterator After = std::next(MI.getIterator());
    MachineIRBuilder PostBuilder(*MI.getParent(), After);
    pullStagingReg(PostBuilder, StagedOffReal);
  }
}

void MC6809InstrInfo::expandImm(ContextImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto operandCount = MI.getNumExplicitOperands();
  auto DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg)) {
    // Preserve the staging real around the whole read-modify-write window
    // (see pushStagingReg).
    pushStagingReg(Builder, getPhysRegFor(DestReg));
    Register RealReg = materializeReg(Builder, DestReg, MF);
    MI.getOperand(0).setReg(RealReg);
    MI.getOperand(0).setIsDead(false); // demat below reads the staging real
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
  // Bug #272 Phase B Scope A followup: AND with 0 is an annihilator, not an
  // identity — `X & 0 == 0` regardless of X.  ANDA/ANDB #0 emits an
  // implicit USE of $aa / $ab that the post-Scope-A/B/C verifier can flag
  // as undef-reading (because the dst's prior value isn't actually
  // reachable in all paths).  CLRA / CLRB is strictly better: same NZV
  // result, 1 byte instead of 2, and no read of the input register.
  // Manifest at test-double-free.c:133 -Og on the high-byte clear of an
  // i8-zext-to-i16 return value.
  if (Context.Opcode == &ANDImmediateOpcode && Val == 0) {
    unsigned ClrOpc = 0;
    switch (DestReg) {
    // Bug #311 Phase 1 step 1.5: *LSB physregs retired.
    case MC6809::AA: ClrOpc = MC6809::CLRAa; break;
    case MC6809::AB: ClrOpc = MC6809::CLRBa; break;
    case MC6809::AE: ClrOpc = MC6809::CLREa; break;
    case MC6809::AF: ClrOpc = MC6809::CLRFa; break;
    case MC6809::AD: ClrOpc = MC6809::CLRDa; break;
    case MC6809::AW: ClrOpc = MC6809::CLRWa; break;
    default: break;
    }
    if (ClrOpc != 0) {
      Builder.buildInstr(ClrOpc).addDef(DestReg, RegState::Implicit);
      // Store result back BEFORE erasing MI (same as the post-op path).
      if (needsMaterialization(OrigDest)) {
        MachineBasicBlock &MBB = *MI.getParent();
        auto NextIt = std::next(MachineBasicBlock::iterator(MI));
        MachineIRBuilder StoreBuilder(MBB, NextIt);
        // dematerializeReg, not emitSpillStore: the destination may be a
        // DP imaginary, which emitSpillStore cannot address.
        dematerializeReg(StoreBuilder, DestReg, OrigDest, MF);
        pullStagingReg(StoreBuilder, DestReg);
      }
      MI.eraseFromParent();
      return;
    }
  }
  if (Context.NeverSkip || Val != Context.IdentityValue) {
    auto OpcodePair = Context.Opcode->find(DestReg);
    if (OpcodePair == Context.Opcode->end()) {
      // HD6309 cheat path: hardware has no AND/OR/XOR/SBC variants for the
      // page-3 sub-registers (E/F/W). When regalloc picks AE/AF/AW for one
      // of those ops, route through the corresponding page-1 register via
      // an EXG bracket. AE↔AA (byte halves of AW), AF↔AB likewise; AW↔AD
      // for the 16-bit case that was already handled. Bug #161.
      Register CheatReg = MC6809::NoRegister;
      switch (DestReg) {
      case MC6809::AE: CheatReg = MC6809::AA; break;
      case MC6809::AF: CheatReg = MC6809::AB; break;
      case MC6809::AW: CheatReg = MC6809::AD; break;
      default: break;
      }
      if (CheatReg != MC6809::NoRegister) {
        OpcodePair = Context.Opcode->find(CheatReg);
        assert((OpcodePair != Context.Opcode->end()) && "Cheat-target register lacks an immediate-form opcode");
        // Bug #161 round 6: emit the EXG-cheat as plain serial MIs (no
        // BUNDLE wrap). The original AW-only path used finalizeBundle
        // for atomicity, but that diverges MachineBasicBlock::size()
        // (instr-based) from the post-RA scheduler's bundle-aware
        // iteration count, surfacing as the Count == 0 mismatch
        // assertion in PostRASchedulerList.cpp:341. The implicit-defs
        // of CheatReg / DestReg already chain the three MIs through
        // liveness without needing the bundle: nothing reorders them.
        // Bug #271 cat-1: the cheat scratch register (AA / AB / AD)
        // holds no meaningful value before the first EXG — only DestReg
        // is live. Mark the scratch read as RegState::Undef so the
        // verifier doesn't flag it as "Using an undefined physical
        // register". After the first EXG and the operation, DestReg
        // is the scratch side and is undef going into the second EXG;
        // mark its read Undef there too.
        Builder.buildInstr(MC6809::EXGp).addDef(CheatReg).addDef(DestReg).addUse(CheatReg, RegState::Undef).addUse(DestReg);
        Builder.buildInstr(OpcodePair->getSecond()).addDef(CheatReg, RegState::Implicit).addImm(Val);
        Builder.buildInstr(MC6809::EXGp).addDef(DestReg).addDef(CheatReg).addUse(DestReg, RegState::Undef).addUse(CheatReg);
      } else {
        llvm_unreachable("Cannot find machine instruction with this immediate operand");
      }
    } else {
      Builder.buildInstr(OpcodePair->getSecond()).addDef(DestReg, RegState::Implicit).addImm(Val);
    }
  }
  // Store result back BEFORE erasing MI.
  if (needsMaterialization(OrigDest)) {
    MachineBasicBlock &MBB = *MI.getParent();
    auto NextIt = std::next(MachineBasicBlock::iterator(MI));
    MachineIRBuilder StoreBuilder(MBB, NextIt);
    dematerializeReg(StoreBuilder, DestReg, OrigDest, MF);
    pullStagingReg(StoreBuilder, DestReg);
  }
  MI.eraseFromParent();
}

static unsigned indirectSiblingOf(unsigned Opc);
static bool isMemIndirectPseudo(unsigned Opc);

void MC6809InstrInfo::expandIdxImm(ContextIndexImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const {
  auto operandCount = MI.getNumExplicitOperands();
  // P3b: indirect-indexed consumer (`op [n,r]`) -- build the indirect-sibling
  // opcode (the memory operand is `*(mem[r+n])`), exactly as expandLoadIdx
  // swaps a load to its `[n,r]` form. Applies at every build site below,
  // including the AW/E-F cheat ops (their memory access is the cheat's op).
  const bool Indirect = isMemIndirectPseudo(MI.getOpcode());
  auto IndOpc = [&](unsigned O) {
    if (!Indirect)
      return O;
    unsigned I = indirectSiblingOf(O);
    assert(I && "no indirect-indexed sibling for this opcode (add an IND3 row)");
    return I;
  };
  auto DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg)) {
    // Preserve the staging real around the read-modify-write window; the
    // memory operand may be S-relative and must see the shifted S.
    Register StageReal = getPhysRegFor(DestReg);
    pushStagingReg(Builder, StageReal);
    compensateSSOperands(MI, stagingPushSize(StageReal));
    Register RealReg = materializeReg(Builder, DestReg, MF);
    MI.getOperand(0).setReg(RealReg);
    MI.getOperand(0).setIsDead(false); // demat below reads the staging real
    // Also fix the tied source operand (operand 1 for most arith pseudos).
    if (operandCount >= 3 && MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == DestReg)
      MI.getOperand(1).setReg(RealReg);
    DestReg = RealReg;
  }
  // Materialize a spilled index base into IY (the P3a consumer fold can keep the
  // base pointer live to the arith op; under pressure it can spill to an
  // index-spill reg, which has no real encoding). Mirrors expandStoreIdx /
  // expandCompareIdx. DestReg is an accumulator here, so IY is free.
  if (isIndexSpillReg(MI.getOperand(operandCount - 2).getReg())) {
    Register SpillReg = MI.getOperand(operandCount - 2).getReg();
    Register StageReg = MC6809::IY;
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    emitSpillLoadInto(PreBuilder, StageReg, SpillReg, /*ExtraOffset=*/0, MF);
    MI.getOperand(operandCount - 2).setReg(StageReg);
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
      // Bug #271 cat-1: AD is the cheat-scratch register here, holding
      // no meaningful value before the first EXG. Mark the AD read Undef
      // so the verifier doesn't flag it. After the first EXG + the
      // indexed op, DestReg is the scratch side; mark its read Undef
      // on the second EXG too.
      B = Builder.buildInstr(MC6809::EXGp).addDef(MC6809::AD).addDef(DestReg).addUse(MC6809::AD, RegState::Undef).addUse(DestReg);
      auto Instr = Builder.buildInstr(IndOpc(OpcodePair->getSecond())).addDef(MC6809::AD, RegState::Implicit);
      if (OffsetSize == 0)
        Instr.addReg(IndexReg);
      else
        Instr.addImm(Offset).addReg(IndexReg);
      E = Builder.buildInstr(MC6809::EXGp).addDef(DestReg).addDef(MC6809::AD).addUse(DestReg, RegState::Undef).addUse(MC6809::AD);
      auto Bundler = MIBundleBuilder(MBB, B, ++E);
      finalizeBundle(MBB, Bundler.begin(), Bundler.end());
      LLVM_DEBUG(for (auto &I : Bundler) {
        I.dump();
      });
    } else if (DestReg == MC6809::AE || DestReg == MC6809::AF) {
      // Bug #382: HD6309 E/F have no indexed memory-arith encoding (the IdxImm
      // map has only A/B/D), so a byte _Mem/_Pull op whose tied accumulator
      // regalloc placed in E/F would otherwise be unselectable. Stage through B:
      // copy E/F->B, do the B-form indexed op, copy B->E/F. COPY (not EXG) never
      // reads B's incoming value, so this is verifier-clean when B is dead; when
      // B is *live* its value must be preserved, so wrap in pshs b / puls b.
      MachineBasicBlock &MBB = *MI.getParent();
      const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
      LivePhysRegs LiveRegs(TRI);
      LiveRegs.addLiveIns(MBB);
      SmallVector<std::pair<MCPhysReg, const MachineOperand *>, 4> Clobbers;
      for (auto It = MBB.begin(); It != MI.getIterator(); ++It) {
        Clobbers.clear();
        LiveRegs.stepForward(*It, Clobbers);
      }
      bool BLive = LiveRegs.contains(MC6809::AB);

      // pshs b shifts S down by 1, so an S-relative offset (and thus the chosen
      // opcode size) must be bumped by 1 while B sits on the stack. Use o8 for
      // the bumped offset (always encodable; o5 might no longer fit).
      int UseOffset = Offset;
      int UseSize = OffsetSize;
      if (BLive && IndexReg == MC6809::SS) {
        UseOffset = Offset + 1;
        UseSize = (UseOffset == 0) ? 0 : isInt<8>(UseOffset) ? 8 : 16;
      }
      RegPlusOffsetLen LookupB{MC6809::AB, UseSize};
      OpcodePair = Context.Opcode->find(LookupB);
      assert((OpcodePair != Context.Opcode->end()) && "B-form indexed arith must exist");

      if (BLive)
        Builder.buildInstr(MC6809::PSHSs).addUse(MC6809::AB); // pshs b
      Builder.buildCopy(Register(MC6809::AB), Register(DestReg)); // B = E/F value
      auto Instr = Builder.buildInstr(IndOpc(OpcodePair->getSecond())).addDef(MC6809::AB, RegState::Implicit);
      if (UseSize == 0)
        Instr.addReg(IndexReg);
      else
        Instr.addImm(UseOffset).addReg(IndexReg);
      Builder.buildCopy(Register(DestReg), Register(MC6809::AB)); // E/F = result
      if (BLive)
        Builder.buildInstr(MC6809::PULSs).addDef(MC6809::AB); // puls b
    } else
      llvm_unreachable("Cannot find machine instruction with these immediate indexed operands");
  } else {
    auto Instr = Builder.buildInstr(IndOpc(OpcodePair->getSecond())).addDef(DestReg, RegState::Implicit);
    if (OffsetSize == 0)
      Instr.addReg(IndexReg);
    else
      Instr.addImm(Offset).addReg(IndexReg);
  }
  // Store result back BEFORE erasing MI.
  if (needsMaterialization(OrigDest)) {
    MachineBasicBlock &MBB = *MI.getParent();
    auto NextIt = std::next(MachineBasicBlock::iterator(MI));
    MachineIRBuilder StoreBuilder(MBB, NextIt);
    dematerializeReg(StoreBuilder, DestReg, OrigDest, MF);
    pullStagingReg(StoreBuilder, DestReg);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCarryImm16(bool IsAdd, MachineIRBuilder &Builder,
                                       MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    // 6309 has ADCD/SBCD — use the carry-chain members which have
    // NeverSkip=true (bug #93 structural cleanup).
    expandImm(IsAdd ? AddCarryImm : SubBorrowImm, Builder, MI);
    return;
  }
  // 6809: split 16-bit carry immediate into two 8-bit operations.
  // ADCB #lo / ADCA #hi  or  SBCB #lo / SBCA #hi
  auto DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  bool StagedDst = needsMaterialization(DestReg);
  // PSHS/PULS and the materializing loads leave C intact, so the incoming
  // carry survives the preserve bracket.
  if (StagedDst)
    pushStagingReg(Builder, getPhysRegFor(DestReg));
  DestReg = materializeReg(Builder, DestReg, MF);
  auto operandCount = MI.getNumExplicitOperands();
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int Lo = Val & 0xFF;
  int Hi = (Val >> 8) & 0xFF;
  unsigned AdcbOpc = IsAdd ? MC6809::ADCBi8 : MC6809::SBCBi8;
  unsigned AdcaOpc = IsAdd ? MC6809::ADCAi8 : MC6809::SBCAi8;
  Builder.buildInstr(AdcbOpc).addDef(MC6809::AB, RegState::Implicit).addImm(Lo);
  Builder.buildInstr(AdcaOpc).addDef(MC6809::AA, RegState::Implicit).addImm(Hi);
  dematerializeReg(Builder, DestReg, OrigDest, MF);
  if (StagedDst)
    pullStagingReg(Builder, DestReg);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCarryMem16(bool IsAdd, MachineIRBuilder &Builder,
                                       MachineInstr &MI) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309()) {
    expandIdxImm(IsAdd ? AddCarryIdxImm : SubBorrowIdxImm, Builder, MI);
    return;
  }
  auto DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  bool StagedDst = needsMaterialization(DestReg);
  if (StagedDst) {
    // Preserve bracket (C survives -- see expandCarryImm16); the memory
    // operand may be S-relative and must see the shifted S.
    Register StageReal = getPhysRegFor(DestReg);
    pushStagingReg(Builder, StageReal);
    compensateSSOperands(MI, stagingPushSize(StageReal));
    DestReg = materializeReg(Builder, DestReg, MF);
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
  dematerializeReg(Builder, DestReg, OrigDest, MF);
  if (StagedDst)
    pullStagingReg(Builder, DestReg);
  MI.eraseFromParent();
}

// Bug #297 (2026-05-15) commit 2/6: native HD6309 i32 ADD/SUB, _Mem form.
//
// Pseudo shape (from MC6809Arithmetic / MC6809ArithmeticCarry):
//   %dst:ACC32 = Add_i32_Mem %src:ACC32(tied), INDEX16:$idx, unknown:$offset
// The pseudo's `let Constraints = "$dst = $src"` makes operand 0 a tied
// def-use; the explicit operands after $src are the index reg and the
// offset (immediate, possibly typed as CImm by some patterns).
//
// HD6309 lowering (assumes the operand is materialised into physical AQ;
// SPILL_Q*N sources go through materializeReg → emitSpillLoad which
// emits LDQ from the slot, and dematerializeReg → emitSpillStore for the
// store-back via STQ):
//   ADDW <off+2>, $idx   ; low word (AW = sub_lo_word of AQ) + memory low
//                          ; (in big-endian memory, low word is at
//                          ; address+2..3); sets CC.C.
//   ADCD <off+0>, $idx   ; high word (AD = sub_hi_word of AQ) + memory
//                          ; high (at address+0..1) + carry-in from
//                          ; ADDW.
//
// SubSetCarry_i32 and SubSetOverflow_i32 share the same emission as
// Sub_i32 (mirrors the bug #147 sharing of AddSetCarry vs AddSetOverflow
// for the i8/i16 dispatch — same hardware instructions; the pseudo
// opcode is preserved only as metadata for the CC.C / CC.V flag-chain
// tracker).
void MC6809InstrInfo::expandAddSub_i32_Mem(MachineIRBuilder &Builder,
                                            MachineInstr &MI,
                                            bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #297: i32 ADD/SUB expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg))
    DestReg = materializeReg(Builder, DestReg, MF);
  // After materialisation the value lives in physical $aq (or its
  // sub-regs AD high / AW low).  ACC32-class operands can ONLY be $aq
  // post-Phase-B-revert (AQc is still `{AQ}`), but ACC32 itself
  // includes SPILL_Q0..31 so a spilled value is loaded into AQ here.
  assert(DestReg == MC6809::AQ &&
         "Bug #297: i32 ADD/SUB requires AQ for the ALU-side operations");

  // Operand layout for _Mem (after the tied $dst=$src pair which counts
  // as one explicit operand):
  //   op 0: $dst (def-use, tied — ACC32)
  //   op 1: $idx (INDEX16 register)
  //   op 2: $offset (immediate, possibly CImm — see Bug #272 Phase B
  //                  CImm-vs-Imm note in expandStoreIdx/Load_i32_Mem).
  unsigned NumOps = MI.getNumExplicitOperands();
  Register IndexReg = MI.getOperand(NumOps - 2).getReg();
  MachineOperand OffsetOp = MI.getOperand(NumOps - 1);
  int Offset = OffsetOp.isImm() ? OffsetOp.getImm()
                                : int(OffsetOp.getCImm()->getSExtValue());
  // Big-endian i32 in memory: byte 0 = msb of high word (at offset+0),
  // byte 2 = msb of low word (at offset+2).
  int OffsetLo = Offset + 2;
  int OffsetHi = Offset;
  int OffsetLoSize = offsetSizeInBitsForValue(OffsetLo);
  int OffsetHiSize = offsetSizeInBitsForValue(OffsetHi);

  // Look up the page-2 16-bit opcodes:
  //   IsAdd → ADDW (low, no carry-in) + ADCD (high, carry-in).
  //   !IsAdd → SUBW (low, no borrow-in) + SBCD (high, borrow-in).
  auto &LowMap = IsAdd ? AddIdxImmOpcode : SubIdxImmOpcode;
  auto &HighMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
  auto LowLookup = LowMap.find({MC6809::AW, OffsetLoSize});
  auto HighLookup = HighMap.find({MC6809::AD, OffsetHiSize});
  assert(LowLookup != LowMap.end() && HighLookup != HighMap.end() &&
         "Bug #297: missing ADDW/ADCD/SUBW/SBCD opcode entry");

  auto LowInstr =
      Builder.buildInstr(LowLookup->getSecond())
          .addDef(MC6809::AW, RegState::Implicit);
  if (OffsetLoSize == 0)
    LowInstr.addReg(IndexReg);
  else
    LowInstr.addImm(OffsetLo).addReg(IndexReg);

  auto HighInstr =
      Builder.buildInstr(HighLookup->getSecond())
          .addDef(MC6809::AD, RegState::Implicit);
  if (OffsetHiSize == 0)
    HighInstr.addReg(IndexReg);
  else
    HighInstr.addImm(OffsetHi).addReg(IndexReg);

  if (needsMaterialization(OrigDest))
    dematerializeReg(Builder, DestReg, OrigDest, MF);
  MI.eraseFromParent();
}

// Static-stack sibling of expandAddSub_i32_Mem. The pseudo carries a
// TI_STATIC_STACK target index (assigned in eliminateFrameIndex when its
// _Mem form's frame index was moved to the static frame) in place of the
// index-register + offset pair. The carry chain is identical — ADDW/ADCD
// (SUBW/SBCD) against the two 16-bit halves — but each half is reached by an
// extended (or PC-relative under PIC) access to static_stack + base + N, with
// the big-endian +2 (low word) / +0 (high word) split applied to the base.
void MC6809InstrInfo::expandAddSub_i32_Sym(MachineIRBuilder &Builder,
                                            MachineInstr &MI,
                                            bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "i32 ADD/SUB static-stack expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg))
    DestReg = materializeReg(Builder, DestReg, MF);
  assert(DestReg == MC6809::AQ &&
         "i32 ADD/SUB static-stack requires AQ for the ALU-side operations");

  // Operand layout after the tied $dst=$src pair: the last operand is the
  // TI_STATIC_STACK target index. Its per-function byte offset lives in the
  // operand's offset field (the second addTargetIndex argument in
  // eliminateFrameIndex); getIndex() would return the TI_STATIC_STACK kind.
  unsigned NumOps = MI.getNumExplicitOperands();
  int Base = MI.getOperand(NumOps - 1).getOffset();
  bool IsPIC = MF.getTarget().isPositionIndependent();

  // Big-endian i32 in memory: low word at base+2, high word at base+0. Reuse
  // the indexed ADDW/ADCD (SUBW/SBCD) opcodes with an o16 offset size, then
  // map them to their extended/PC-relative static-stack variants — the offset
  // size only picks the widest indexed form to translate.
  auto &LowMap = IsAdd ? AddIdxImmOpcode : SubIdxImmOpcode;
  auto &HighMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
  auto LowLookup = LowMap.find({MC6809::AW, 16});
  auto HighLookup = HighMap.find({MC6809::AD, 16});
  assert(LowLookup != LowMap.end() && HighLookup != HighMap.end() &&
         "missing ADDW/ADCD/SUBW/SBCD opcode entry");

  Builder.buildInstr(getStaticStackOpcode(LowLookup->getSecond(), IsPIC))
      .addDef(MC6809::AW, RegState::Implicit)
      .addTargetIndex(MC6809::TI_STATIC_STACK, Base + 2);
  Builder.buildInstr(getStaticStackOpcode(HighLookup->getSecond(), IsPIC))
      .addDef(MC6809::AD, RegState::Implicit)
      .addTargetIndex(MC6809::TI_STATIC_STACK, Base + 0);

  if (needsMaterialization(OrigDest))
    dematerializeReg(Builder, DestReg, OrigDest, MF);
  MI.eraseFromParent();
}

// Bug #297 commit 2.1/6 (2026-05-15): native HD6309 i32 ADD/SUB, _Imm form.
//
// Pseudo shape:
//   %dst:ACC32 = Add_i32_Imm %src:ACC32(tied), i32imm:$val
//
// Lowering: split the 32-bit immediate into two 16-bit halves and emit
//   ADDW #imm_lo  ; AW += low_word, sets CC.C
//   ADCD #imm_hi  ; AD += high_word + CC.C
// (SUBW + SBCD for IsAdd=false).
//
// Big-endian convention is irrelevant for immediates (no memory order);
// arithmetic order is fixed: low 16 bits first to set carry, high 16
// bits second to consume it.
void MC6809InstrInfo::expandAddSub_i32_Imm(MachineIRBuilder &Builder,
                                            MachineInstr &MI,
                                            bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #297: i32 ADD/SUB expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg))
    DestReg = materializeReg(Builder, DestReg, MF);
  assert(DestReg == MC6809::AQ &&
         "Bug #297: i32 ADD/SUB Imm requires AQ for the ALU-side ops");

  unsigned NumOps = MI.getNumExplicitOperands();
  MachineOperand ValOp = MI.getOperand(NumOps - 1);
  int64_t Val = ValOp.isImm() ? ValOp.getImm()
                              : ValOp.getCImm()->getSExtValue();
  int Lo = int(Val & 0xFFFF);
  int Hi = int((Val >> 16) & 0xFFFF);

  // Low half opcode (ADDW / SUBW) — touches AW; sets CC.C.
  auto LowMap = IsAdd ? AddImmediateOpcode : SubImmediateOpcode;
  auto LowLookup = LowMap.find({MC6809::AW});
  // High half opcode (ADCD / SBCD) — touches AD; consumes CC.C.
  auto &HighMap = IsAdd ? AddCarryImmediateOpcode : SubBorrowImmediateOpcode;
  auto HighLookup = HighMap.find({MC6809::AD});
  assert(LowLookup != LowMap.end() && HighLookup != HighMap.end() &&
         "Bug #297: missing ADDW/ADCD/SUBW/SBCD immediate opcode entry");

  Builder.buildInstr(LowLookup->getSecond())
      .addDef(MC6809::AW, RegState::Implicit)
      .addImm(Lo);
  Builder.buildInstr(HighLookup->getSecond())
      .addDef(MC6809::AD, RegState::Implicit)
      .addImm(Hi);

  if (needsMaterialization(OrigDest))
    dematerializeReg(Builder, DestReg, OrigDest, MF);
  MI.eraseFromParent();
}

// Bug #311 Phase 2 (2026-05-21): native HD6309 i32 ADD/SUB carry-USE
// variant, _Imm form.  Same shape as expandAddSub_i32_Imm above but
// the LOW half consumes CC.C as carry-in.
//
// HD6309 has ADCD/SBCD for D's i16 add-with-carry but NO equivalent
// for W (no ADCW / SBCW exists).  To get a carry-in into the W half
// without losing CC.C, we route W's bytes through D via EXG D,W:
//
//   EXG  D,W                ; D ↔ W, CC.C preserved
//   ADCB lo16(value) & 0xFF ; D[lsb] = old W's F + value_lo_byte_lsb + Cin
//   ADCA (lo16(value)>>8)   ; D[msb] = old W's E + value_lo_byte_msb + Cout
//   EXG  D,W                ; swap back: W now holds new low result
//   ADCD hi16(value)        ; D = old D + value_hi + Cout from above
//
// EXGp doesn't modify CC (per HD6309 datasheet), so CC.C threads
// through both EXG instructions and the two ADCB/ADCA hops.  Net
// behaviour: AQ = AQ_in + value + Cin, with carry-out in CC.C and
// the IR's phantom carry-out vreg implicit-defined by the pseudo.
//
// Cost vs the non-Use form (ADDW + ADCD = 7 bytes / ~11 cycles):
// 2× EXGp + ADCB + ADCA + ADCD = 2×2 + 2 + 2 + 3 = 11 bytes / ~14
// cycles.  Only emitted when the IR's add/sub-with-carry chain
// crosses an i32 boundary (Phase 2.4's i64 narrowing).
void MC6809InstrInfo::expandAddSubCarryUse_i32_Imm(
    MachineIRBuilder &Builder, MachineInstr &MI, bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #311 Phase 2: i32 carry-use expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();

  unsigned NumOps = MI.getNumExplicitOperands();
  MachineOperand ValOp = MI.getOperand(NumOps - 1);
  int64_t Val = ValOp.isImm() ? ValOp.getImm()
                              : ValOp.getCImm()->getSExtValue();
  int LoB0 = int(Val & 0xFF);
  int LoB1 = int((Val >> 8) & 0xFF);
  int Hi   = int((Val >> 16) & 0xFFFF);

  // Bug #311 Phase 2 follow-up: when DstIsSpill, materializeReg's LDQ
  // overwrites $aq.  Without preserving the caller's prior $aq value
  // (typically a downstream-live LO i32 result from the preceding
  // AddSetCarry_i32_* op), the trailing Store_i32_Mem $aq writes the
  // wrong half of the i64.  Use emitAQPreservedOverHardStackScratch
  // so the LDQ trailer restores the caller's $aq before the body
  // returns.  See expandAddSubCarryUse_i32_Reg for the full rationale.
  bool DstIsSpill = needsMaterialization(DestReg);
  auto Body = [&]() {
    if (DstIsSpill)
      DestReg = materializeReg(Builder, DestReg, MF);
    assert(DestReg == MC6809::AQ &&
           "Bug #311 Phase 2: i32 carry-use Imm requires AQ for the ALU ops");

    // EXG D,W — swap; CC.C survives.
    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    unsigned LowLowOpc = IsAdd ? MC6809::ADCBi8 : MC6809::SBCBi8;
    Builder.buildInstr(LowLowOpc)
        .addDef(MC6809::AB, RegState::Implicit)
        .addImm(LoB0);

    unsigned LowHighOpc = IsAdd ? MC6809::ADCAi8 : MC6809::SBCAi8;
    Builder.buildInstr(LowHighOpc)
        .addDef(MC6809::AA, RegState::Implicit)
        .addImm(LoB1);

    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    auto &HighMap = IsAdd ? AddCarryImmediateOpcode : SubBorrowImmediateOpcode;
    auto HighLookup = HighMap.find({MC6809::AD});
    assert(HighLookup != HighMap.end() &&
           "Bug #311 Phase 2: missing ADCD/SBCD immediate opcode entry");
    Builder.buildInstr(HighLookup->getSecond())
        .addDef(MC6809::AD, RegState::Implicit)
        .addImm(Hi);

    if (DstIsSpill)
      dematerializeReg(Builder, DestReg, OrigDest, MF);
  };

  if (DstIsSpill)
    emitAQPreservedOverHardStackScratch(Builder, Body);
  else
    Body();

  MI.eraseFromParent();
}

// Bug #311 Phase 2: native HD6309 i32 carry-USE _Mem form.
//
// Same EXG-D,W shape as the _Imm Use form, but the four 16-bit halves
// of "value" come from memory bytes at offset+3, offset+2, offset+0
// (big-endian i32 layout, same as expandAddSub_i32_Mem):
//
//   EXG  D,W
//   ADCB <off+3>,<idx>    ; D[lsb] += byte_3 + Cin
//   ADCA <off+2>,<idx>    ; D[msb] += byte_2 + Cout
//   EXG  D,W
//   ADCD <off+0>,<idx>    ; D = old D + hi word + Cout
void MC6809InstrInfo::expandAddSubCarryUse_i32_Mem(
    MachineIRBuilder &Builder, MachineInstr &MI, bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #311 Phase 2: i32 carry-use expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();

  // Operand layout (post-tied $dst=$src):
  //   op 0: $dst (def-use, tied)
  //   op 1: $idx (INDEX16)
  //   op 2: $offset (imm or CImm)
  unsigned NumOps = MI.getNumExplicitOperands();
  Register IndexReg = MI.getOperand(NumOps - 2).getReg();
  MachineOperand OffsetOp = MI.getOperand(NumOps - 1);
  int Offset = OffsetOp.isImm() ? OffsetOp.getImm()
                                : int(OffsetOp.getCImm()->getSExtValue());
  int OffB3 = Offset + 3;
  int OffB2 = Offset + 2;
  int OffHi = Offset;
  int OffB3Size = offsetSizeInBitsForValue(OffB3);
  int OffB2Size = offsetSizeInBitsForValue(OffB2);
  int OffHiSize = offsetSizeInBitsForValue(OffHi);

  // See expandAddSubCarryUse_i32_Imm for the DstIsSpill rationale.
  bool DstIsSpill = needsMaterialization(DestReg);
  auto Body = [&]() {
    if (DstIsSpill)
      DestReg = materializeReg(Builder, DestReg, MF);
    assert(DestReg == MC6809::AQ &&
           "Bug #311 Phase 2: i32 carry-use Mem requires AQ for the ALU ops");

    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    auto &ByteCarryMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
    auto LowLowLookup  = ByteCarryMap.find({MC6809::AB, OffB3Size});
    auto LowHighLookup = ByteCarryMap.find({MC6809::AA, OffB2Size});
    auto HighLookup    = ByteCarryMap.find({MC6809::AD, OffHiSize});
    assert(LowLowLookup != ByteCarryMap.end() &&
           LowHighLookup != ByteCarryMap.end() &&
           HighLookup != ByteCarryMap.end() &&
           "Bug #311 Phase 2: missing ADC[B/A/D] / SBC[B/A/D] indexed entry");

    auto Emit = [&](unsigned Opc, MCPhysReg Reg, int Off, int OffSize) {
      auto Inst = Builder.buildInstr(Opc).addDef(Reg, RegState::Implicit);
      if (OffSize == 0)
        Inst.addReg(IndexReg);
      else
        Inst.addImm(Off).addReg(IndexReg);
    };
    Emit(LowLowLookup->getSecond(),  MC6809::AB, OffB3, OffB3Size);
    Emit(LowHighLookup->getSecond(), MC6809::AA, OffB2, OffB2Size);

    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    Emit(HighLookup->getSecond(), MC6809::AD, OffHi, OffHiSize);

    if (DstIsSpill)
      dematerializeReg(Builder, DestReg, OrigDest, MF);
  };

  if (DstIsSpill)
    emitAQPreservedOverHardStackScratch(Builder, Body);
  else
    Body();

  MI.eraseFromParent();
}

// Bug #311 Phase 2: native HD6309 i32 carry-USE _Reg form.
//
// Same emergency-slot pattern as expandAddSub_i32_Reg (Bug #298): save
// $aq to a hard-stack scratch slot, materialise $dst into $aq, then
// read $src2's value via memory (S-relative for the scratch when src2
// was pre-placed in $aq, U-relative when src2 is SPILL_Q*N).
//
// CC.C threads through LEAS / STQ / materialiseReg / EXG unchanged
// (none of these ops modify the carry flag on HD6309).
void MC6809InstrInfo::expandAddSubCarryUse_i32_Reg(
    MachineIRBuilder &Builder, MachineInstr &MI, bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #311 Phase 2: i32 carry-use expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register Src2Reg = MI.getOperand(2).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();

  // Bug #311 Phase 2 follow-up: choose the right scratch primitive
  // based on where the tied dst lives at expansion time.
  //
  //   - dst == $aq        — emitAQOnHardStackScratch.  The body
  //     mutates $aq IN PLACE to become the result; the caller's
  //     "OLD $aq" value (= the tied src) was overwritten by the body
  //     anyway, so the LEAS-without-LDQ epilogue is correct.
  //
  //   - dst is SPILL_Q*N  — emitAQPreservedOverHardStackScratch.
  //     The body uses $aq as a SCRATCH (loading dst's value into $aq,
  //     computing the result, dematerialising back to dst's slot).
  //     The caller's prior $aq value MUST survive — without the
  //     trailing LDQ, an unrelated live value in $aq (e.g. a
  //     previously-computed LO i32 result waiting for a downstream
  //     store) gets silently destroyed.
  //
  // This is exactly the memcpy-1 / random() i64-add miscompile that
  // Phase 2's first cut produced: the HIGH-i32 expansion clobbered
  // the LOW-i32 result in $aq, and the trailing Store_i32_Mem $aq
  // then wrote HIGH bytes to the LOW slot of the i64 return.
  bool DstIsSpill = needsMaterialization(DestReg);
  auto Body = [&]() {
    if (DstIsSpill)
      DestReg = materializeReg(Builder, DestReg, MF);
    assert(DestReg == MC6809::AQ &&
           "Bug #311 Phase 2: i32 carry-use Reg requires AQ for the ALU ops");

    bool Src2InEmergency = !isQSpillReg(Src2Reg);
    int Src2Off = Src2InEmergency
                      ? 0
                      : computeSpillStackOffset(Src2Reg, MF);
    int Src2OffB3 = Src2Off + 3;
    int Src2OffB2 = Src2Off + 2;
    int Src2OffHi = Src2Off;
    int Src2OffB3Size = offsetSizeInBitsForValue(Src2OffB3);
    int Src2OffB2Size = offsetSizeInBitsForValue(Src2OffB2);
    int Src2OffHiSize = offsetSizeInBitsForValue(Src2OffHi);
    Register Src2BaseReg = Src2InEmergency ? MC6809::SS : MC6809::SU;
    // Bug #387: a static-stack $src2 slot is reached by an extended (absolute,
    // PC-relative under PIC) carry op against the static_stack global, not a
    // SU-relative one. The byte offset within the slot is (Off - Src2Off).
    bool Src2IsStatic = !Src2InEmergency && isStaticSpillSlot(Src2Reg, MF);
    int Src2StaticBase = Src2IsStatic ? staticSpillOffset(Src2Reg, MF) : 0;
    bool Src2IsPIC = MF.getTarget().isPositionIndependent();

    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    auto &ByteCarryMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
    auto LowLowLookup  = ByteCarryMap.find({MC6809::AB, Src2OffB3Size});
    auto LowHighLookup = ByteCarryMap.find({MC6809::AA, Src2OffB2Size});
    auto HighLookup    = ByteCarryMap.find({MC6809::AD, Src2OffHiSize});
    assert(LowLowLookup != ByteCarryMap.end() &&
           LowHighLookup != ByteCarryMap.end() &&
           HighLookup != ByteCarryMap.end() &&
           "Bug #311 Phase 2: missing ADC[B/A/D] / SBC[B/A/D] indexed entry");

    auto Emit = [&](unsigned Opc, MCPhysReg Reg, int Off, int OffSize) {
      if (Src2IsStatic) {
        Builder.buildInstr(getStaticStackOpcode(Opc, Src2IsPIC))
            .addDef(Reg, RegState::Implicit)
            .addTargetIndex(MC6809::TI_STATIC_STACK,
                            Src2StaticBase + (Off - Src2Off));
        return;
      }
      auto Inst = Builder.buildInstr(Opc).addDef(Reg, RegState::Implicit);
      if (OffSize == 0)
        Inst.addReg(Src2BaseReg);
      else
        Inst.addImm(Off).addReg(Src2BaseReg);
    };
    Emit(LowLowLookup->getSecond(),  MC6809::AB, Src2OffB3, Src2OffB3Size);
    Emit(LowHighLookup->getSecond(), MC6809::AA, Src2OffB2, Src2OffB2Size);

    Builder.buildInstr(MC6809::EXGp)
        .addDef(MC6809::AD).addDef(MC6809::AW)
        .addUse(MC6809::AD).addUse(MC6809::AW);

    Emit(HighLookup->getSecond(), MC6809::AD, Src2OffHi, Src2OffHiSize);

    if (DstIsSpill)
      dematerializeReg(Builder, DestReg, OrigDest, MF);
  };

  if (DstIsSpill)
    emitAQPreservedOverHardStackScratch(Builder, Body);
  else
    emitAQOnHardStackScratch(Builder, Body);

  MI.eraseFromParent();
}

// Bug #297 commit 2.1/6 (2026-05-15): native HD6309 i32 ADD/SUB, _Reg form.
//
// Pseudo shape:
//   %dst:ACC32 = Add_i32_Reg %src:ACC32(tied), %src2:ACC32
//
// HD6309 has no direct AQ-AQ register-level i32 ADD/SUB op (ADCR/SBCR
// exist but only at i8/i16 register pairs).  Strategy:
//
//   1. Allocate a 4-byte emergency slot on the hard stack via `LEAS
//      -4,S` and save the current $aq value to it via `STQ ,S`.  This
//      unconditionally costs an STQ; the alternative is a conditional
//      save that pessimises the common case (both operands in
//      SPILL_Q*N) but optimises the AQ-source case, and the complexity
//      isn't worth the savings.
//   2. Materialize $dst (which is also $src via tying) into physical
//      AQ.  If $dst was in SPILL_Q*N, materializeReg emits LDQ from its
//      slot using U-relative addressing (U is the stable frame ptr).
//      If $dst was already in AQ, the materialise is a no-op (but the
//      previous STQ saved that very value to the emergency slot, which
//      is harmless dead-store).
//   3. Compute the stack offset of $src2's operand:
//        - SPILL_Q*N: use computeSpillStackOffset (its own U-relative slot).
//        - AQ: use the emergency slot (we just stashed it there, S-relative).
//   4. Emit ADDW <off+2>, <base> / ADCD <off+0>, <base> (or SUBW/SBCD).
//      Base is U for SPILL_Q*N slot, S for the emergency slot.
//   5. Dematerialize $dst back to its origin if needed.
//   6. Release the emergency slot via `LEAS 4,S`.
//
// Bug #298 fix 2026-05-15: the previous implementation used
// `MFI.CreateStackObject` + `getObjectOffset` to allocate the emergency
// slot.  That's broken because expandPostRAPseudo runs AFTER PEI — the
// freshly-created stack object never gets laid out, so getObjectOffset
// returns 0.  An emergency-slot offset of 0 from U collides with the
// pshs-saved Y/U area at the head of every function with a `pshs y,u;
// tfr s,u` prologue.  The wild `stq ,u` overwrote the saved registers,
// corrupting the function's epilogue puls/rts chain (manifest:
// muld_neg60_probe at Os-lto-hd6309-mame hung because vfprintf's saved
// Y/U slot got overwritten with the i32 value -1740 during `%d`
// processing).
//
// LEAS-based push/pop avoids the FI mechanism entirely.  S is moved
// down 4, the emergency value lives at [S+0..3], and S is restored
// before MI.eraseFromParent().  Intermediate materialize/dematerialize
// uses U-relative addressing which is stable across S-manipulations.
void MC6809InstrInfo::expandAddSub_i32_Reg(MachineIRBuilder &Builder,
                                            MachineInstr &MI,
                                            bool IsAdd) const {
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  (void)STI;
  assert(STI.has6309() &&
         "Bug #297: i32 ADD/SUB expansion is HD6309-only");

  Register DestReg = MI.getOperand(0).getReg();
  Register Src2Reg = MI.getOperand(2).getReg();
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();

  // The body of the helper saves $aq into a 4-byte hard-stack scratch
  // slot (so SRC2-in-AQ is readable via $ss+0..3) and DOES NOT restore
  // $aq after the body — the body's ADDW+ADCD result becomes the new
  // $aq, which is the DST.
  auto Body = [&]() {
    // Step 1: materialize $dst into AQ.  Uses U-relative addressing
    // for SPILL_Q*N → AQ; U is unchanged by the LEAS on S.
    if (needsMaterialization(DestReg))
      DestReg = materializeReg(Builder, DestReg, MF);
    assert(DestReg == MC6809::AQ &&
           "Bug #297: i32 ADD/SUB Reg requires AQ for the ALU-side ops");

    // Step 2: pick $src2's stack offset + base register.
    //   SPILL_Q*N → computeSpillStackOffset (U-relative)
    //   AQ        → emergency slot at S+0 (S-relative)
    bool Src2InEmergency = !isQSpillReg(Src2Reg);
    int Src2Off = Src2InEmergency
                      ? 0
                      : computeSpillStackOffset(Src2Reg, MF);
    int Src2OffLo = Src2Off + 2;
    int Src2OffHi = Src2Off;
    int Src2OffLoSize = offsetSizeInBitsForValue(Src2OffLo);
    int Src2OffHiSize = offsetSizeInBitsForValue(Src2OffHi);
    Register Src2BaseReg = Src2InEmergency ? MC6809::SS : MC6809::SU;
    // Bug #387: a static-stack $src2 reaches the static_stack global by an
    // extended (PC-relative under PIC) op; the byte offsets are +2 (lo) / +0 (hi).
    bool Src2IsStatic = !Src2InEmergency && isStaticSpillSlot(Src2Reg, MF);
    int Src2StaticBase = Src2IsStatic ? staticSpillOffset(Src2Reg, MF) : 0;
    bool Src2IsPIC = MF.getTarget().isPositionIndependent();

    // Step 3: emit ADDW/SUBW + ADCD/SBCD pair against $src2's base.
    auto &LowMap = IsAdd ? AddIdxImmOpcode : SubIdxImmOpcode;
    auto &HighMap = IsAdd ? AddCarryIdxImmOpcode : SubBorrowIdxImmOpcode;
    auto LowLookup = LowMap.find({MC6809::AW, Src2OffLoSize});
    auto HighLookup = HighMap.find({MC6809::AD, Src2OffHiSize});
    assert(LowLookup != LowMap.end() && HighLookup != HighMap.end() &&
           "Bug #297: missing ADDW/ADCD/SUBW/SBCD opcode entry");

    if (Src2IsStatic) {
      Builder.buildInstr(getStaticStackOpcode(LowLookup->getSecond(), Src2IsPIC))
          .addDef(MC6809::AW, RegState::Implicit)
          .addTargetIndex(MC6809::TI_STATIC_STACK, Src2StaticBase + 2);
      Builder.buildInstr(getStaticStackOpcode(HighLookup->getSecond(), Src2IsPIC))
          .addDef(MC6809::AD, RegState::Implicit)
          .addTargetIndex(MC6809::TI_STATIC_STACK, Src2StaticBase + 0);
    } else {
      auto LowInstr =
          Builder.buildInstr(LowLookup->getSecond())
              .addDef(MC6809::AW, RegState::Implicit);
      if (Src2OffLoSize == 0)
        LowInstr.addReg(Src2BaseReg);
      else
        LowInstr.addImm(Src2OffLo).addReg(Src2BaseReg);

      auto HighInstr =
          Builder.buildInstr(HighLookup->getSecond())
              .addDef(MC6809::AD, RegState::Implicit);
      if (Src2OffHiSize == 0)
        HighInstr.addReg(Src2BaseReg);
      else
        HighInstr.addImm(Src2OffHi).addReg(Src2BaseReg);
    }

    // Step 4: dematerialize $dst back if it was spilled.  U-relative
    // addressing; safe across the in-progress S-displacement.
    if (needsMaterialization(OrigDest))
      dematerializeReg(Builder, DestReg, OrigDest, MF);
  };

  if (isQSpillReg(Src2Reg)) {
    // $src2 reads from its own U-relative spill slot; the hard-stack
    // scratch would never be read -- and its save-STQ of a possibly
    // partially-defined $aq (both operands spilled, nothing yet staged)
    // is a verifier reject. Run the body without the bracket.
    Body();
  } else {
    emitAQOnHardStackScratch(Builder, Body);
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
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg)) {
    // Save D — it may hold a live value while we operate on the spill/imag.
    // Undef-marked: pushing then popping garbage is the identity when D was
    // in fact dead/undefined, and no local probe can prove which.
    Builder.buildInstr(MC6809::PSHSs).addUse(MC6809::AD, RegState::Undef);
    DestReg = materializeReg(Builder, DestReg, MF);
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto ValOp = MI.getOperand(operandCount - 1);
  int Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
  int Lo = Val & 0xFF;
  int Hi = (Val >> 8) & 0xFF;
  // Bug #272 Phase B Scope A followup: AND with 0 is an annihilator; emit
  // CLRA/CLRB instead of ANDA/ANDB #0 to avoid the implicit USE of the
  // accumulator (which the post-Scope-A/B/C verifier flags as undef-read
  // when the byte's value isn't reachable on all paths).
  bool IsAND = (Context.Opcode == &ANDImmediateOpcode);
  if (Lo != Context.IdentityValue) {
    if (IsAND && Lo == 0)
      Builder.buildInstr(MC6809::CLRBa).addDef(MC6809::AB, RegState::Implicit);
    else
      Builder.buildInstr(OpcB).addDef(MC6809::AB, RegState::Implicit).addImm(Lo);
  }
  if (Hi != Context.IdentityValue) {
    if (IsAND && Hi == 0)
      Builder.buildInstr(MC6809::CLRAa).addDef(MC6809::AA, RegState::Implicit);
    else
      Builder.buildInstr(OpcA).addDef(MC6809::AA, RegState::Implicit).addImm(Hi);
  }
  if (needsMaterialization(OrigDest)) {
    dematerializeReg(Builder, DestReg, OrigDest, MF);
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
  Register OrigDest = DestReg;
  MachineFunction &MF = *MI.getMF();
  if (needsMaterialization(DestReg)) {
    // Undef-marked push -- see expandBitwiseImm16. The push moved S down
    // by 2: an S-relative memory operand must be displacement-compensated
    // before the offset is read below.
    Builder.buildInstr(MC6809::PSHSs).addUse(MC6809::AD, RegState::Undef);
    compensateSSOperands(MI, 2);
    DestReg = materializeReg(Builder, DestReg, MF);
  }
  auto operandCount = MI.getNumExplicitOperands();
  auto IndexReg = MI.getOperand(operandCount - 2).getReg();
  auto OffsetOp = MI.getOperand(operandCount - 1);
  auto Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
  // P3b indirect (6809): materialize the pointer (at [IndexReg+Offset]) into IY,
  // then split off [0,IY]/[1,IY] below. The direct two-byte split addresses
  // [n,r]/[n+1,r], which is wrong for an indirect operand whose two bytes live
  // at *ptr and *ptr+1. (6309 took the expandIdxImm path above with ANDD/etc.)
  if (isMemIndirectPseudo(MI.getOpcode())) {
    // getLoadIdxOpcode always returns the o8/o16 form (which takes an explicit
    // offset operand), so always pass it -- exactly like the spill-base helper.
    unsigned LdOpc = getLoadIdxOpcode(MC6809::IY, Offset);
    Builder.buildInstr(LdOpc)
        .addDef(MC6809::IY, RegState::Implicit)
        .addImm(Offset)
        .addReg(IndexReg);
    IndexReg = Register(MC6809::IY);
    Offset = 0;
  }
  int OffsetLo = Offset + 1;
  int OffsetHi = Offset;
  int OffsetLoSize = offsetSizeInBitsForValue(OffsetLo);
  int OffsetHiSize = offsetSizeInBitsForValue(OffsetHi);
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
  if (needsMaterialization(OrigDest)) {
    dematerializeReg(Builder, DestReg, OrigDest, MF);
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

  // Handle spill/imaginary registers: load to real register, negate, store back.
  if (needsMaterialization(Reg)) {
    Register OrigReg = Reg;
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock &MBB = *MI.getParent();
    // The recursive expansion may erase MI: capture the resume point now
    // and rebuild the trailing code with a fresh builder. Preserve the
    // staging real around the window (see pushStagingReg).
    MachineBasicBlock::iterator NextPt = std::next(MI.getIterator());
    Register RealReg = getPhysRegFor(OrigReg);
    MachineIRBuilder PreBuilder(MBB, MI.getIterator());
    PreBuilder.buildInstr(MC6809::PSHSs).addUse(RealReg, RegState::Undef);
    materializeReg(Builder, Reg, MF);
    // Rewrite operands to the real register.
    MI.getOperand(0).setReg(RealReg);
    MI.getOperand(0).setIsDead(false); // demat below reads the staging real
    if (MI.getNumOperands() > 1 && MI.getOperand(1).isReg() &&
        MI.getOperand(1).getReg() == OrigReg)
      MI.getOperand(1).setReg(RealReg);
    // Recursively expand with the real register.
    expandNegate(Builder, MI);
    // Store back.
    MachineIRBuilder PostBuilder(MBB, NextPt);
    dematerializeReg(PostBuilder, RealReg, OrigReg, MF);
    PostBuilder.buildInstr(MC6809::PULSs).addDef(RealReg);
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
  Register Reg = MI.getOperand(0).getReg();
  if (needsMaterialization(Reg)) {
    Register OrigReg = Reg;
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock &MBB = *MI.getParent();
    // The recursive expansion erases MI, so capture the resume point now
    // and rebuild the trailing code with a fresh builder -- inserting via
    // the caller's builder after the erase trips the
    // iterator-points-outside-of-basic-block assert.
    MachineBasicBlock::iterator NextPt = std::next(MI.getIterator());
    // The staging clobbers the real accumulator, and regalloc picked an
    // imaginary home precisely because that accumulator may be busy --
    // preserve it around the whole shift (undef-marked push: garbage
    // in/garbage out is the identity when it was in fact dead).
    Register RealReg = getPhysRegFor(OrigReg);
    MachineIRBuilder PreBuilder(MBB, MI.getIterator());
    PreBuilder.buildInstr(MC6809::PSHSs).addUse(RealReg, RegState::Undef);
    Reg = materializeReg(Builder, Reg, MF);
    MI.getOperand(0).setReg(Reg);
    MI.getOperand(0).setIsDead(false); // demat below reads the staging real
    if (MI.getNumOperands() > 1 && MI.getOperand(1).isReg() &&
        MI.getOperand(1).getReg() == OrigReg)
      MI.getOperand(1).setReg(Reg);
    expandShiftLeft(Builder, MI);
    MachineIRBuilder PostBuilder(MBB, NextPt);
    dematerializeReg(PostBuilder, Reg, OrigReg, MF);
    PostBuilder.buildInstr(MC6809::PULSs).addDef(RealReg);
    return;
  }
  // LSL_i8/i16_Reg is (outs reg:$dst), (ins reg:$src, i8imm:$val) with $dst
  // tied to $src: "shift $src left by $val into $dst". Emit $val single-bit
  // hardware shifts in place, then erase the pseudo. i8 (AA/AB) is one ASL per
  // bit; i16 (AD) is ASLD per bit on hd6309, ASLB+ROLA per bit on base 6809.
  uint64_t Count = MI.getOperand(2).getImm();
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  // buildInstr already attaches the opcode's implicit defs/uses from its
  // MCInstrDesc -- do NOT also call addImplicitDefUseOperands (that doubles
  // them, leaving a stray undefined $c use the machine verifier rejects).
  for (uint64_t I = 0; I < Count; ++I) {
    switch (Reg) {
    default:
      llvm_unreachable("Illegal register for ASL/LSL");
    case MC6809::AA:
      Builder.buildInstr(MC6809::ASLAa);
      break;
    case MC6809::AB:
      Builder.buildInstr(MC6809::ASLBa);
      break;
    case MC6809::AD:
      if (STI.has6309()) {
        Builder.buildInstr(MC6809::ASLDa);
      } else {
        // ASLB then ROLA (carry from B propagates into A).
        Builder.buildInstr(MC6809::ASLBa);
        Builder.buildInstr(MC6809::ROLAa);
      }
      break;
    }
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandShiftRight(MachineIRBuilder &Builder, MachineInstr &MI, bool Arithmetic) const {
  Register Reg = MI.getOperand(0).getReg();
  if (needsMaterialization(Reg)) {
    Register OrigReg = Reg;
    MachineFunction &MF = *MI.getMF();
    MachineBasicBlock &MBB = *MI.getParent();
    // The recursive expansion erases MI, so capture the resume point now
    // and rebuild the trailing code with a fresh builder -- inserting via
    // the caller's builder after the erase trips the
    // iterator-points-outside-of-basic-block assert.
    MachineBasicBlock::iterator NextPt = std::next(MI.getIterator());
    // The staging clobbers the real accumulator, and regalloc picked an
    // imaginary home precisely because that accumulator may be busy --
    // preserve it around the whole shift (undef-marked push: garbage
    // in/garbage out is the identity when it was in fact dead).
    Register RealReg = getPhysRegFor(OrigReg);
    MachineIRBuilder PreBuilder(MBB, MI.getIterator());
    PreBuilder.buildInstr(MC6809::PSHSs).addUse(RealReg, RegState::Undef);
    Reg = materializeReg(Builder, Reg, MF);
    MI.getOperand(0).setReg(Reg);
    MI.getOperand(0).setIsDead(false); // demat below reads the staging real
    if (MI.getNumOperands() > 1 && MI.getOperand(1).isReg() &&
        MI.getOperand(1).getReg() == OrigReg)
      MI.getOperand(1).setReg(Reg);
    expandShiftRight(Builder, MI, Arithmetic);
    MachineIRBuilder PostBuilder(MBB, NextPt);
    dematerializeReg(PostBuilder, Reg, OrigReg, MF);
    PostBuilder.buildInstr(MC6809::PULSs).addDef(RealReg);
    return;
  }
  // Emit $val (operand 2) single-bit right shifts in place, then erase.
  // i16 (AD): ASRD/LSRD per bit on hd6309; on base 6809 shift the high byte
  // first (ASRA/LSRA, bit0 -> carry) then ROR the low byte (carry -> bit7).
  uint64_t Count = MI.getOperand(2).getImm();
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  // buildInstr already attaches each opcode's implicit defs/uses; don't add
  // them again (see expandShiftLeft). For ASHR the high byte is shifted with
  // ASRA/ASRD (arithmetic, sign-fill -- no carry-in), then the low byte rotates
  // through the carry ASRA set.
  for (uint64_t I = 0; I < Count; ++I) {
    switch (Reg) {
    default:
      llvm_unreachable("Illegal register for LSR/ASR");
    case MC6809::AA:
      Builder.buildInstr(Arithmetic ? MC6809::ASRAa : MC6809::LSRAa);
      break;
    case MC6809::AB:
      Builder.buildInstr(Arithmetic ? MC6809::ASRBa : MC6809::LSRBa);
      break;
    case MC6809::AD:
      if (STI.has6309()) {
        Builder.buildInstr(Arithmetic ? MC6809::ASRDa : MC6809::LSRDa);
      } else {
        Builder.buildInstr(Arithmetic ? MC6809::ASRAa : MC6809::LSRAa);
        Builder.buildInstr(MC6809::RORBa);
      }
      break;
    }
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandRotate(MachineIRBuilder &Builder, MachineInstr &MI, bool Left) const {
  Register Reg = MI.getOperand(0).getReg();
  if (needsMaterialization(Reg)) {
    Register OrigReg = Reg;
    MachineFunction &MF = *MI.getMF();
    Reg = materializeReg(Builder, Reg, MF);
    MI.getOperand(0).setReg(Reg);
    if (MI.getNumOperands() > 1 && MI.getOperand(1).isReg() &&
        MI.getOperand(1).getReg() == OrigReg)
      MI.getOperand(1).setReg(Reg);
    expandRotate(Builder, MI, Left);
    dematerializeReg(Builder, Reg, OrigReg, MF);
    return;
  }
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
  // Bug #271 (category 4): same off-by-one as expandShiftLeft /
  // expandShiftRight. ROL_i8 / ROR_i8 use MC6809Shift<ACC8_AB> too,
  // i.e. (outs reg:$dst), (ins reg:$src, i8imm:$val) — three explicit
  // operands. Remove operand 2 (the i8imm shift count) first.
  MI.removeOperand(2); // remove immediate (the i8imm shift count)
  MI.removeOperand(1); // remove src register (now implicit)
  MI.removeOperand(0); // remove dst register (now implicit)
  MI.addImplicitDefUseOperands(*MI.getMF());
}

void MC6809InstrInfo::expandMulD(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOpcode() == MC6809::MUL_D && "Invalid multiply opcode");
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();

  if (DstReg == MC6809::AD && SrcReg == MC6809::AD) {
    MI.setDesc(Builder.getTII().get(MC6809::MULx));
    MI.getOperand(0).setImplicit();
    MI.getOperand(1).setImplicit();
    return;
  }

  // Spill/imaginary operand: materialize → MUL → dematerialize.
  MachineFunction &MF = *MI.getMF();
  MachineBasicBlock &MBB = *MI.getParent();

  if (SrcReg != MC6809::AD) {
    MachineIRBuilder PreBuilder(MBB, MI.getIterator());
    materializeReg(PreBuilder, SrcReg, MF);
  }

  MI.setDesc(Builder.getTII().get(MC6809::MULx));
  MI.getOperand(0).setReg(MC6809::AD);
  MI.getOperand(0).setImplicit();
  MI.getOperand(1).setReg(MC6809::AD);
  MI.getOperand(1).setImplicit();

  if (DstReg != MC6809::AD) {
    MachineBasicBlock::iterator InsertPt = MI.getIterator();
    ++InsertPt;
    MachineIRBuilder PostBuilder(MBB, InsertPt);
    dematerializeReg(PostBuilder, MC6809::AD, DstReg, MF);
  }
}

void MC6809InstrInfo::expandMul16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #161 round 14: dst class widened from AWc to ACC16. After MULD
  // the result lives in AW; copy out to dst if dst != AW.
  Register Dst = MI.getOperand(0).getReg();
  auto ValueOp = MI.getOperand(2);
  auto Value = ValueOp.isImm() ? ValueOp.getImm() : ValueOp.getCImm()->getSExtValue();
  Builder.buildInstr(MC6809::MULDi16)
      .addDef(MC6809::AW, RegState::ImplicitDefine)
      .addUse(MI.getOperand(1).getReg(), RegState::Implicit)
      .addImm(Value);
  if (Dst != MC6809::AW)
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(), Dst, MC6809::AW,
                /*KillSrc=*/false);
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
  // Bug #161 round 14: dst class widened from AWc to ACC16. Same TFR-out
  // fixup as expandMul16Imm.
  Register Dst = MI.getOperand(0).getReg();
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
                     .addDef(MC6809::AW, RegState::ImplicitDefine)
                     .addUse(MI.getOperand(1).getReg(), RegState::Implicit);
    if (OffsetSize == 0) {
      Instr.addReg(IndexReg);
    } else {
      Offset = OffsetOp.isImm() ? OffsetOp.getImm() : OffsetOp.getCImm()->getSExtValue();
      Instr.addImm(Offset).addReg(IndexReg);
    }
  } else
    llvm_unreachable("Unknown offset type");
  if (Dst != MC6809::AW)
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(), Dst, MC6809::AW,
                /*KillSrc=*/false);
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
  // Bug #161 round 14: dst class widened from AWc to ACC16. Same TFR-out
  // fixup as expandMul16Imm.
  Register Dst = MI.getOperand(0).getReg();
  Builder.buildInstr(Opcode)
      .addDef(MC6809::AW, RegState::ImplicitDefine)
      .addDef(MI.getOperand(1).getReg(), RegState::ImplicitDefine)
      .addReg(OffsetReg, RegState::Implicit).addReg(IndexReg);
  if (Dst != MC6809::AW)
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(), Dst, MC6809::AW,
                /*KillSrc=*/false);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  llvm_unreachable("Write me!");
}

void MC6809InstrInfo::expandMul16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #161 round 14: dst class widened from AWc to ACC16 so back-to-back
  // multiplies in i64 chains don't all collide on AW. After MULD the
  // result is physically in AW; if the regalloc-assigned dst is anything
  // else (AD, an SPILL_D, an RS imag reg, etc.) emit a TFR W,<dst> via
  // copyPhysReg. AD is also clobbered by MULD (high-half product) — Defs
  // already lists it.
  //
  Register Dst = MI.getOperand(0).getReg();
  Register Reg = MI.getOperand(MI.getNumExplicitOperands() - 1).getReg();
  // PSHS encodes reg-list bits for CC/A/B/D/DP/X/Y/U/PC only; pushing AW
  // (or any reg outside that set) needs the HD6309 PSHSW (page-2 0x10 0x38).
  // Dispatch by source-reg class. Both push 2 bytes big-endian onto S, so
  // the following MULDi_Inc2 with `,S++` works identically.
  if (Reg == MC6809::AW) {
    Builder.buildInstr(MC6809::PSHSWx);
  } else if (needsMaterialization(Reg)) {
    // Imaginary/spill operand: PSHS can't encode it. Stage through AD --
    // safe, MULD consumes and clobbers D (the pseudo's Defs list it).
    Register Real = materializeReg(Builder, Reg, *MI.getMF());
    Builder.buildInstr(MC6809::PSHSs).addReg(Real);
  } else {
    Builder.buildInstr(MC6809::PSHSs).addReg(Reg);
  }
  Builder.buildInstr(MC6809::MULDi_Inc2).addReg(MC6809::SS);
  if (Dst != MC6809::AW)
    copyPhysReg(*MI.getParent(), MI, MI.getDebugLoc(), Dst, MC6809::AW,
                /*KillSrc=*/false);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandMulH16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #161: i16 mulh — high-16 of (D)*(src2) on HD6309. Same expansion
  // as expandMul16Reg (push the register operand, MULD ,S++); MULD
  // natively places the high half in D and the low half in W. The
  // pseudo's `Defs = [NZ, AW]` already lists W as clobbered, so the
  // dst (AD = high half) is just D after the MULD. Plain serial MIs
  // (no MIBundleBuilder) — see expandMul16Reg comment for why.
  auto Reg = MI.getOperand(MI.getNumExplicitOperands() - 1).getReg();
  // See expandMul16Reg for rationale; AW needs PSHSW, others use PSHS.
  if (Reg == MC6809::AW) {
    Builder.buildInstr(MC6809::PSHSWx);
  } else if (needsMaterialization(Reg)) {
    // See expandMul16Reg: stage through AD, which MULD clobbers anyway.
    Register Real = materializeReg(Builder, Reg, *MI.getMF());
    Builder.buildInstr(MC6809::PSHSs).addReg(Real);
  } else {
    Builder.buildInstr(MC6809::PSHSs).addReg(Reg);
  }
  Builder.buildInstr(MC6809::MULDi_Inc2).addReg(MC6809::SS);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandLoad1Imm(MachineIRBuilder &Builder, MachineInstr &MI) const {
  Register DestReg = MI.getOperand(0).getReg();
  int64_t Val = MI.getOperand(1).getImm();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    // Default path: byte dst (the only non-CC bit destination).
    assert(MC6809::ACC8RegClass.contains(DestReg));
    Opcode = MC6809::Load_i8_Imm;
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

  // If the destination is a spill or imaginary register, load immediate into
  // the real hardware register, then dematerialize back.
  // Don't recurse into expandLoadImm — it removes MI and invalidates Builder.
  if (needsMaterialization(DestRegOp.getReg())) {
    // INDEX spills use IY as staging; ACC spills and imaginary use their
    // natural physical register (via getPhysRegFor).
    Register RealReg = isIndexSpillReg(DestRegOp.getReg()) ? MC6809::IY
                                                           : getPhysRegFor(DestRegOp.getReg());
    MachineFunction &MF = Builder.getMF();
    auto ValOp = MI.getOperand(1);
    // Bug #300 residue (2026-05-15): for SPILL_Q*N (i32) dst, the
    // staging through physical $aq via LDQ silently clobbers any
    // other vreg currently live in $aq.  The pseudo's Defs don't
    // declare AD (adding it regresses test_return_i32_constant), and
    // the dst class AQc is widened to include SPILL_Q*N (Phase B),
    // so regalloc may legitimately place a different vreg in $aq
    // across this pseudo.  Bracket the LDQ + STQ with a LEAS-based
    // save/restore of $aq on the hard stack to preserve the live
    // value.  Cost: 4 extra instructions per i32-Imm-to-spill load.
    bool IsQSpill = !isIndexSpillReg(DestRegOp.getReg()) &&
                    RealReg == MC6809::AQ;
    auto Body = [&]() {
      // Load immediate into staging register, then store to spill/
      // imaginary slot.
      auto OpcodePair = LoadImmediateOpcode.find(RealReg);
      assert(OpcodePair != LoadImmediateOpcode.end());
      auto NewMI = Builder.buildInstr(OpcodePair->getSecond()).addDef(RealReg, RegState::Implicit);
      if (ValOp.isGlobal())
        NewMI.addGlobalAddress(ValOp.getGlobal(), ValOp.getOffset(), ValOp.getTargetFlags());
      else if (ValOp.isSymbol())
        NewMI.addExternalSymbol(ValOp.getSymbolName(), ValOp.getTargetFlags());
      else {
        int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
        NewMI.addImm(Val);
      }
      // Store to spill slot (use STY for INDEX spills) or dematerialize.
      if (isIndexSpillReg(DestRegOp.getReg())) {
        emitSpillStoreFrom(Builder, MC6809::IY, DestRegOp.getReg(),
                           /*ExtraOffset=*/0, MF);
      } else {
        dematerializeReg(Builder, RealReg, DestRegOp.getReg(), MF);
      }
    };
    if (IsQSpill) {
      // Bug #305 (2026-05-18): only wrap in save/restore when $aq or
      // any of its sub-registers ($ad, $aw, $aa, $ab, $ae, $af) is live
      // at this expansion point.  Bug #300's unconditional wrap emits
      // `LEAS -4,$ss; STQ ,$ss; <body>; LDQ ,$ss; LEAS 4,$ss` whose
      // save-STQ reads $aq.  When the whole $aq family is dead
      // (typical after an LDD that killed $aq via Bug #299's AQ-in-Defs
      // with no surviving sub-reg consumer), the read is undefined and
      // `-verify-machineinstrs` rejects it (61 hits across 7 functions
      // at -Og-hd6309-mame).
      //
      // CRITICAL: `LivePhysRegs::contains(AQ)` returns false when only
      // a sub-reg ($ad/$aw/etc.) is live (header comment: "Returns
      // false if just some sub registers are live, use available() when
      // searching a free register").  We must iterate sub-regs
      // explicitly — skipping save/restore when $ad is live would let
      // the body's LDQ #imm clobber the still-live sub-reg.
      MachineBasicBlock *MBB = MI.getParent();
      const TargetRegisterInfo &TRI =
          *Builder.getMF().getSubtarget().getRegisterInfo();
      LivePhysRegs LiveRegs(TRI);
      LiveRegs.addLiveIns(*MBB);
      SmallVector<std::pair<MCPhysReg, const MachineOperand *>, 4> Clobbers;
      for (auto It = MBB->begin(); It != MI.getIterator(); ++It) {
        Clobbers.clear();
        LiveRegs.stepForward(*It, Clobbers);
      }
      bool AqFamilyLive = false;
      for (MCPhysReg R : {MC6809::AQ, MC6809::AD, MC6809::AW,
                          MC6809::AA, MC6809::AB,
                          MC6809::AE, MC6809::AF}) {
        if (LiveRegs.contains(R)) { AqFamilyLive = true; break; }
      }
      if (AqFamilyLive)
        emitAQPreservedOverHardStackScratch(Builder, Body);
      else
        Body();
    } else if (!isIndexSpillReg(DestRegOp.getReg())) {
      // AD/AA/AB staging: the LD #imm clobbers the staging real, which
      // regalloc may have left live across this pseudo (its Defs are
      // deliberately empty). Preserve it (see pushStagingReg).
      pushStagingReg(Builder, RealReg);
      Body();
      pullStagingReg(Builder, RealReg);
    } else {
      Body();
    }
    MI.removeFromParent();
    return;
  }

  auto ValOp = MI.getOperand(1);
  auto OpcodePair = LoadImmediateOpcode.find(MI.getOperand(0).getReg());
  if (OpcodePair == LoadImmediateOpcode.end())
    llvm_unreachable("Unexpected LoadImm register.");
  auto NewMI = Builder.buildInstr(OpcodePair->getSecond()).addDef(DestRegOp.getReg(), RegState::Implicit);
  // Preserve the operand type: integer immediate, global, or external symbol.
  if (ValOp.isGlobal())
    NewMI.addGlobalAddress(ValOp.getGlobal(), ValOp.getOffset(), ValOp.getTargetFlags());
  else if (ValOp.isSymbol())
    NewMI.addExternalSymbol(ValOp.getSymbolName(), ValOp.getTargetFlags());
  else {
    int64_t Val = ValOp.isImm() ? ValOp.getImm() : ValOp.getCImm()->getSExtValue();
    NewMI.addImm(Val);
  }
  MI.removeFromParent();
}

// Concrete reg-specific opcode for a global-symbol load/store, by the addressing
// mode the post-RA expander resolves: direct page (IsDP), PC-relative (IsPIC),
// or extended (absolute). Driven off the register the value landed in.
static unsigned getSymLoadOpcode(Register Reg, bool IsDP, bool IsPIC) {
  struct Row { unsigned R, D, E, PC; };
  static const Row T[] = {
    {MC6809::AA, MC6809::LDAd, MC6809::LDAe, MC6809::LDAi_o16PC},
    {MC6809::AB, MC6809::LDBd, MC6809::LDBe, MC6809::LDBi_o16PC},
    {MC6809::AD, MC6809::LDDd, MC6809::LDDe, MC6809::LDDi_o16PC},
    {MC6809::AE, MC6809::LDEd, MC6809::LDEe, MC6809::LDEi_o16PC},
    {MC6809::AF, MC6809::LDFd, MC6809::LDFe, MC6809::LDFi_o16PC},
    {MC6809::AW, MC6809::LDWd, MC6809::LDWe, MC6809::LDWi_o16PC},
    {MC6809::IX, MC6809::LDXd, MC6809::LDXe, MC6809::LDXi_o16PC},
    {MC6809::IY, MC6809::LDYd, MC6809::LDYe, MC6809::LDYi_o16PC},
    {MC6809::SU, MC6809::LDUd, MC6809::LDUe, MC6809::LDUi_o16PC},
    {MC6809::AQ, MC6809::LDQd, MC6809::LDQe, MC6809::LDQi_o16PC},
  };
  for (const Row &E : T)
    if (E.R == Reg)
      return IsDP ? E.D : IsPIC ? E.PC : E.E;
  return 0;
}

static unsigned getSymStoreOpcode(Register Reg, bool IsDP, bool IsPIC) {
  struct Row { unsigned R, D, E, PC; };
  static const Row T[] = {
    {MC6809::AA, MC6809::STAd, MC6809::STAe, MC6809::STAi_o16PC},
    {MC6809::AB, MC6809::STBd, MC6809::STBe, MC6809::STBi_o16PC},
    {MC6809::AD, MC6809::STDd, MC6809::STDe, MC6809::STDi_o16PC},
    {MC6809::AE, MC6809::STEd, MC6809::STEe, MC6809::STEi_o16PC},
    {MC6809::AF, MC6809::STFd, MC6809::STFe, MC6809::STFi_o16PC},
    {MC6809::AW, MC6809::STWd, MC6809::STWe, MC6809::STWi_o16PC},
    {MC6809::IX, MC6809::STXd, MC6809::STXe, MC6809::STXi_o16PC},
    {MC6809::IY, MC6809::STYd, MC6809::STYe, MC6809::STYi_o16PC},
    {MC6809::SU, MC6809::STUd, MC6809::STUe, MC6809::STUi_o16PC},
    {MC6809::AQ, MC6809::STQd, MC6809::STQe, MC6809::STQi_o16PC},
  };
  for (const Row &E : T)
    if (E.R == Reg)
      return IsDP ? E.D : IsPIC ? E.PC : E.E;
  return 0;
}

// The staging register a spilled _Sym value transits through, by pseudo width.
static Register symStageReg(unsigned PseudoOpc) {
  switch (PseudoOpc) {
  case MC6809::Load_i8_Sym:  case MC6809::Store_i8_Sym:  return MC6809::AA;
  case MC6809::Load_i16_Sym: case MC6809::Store_i16_Sym: return MC6809::AD;
  case MC6809::Load_iPtr_Sym:case MC6809::Store_iPtr_Sym:return MC6809::IY;
  case MC6809::Load_i32_Sym: case MC6809::Store_i32_Sym: return MC6809::AQ;
  default:                                               return MC6809::NoRegister;
  }
}

static bool symIsDirectPage(const MachineOperand &SymOp) {
  return SymOp.isGlobal() &&
         SymOp.getGlobal()->getAddressSpace() == MC6809::AS_DirectPage;
}

void MC6809InstrInfo::expandLoadSym(MachineIRBuilder &Builder, MachineInstr &MI) const {
  MachineFunction &MF = *MI.getMF();
  Register Dst = MI.getOperand(0).getReg();
  const MachineOperand &SymOp = MI.getOperand(1);
  bool IsDP = symIsDirectPage(SymOp);
  bool IsPIC = MF.getTarget().isPositionIndependent();
  MachineBasicBlock &MBB = *MI.getParent();

  // Imaginary dst: load the global into the imaginary's staging real and
  // store it to the DP slot, preserving the staging real around the window
  // (see pushStagingReg). isStaticSpillSlot/computeSpillStackOffset below
  // are spill-only and would assert on an RS/RC register.
  if (needsMaterialization(Dst) && !isSpillReg(Dst)) {
    Register Stage = getPhysRegFor(Dst);
    MachineIRBuilder B(MBB, MI.getIterator());
    pushStagingReg(B, Stage);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(getSymLoadOpcode(Stage, IsDP, IsPIC)))
        .add(SymOp)
        .addDef(Stage, RegState::Implicit);
    dematerializeReg(B, Stage, Dst, MF);
    pullStagingReg(B, Stage);
    MI.eraseFromParent();
    return;
  }

  // Spill dst: load the global into a width-appropriate staging register, then
  // store that register to the spill slot. Mirrors expandLoadIdx's spill path;
  // the global op needs no address register, so no staging-base conflict.
  if (needsMaterialization(Dst)) {
    Register Stage = symStageReg(MI.getOpcode());
    unsigned LoadOpc = getSymLoadOpcode(Stage, IsDP, IsPIC);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(LoadOpc))
        .add(SymOp)
        .addDef(Stage, RegState::Implicit);
    // Bug #387: a static-stack spill slot is reached by an extended (or
    // PC-relative) store, not SpillOff,U.
    if (isStaticSpillSlot(Dst, MF)) {
      BuildMI(MBB, MI, MI.getDebugLoc(),
              get(getSymStoreOpcode(Stage, /*IsDP=*/false, IsPIC)))
          .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(Dst, MF))
          .addUse(Stage, RegState::Implicit);
    } else {
      int SpillOff = computeSpillStackOffset(Dst, MF);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(getStoreIdxOpcode(Stage, SpillOff)))
          .addUse(Stage, RegState::Implicit)
          .addImm(SpillOff)
          .addReg(MC6809::SU);
    }
    MI.eraseFromParent();
    return;
  }

  unsigned Opc = getSymLoadOpcode(Dst, IsDP, IsPIC);
  assert(Opc && "no global-symbol load opcode for destination register");
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Opc))
      .add(SymOp)
      .addDef(Dst, RegState::Implicit);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandStoreSym(MachineIRBuilder &Builder, MachineInstr &MI) const {
  MachineFunction &MF = *MI.getMF();
  Register Src = MI.getOperand(0).getReg();
  const MachineOperand &SymOp = MI.getOperand(1);
  bool IsDP = symIsDirectPage(SymOp);
  bool IsPIC = MF.getTarget().isPositionIndependent();
  MachineBasicBlock &MBB = *MI.getParent();

  // Imaginary src: read the value from its DP slot into the staging real,
  // store that to the global, preserving the staging real (see
  // pushStagingReg). The spill-slot machinery below would assert on RS/RC.
  if (needsMaterialization(Src) && !isSpillReg(Src)) {
    MachineIRBuilder B(MBB, MI.getIterator());
    Register Stage = getPhysRegFor(Src);
    pushStagingReg(B, Stage);
    materializeReg(B, Src, MF);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(getSymStoreOpcode(Stage, IsDP, IsPIC)))
        .add(SymOp)
        .addUse(Stage, RegState::Implicit);
    pullStagingReg(B, Stage);
    MI.eraseFromParent();
    return;
  }

  // Spill src: reload the value from its spill slot into a staging register,
  // then store that register to the global.
  if (needsMaterialization(Src)) {
    Register Stage = symStageReg(MI.getOpcode());
    // Bug #387: a static-stack spill slot is reached by an extended (or
    // PC-relative) load, not SpillOff,U.
    if (isStaticSpillSlot(Src, MF)) {
      BuildMI(MBB, MI, MI.getDebugLoc(),
              get(getSymLoadOpcode(Stage, /*IsDP=*/false, IsPIC)))
          .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(Src, MF))
          .addDef(Stage, RegState::Implicit);
    } else {
      int SpillOff = computeSpillStackOffset(Src, MF);
      BuildMI(MBB, MI, MI.getDebugLoc(), get(getLoadIdxOpcode(Stage, SpillOff)))
          .addDef(Stage, RegState::Implicit)
          .addImm(SpillOff)
          .addReg(MC6809::SU);
    }
    unsigned StoreOpc = getSymStoreOpcode(Stage, IsDP, IsPIC);
    BuildMI(MBB, MI, MI.getDebugLoc(), get(StoreOpc))
        .add(SymOp)
        .addUse(Stage, RegState::Implicit);
    MI.eraseFromParent();
    return;
  }

  unsigned Opc = getSymStoreOpcode(Src, IsDP, IsPIC);
  assert(Opc && "no global-symbol store opcode for source register");
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Opc))
      .add(SymOp)
      .addUse(Src, RegState::Implicit);
  MI.eraseFromParent();
}

// The indexed post-modify (,R+ / ,R++ / ,-R / ,--R) opcode for a value in Reg.
// Element size (1 vs 2) is implied by the register, so Inc1/Dec1 for the 8-bit
// accumulators and Inc2/Dec2 for the 16-bit ones.
static unsigned getPostModOpcode(Register Val, bool IsInc, bool IsLoad) {
  struct Row { unsigned R, LdI, LdD, StI, StD; };
  static const Row T[] = {
    {MC6809::AA, MC6809::LDAi_Inc1, MC6809::LDAi_Dec1, MC6809::STAi_Inc1, MC6809::STAi_Dec1},
    {MC6809::AB, MC6809::LDBi_Inc1, MC6809::LDBi_Dec1, MC6809::STBi_Inc1, MC6809::STBi_Dec1},
    {MC6809::AE, MC6809::LDEi_Inc1, MC6809::LDEi_Dec1, MC6809::STEi_Inc1, MC6809::STEi_Dec1},
    {MC6809::AF, MC6809::LDFi_Inc1, MC6809::LDFi_Dec1, MC6809::STFi_Inc1, MC6809::STFi_Dec1},
    {MC6809::AD, MC6809::LDDi_Inc2, MC6809::LDDi_Dec2, MC6809::STDi_Inc2, MC6809::STDi_Dec2},
    {MC6809::AW, MC6809::LDWi_Inc2, MC6809::LDWi_Dec2, MC6809::STWi_Inc2, MC6809::STWi_Dec2},
    {MC6809::IX, MC6809::LDXi_Inc2, MC6809::LDXi_Dec2, MC6809::STXi_Inc2, MC6809::STXi_Dec2},
    {MC6809::IY, MC6809::LDYi_Inc2, MC6809::LDYi_Dec2, MC6809::STYi_Inc2, MC6809::STYi_Dec2},
    {MC6809::SU, MC6809::LDUi_Inc2, MC6809::LDUi_Dec2, MC6809::STUi_Inc2, MC6809::STUi_Dec2},
  };
  for (const Row &E : T)
    if (E.R == Val)
      return IsLoad ? (IsInc ? E.LdI : E.LdD) : (IsInc ? E.StI : E.StD);
  return 0;
}

void MC6809InstrInfo::expandLoadPostMod(MachineIRBuilder &Builder, MachineInstr &MI, bool IsInc) const {
  // Pseudo: outs ($dst value, $ptr_out), in ($ptr_in); $ptr_in == $ptr_out.
  MachineFunction &MF = *MI.getMF();
  Register Val = MI.getOperand(0).getReg();
  Register Ptr = MI.getOperand(2).getReg();
  // A spilled pointer is staged into a hardware index register (advanced in
  // place, written back); a spilled value is produced into its hardware
  // register and stored to its slot. Non-spill operands pass straight through.
  Register PtrReg = materializeReg(Builder, Ptr, MF);
  bool StagedVal = needsMaterialization(Val);
  Register ValReg = StagedVal ? getPhysRegFor(Val) : Val;
  assert(ValReg != PtrReg && "post-modify value/pointer register conflict");
  // Preserve the value's staging real (pure def -- see pushStagingReg).
  if (StagedVal)
    pushStagingReg(Builder, ValReg);
  unsigned Opc = getPostModOpcode(ValReg, IsInc, /*IsLoad=*/true);
  assert(Opc && "no post-modify load opcode for value register");
  // The indexed opcode's $ireg is the pointer; the loaded value is in its
  // implicit Defs. The index modification is NOT modelled by the opcode, so add
  // an implicit def of the pointer to keep the advance visible post-expansion.
  Builder.buildInstr(Opc).addUse(PtrReg).addDef(PtrReg, RegState::Implicit);
  dematerializeReg(Builder, ValReg, Val, MF);
  if (StagedVal)
    pullStagingReg(Builder, ValReg);
  dematerializeReg(Builder, PtrReg, Ptr, MF);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandStorePostMod(MachineIRBuilder &Builder, MachineInstr &MI, bool IsInc) const {
  // Pseudo: outs ($ptr_out), in ($src value, $ptr_in); $ptr_in == $ptr_out.
  MachineFunction &MF = *MI.getMF();
  Register Val = MI.getOperand(1).getReg();
  Register Ptr = MI.getOperand(2).getReg();
  Register PtrReg = materializeReg(Builder, Ptr, MF);
  bool StagedVal = needsMaterialization(Val);
  // Preserve the value's staging real around the load + store window.
  if (StagedVal)
    pushStagingReg(Builder, getPhysRegFor(Val));
  Register ValReg = materializeReg(Builder, Val, MF);
  assert(ValReg != PtrReg && "post-modify value/pointer register conflict");
  unsigned Opc = getPostModOpcode(ValReg, IsInc, /*IsLoad=*/false);
  assert(Opc && "no post-modify store opcode for value register");
  // The store opcode models neither the value read nor the index advance, so
  // add both explicitly: implicit use of the value, implicit def of the pointer.
  Builder.buildInstr(Opc)
      .addUse(PtrReg)
      .addUse(ValReg, RegState::Implicit)
      .addDef(PtrReg, RegState::Implicit);
  if (StagedVal)
    pullStagingReg(Builder, ValReg);
  dematerializeReg(Builder, PtrReg, Ptr, MF);
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandJsrIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Pseudo: ins (idx base, offset imm), then the call's regmask + implicits.
  // After PEI the frame-index base is resolved to U/S plus a concrete offset.
  // Pick the indirect JSR opcode by the resolved offset width and emit it in the
  // indexed-indirect operand order (offset, base) — note this is the reverse of
  // the (base, offset) order the pseudo carried (which matched Load_iPtr_Mem so
  // eliminateFrameIndex could resolve the frame index). There is no 5-bit
  // indirect mode, so small offsets use the 8-bit form.
  Register Base = MI.getOperand(0).getReg();
  int64_t Offset = MI.getOperand(1).getImm();
  unsigned Opc = Offset == 0      ? MC6809::JSRi_o0I
                 : isInt<8>(Offset) ? MC6809::JSRi_o8I
                                    : MC6809::JSRi_o16I;
  MachineInstrBuilder MIB = Builder.buildInstr(Opc);
  if (Opc != MC6809::JSRi_o0I)
    MIB.addImm(Offset);
  MIB.addReg(Base);
  // Carry the call's register mask and implicit argument/clobber operands
  // (everything after the base + offset).
  for (unsigned I = 2, E = MI.getNumOperands(); I < E; ++I)
    MIB.add(MI.getOperand(I));
  MI.eraseFromParent();
}

// The PC-relative LEA opcode that computes a global's address into Reg.
static unsigned getLeaSymOpcode(Register Reg) {
  switch (Reg) {
  case MC6809::IX: return MC6809::LEAXi_o16PC;
  case MC6809::IY: return MC6809::LEAYi_o16PC;
  case MC6809::SU: return MC6809::LEAUi_o16PC;
  case MC6809::SS: return MC6809::LEASi_o16PC;
  default:         return 0;
  }
}

void MC6809InstrInfo::expandLeaSym(MachineIRBuilder &Builder, MachineInstr &MI) const {
  MachineFunction &MF = *MI.getMF();
  Register Dst = MI.getOperand(0).getReg();
  const MachineOperand &SymOp = MI.getOperand(1);
  MachineBasicBlock &MBB = *MI.getParent();

  // Spill dst: compute the address into IY staging, then store it to the slot.
  if (needsMaterialization(Dst)) {
    bool IsPIC = MF.getTarget().isPositionIndependent();
    BuildMI(MBB, MI, MI.getDebugLoc(), get(MC6809::LEAYi_o16PC))
        .add(SymOp)
        .addDef(MC6809::IY, RegState::Implicit);
    // Bug #387: a static-stack spill slot is reached by an extended (or
    // PC-relative) store, not SpillOff,U.
    if (isStaticSpillSlot(Dst, MF)) {
      BuildMI(MBB, MI, MI.getDebugLoc(),
              get(getSymStoreOpcode(MC6809::IY, /*IsDP=*/false, IsPIC)))
          .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(Dst, MF))
          .addUse(MC6809::IY, RegState::Implicit);
    } else {
      int SpillOff = computeSpillStackOffset(Dst, MF);
      BuildMI(MBB, MI, MI.getDebugLoc(),
              get(getStoreIdxOpcode(MC6809::IY, SpillOff)))
          .addUse(MC6809::IY, RegState::Implicit)
          .addImm(SpillOff)
          .addReg(MC6809::SU);
    }
    MI.eraseFromParent();
    return;
  }

  unsigned Opc = getLeaSymOpcode(Dst);
  assert(Opc && "no PC-relative LEA opcode for destination register");
  BuildMI(MBB, MI, MI.getDebugLoc(), get(Opc))
      .add(SymOp)
      .addDef(Dst, RegState::Implicit);
  MI.eraseFromParent();
}

// Map a concrete indexed load/store opcode to its indirect-indexed (`[n,r]`)
// sibling. The indirect addressing mode has no 5-bit-offset form, so an o5 pick
// is promoted to o8I. Returns 0 if no indirect form exists.
static unsigned indirectSiblingOf(unsigned Opc) {
  switch (Opc) {
#define IND3(R)                                                                 \
  case MC6809::R##i_o0: return MC6809::R##i_o0I;                                \
  case MC6809::R##i_o5: case MC6809::R##i_o8: return MC6809::R##i_o8I;          \
  case MC6809::R##i_o16: return MC6809::R##i_o16I;
  IND3(LDA) IND3(LDB) IND3(LDE) IND3(LDF) IND3(LDD) IND3(LDW) IND3(LDX) IND3(LDY)
  IND3(STA) IND3(STB) IND3(STE) IND3(STF) IND3(STD) IND3(STW) IND3(STX) IND3(STY)
  IND3(LDQ) IND3(STQ)
  // P3b: indirect-indexed arith/logical consumers (`op [n,r]`).
  IND3(ADDA) IND3(ADDB) IND3(ADDD) IND3(ADCA) IND3(ADCB)
  IND3(SUBA) IND3(SUBB) IND3(SUBD) IND3(SBCA) IND3(SBCB)
  IND3(ANDA) IND3(ANDB) IND3(ORA) IND3(ORB) IND3(EORA) IND3(EORB)
  IND3(ANDD) IND3(ORD) IND3(EORD) // HD6309 16-bit logical (expandIdxImm path)
  IND3(ADDW) IND3(SUBW)           // HD6309 W-accumulator i16 arith
  IND3(CMPA) IND3(CMPB) IND3(CMPD) IND3(CMPW) IND3(CMPX) IND3(CMPY)
#undef IND3
  default: return 0;
  }
}

// True if the pseudo is an indirect-indexed load/store variant (its concrete
// opcode, once chosen, is swapped to the `[n,r]` indirect sibling).
static bool isMemIndirectPseudo(unsigned Opc) {
  switch (Opc) {
  case MC6809::Load_i8_MemIndirect:  case MC6809::Load_i16_MemIndirect:
  case MC6809::Load_i32_MemIndirect: case MC6809::Store_i32_MemIndirect:
  case MC6809::Load_iPtr_MemIndirect:
  case MC6809::Store_i8_MemIndirect: case MC6809::Store_i16_MemIndirect:
  case MC6809::Store_iPtr_MemIndirect:
  // P3b: indirect-indexed arith/logical consumers.
  case MC6809::Add_i8_MemIndirect: case MC6809::Add_i16_MemIndirect:
  case MC6809::Sub_i8_MemIndirect: case MC6809::Sub_i16_MemIndirect:
  case MC6809::AND_i8_MemIndirect: case MC6809::AND_i16_MemIndirect:
  case MC6809::OR_i8_MemIndirect:  case MC6809::OR_i16_MemIndirect:
  case MC6809::XOR_i8_MemIndirect: case MC6809::XOR_i16_MemIndirect:
  // P3b: indirect-indexed compares (setcc + fused compare-branch).
  case MC6809::Compare_i8_MemIndirect: case MC6809::Compare_i16_MemIndirect:
  case MC6809::CompareBranch_i8_MemIndirect:
  case MC6809::CompareBranch_i16_MemIndirect:
    return true;
  default:
    return false;
  }
}

void MC6809InstrInfo::expandLoadIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Captured before any mutation: this expands the indirect-indexed pseudo, so
  // the concrete opcode chosen below is swapped to its `[n,r]` sibling.
  const bool Indirect = isMemIndirectPseudo(MI.getOpcode());

  auto DestRegOp = MI.getOperand(0);

  // If the destination is a spill register, load into D then store to the
  // spill slot. The SpillDSaveRestore pass (addPrePEI) handles saving and
  // restoring D around this operation when D is live — we don't need to
  // do emergency save/restore here. (The old emergency mechanism created
  // frame slots after PEI with unreliable offsets.)
  if (needsMaterialization(DestRegOp.getReg())) {
    MachineFunction &MF = *MI.getMF();
    if (isIndexSpillReg(DestRegOp.getReg())) {
      // INDEX spill dest: load into IY (staging), then store to spill slot.
      Register StageReg = MC6809::IY;
      MI.getOperand(0).setReg(StageReg);
      expandLoadIdx(Builder, MI);
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      emitSpillStoreFrom(PostBuilder, StageReg, DestRegOp.getReg(),
                         /*ExtraOffset=*/0, MF);
    } else if (isQSpillReg(DestRegOp.getReg())) {
      // Bug #308 (2026-05-19): Q-spill destination — conditional
      // expansion to balance Bug #308's correctness need against
      // the cycle/byte cost of an unconditional save/restore wrap
      // (catastrophic at LTO levels, +11% cycles).
      //
      // - AD-family dead: use emitTwoLDDSlotCopy (the legacy Bug #221
      //   path).  Cheap (8 bytes, only $ad clobbered).  No Bug #308
      //   manifest because there's no AD-vreg to corrupt.
      // - AD-family live: use LDQ + STQ through $aq, bracketed by
      //   emitAQPreservedOverHardStackScratch (Bug #298/#300/#305
      //   part 1 pattern).  Preserves all AQ sub-regs across the
      //   load → correct cross-load behaviour.  Costs ~15 bytes,
      //   but only in the cases where the OLD path would silently
      //   miscompile.
      //
      // History:
      //   Bug #221 (pre-Phase-B era) avoided LDQ here because it
      //   clobbers the full AQ family (AA/AB/AE/AF/AD/AW) — back when
      //   $aq couldn't be safely save/restored, an unconditional LDQ
      //   risked corrupting whatever vregs regalloc had placed in those
      //   sub-regs.  The chosen alternative was `emitTwoLDDSlotCopy`
      //   via $ad as transit — narrower clobber footprint.
      //
      //   But the two-LDD path has its own structural gap (Bug #308):
      //   the pseudo's `Defs=[AD]` (Bug #299) gets dead-marked by
      //   regalloc because no MI between this load and the next AD
      //   def reads the LOADED $ad value (it goes to the spill slot
      //   via STD).  Dead-marked = zero-width interval = doesn't
      //   conflict with cross-load AD-vreg ranges → vregs in $ad
      //   survive in regalloc's view while the runtime two-LDD
      //   silently clobbers them.  Verifier rejects + runtime
      //   miscompile.
      //
      //   Bug #297 (2026-05-15) closed and re-landed Phase B with
      //   native HD6309 i32 G_ADD/G_SUB + LDQ/STQ for the AQ-dst
      //   case, plus the `emitAQPreservedOverHardStackScratch`
      //   infrastructure for safe AQ save/restore.  That's the
      //   missing piece — with the wrap, the wider LDQ clobber is
      //   fully transparent to surrounding code.
      //
      // Now: just use LDQ → AQ; STQ AQ → slot, bracketed when needed.
      // When AQ-family is dead (typical, since cross-load values
      // usually live in $ad — Bug #308's affected pattern), no wrap
      // → 6 bytes (LDQ + STQ via Page 2).  When AQ-family is live,
      // add 9-byte wrap → 15 bytes.  Compare: old two-LDD path was
      // 8 bytes always, but silently miscompiled the cross-load
      // case.
      auto SrcIndex = MI.getOperand(1);
      auto SrcOffset = MI.getOperand(2);
      int DstSpillOff = computeSpillStackOffset(DestRegOp.getReg(), MF);
      Register DstSpillReg = DestRegOp.getReg();

      // Bug #272 Phase B core: accept CImm offsets too.
      // Bug #381: the optimized two-LDD / LDQ slot-copy below loads DIRECTLY
      // from `off,base` and never applies the indirect-sibling swap, so for an
      // indirect-indexed pseudo (`ld [off,base]`) it would silently drop the
      // indirection (qsort's `*(long*)b` -> `ldq 20,u` instead of `ldq [20,u]`).
      // Route the indirect case through the generic materialize -> recurse ->
      // dematerialize path below, which expands the load with the indirect swap.
      // A static-stack destination slot is NOT at DstSpillOff,U — the
      // optimized two-LDD / LDQ+STQ paths below hard-code that U-relative
      // store and would write into the (shrunk) dynamic frame. Skip them for a
      // static slot and fall through to the generic materialize -> recurse ->
      // dematerialize path, whose emitSpillStore emits the extended STQ.
      bool SrcOffIsImmediate =
          (SrcOffset.isImm() || SrcOffset.isCImm()) && !Indirect &&
          !isStaticSpillSlot(DestRegOp.getReg(), MF);
      if (SrcOffIsImmediate) {
        int SrcOffBytes = SrcOffset.isImm()
                              ? int(SrcOffset.getImm())
                              : int(SrcOffset.getCImm()->getSExtValue());

        // AD-family liveness gate — narrower than AQ-family because
        // emitTwoLDDSlotCopy only clobbers AD (and via sub-reg
        // aliasing AA/AB).  AW/AE/AF being live alone doesn't
        // require the save/restore wrap.
        MachineBasicBlock *MBB = MI.getParent();
        const TargetRegisterInfo &TRI =
            *Builder.getMF().getSubtarget().getRegisterInfo();
        LivePhysRegs LiveRegs(TRI);
        LiveRegs.addLiveIns(*MBB);
        SmallVector<std::pair<MCPhysReg, const MachineOperand *>, 4>
            Clobbers;
        for (auto It = MBB->begin(); It != MI.getIterator(); ++It) {
          Clobbers.clear();
          LiveRegs.stepForward(*It, Clobbers);
        }
        bool AdFamilyLive = false;
        for (MCPhysReg R : {MC6809::AD, MC6809::AA, MC6809::AB}) {
          if (LiveRegs.contains(R)) { AdFamilyLive = true; break; }
        }

        if (AdFamilyLive) {
          // Cross-load AD-vreg is live → use LDQ + STQ via $aq,
          // bracketed by AQ save/restore so AD's value is preserved
          // at runtime.  Bug #305 part 2 cluster A: add
          // `implicit-def $spill_q*` on the STQ to keep downstream
          // FAKE_USE / consumers happy.
          //
          // Wrapper does `LEAS -4,$ss` before body and `LEAS 4,$ss`
          // after.  If the body's LDQ uses $ss as its base register
          // (typical when the function has no frame pointer — common
          // after LTO inlining at -Os), the saved offset is now off
          // by 4 because $ss has been bumped down by 4.  Compensate
          // by adding 4 to the source offset when the base is $ss.
          // The U-relative STQ to the spill slot is unaffected (the
          // wrapper does not touch $su).
          int AdjSrcOff =
              (SrcIndex.getReg() == MC6809::SS)
                  ? SrcOffBytes + 4
                  : SrcOffBytes;
          auto Body = [&, AdjSrcOff]() {
            unsigned LdOpc =
                getLoadIdxOpcode(MC6809::AQ, AdjSrcOff);
            Builder.buildInstr(LdOpc)
                .addDef(MC6809::AQ, RegState::Implicit)
                .addImm(AdjSrcOff)
                .addReg(SrcIndex.getReg());
            unsigned StOpc =
                getStoreIdxOpcode(MC6809::AQ, DstSpillOff);
            Builder.buildInstr(StOpc)
                .addUse(MC6809::AQ, RegState::Implicit)
                .addImm(DstSpillOff)
                .addReg(MC6809::SU)
                .addDef(DstSpillReg, RegState::Implicit);
          };
          emitAQPreservedOverHardStackScratch(Builder, Body);
        } else {
          // AD-family dead → legacy two-LDD path is safe and cheap.
          emitTwoLDDSlotCopy(Builder, SrcOffBytes, SrcIndex.getReg(),
                             DstSpillOff, MC6809::SU, DstSpillReg);
        }
        MI.eraseFromParent();
        return;
      }
      // Reg-offset source path: fall through to the generic Q-via-AQ
      // approach below (rare; correctness over generality).
      Register RealReg = getPhysRegFor(DestRegOp.getReg());
      MI.getOperand(0).setReg(RealReg);
      MI.getOperand(0).setIsDead(false); // demat below reads the staging real
      expandLoadIdx(Builder, MI);
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      dematerializeReg(PostBuilder, RealReg, DestRegOp.getReg(), MF);
    } else {
      // ACC spill or imaginary dest: load into a real accumulator, then
      // store back. The staging clobbers that accumulator, and regalloc
      // believes it survives this pseudo (the pseudo deliberately declares
      // no accumulator Defs -- a fixed dead-def would veto allocating the
      // register at all). With the RS imaginaries allocatable, a live $ad
      // routinely spans an RS-destined load, so preserve it around the
      // staging via the hard stack (an emergency frame slot cannot be
      // created this late -- PEI has already laid the frame out). The
      // preserve is unconditional with an undef-marked push: no
      // neighborhood liveness probe can decide "holds a value someone
      // reads" once sub-register halves are defined and consumed
      // downstream, and pushing then popping garbage is the identity.
      Register RealReg = getPhysRegFor(DestRegOp.getReg());
      MachineBasicBlock &MBB = *MI.getParent();
      MachineIRBuilder PreBuilder(MBB, MI.getIterator());
      PreBuilder.buildInstr(MC6809::PSHSs)
          .addUse(RealReg, RegState::Undef);
      // The push moved S down: any pre-existing S-relative operand inside
      // the window must be displacement-compensated.
      compensateSSOperands(MI, stagingPushSize(RealReg));
      MI.getOperand(0).setReg(RealReg);
      // The pseudo's def may be dead-marked (unused imaginary dest); the
      // dematerialising store below READS the staging real, so the flag
      // must not survive onto the rewritten load's def.
      MI.getOperand(0).setIsDead(false);
      expandLoadIdx(Builder, MI);
      MachineBasicBlock::iterator InsertPt = MI.getIterator();
      ++InsertPt;
      MachineIRBuilder PostBuilder(*MI.getParent(), InsertPt);
      dematerializeReg(PostBuilder, RealReg, DestRegOp.getReg(), MF);
      PostBuilder.buildInstr(MC6809::PULSs).addDef(RealReg);
    }
    return;
  }

  // If the index register is an INDEX spill, load into a staging index reg.
  // Use IY if the dest operand is IX (avoid conflict), otherwise IX.
  if (isIndexSpillReg(MI.getOperand(1).getReg())) {
    Register SpillReg = MI.getOperand(1).getReg();
    //Register DestReg = MI.getOperand(0).getReg();
    Register StageReg = MC6809::IY;  // Prefer IY (callee-saved)
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    emitSpillLoadInto(PreBuilder, StageReg, SpillReg, /*ExtraOffset=*/0, MF);
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
      // No (dest reg, offset width) entry. offsetSizeInBits returns >16 only
      // for an offset outside the 16-bit range; otherwise the width is valid
      // (o5/o8/o16) and the destination register simply has no indexed-load
      // form in the table (e.g. a register class never expected here).
      llvm_unreachable(OffsetSize > 16
          ? "LoadIdx offset out of range for any indexed addressing form"
          : "no LoadIdx opcode for this destination register at a valid offset "
            "width (unmapped/unsupported destination register)");
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
    // Accumulator-offset load (ld d,x / b,x): exactly like the immediate path
    // above, but the offset rides a fixed accumulator (encoded by the opcode)
    // and so is an IMPLICIT use, not an explicit operand. Only the base index
    // is explicit; the value-def is operand 0 (made implicit). Adding the offset
    // as an already-implicit operand avoids the explicit-before-implicit
    // reordering that mis-marked it as a second explicit operand (the `d,d`
    // mis-render). reg-offset never combines with the indirect swap below.
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    MI.addOperand(IndexRegOp);
    MI.addOperand(MachineOperand::CreateReg(OffsetOp.getReg(), /*isDef=*/false,
                                            /*isImp=*/true, /*isKill=*/true));
  } else
    llvm_unreachable("Unknown offset type for LoadIdx");

  // The indirect-indexed form has identical operands -- only the postbyte (the
  // indirect bit, baked into the opcode) differs -- so swap the concrete opcode
  // for its `[n,r]` sibling.
  if (Indirect) {
    unsigned Ind = indirectSiblingOf(MI.getOpcode());
    assert(Ind && "no indirect-indexed sibling for this load opcode");
    MI.setDesc(Builder.getTII().get(Ind));
  }
}

void MC6809InstrInfo::expandStoreIdx(MachineIRBuilder &Builder, MachineInstr &MI) const {
  const bool Indirect = isMemIndirectPseudo(MI.getOpcode());

  // If the source is a spill or imaginary register, materialize it.
  // INDEX spills use IY (avoids clobbering IX which may be the index base).
  // ACC spills and imaginary use D with emergency save/restore.
  bool NeedRestore = false;
  if (isIndexSpillReg(MI.getOperand(0).getReg())) {
    Register SpillReg = MI.getOperand(0).getReg();
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder LoadBuilder(*MI.getParent(), MI.getIterator());
    emitSpillLoadInto(LoadBuilder, MC6809::IY, SpillReg, /*ExtraOffset=*/0, MF);
    MI.getOperand(0).setReg(MC6809::IY);
  } else if (isQSpillReg(MI.getOperand(0).getReg())) {
    // Bug #308 (2026-05-19): symmetric to expandLoadIdx's Q-spill
    // DEST path — switch from the AD-transit two-LDD slot-to-slot
    // approach (which has the regalloc-dead-marks-AD-def gap) to
    // LDQ-from-slot + STQ-to-dst via $aq, bracketed by
    // `emitAQPreservedOverHardStackScratch` when AQ-family is live.
    // See the long comment block in expandLoadIdx's Q-spill DEST
    // path for the full design rationale.
    auto SrcSpill = MI.getOperand(0);
    auto DstIndex = MI.getOperand(1);
    auto DstOffset = MI.getOperand(2);
    MachineFunction &MF = *MI.getMF();
    int SrcSpillOff = computeSpillStackOffset(SrcSpill.getReg(), MF);

    // Bug #272 Phase B core: accept CImm offsets too.
    // Bug #381: symmetric to expandLoadIdx -- the optimized two-LDD / STQ
    // slot-copy stores DIRECTLY to `off,base` and never applies the indirect
    // swap, so route the indirect-indexed case (`st [off,base]`) through the
    // generic materialize -> generic-build -> indirect-swap path below.
    bool DstOffIsImmediate =
        (DstOffset.isImm() || DstOffset.isCImm()) && !Indirect;
    if (DstOffIsImmediate) {
      int DstOffBytes = DstOffset.isImm()
                            ? int(DstOffset.getImm())
                            : int(DstOffset.getCImm()->getSExtValue());

      // AD-family liveness gate (same conditional as expandLoadIdx
      // Q-spill DST above — narrower than AQ-family to minimise LTO
      // overhead; emitTwoLDDSlotCopy only clobbers AD/AA/AB so
      // AW/AE/AF being live alone doesn't require save/restore).
      MachineBasicBlock *MBB = MI.getParent();
      const TargetRegisterInfo &TRI =
          *Builder.getMF().getSubtarget().getRegisterInfo();
      LivePhysRegs LiveRegs(TRI);
      LiveRegs.addLiveIns(*MBB);
      SmallVector<std::pair<MCPhysReg, const MachineOperand *>, 4>
          Clobbers;
      for (auto It = MBB->begin(); It != MI.getIterator(); ++It) {
        Clobbers.clear();
        LiveRegs.stepForward(*It, Clobbers);
      }
      bool AdFamilyLive = false;
      for (MCPhysReg R : {MC6809::AD, MC6809::AA, MC6809::AB}) {
        if (LiveRegs.contains(R)) { AdFamilyLive = true; break; }
      }

      // Bug #387: a static-stack source slot is reached by an extended (or
      // PC-relative) LDQ, not `SrcSpillOff,U`. The U-relative optimized paths
      // below would read from the (shrunk) dynamic frame. Load the slot into
      // $aq via the static opcode, then STQ to the (dynamic) destination —
      // wrapped in AQ-preservation when an AD-family value is live across it.
      if (isStaticSpillSlot(SrcSpill.getReg(), MF)) {
        bool IsPIC = MF.getTarget().isPositionIndependent();
        int StaticOff = staticSpillOffset(SrcSpill.getReg(), MF);
        auto Body = [&]() {
          unsigned LdOpc = getSymLoadOpcode(MC6809::AQ, /*IsDP=*/false, IsPIC);
          Builder.buildInstr(LdOpc)
              .addTargetIndex(MC6809::TI_STATIC_STACK, StaticOff)
              .addDef(MC6809::AQ, RegState::Implicit);
          // The STQ base offset shifts by 4 only when the wrapper's LEAS moved
          // $ss (i.e. AD-family live) and the destination base IS $ss.
          int AdjDstOff = (AdFamilyLive && DstIndex.getReg() == MC6809::SS)
                              ? DstOffBytes + 4
                              : DstOffBytes;
          unsigned StOpc = getStoreIdxOpcode(MC6809::AQ, AdjDstOff);
          Builder.buildInstr(StOpc)
              .addUse(MC6809::AQ, RegState::Implicit)
              .addImm(AdjDstOff)
              .addReg(DstIndex.getReg());
        };
        if (AdFamilyLive)
          emitAQPreservedOverHardStackScratch(Builder, Body);
        else
          Body();
        MI.eraseFromParent();
        return;
      }

      if (AdFamilyLive) {
        // Use LDQ + STQ via $aq + save/restore — preserves AD-family
        // across the two-LDD's transient clobber.
        //
        // Wrapper does `LEAS -4,$ss` before body and `LEAS 4,$ss`
        // after.  If the body's STQ uses $ss as its base register
        // (typical when the function has no frame pointer — common
        // after LTO inlining at -Os), the saved offset is now off by
        // 4 because $ss has been bumped down by 4.  Compensate by
        // adding 4 to the destination offset when the base is $ss.
        // The U-relative LDQ from the spill slot is unaffected (the
        // wrapper does not touch $su).
        int AdjDstOff =
            (DstIndex.getReg() == MC6809::SS)
                ? DstOffBytes + 4
                : DstOffBytes;
        auto Body = [&, AdjDstOff]() {
          unsigned LdOpc = getLoadIdxOpcode(MC6809::AQ, SrcSpillOff);
          Builder.buildInstr(LdOpc)
              .addDef(MC6809::AQ, RegState::Implicit)
              .addImm(SrcSpillOff)
              .addReg(MC6809::SU);
          unsigned StOpc = getStoreIdxOpcode(MC6809::AQ, AdjDstOff);
          Builder.buildInstr(StOpc)
              .addUse(MC6809::AQ, RegState::Implicit)
              .addImm(AdjDstOff)
              .addReg(DstIndex.getReg());
        };
        emitAQPreservedOverHardStackScratch(Builder, Body);
      } else {
        // AD-family dead → legacy two-LDD path is safe and cheap.
        emitTwoLDDSlotCopy(Builder, SrcSpillOff, MC6809::SU,
                           DstOffBytes, DstIndex.getReg());
      }
      MI.eraseFromParent();
      return;
    }
    // Reg-offset dst path: fall through to the generic
    // emergency-save + materialise approach (rare; correctness over
    // generality).
    MachineIRBuilder LoadBuilder(*MI.getParent(), MI.getIterator());
    // Preserve $ad around the materialisation via the hard stack: PEI has
    // already laid the frame out, so a CreateStackObject here would never
    // receive an offset (every such "emergency" slot silently resolved to
    // 0,U and smashed the local living there). The push is unconditional
    // and undef-marked: no neighborhood liveness probe can decide "holds
    // a value someone reads" once sub-register halves are defined and
    // consumed downstream, and pushing then popping garbage is the
    // identity.
    LoadBuilder.buildInstr(MC6809::PSHSs).addUse(MC6809::AD, RegState::Undef);
    NeedRestore = true;
    // The push moved S down by 2: any pre-existing S-relative operand
    // inside the window must be displacement-compensated.
    compensateSSOperands(MI, 2);
    Register RealReg = materializeReg(LoadBuilder, SrcSpill.getReg(), MF);
    MI.getOperand(0).setReg(RealReg);
  } else if (needsMaterialization(MI.getOperand(0).getReg())) {
    Register SrcReg = MI.getOperand(0).getReg();
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder LoadBuilder(*MI.getParent(), MI.getIterator());

    // Preserve $ad unconditionally (see the spill-source arm above --
    // same hard-stack + undef-push rationale).
    LoadBuilder.buildInstr(MC6809::PSHSs).addUse(MC6809::AD, RegState::Undef);
    NeedRestore = true;
    // The push moved S down by 2: any pre-existing S-relative operand
    // inside the window must be displacement-compensated.
    compensateSSOperands(MI, 2);

    Register RealReg = materializeReg(LoadBuilder, SrcReg, MF);
    MI.getOperand(0).setReg(RealReg);
  }

  // If the index register is an INDEX spill, load into a staging index reg.
  // Use IY if the source operand is IX (avoid conflict), otherwise IX.
  if (isIndexSpillReg(MI.getOperand(1).getReg())) {
    Register SpillReg = MI.getOperand(1).getReg();
    //Register SrcReg = MI.getOperand(0).getReg();
    Register StageReg = MC6809::IY;  // Prefer IY (callee-saved)
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    emitSpillLoadInto(PreBuilder, StageReg, SpillReg, /*ExtraOffset=*/0, MF);
    MI.getOperand(1).setReg(StageReg);
  }

  auto SrcRegOp = MI.getOperand(0); // re-read AFTER spill fix
  auto IndexRegOp = MI.getOperand(1);
  auto OffsetOp = MI.getOperand(2);

  // Bug #272 Phase B Scope A followup: probe true liveness of the source
  // register at this MI. If it isn't definitely live (LQR_Live), mark the
  // expanded implicit-use Undef so the verifier accepts the read. The
  // accumulator-hierarchy Defs cleanup (Scope A/B/C) made regalloc more
  // aggressive about sub-register packing, surfacing cases where a vreg
  // assigned to (e.g.) $aa appears in a spill store after the byte's value
  // has been killed via super-reg containment — the verifier sees a bare
  // `implicit $aa` and trips.  Same Undef-protection shape as Bug #275 /
  // the MS-pass fix at the save-emission sites, applied at the expansion
  // layer so it also covers regalloc-emitted spills (not just MS-pass
  // saves).
  // Bug #272 Phase B Scope A followup: probe true liveness of the source
  // operand using a LivePhysRegs walk from MBB.begin() — matches the
  // analysis the post-RA verifier uses.  computeRegisterLiveness with a
  // bounded neighborhood mis-classifies cases where the search window
  // doesn't reach a def/kill (returns LQR_Unknown) and also sees the MI's
  // own explicit USE as a "live" signal.
  //
  // The accumulator-hierarchy Defs cleanup (Scope A/B/C) made regalloc
  // more aggressive about byte-granular packing and surfaced cases where
  // a vreg assigned to (e.g.) $aa appears in a spill store after the
  // byte's value was killed via super-reg containment — the verifier
  // sees a bare `implicit $aa` read and trips.  Mark Undef when our
  // walked-from-livein analysis agrees with the verifier that the
  // source is dead.
  Register SrcReg = SrcRegOp.getReg();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();
  bool IsAccByte = SrcReg.isPhysical() &&
      (SrcReg == MC6809::AA || SrcReg == MC6809::AB ||
       SrcReg == MC6809::AE || SrcReg == MC6809::AF ||
       SrcReg == MC6809::AD || SrcReg == MC6809::AW);
  bool SrcIsImpliedUndef = false;
  if (IsAccByte) {
    MachineBasicBlock *MBB = MI.getParent();
    LivePhysRegs LiveRegs(TRI);
    LiveRegs.addLiveIns(*MBB);
    SmallVector<std::pair<MCPhysReg, const MachineOperand *>, 4> Clobbers;
    for (auto It = MBB->begin(); It != MI.getIterator(); ++It) {
      Clobbers.clear();
      LiveRegs.stepForward(*It, Clobbers);
    }
    SrcIsImpliedUndef = !LiveRegs.contains(SrcReg);
  }

  MI.removeOperand(2);
  MI.removeOperand(1);

  int OffsetSize = offsetSizeInBits(OffsetOp);
  if (OffsetSize >= 0) {
    RegPlusOffsetLen Lookup{SrcRegOp.getReg(), OffsetSize};
    auto OpcodePair = StoreIdxImmOpcode.find(Lookup);
    if (OpcodePair == StoreIdxImmOpcode.end())
      // Mirror of the LoadIdx case above: >16 means the offset is out of range,
      // otherwise the source register has no indexed-store form in the table.
      llvm_unreachable(OffsetSize > 16
          ? "StoreIdx offset out of range for any indexed addressing form"
          : "no StoreIdx opcode for this source register at a valid offset "
            "width (unmapped/unsupported source register)");
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    if (SrcIsImpliedUndef)
      MI.getOperand(0).setIsUndef(true);
    if (OffsetSize > 0)
      MI.addOperand(OffsetOp);
    MI.addOperand(IndexRegOp);
  } else if (OffsetOp.isReg()) {
    RegPlusReg Lookup{SrcRegOp.getReg(), OffsetOp.getReg()};
    auto OpcodePair = StoreIdxRegOpcode.find(Lookup);
    if (OpcodePair == StoreIdxRegOpcode.end())
      llvm_unreachable("Unexpected StoreIdx register offset operand.");
    // Accumulator-offset store (st d,x / b,x): value (operand 0) is an implicit
    // use, the base index is explicit, the offset accumulator is an implicit
    // use. Operands 1 and 2 were removed above, so re-add them (the old code
    // called getOperand(2) on the already-shortened operand list -> OOB crash).
    MI.setDesc(Builder.getTII().get(OpcodePair->getSecond()));
    MI.getOperand(0).setImplicit();
    if (SrcIsImpliedUndef)
      MI.getOperand(0).setIsUndef(true);
    MI.addOperand(IndexRegOp);
    MI.addOperand(MachineOperand::CreateReg(OffsetOp.getReg(), /*isDef=*/false,
                                            /*isImp=*/true, /*isKill=*/true));
  } else
    llvm_unreachable("Unknown offset type for StoreIdx");

  // Swap to the indirect-indexed sibling (same operands, indirect postbyte).
  if (Indirect) {
    unsigned Ind = indirectSiblingOf(MI.getOpcode());
    assert(Ind && "no indirect-indexed sibling for this store opcode");
    MI.setDesc(Builder.getTII().get(Ind));
  }

  // Restore the preserved $ad from the hard stack.
  if (NeedRestore) {
    MachineBasicBlock::iterator RestorePt = MI.getIterator();
    ++RestorePt;
    MachineIRBuilder RestoreBuilder(*MI.getParent(), RestorePt);
    RestoreBuilder.buildInstr(MC6809::PULSs).addDef(MC6809::AD);
  }
}

// Forward declarations for 6809 register-to-memory helpers.
//
// Bug #122 / #130: every caller MUST pass the `_o16` opcode explicitly.
// A `0` `_o16` silently falls back to the `_o8` form inside the helpers,
// which wraps offsets >127 to negative — a wrong-code class of bug. The
// parameter is required (no default) so the compiler refuses any new
// caller that forgets it. The relaxation pass TODO is the long-term fix;
// this is the short-term footgun-removal.
static void emit6809RegByteFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned Opc_o8, unsigned Opc_o5,
                                   unsigned Opc_o16);
static void emit6809RegPairFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned OpcB_o8, unsigned OpcA_o8,
                                   unsigned OpcB_o5, unsigned OpcA_o0,
                                   unsigned OpcB_o16,
                                   unsigned OpcA_o16);
static void getByteOpcodes(Register LHS,
                           unsigned OpcA_o8, unsigned OpcB_o8,
                           unsigned OpcA_o5, unsigned OpcB_o5,
                           unsigned OpcA_o16, unsigned OpcB_o16,
                           unsigned &Opc_o8, unsigned &Opc_o5,
                           unsigned &Opc_o16);

void MC6809InstrInfo::expandANDReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for ANDReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ANDRp)) {
    // page-3 reg-reg path emitted
  } else if (MI.getOpcode() == MC6809::AND_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ANDBi_o8, MC6809::ANDAi_o8,
                           MC6809::ANDBi_o5, MC6809::ANDAi_o0,
                           MC6809::ANDBi_o16, MC6809::ANDAi_o16);
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::ANDAi_o8,  MC6809::ANDBi_o8,
                   MC6809::ANDAi_o5,  MC6809::ANDBi_o5,
                   MC6809::ANDAi_o16, MC6809::ANDBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}




void MC6809InstrInfo::expandORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for ORReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ORRp)) {
    // page-3 reg-reg path emitted
  } else if (MI.getOpcode() == MC6809::OR_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ORBi_o8, MC6809::ORAi_o8,
                           MC6809::ORBi_o5, MC6809::ORAi_o0,
                           MC6809::ORBi_o16, MC6809::ORAi_o16);
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::ORAi_o8,  MC6809::ORBi_o8,
                   MC6809::ORAi_o5,  MC6809::ORBi_o5,
                   MC6809::ORAi_o16, MC6809::ORBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandXORReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for XORReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::EORRp)) {
    // page-3 reg-reg path emitted
  } else if (MI.getOpcode() == MC6809::XOR_i16_Reg) {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::EORBi_o8, MC6809::EORAi_o8,
                           MC6809::EORBi_o5, MC6809::EORAi_o0,
                           MC6809::EORBi_o16, MC6809::EORAi_o16);
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::EORAi_o8,  MC6809::EORBi_o8,
                   MC6809::EORAi_o5,  MC6809::EORBi_o5,
                   MC6809::EORAi_o16, MC6809::EORBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source 1 must be same for AddReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ADDRp)) {
    // page-3 reg-reg path emitted
  } else if (MI.getOpcode() == MC6809::Add_i8_Reg) {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::ADDAi_o8,  MC6809::ADDBi_o8,
                   MC6809::ADDAi_o5,  MC6809::ADDBi_o5,
                   MC6809::ADDAi_o16, MC6809::ADDBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           Opc_o8, Opc_o5, Opc_o16);
  } else {
    // i16 reg-reg add: low byte ADDB sets carry, high byte ADCA must
    // consume it. Earlier code used ADDA for the high byte and silently
    // dropped the carry between bytes 0 and 1 of the i16 add — so e.g.
    // 0xFFFF + 0x0001 produced 0xFF00 instead of 0x0000-with-carry-out.
    // The bug was dormant because the legalizer/selector almost always
    // chose the memory-form ADDD instead of routing through this
    // register-register expansion path. Compare with expandSubReg
    // which has always used SUBB; SBCA correctly.
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ADDBi_o8, MC6809::ADCAi_o8,
                           MC6809::ADDBi_o5, MC6809::ADCAi_o0,
                           MC6809::ADDBi_o16, MC6809::ADCAi_o16);
  }
  MI.eraseFromParent();
}

// Expand `AddSetCarry_i{8,16}_Reg` (i32-narrow lo half — does the
// add and PRODUCES carry-out for the upper half to consume).
//
// Operand layout (this is the NON-Use variant — 4 operands)
// ---------------------------------------------------------
// From the MC6809ArithmeticBaseCarry multiclass in
// MC6809InstrFamilies.td:
//
//   class MC6809ArithmeticBaseCarry<dst, carry, src, operand> {
//     OutOperandList = (outs dst:$dst, carry:$carry);
//     InOperandList  = !con((ins src:$src), operand);  // adds src2
//     Constraints    = "$dst = $src";                  // tied
//   }
//
// Resulting MachineInstr operand indices:
//
//   op0 : dst    (def, ACC8 / ACC16 / ACC32) — tied to op2
//   op1 : carry  (def, s1 vreg in PHANTOM_CARRY pool)
//   op2 : src    (use, == op0 by tie)
//   op3 : src2   (use)
//
// Then the implicit-defs from `let Defs = [NZ, V]` follow at op4
// ($nz) and op5 ($v). It is critically important not to confuse
// op3 (the real src2) with op4 (an implicit-def $nz).
//
// Bug #64
// -------
// Earlier code here used `MI.getOperand(4)` for src2 — that was
// the implicit-def $nz, not src2. The result was a passed-as-RHS
// register that didn't make sense, producing garbage assembly
// like `pshs NZ; sbcb 1,s; sbca ,s` once the call reached
// emit6809RegPairFromMem. The Use variant (`AddSetCarryUse_*_Reg`)
// has 5 operands and op4 IS its src2 — the bug came from copying
// the Use-variant code without adjusting the index. Fixed.
void MC6809InstrInfo::expandAddSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #186 v5: pseudo operand layout shifted (no carry-out at op 1).
  // New layout: dst (op 0), src tied (op 1), src2 (op 2).
  // CC.C/V write is implicit via Defs=[NZ,V,C].
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for AddSetCarryReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ADDRp)) {
    // page-3 reg-reg path emitted
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(),  // LHS = dst (== src by tie)
                           MI.getOperand(2).getReg(),  // RHS = src2
                           MC6809::ADDBi_o8, MC6809::ADCAi_o8,
                           MC6809::ADDBi_o5, MC6809::ADCAi_o0,
                           MC6809::ADDBi_o16, MC6809::ADCAi_o16);
  }
  MI.eraseFromParent();
}

// Expand `AddSetCarry_i8_Reg` — byte-level add that PRODUCES carry-out.
// Same operand layout as the i16 variant (4 operands: dst, carry, src, src2),
// but uses emit6809RegByteFromMem instead of emit6809RegPairFromMem.
/// Determine the correct A or B opcode variant for a byte-level
/// carry chain operation based on which accumulator the LHS maps to,
/// including the `_o16` variant for large frame offsets (bug #122).
///
/// Every `expand*Reg` expansion that emits via `emit6809RegByteFromMem`
/// MUST route the opcode pick through this helper — hardcoding `B`
/// variants is silently wrong when the LHS maps to AA (bug #130).
static void getByteOpcodes(Register LHS,
                           unsigned OpcA_o8, unsigned OpcB_o8,
                           unsigned OpcA_o5, unsigned OpcB_o5,
                           unsigned OpcA_o16, unsigned OpcB_o16,
                           unsigned &Opc_o8, unsigned &Opc_o5,
                           unsigned &Opc_o16) {
  Register RealLHS = needsMaterialization(LHS) ? getPhysRegFor(LHS) : LHS;
  // Bug #311 Phase 1 step 1.5: AALSB retired.
  bool UseA = (RealLHS == MC6809::AA);
  Opc_o8 = UseA ? OpcA_o8 : OpcB_o8;
  Opc_o5 = UseA ? OpcA_o5 : OpcB_o5;
  Opc_o16 = UseA ? OpcA_o16 : OpcB_o16;
}

// Bug #186 v5: all SetCarry/SetCarryUse expansions migrate to the new
// operand layout (no explicit carry operand). Layouts:
//   SetCarry:    op 0 = dst, op 1 = src tied, op 2 = src2
//   SetCarryUse: op 0 = dst, op 1 = src tied, op 2 = src2  (carry-in
//                read implicitly via Uses=[C])
// CC.C/V write is implicit via Defs=[NZ,V,C].
void MC6809InstrInfo::expandAddSetCarryByteReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg());
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ADDRp)) {
    // page-3 reg-reg path emitted
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::ADDAi_o8, MC6809::ADDBi_o8,
                   MC6809::ADDAi_o5, MC6809::ADDBi_o5,
                   MC6809::ADDAi_o16, MC6809::ADDBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder, MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(), Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandAddSetCarryUseByteReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg());
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ADCRp)) {
    // page-3 reg-reg path emitted
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::ADCAi_o8, MC6809::ADCBi_o8,
                   MC6809::ADCAi_o5, MC6809::ADCBi_o5,
                   MC6809::ADCAi_o16, MC6809::ADCBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder, MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(), Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryByteReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg());
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SUBRp)) {
    // page-3 reg-reg path emitted
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::SUBAi_o8, MC6809::SUBBi_o8,
                   MC6809::SUBAi_o5, MC6809::SUBBi_o5,
                   MC6809::SUBAi_o16, MC6809::SUBBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder, MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(), Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryUseByteReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg());
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SBCRp)) {
    // page-3 reg-reg path emitted
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::SBCAi_o8, MC6809::SBCBi_o8,
                   MC6809::SBCAi_o5, MC6809::SBCBi_o5,
                   MC6809::SBCAi_o16, MC6809::SBCBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder, MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(), Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

/// Map an indexed opcode (e.g. ADDBi_o8) to its direct-page variant
/// (e.g. ADDBd). Used when RHS is an Imag8 register — the imaginary
/// regs live in the direct page and can be addressed with the shorter
/// direct-page instruction form instead of pushing onto the stack.
/// Byte sub-registers of an Imag16 parent (big-endian: HI at the base
/// DP address, LO at base+1).
static Register imag16LoByte(Register R) {
  switch (R) {
  case MC6809::RS0: return MC6809::RS0LO;
  case MC6809::RS1: return MC6809::RS1LO;
  case MC6809::RS2: return MC6809::RS2LO;
  case MC6809::RS3: return MC6809::RS3LO;
  default: llvm_unreachable("not an Imag16 register");
  }
}
static Register imag16HiByte(Register R) {
  switch (R) {
  case MC6809::RS0: return MC6809::RS0HI;
  case MC6809::RS1: return MC6809::RS1HI;
  case MC6809::RS2: return MC6809::RS2HI;
  case MC6809::RS3: return MC6809::RS3HI;
  default: llvm_unreachable("not an Imag16 register");
  }
}

static unsigned getDirectPageOpcode(unsigned IdxOpc) {
  switch (IdxOpc) {
  case MC6809::ADDBi_o8: case MC6809::ADDBi_o5: return MC6809::ADDBd;
  case MC6809::ADDAi_o8: case MC6809::ADDAi_o0: return MC6809::ADDAd;
  case MC6809::ADCAi_o8: case MC6809::ADCAi_o0: return MC6809::ADCAd;
  case MC6809::ADCBi_o8: case MC6809::ADCBi_o5: return MC6809::ADCBd;
  case MC6809::SUBAi_o8: case MC6809::SUBAi_o0: return MC6809::SUBAd;
  case MC6809::SUBBi_o8: case MC6809::SUBBi_o5: return MC6809::SUBBd;
  case MC6809::SBCAi_o8: case MC6809::SBCAi_o0: return MC6809::SBCAd;
  case MC6809::SBCBi_o8: case MC6809::SBCBi_o5: return MC6809::SBCBd;
  case MC6809::ANDBi_o8: case MC6809::ANDBi_o5: return MC6809::ANDBd;
  case MC6809::ANDAi_o8: case MC6809::ANDAi_o0: return MC6809::ANDAd;
  case MC6809::ORBi_o8:  case MC6809::ORBi_o5:  return MC6809::ORBd;
  case MC6809::ORAi_o8:  case MC6809::ORAi_o0:  return MC6809::ORAd;
  case MC6809::EORBi_o8: case MC6809::EORBi_o5: return MC6809::EORBd;
  case MC6809::EORAi_o8: case MC6809::EORAi_o0: return MC6809::EORAd;
  default: llvm_unreachable("No direct-page variant for this opcode");
  }
}

// The extended (absolute) or PC-relative variant of an indexed
// spill-access opcode, for a slot the static-stack allocator moved into the
// static_stack global. Mirrors getDirectPageOpcode; the value operand(s) stay
// the same, only the addressing tail changes (a TI_STATIC_STACK target index
// replaces the offset,U pair). PIC uses the PC-relative form because the
// static_stack global is materialised relative to PC.
static unsigned getStaticStackOpcode(unsigned IdxOpc, bool IsPIC) {
#define SS(BASE)                                                               \
  case MC6809::BASE##i_o8: case MC6809::BASE##i_o5:                            \
  case MC6809::BASE##i_o0: case MC6809::BASE##i_o16:                           \
    return IsPIC ? MC6809::BASE##i_o16PC : MC6809::BASE##e;
  switch (IdxOpc) {
  SS(ADDB) SS(ADDA) SS(ADCA) SS(ADCB) SS(SUBA) SS(SUBB) SS(SBCA) SS(SBCB)
  SS(ANDB) SS(ANDA) SS(ORB) SS(ORA) SS(EORB) SS(EORA)
  SS(ADDD) SS(SUBD) SS(ADCD) SS(SBCD) SS(ADDW) SS(SUBW)
  SS(CMPA) SS(CMPB) SS(CMPD)
  // Index-domain pointer compares (pickCmpO8O16 emits these when the
  // spilled operand is a pointer/index register).
  SS(CMPX) SS(CMPY) SS(CMPU) SS(CMPS)
  default:
    llvm_unreachable("No static-stack variant for this opcode");
  }
#undef SS
}

unsigned MC6809InstrInfo::getStaticSymOpcode(unsigned MemOpc) const {
  switch (MemOpc) {
  case MC6809::Load_i8_Mem:        return MC6809::Load_i8_Sym;
  case MC6809::Load_i16_Mem:       return MC6809::Load_i16_Sym;
  case MC6809::Load_iPtr_Mem:      return MC6809::Load_iPtr_Sym;
  case MC6809::Load_i32_Mem:
  case MC6809::SpillLoad_i32_Mem:  return MC6809::Load_i32_Sym;
  case MC6809::Store_i8_Mem:       return MC6809::Store_i8_Sym;
  case MC6809::Store_i16_Mem:      return MC6809::Store_i16_Sym;
  case MC6809::Store_iPtr_Mem:     return MC6809::Store_iPtr_Sym;
  case MC6809::Store_i32_Mem:
  case MC6809::SpillStore_i32_Mem: return MC6809::Store_i32_Sym;
  // i32 add/sub whose memory operand is a static-frame local, and the
  // address-of (LEA) of a static-frame local — the escaping-object cases
  // that let the whole frame move to the static region.
  case MC6809::Add_i32_Mem:        return MC6809::Add_i32_Sym;
  case MC6809::Sub_i32_Mem:        return MC6809::Sub_i32_Sym;
  case MC6809::LEA_Ptr_Imm:        return MC6809::Lea_iPtr_Sym;
  default:                         return 0;
  }
}

/// Emit a 6809 8-bit register-register operation by loading LHS into
/// an accumulator and reading RHS from memory.
///
/// The 6809 has no native register-register byte arithmetic — every
/// instruction takes one ACC source and one memory source. So a
/// "byte op two regs" pseudo expands to:
///   1. Make sure LHS is in an accumulator (A or B).
///   2. Get RHS into a memory location we can address.
///   3. Emit the op as `<acc> := <acc> op <mem>`.
///   4. Store the result back if LHS was a spill or imaginary reg.
///
/// `Opc_o8` is the opcode for the U-relative form (8-bit displacement,
/// used for the spill RHS path). `Opc_o5` is the opcode for the
/// 5-bit-displacement form (used for the S-relative `0,s` access
/// after PSHS). The two forms encode differently in the postbyte.
///
/// LHS handling
/// ------------
/// `RealLHS` is the actual hardware register the accumulator op will
/// touch. For LHS = an imaginary or A/B-spill register, materializeReg
/// emits an LDA/LDB and `RealLHS` is the AA or AB it loaded into.
///
/// RHS handling — THREE PATHS
/// --------------------------
///   (a) RHS is a SPILL_A*/SPILL_B*: read it directly via U-relative
///       addressing from the spill slot — no need to push anything.
///       Used by bug #63's "skip the second ACC spill" path in
///       MaterializeSpills (see MC6809MaterializeSpills.cpp). The
///       byte spills use getSpillByteOffset for the correct offset
///       within the parent frame slot (big-endian: A at slot+0,
///       B at slot+1).
///
///   (b) RHS is an Imag8 register: use the direct-page opcode to
///       read it directly from its DP location. No push needed.
///
///   (c) RHS is a real register or other imaginary register: push it
///       onto the S stack with PSHS, operate from `0,s`, then LEAS
///       to deallocate. Imaginary regs are materialized first.
static void emit6809RegByteFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned Opc_o8, unsigned Opc_o5,
                                   unsigned Opc_o16) {
  assert(Opc_o16 != 0 &&
         "emit6809RegByteFromMem: _o16 opcode is required (bug #122/#130). "
         "A 0 here silently falls back to _o8 and wraps offsets >127 to "
         "negative. Pass the correct *i_o16 variant explicitly.");
  MachineFunction &MF = Builder.getMF();
  Register OrigLHS = LHS;
  Register RealLHS = needsMaterialization(LHS) ? getPhysRegFor(LHS) : LHS;
  // The accumulator half (A or B) is determined by where LHS lives.
  // SPILL_A* / Imag8-A maps to AA; SPILL_B* / Imag8-B maps to AB.
  // Bug #311 Phase 1 step 1.5: AALSB retired.
  Register AccReg = (RealLHS == MC6809::AA) ? MC6809::AA : MC6809::AB;
  // An imaginary/spill LHS stages through AccReg, which the pseudo does
  // not declare and regalloc may have left live -- preserve it for the
  // whole window (self-referencing 0,s reads below are emitted after
  // this push, so they stay consistent).
  bool StagedLHS = needsMaterialization(LHS);
  if (StagedLHS)
    pushStagingReg(Builder, AccReg);
  // If the RHS is a physical register that's the SAME as the LHS
  // accumulator, push it BEFORE materializing the LHS. Otherwise
  // the LHS load clobbers the RHS value. This happens when the
  // byte-level carry chain has a spill-backed LHS and the RHS was
  // left in A/B by a preceding operation (e.g., libcall result).
  Register RealRHS = needsMaterialization(RHS) ? getPhysRegFor(RHS) : RHS;
  bool PushedEarly = false;
  if (!isSpillReg(RHS) &&
      !(RHS.isPhysical() && MC6809::Imag8RegClass.contains(RHS)) &&
      !isImag16ByteSubReg(RHS) && RealRHS == AccReg) {
    if (needsMaterialization(RHS))
      RealRHS = materializeReg(Builder, RHS, MF);
    Builder.buildInstr(MC6809::PSHSs, {}, {RealRHS});
    PushedEarly = true;
  }
  if (needsMaterialization(LHS))
    materializeReg(Builder, LHS, MF);
  RealRHS = RHS;
  if (isSpillReg(RHS)) {
    // Path (a): read RHS directly from its spill slot. emitSpillArith emits the
    // U-relative form for a dynamic slot (picking o8 vs o16 by the offset —
    // Bug #122's large-frame guard) or the extended TI_STATIC_STACK form for a
    // static-stack slot. computeSpillStackOffset already includes the
    // byte offset within the parent D slot (A at +0, B at +1 on big-endian).
    LLVM_DEBUG(dbgs() << "emit6809RegByteFromMem: path(a) RHS="
               << printReg(RHS) << "\n");
    emitSpillArith(Builder, Opc_o8, Opc_o16, AccReg, RHS, MF);
  } else if (RHS.isPhysical() && (MC6809::Imag8RegClass.contains(RHS) ||
                                  isImag16ByteSubReg(RHS))) {
    // Path (b): RHS is an Imag8 register or an RS byte sub-register —
    // use the direct-page opcode to read it directly from its DP
    // location. This avoids the push/pop overhead (and any staging
    // that would clobber the LHS accumulator) and is correct because
    // the imaginaries live in the direct page at fixed addresses.
    unsigned DPOpc = getDirectPageOpcode(Opc_o8);
    Builder.buildInstr(DPOpc).addReg(RHS);
  } else if (PushedEarly) {
    // RHS was pushed to S before LHS materialization (collision case
    // above). The real register no longer holds the value, so we must
    // consume it from the stack via 0,S rather than re-pushing.
    Builder.buildInstr(Opc_o5)
        .addDef(AccReg, RegState::Implicit)
        .addImm(0).addReg(MC6809::SS);
    Builder.buildInstr(MC6809::LEASi_o5)
        .addImm(1).addReg(MC6809::SS);
  } else {
    // Path (c): PSHS RealRHS; op 0,S; LEAS 1,S.
    // Stage 5 of MC6809PostRASpillOpt (bug #185) folds this triple to
    // PSHS RealRHS; op ,S+ — saving the LEAS at any optimisation level
    // where the pass fires.  Using the stack avoids the __scratch DP byte.
    if (needsMaterialization(RHS))
      RealRHS = materializeReg(Builder, RHS, MF);
    // Bug #161 round 11: AE / AF (HD6309 byte sub-regs of AW) can't be
    // pushed by page-1 PSHS. Use page-2 PSHSWx which pushes W (E:F)
    // as 2 bytes (E at 0,S, F at 1,S, big-endian). For AE the op reads
    // 0,S; for AF it reads 1,S. LEAS pops 2 bytes either way.
    if (RealRHS == MC6809::AE || RealRHS == MC6809::AF) {
      Builder.buildInstr(MC6809::PSHSWx);
      int ByteOff = (RealRHS == MC6809::AE) ? 0 : 1;
      Builder.buildInstr(Opc_o5)
          .addDef(AccReg, RegState::Implicit)
          .addImm(ByteOff).addReg(MC6809::SS);
      Builder.buildInstr(MC6809::LEASi_o5)
          .addImm(2).addReg(MC6809::SS);
    } else {
      Builder.buildInstr(MC6809::PSHSs, {}, {RealRHS});
      Builder.buildInstr(Opc_o5)
          .addDef(AccReg, RegState::Implicit)
          .addImm(0).addReg(MC6809::SS);
      Builder.buildInstr(MC6809::LEASi_o5)
          .addImm(1).addReg(MC6809::SS);
    }
  }
  if (needsMaterialization(OrigLHS))
    dematerializeReg(Builder, RealLHS, OrigLHS, MF);
  if (StagedLHS)
    pullStagingReg(Builder, AccReg);
}

/// Emit a 6809 16-bit register-register operation as a chained pair
/// of 8-bit operations: byte-low first (sets carry), byte-high second
/// (uses carry).
///
/// The 6809 has no native register-register 16-bit ADD/SUB. Instead
/// we expand to:
///
///     <op-low>  AB, lo-byte-of-RHS    ; ADDB / SUBB / ADCB / SBCB
///     <op-high> AA, hi-byte-of-RHS    ; ADCA / SBCA / etc.
///
/// The carry from the low byte propagates to the high byte via the
/// processor's CC.C flag — meaning NOTHING that touches CC may be
/// scheduled between the two halves. (This is the structural fragility
/// behind bug #57.) The expansion here is straight-line; downstream
/// passes don't reorder these instructions because they're emitted as
/// a contiguous group.
///
/// `OpcB_o8`/`OpcA_o8` are the U-relative (8-bit displacement)
/// variants for the spill RHS path. `OpcB_o5`/`OpcA_o0` are the
/// 5-bit / 0-byte displacement variants used after a PSHS — `0,s`
/// and `1,s` access the just-pushed bytes.
///
/// LHS handling
/// ------------
/// LHS is loaded into D (the only 16-bit accumulator) if it's a
/// spill or imaginary register. The 8-bit ops then act on AA and AB
/// implicitly. The result is in D at the end and stored back to the
/// spill slot if needed.
///
/// RHS handling — TWO PATHS  (mirror image of emit6809RegByteFromMem)
/// ------------------------
///   (a) RHS is a SPILL_D / SPILL_A* / SPILL_B*: read it directly
///       from the parent's spill slot via U-relative addressing.
///       This path was previously dead code (MaterializeSpills used
///       to rewrite all spill operands before this function ran),
///       but bug #63's fix re-enables it: MaterializeSpills now
///       deliberately leaves the SECOND distinct ACC spill alone for
///       Add/Sub/SetCarry _Reg pseudos so this path can handle it.
///       Big-endian byte order: high byte at slot+0, low byte at
///       slot+1.
///
///   (b) RHS is a real or imaginary register: materialize if needed,
///       PSHS to push the i16 onto the S stack (high byte at 0,s,
///       low byte at 1,s), operate from there, then LEAS to pop.
static void emit6809RegPairFromMem(MachineIRBuilder &Builder,
                                   Register LHS, Register RHS,
                                   unsigned OpcB_o8, unsigned OpcA_o8,
                                   unsigned OpcB_o5, unsigned OpcA_o0,
                                   unsigned OpcB_o16,
                                   unsigned OpcA_o16) {
  assert(OpcB_o16 != 0 && OpcA_o16 != 0 &&
         "emit6809RegPairFromMem: _o16 opcodes are required (bug #122/#130). "
         "A 0 here silently falls back to _o8 and wraps offsets >127 to "
         "negative. Pass the correct *i_o16 variants explicitly.");
  MachineFunction &MF = Builder.getMF();
  Register OrigLHS = LHS;
  Register RealRHS = RHS;

  // An imaginary/spill LHS stages through D, which the pseudo does not
  // declare and regalloc may have left live (e.g. a loop counter carried
  // in D across an RS-homed accumulate) -- preserve it for the whole
  // window. PULS is CC-neutral, so the carry chain the ops produce
  // survives to its consumer. Path (b)'s own RHS push happens after
  // this one, so its 0,s/1,s reads stay consistent.
  bool StagedLHS = needsMaterialization(LHS);
  if (StagedLHS)
    pushStagingReg(Builder, MC6809::AD);

  if (isSpillReg(RHS)) {
    // Path (a): read RHS bytes directly from its U-relative spill
    // slot. Byte order: hi at slot+0, lo at slot+1.
    // LHS materialization can happen now — RHS reads from memory and
    // doesn't need a register.
    if (needsMaterialization(LHS))
      materializeReg(Builder, LHS, MF);

    if (isStaticSpillSlot(RHS, MF)) {
      // Static-stack: extended byte ops against static_stack global (big-endian:
      // A high at +0, B low at +1). Low byte first so CC.C carries into the high.
      bool IsPIC = MF.getTarget().isPositionIndependent();
      int Base = staticSpillOffset(RHS, MF);
      Builder.buildInstr(getStaticStackOpcode(OpcB_o8, IsPIC))
          .addDef(MC6809::AB, RegState::Implicit)
          .addTargetIndex(MC6809::TI_STATIC_STACK, Base + 1);
      Builder.buildInstr(getStaticStackOpcode(OpcA_o8, IsPIC))
          .addDef(MC6809::AA, RegState::Implicit)
          .addTargetIndex(MC6809::TI_STATIC_STACK, Base);
    } else {
      int Offset = computeSpillStackOffset(RHS, MF);
      // Bug #122 / #125: for large stack frames (>127 bytes), the spill
      // offset may exceed the 8-bit signed range (-128..+127). Use the
      // _o16 opcode variant for large offsets — _o8 wraps >127 to
      // negative, reading from below the frame. Note: Offset+1 (the low
      // byte) overflows first, so check that.
      bool Fits8 = (Offset >= -128 && (Offset + 1) <= 127);
      unsigned OpcB = (Fits8 || !OpcB_o16) ? OpcB_o8 : OpcB_o16;
      unsigned OpcA = (Fits8 || !OpcA_o16) ? OpcA_o8 : OpcA_o16;
      // Low byte first — sets CC.C if this op carries.
      Builder.buildInstr(OpcB)
          .addDef(MC6809::AB, RegState::Implicit)
          .addImm(Offset + 1).addReg(MC6809::SU);
      // High byte second — uses (and possibly sets) CC.C from the low op.
      Builder.buildInstr(OpcA)
          .addDef(MC6809::AA, RegState::Implicit)
          .addImm(Offset).addReg(MC6809::SU);
    }
  } else if (RHS.isPhysical() && MC6809::Imag16RegClass.contains(RHS)) {
    // Path (b-dp): RHS lives at a fixed direct-page address -- read its
    // bytes straight from memory (lo first so CC.C chains into the hi).
    // No staging register is touched, so this also avoids the Bug #242
    // mirror hazard: materializing an imaginary RHS through AD would
    // destroy a real LHS already living there.
    if (needsMaterialization(LHS))
      materializeReg(Builder, LHS, MF);
    Builder.buildInstr(getDirectPageOpcode(OpcB_o8))
        .addDef(MC6809::AB, RegState::Implicit)
        .addReg(imag16LoByte(RHS));
    Builder.buildInstr(getDirectPageOpcode(OpcA_o8))
        .addDef(MC6809::AA, RegState::Implicit)
        .addReg(imag16HiByte(RHS));
  } else {
    // Path (b): push RHS onto S, operate from 0,s and 1,s, then LEAS.
    //
    // Bug #242: we MUST push RHS BEFORE materializing LHS. If LHS is a
    // spill (needs LDD into AD) and RHS == $ad (the typical case for
    // ALU16 ops where the regalloc kept one operand in AD and spilled
    // the other), the LHS materialization clobbers AD — so RHS's value
    // is lost before we push it. The push then captures the post-LDD
    // AD (= LHS), the AND becomes `LHS AND LHS = LHS` (wrong).
    //
    // Push the (still-correct) RHS first, then materialize LHS into AD.
    if (needsMaterialization(RHS))
      RealRHS = materializeReg(Builder, RHS, MF);

    // Bug #161 round 11: page-1 PSHS doesn't accept the HD6309 W
    // register. Use the page-2 PSHSWx (which pushes W = E:F as 2 bytes
    // onto SS, big-endian: E at offset 0, F at offset 1) when RealRHS
    // is AW. Same stack layout as `PSHS D` (A at 0, B at 1) so the
    // downstream OpcA/OpcB at 0,S and 1,S reads work unchanged.
    // Bug #65: see emit6809RegByteFromMem for the full explanation
    // of why we use the (opc, defs, srcs) buildInstr form here. TL;DR:
    // the chained .addUse(..., Implicit) form leaves RealRHS off the
    // printed register list and the asm printer emits garbage like
    // "pshs s,s".
    if (RealRHS == MC6809::AW)
      Builder.buildInstr(MC6809::PSHSWx);
    else
      Builder.buildInstr(MC6809::PSHSs, {}, {RealRHS});

    // Now LHS materialization is safe — RHS is captured on the stack.
    if (needsMaterialization(LHS))
      materializeReg(Builder, LHS, MF);

    // PSHS pushed 2 bytes; SS is now 2 lower than at the LHS load.
    // Low byte at S+1, high byte at S+0 (big-endian on the stack).
    Builder.buildInstr(OpcB_o5)
        .addDef(MC6809::AB, RegState::Implicit)
        .addImm(1).addReg(MC6809::SS);
    Builder.buildInstr(OpcA_o0)
        .addDef(MC6809::AA, RegState::Implicit)
        .addReg(MC6809::SS);
    // Pop the i16 we pushed.
    Builder.buildInstr(MC6809::LEASi_o5)
        .addImm(2).addReg(MC6809::SS);
  }
  // Store the i16 result back to LHS's slot if LHS was a spill /
  // imaginary register. AD now holds the result.
  if (needsMaterialization(OrigLHS))
    dematerializeReg(Builder, MC6809::AD, OrigLHS, MF);
  if (StagedLHS)
    pullStagingReg(Builder, MC6809::AD);
}

void MC6809InstrInfo::expandAddSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #186 v5: 3-op layout (no carry def/use operand).
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for AddSetCarryUseReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::ADCRp)) {
    // page-3 reg-reg path emitted
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::ADCBi_o8, MC6809::ADCAi_o8,
                           MC6809::ADCBi_o5, MC6809::ADCAi_o0,
                           MC6809::ADCBi_o16, MC6809::ADCAi_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for SubReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SUBRp)) {
    // page-3 reg-reg path emitted
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::SUBBi_o8, MC6809::SBCAi_o8,
                           MC6809::SUBBi_o5, MC6809::SBCAi_o0,
                           MC6809::SUBBi_o16, MC6809::SBCAi_o16);
  }
  MI.eraseFromParent();
}

// Expand `Sub_i8_Reg` — plain byte subtraction (no carry chain).
// Same as expandSubReg but uses emit6809RegByteFromMem with correct
// A/B accumulator selection, avoiding the 16-bit pair expansion
// that clobbers when LHS and RHS share the same register.
void MC6809InstrInfo::expandSubByteReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg());
  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SUBRp)) {
    // page-3 reg-reg path emitted
  } else {
    unsigned Opc_o8, Opc_o5, Opc_o16;
    getByteOpcodes(MI.getOperand(0).getReg(),
                   MC6809::SUBAi_o8, MC6809::SUBBi_o8,
                   MC6809::SUBAi_o5, MC6809::SUBBi_o5,
                   MC6809::SUBAi_o16, MC6809::SUBBi_o16,
                   Opc_o8, Opc_o5, Opc_o16);
    emit6809RegByteFromMem(Builder, MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(), Opc_o8, Opc_o5, Opc_o16);
  }
  MI.eraseFromParent();
}

// Expand `SubSetCarry_i{8,16}_Reg` (i32-narrow lo half — does the
// subtract and PRODUCES borrow-out for the upper half to consume).
//
// Same operand layout and bug #64 history as expandAddSetCarryReg
// above (4 operands: dst, carry, src(==dst), src2). RHS lives at
// op3, NOT op4. See expandAddSetCarryReg for the full write-up.
void MC6809InstrInfo::expandSubSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #186 v5: 3-op layout (no carry def operand).
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for SubSetCarryReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SUBRp)) {
    // page-3 reg-reg path emitted
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(),
                           MI.getOperand(2).getReg(),
                           MC6809::SUBBi_o8, MC6809::SBCAi_o8,
                           MC6809::SUBBi_o5, MC6809::SBCAi_o0,
                           MC6809::SUBBi_o16, MC6809::SBCAi_o16);
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandSubSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Bug #186 v5: 3-op layout (no carry def/use operand).
  assert(MI.getOperand(0).getReg() == MI.getOperand(1).getReg() && "Dest and Source must be same for SubSetCarryUseReg");

  const auto &STI = MI.getMF()->getSubtarget<MC6809Subtarget>();
  if (STI.has6309() && emitHD6309RegRegOp(Builder, MI, MC6809::SBCRp)) {
    // page-3 reg-reg path emitted
  } else {
    emit6809RegPairFromMem(Builder,
                           MI.getOperand(0).getReg(), MI.getOperand(2).getReg(),
                           MC6809::SBCBi_o8, MC6809::SBCAi_o8,
                           MC6809::SBCBi_o5, MC6809::SBCAi_o0,
                           MC6809::SBCBi_o16, MC6809::SBCAi_o16);
  }
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
  MachineFunction &MF = *MI.getMF();
  SrcReg = materializeReg(Builder, SrcReg, MF);

  // Bug #49 fix: if source is AD and D was loaded from a frame slot that was
  // stored from X or Y, use CMPX/CMPY instead. This preserves D's PHI value
  // which would otherwise be clobbered by the LDD reload.
  if (SrcReg == MC6809::AD && !DisableCmpSubst) {
    MachineBasicBlock &MBB = *MI.getParent();
    MachineInstr *LDDInstr = nullptr;
    int LoadOffset = 0;

    // Find the LDD that loaded the comparison value into D.
    //
    // The scan must stop at ANY write that overlaps AD — including
    // sub-register writes of AA/AB (TFR into a half, byte loads, byte
    // arithmetic). The original exact-operand check (definesRegister with
    // TRI == nullptr) was blind to those, so a D value assembled from
    // byte halves after an unrelated earlier LDD was mis-attributed to
    // that stale LDD, and the substituted CMPX compared a register that
    // never held the value (manifest: the libc-testsuite strtol position
    // checks compared the still-live `c` pointer in X instead of the
    // freshly byte-built `c - s` in D, "10 != 10"). Byte-assembled D
    // values are the norm once byte spills flow through the stock
    // spiller, so overlap-awareness is load-bearing.
    const TargetRegisterInfo &CmpTRI =
        *MF.getSubtarget().getRegisterInfo();
    for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
         It != MBB.rend(); ++It) {
      if (It->modifiesRegister(MC6809::AD, &CmpTRI)) {
        unsigned Opc = It->getOpcode();
        if ((Opc == MC6809::LDDi_o0 || Opc == MC6809::LDDi_o5 ||
             Opc == MC6809::LDDi_o8 || Opc == MC6809::LDDi_o16) &&
            It->getNumOperands() >= 2 && It->getOperand(1).isReg() &&
            It->getOperand(1).getReg() == MC6809::SU) {
          LDDInstr = &*It;
          LoadOffset = It->getOperand(0).getImm();
        }
        break; // Stop at the first write overlapping D.
      }
    }

    if (LDDInstr) {
      // Look further back for STX/STY to the same frame offset.
      //
      // Bug #87: the scan must bail out if ANY byte- or word-level store
      // to the slot's 2-byte footprint (offsets LoadOffset and
      // LoadOffset+1) occurs between the candidate STX/STY and the LDD.
      // Those stores mutate the in-memory value *after* the STX, so the
      // index register is stale compared to the slot contents. Without
      // this check, sequences like
      //
      //    stx     4,u          ; original c
      //    ldb     5,u          ; \
      //    addb    #-97         ;  |  byte-level c -= 97 in memory
      //    stb     5,u          ; /
      //    lda     4,u          ; \
      //    adca    #-1          ;  |  (propagates borrow)
      //    sta     4,u          ; /
      //    [cmpd   4,u]         ; <- erased; replaced by cmpx below
      //    cmpx    #26          ; WRONG: X still holds original c
      //
      // silently miscompile (test_toupper('a') returns 'a' unchanged in
      // picolibc at -O0, and similarly for test_tolower / test_isspace).
      Register IndexSrc;
      auto OverlapsSlot = [LoadOffset](int Off, int Size) {
        return Off < LoadOffset + 2 && Off + Size > LoadOffset;
      };
      auto IsStoreToSU = [](const MachineInstr &I, int &OffOut, int &SizeOut) {
        // Recognise STB/STA/STD/STX/STY through $su with an immediate
        // offset. Returns the byte offset and the store size. STD/STX/STY
        // are 2 bytes; STA/STB are 1 byte.
        auto CheckOpnd = [&](unsigned OffOpIdx) {
          if (I.getNumOperands() <= OffOpIdx + 1) return false;
          const MachineOperand &OffMO = I.getOperand(OffOpIdx);
          const MachineOperand &BaseMO = I.getOperand(OffOpIdx + 1);
          if (!OffMO.isImm() || !BaseMO.isReg()) return false;
          if (BaseMO.getReg() != MC6809::SU) return false;
          OffOut = OffMO.getImm();
          return true;
        };
        switch (I.getOpcode()) {
        case MC6809::STAi_o5: case MC6809::STAi_o8: case MC6809::STAi_o16:
        case MC6809::STBi_o5: case MC6809::STBi_o8: case MC6809::STBi_o16:
          SizeOut = 1;
          return CheckOpnd(0);
        case MC6809::STDi_o5: case MC6809::STDi_o8: case MC6809::STDi_o16:
        case MC6809::STXi_o5: case MC6809::STXi_o8: case MC6809::STXi_o16:
        case MC6809::STYi_o5: case MC6809::STYi_o8: case MC6809::STYi_o16:
          SizeOut = 2;
          return CheckOpnd(0);
        }
        return false;
      };
      for (auto It = MachineBasicBlock::reverse_iterator(LDDInstr->getIterator());
           It != MBB.rend(); ++It) {
        unsigned Opc = It->getOpcode();
        if ((Opc == MC6809::STXi_o5 || Opc == MC6809::STXi_o8 ||
             Opc == MC6809::STXi_o16) &&
            It->getOperand(0).isImm() &&
            It->getOperand(0).getImm() == LoadOffset) {
          IndexSrc = MC6809::IX;
          break;
        }
        if ((Opc == MC6809::STYi_o5 || Opc == MC6809::STYi_o8 ||
             Opc == MC6809::STYi_o16) &&
            It->getOperand(0).isImm() &&
            It->getOperand(0).getImm() == LoadOffset) {
          IndexSrc = MC6809::IY;
          break;
        }
        // Bug #87: any intervening store to the slot footprint makes the
        // index register stale — bail.
        int Off = 0, Sz = 0;
        if (IsStoreToSU(*It, Off, Sz) && OverlapsSlot(Off, Sz))
          break;
        // Bug #263: bail on any call. Calls clobber IX (caller-saved per
        // the MC6809 ABI; only IY and SU are in MC6809_CSR). The slot's
        // bytes survive the call (it's a frame-local slot the callee
        // can't touch), but the IX register does NOT — so the
        // "compare via IX" optimization is unsafe when a call sits
        // between the STX and the LDD. Specifically, this manifested at
        // -*-lto levels in picolibc's muld_neg60_probe: a printf call
        // between the STX (saving sec_calc's i16 return) and the LDD
        // (reloading for an `if (v != 43)` check) clobbered IX, so the
        // resulting `cmpx #43` compared printf's return value instead
        // of sec_calc's. The regmask on the call DOES list IX as
        // clobbered, but `definesRegister` with TRI=nullptr doesn't
        // consult the regmask. Using `MI.isCall()` is the robust
        // catch-all: any call invalidates the index-source assumption.
        if (It->isCall())
          break;
        // If X or Y is redefined before we find the store, stop.
        if (It->definesRegister(MC6809::IX, /*TRI=*/nullptr) ||
            It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
          break;
      }

      if (IndexSrc.isValid()) {
        SrcReg = IndexSrc;
        LDDInstr->eraseFromParent();
      }
    }
  }

  // (Bug #49 fix part 2 once lived here — a custom PSHS-D / byte-load /
  // CMPB / PULS-D wrap intended to preserve a PHI-carried D value across
  // the byte load that would otherwise clobber D's low byte. Bug #159
  // narrowed the wrap with a live-out gate. Bug #166's spike removed the
  // wrap entirely after confirming the underlying PHI clobber no longer
  // reproduces — regalloc / spill placer handle the D-preservation now.
  // Codegen-shape sentinel: `test/CodeGen/MC6809/cmp_imm_byte_load_phi.ll`.
  // Semantic sentinel: `test_atoi_neg("-3")` in
  // `test/MC/MC6809/Execution/codegen-stdlib-ctype.s`.)

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

  // Materialize a spilled index base (operand 3) into IY, mirroring
  // expandStoreIdx. The P3a consumer memory-fold keeps the base pointer live to
  // the compare; under register pressure it can spill to an index-spill reg,
  // which has no real encoding and would render as `,0` (the strcmp miscompile).
  if (isIndexSpillReg(MI.getOperand(3).getReg())) {
    Register SpillReg = MI.getOperand(3).getReg();
    Register StageReg = MC6809::IY;  // callee-saved; avoids clobbering IX base
    MachineFunction &MF = *MI.getMF();
    MachineIRBuilder PreBuilder(*MI.getParent(), MI.getIterator());
    emitSpillLoadInto(PreBuilder, StageReg, SpillReg, /*ExtraOffset=*/0, MF);
    MI.getOperand(3).setReg(StageReg);
  }

  auto SrcReg = MI.getOperand(2).getReg();
  if (needsMaterialization(SrcReg)) {
    MachineFunction &MF = *MI.getMF();
    Register IndexSrc = Register();
    // Optimization for spill registers: if the spill was stored from an INDEX
    // register (STX/STY) and that register hasn't been redefined, use
    // CMPX/CMPY directly. This avoids clobbering D (bug #30).
    // (Imaginary registers are direct-page based, not stack-based, so skip scan.)
    if (isSpillReg(SrcReg)) {
      int SpillOffset = computeSpillStackOffset(SrcReg, MF);
      unsigned SpillSize = getSpillRegSize(SrcReg);
      MachineBasicBlock &MBB = *MI.getParent();
      for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
           It != MBB.rend(); ++It) {
        unsigned Opc = It->getOpcode();
        auto IsMatchingIndexStore = [&](unsigned O5, unsigned O8, unsigned O16) {
          return (Opc == O5 || Opc == O8 || Opc == O16) &&
                 It->getNumOperands() >= 2 &&
                 It->getOperand(0).isImm() &&
                 It->getOperand(0).getImm() == SpillOffset &&
                 It->getOperand(1).isReg() &&
                 It->getOperand(1).getReg() == MC6809::SU;
        };
        if (IsMatchingIndexStore(MC6809::STXi_o5, MC6809::STXi_o8, MC6809::STXi_o16)) {
          IndexSrc = MC6809::IX;
          break;
        }
        if (IsMatchingIndexStore(MC6809::STYi_o5, MC6809::STYi_o8, MC6809::STYi_o16)) {
          IndexSrc = MC6809::IY;
          break;
        }
        // Bail if an intervening store overwrote any byte of our spill slot —
        // the IX/IY value no longer mirrors the slot even if IX/IY itself is
        // still live (bug #125: SubSetCarry_i8_Reg writes spill_b/_a in place).
        if (storeOverlapsSpillSlot(*It, SpillOffset, SpillSize))
          break;
        // Bug #263: bail on any call. Calls clobber IX (caller-saved
        // per the MC6809 ABI) via the regmask, which
        // `definesRegister(_, TRI=nullptr)` doesn't consult.
        if (It->isCall())
          break;
        if (It->definesRegister(MC6809::IX, /*TRI=*/nullptr) ||
            It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
          break;
      }
    }
    if (IndexSrc.isValid()) {
      SrcReg = IndexSrc;  // Use CMPX/CMPY directly, preserving D.
    } else {
      SrcReg = materializeReg(Builder, SrcReg, MF);
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
    Opcode = OpcodePair->getSecond();
  } else
    llvm_unreachable("Unknown offset type for CompareIdx");
  // The zero-offset form (CMPxi_o0) takes ONLY the base -- no offset operand --
  // exactly like the LOAD expander's `if (OffsetSize > 0)` guard. Adding a `0`
  // offset to the o0 form gives it a spurious operand and the printer renders
  // `cmp ,0` (the strcmp register-base-compare miscompile). This path is only
  // reached with offset 0 by the P3a register-base compare fold; existing
  // frame-index compares resolve to non-zero (o8/o16) offsets post-PEI.
  // P3b: indirect-indexed compare (`cmp [n,r]`) -- swap to the indirect-sibling
  // opcode, mirroring expandIdxImm / expandLoadIdx.
  if (isMemIndirectPseudo(MI.getOpcode())) {
    unsigned Ind = indirectSiblingOf(Opcode);
    assert(Ind && "no indirect-indexed compare sibling (add an IND3 row)");
    Opcode = Ind;
  }
  auto CmpB = Builder.buildInstr(Opcode);
  if (OffsetSize > 0)
    CmpB.add(OffsetOp);
  CmpB.add(IndexOp);
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

  // Bug #161 round 17: materialize spill / imaginary operands into real
  // hardware registers before emitting CMPR. CMPR encodes 4-bit hardware
  // register codes in its postbyte; raw SPILL_* operands collapse the
  // postbyte to 0x00 (= CMPR D,D = always Z=1), breaking the test-strncpy
  // and similar inner-loop comparisons.
  //
  // Bug #161 round 18: when both sources resolve to the same physical
  // register (commonly because both were SPILL_D* and materialize to
  // AD), CMPRp X,X always sets Z=1 / N,V,C=0 — wrong for any non-equal
  // source pair. Detect that and fall back to a 6809-style compare via
  // PSHS / CMPx 0,S / LEAS so the SPILL operand is read separately.
  MachineFunction &MF = *MI.getMF();
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  Register Src1 = MI.getOperand(3).getReg();
  Register Src2 = MI.getOperand(2).getReg();
  auto effectivePhys = [](Register R) -> Register {
    return needsMaterialization(R) ? getPhysRegFor(R) : R;
  };
  Register Src1Phys = effectivePhys(Src1);
  Register Src2Phys = effectivePhys(Src2);
  bool SameHalf = Src1Phys == Src2Phys &&
      (Src1Phys == MC6809::AA || Src1Phys == MC6809::AB ||
       Src1Phys == MC6809::AD || Src1Phys == MC6809::AW ||
       Src1Phys == MC6809::AE || Src1Phys == MC6809::AF ||
       Src1Phys == MC6809::IX || Src1Phys == MC6809::IY ||
       Src1Phys == MC6809::SU || Src1Phys == MC6809::SS);

  // Bug #312: CMPRp is HD6309-only.  On plain MC6809 take the same
  // PSHS / CMPx 0,$ss / LEAS sequence the SameHalf collision-fallback
  // uses below — it works for the general non-SameHalf case too, just
  // costs one stack round-trip.  pickCmpO8O16 (defined further down)
  // picks the right CMPx variant for Src2's physreg.
  //
  // An INDEX spill operand must NOT be materialized into a hardware index
  // register here: MaterializeSpills deliberately leaves the second of
  // two distinct index spills as a SPILL_X for the register-vs-slot path
  // below (CMPY off,u), because materializing it into IX/IY would clobber
  // whatever live value already occupies that register (e.g. a NULL
  // return pointer held in IX across the compare). Loading a spill into
  // IX here silently destroyed that value. So for the pointer compare,
  // when a source is still a spill, fall through to the isSpillReg paths,
  // which read it from its U-relative slot and touch only the other
  // operand's register. (Accumulator compares are unaffected: their D
  // spill is materialized with a D save/restore by MaterializeSpills.)
  unsigned MIOpc = MI.getOpcode();
  auto pickCmpO8O16 = [&](Register PhysReg, unsigned ByteOffset,
                          unsigned &OpcOut) {
    bool Fits8 = (int(ByteOffset) >= -128 && int(ByteOffset) <= 127);
    if (MIOpc == MC6809::Compare_i8_Reg) {
      if (PhysReg == MC6809::AA)
        OpcOut = Fits8 ? MC6809::CMPAi_o8 : MC6809::CMPAi_o16;
      else
        OpcOut = Fits8 ? MC6809::CMPBi_o8 : MC6809::CMPBi_o16;
    } else {
      if      (PhysReg == MC6809::AD) OpcOut = Fits8 ? MC6809::CMPDi_o8 : MC6809::CMPDi_o16;
      else if (PhysReg == MC6809::IX) OpcOut = Fits8 ? MC6809::CMPXi_o8 : MC6809::CMPXi_o16;
      else if (PhysReg == MC6809::IY) OpcOut = Fits8 ? MC6809::CMPYi_o8 : MC6809::CMPYi_o16;
      else if (PhysReg == MC6809::SU) OpcOut = Fits8 ? MC6809::CMPUi_o8 : MC6809::CMPUi_o16;
      else if (PhysReg == MC6809::SS) OpcOut = Fits8 ? MC6809::CMPSi_o8 : MC6809::CMPSi_o16;
      else                             OpcOut = Fits8 ? MC6809::CMPDi_o8 : MC6809::CMPDi_o16;
    }
  };

  // Imaginary (direct-page homed) sources. With the RS/RC imaginaries
  // allocatable, one or both compare operands can live in a DP slot; the
  // materializing CMPR arm above skips SameHalf pairs (both would stage
  // through the same accumulator and CMPR would degenerate -- the custom
  // verifier check catches exactly that shape). Read the DP operand with
  // the d-form compare instead. Direction contract (see above):
  // flags = Src2 - Src1.
  {
    bool Src1Imag = !isSpillReg(Src1) && needsMaterialization(Src1);
    bool Src2Imag = !isSpillReg(Src2) && needsMaterialization(Src2);
    auto PickCmpD = [&](Register PhysReg) -> unsigned {
      if (MIOpc == MC6809::Compare_i8_Reg) {
        if (PhysReg == MC6809::AA) return MC6809::CMPAd;
        if (PhysReg == MC6809::AE) return MC6809::CMPEd;
        if (PhysReg == MC6809::AF) return MC6809::CMPFd;
        return MC6809::CMPBd;
      }
      if (PhysReg == MC6809::IX) return MC6809::CMPXd;
      if (PhysReg == MC6809::IY) return MC6809::CMPYd;
      if (PhysReg == MC6809::SU) return MC6809::CMPUd;
      if (PhysReg == MC6809::SS) return MC6809::CMPSd;
      if (PhysReg == MC6809::AW) return MC6809::CMPWd;
      return MC6809::CMPDd;
    };
    if (Src1Imag && !Src2Imag) {
      // Src2 already real: CMP<Src2>d __src1 gives Src2 - mem(Src1).
      Builder.buildInstr(PickCmpD(Src2Phys)).addReg(Src1);
      MI.eraseFromParent();
      return;
    }
    if (Src2Imag) {
      if (!Src1Imag && Src1 == Src2Phys) {
        // Src1's value lives in the register Src2 must stage through.
        // Push it, load Src2 over it, compare against the pushed copy
        // (flags = reg(Src2) - mem(Src1)), then restore Src1.
        Builder.buildInstr(MC6809::PSHSs, {}, {Src1});
        materializeReg(Builder, Src2, MF);
        unsigned CmpOpc;
        pickCmpO8O16(Src2Phys, 0, CmpOpc);
        Builder.buildInstr(CmpOpc).addImm(0).addReg(MC6809::SS);
        Builder.buildInstr(MC6809::PULSs, {Register(Src1)}, {});
        MI.eraseFromParent();
        return;
      }
      // Src1 is imaginary or a real register distinct from Src2's
      // staging register. Stage Src2 (preserving the staging register
      // -- it is not a declared def of the compare and may be live),
      // then read Src1 from its DP slot / compare register-register.
      // The PULS restore does not touch CC.
      pushStagingReg(Builder, Src2Phys);
      materializeReg(Builder, Src2, MF);
      if (Src1Imag)
        Builder.buildInstr(PickCmpD(Src2Phys)).addReg(Src1);
      else if (STI.has6309())
        Builder.buildInstr(MC6809::CMPRp).addUse(Src1).addUse(Src2Phys);
      else {
        Builder.buildInstr(MC6809::PSHSs, {}, {Register(Src1)});
        unsigned CmpOpc;
        pickCmpO8O16(Src2Phys, 0, CmpOpc);
        Builder.buildInstr(CmpOpc).addImm(0).addReg(MC6809::SS);
        Builder.buildInstr(MC6809::PULSs, {Register(Src1)}, {});
      }
      pullStagingReg(Builder, Src2Phys);
      MI.eraseFromParent();
      return;
    }
  }

  bool PtrSpill = MI.getOpcode() == MC6809::Compare_ptr_Reg &&
                  (isSpillReg(Src1) || isSpillReg(Src2));
  if (!SameHalf && STI.has6309() && !PtrSpill) {
    if (needsMaterialization(Src1)) Src1 = materializeReg(Builder, Src1, MF);
    if (needsMaterialization(Src2)) Src2 = materializeReg(Builder, Src2, MF);
    Builder.buildInstr(MC6809::CMPRp).addUse(Src1).addUse(Src2);
    MI.eraseFromParent();
    return;
  }

  // Same-physreg collision. CMPRp's TableGen semantics are
  // `cmpr reg1, reg2 → flags = reg2 - reg1`. The non-collision path
  // above emits `addUse(Src1).addUse(Src2)`, mapping Src1→reg1 and
  // Src2→reg2, so the resulting flags are `Src2 - Src1`. Every
  // collision fallback below MUST preserve that direction —
  // i.e. flags = (op2 - op3) using MIR convention, which is
  // `physreg(=Src2) - mem(=Src1)` in CMPx-from-spill form.
  //
  // Three sub-cases:
  //
  //   (i)  Src2 is a SPILL_* (Src1 already in physreg, with the same
  //        physreg the spill would materialize into). PSHS Src1's
  //        value to free the physreg, then load Src2 into the
  //        physreg, then `CMPx 0,$ss` so the compare is
  //        physreg(=Src2) - mem(=Src1). LEAS to clean up.
  //
  //   (ii) Src1 is a SPILL_* (Src2 already in physreg). Src2 is in
  //        the right place already; emit `CMPx Src1_offset, $su`
  //        directly so flags = physreg(=Src2) - mem(=Src1). No
  //        PSHS needed.
  //
  //   (iii) Both already physical AND identical (regalloc coalesced
  //         them). The value really is the same, so equal is correct
  //         — emit CMPRp.

  if (isSpillReg(Src2)) {
    // Src2 is the spill, Src1 is in physreg. Push Src1 to stack, load
    // Src2 into the physreg (its RealReg matches Src1Phys), CMP
    // physreg-vs-stack so flags = Src2 - Src1.
    if (needsMaterialization(Src1)) Src1 = materializeReg(Builder, Src1, MF);
    Builder.buildInstr(MC6809::PSHSs, {}, {Src1Phys});
    Register Src2Real = materializeReg(Builder, Src2, MF);
    unsigned CmpOpc;
    pickCmpO8O16(Src2Real, 0, CmpOpc);
    Builder.buildInstr(CmpOpc).addImm(0).addReg(MC6809::SS);
    // Bug #257: PULS Src1Phys instead of LEAS to RESTORE Src1's value.
    // The materializeReg(Src2) above loaded Src2's value into Src1Phys
    // (because SameHalf collision routed both through the same physreg).
    // Without restoring, successor BBs that have Src1Phys as a livein
    // see the wrong value. memcmp's byte-loop hit this: $ab held *s2
    // pre-compare, the load clobbered it with *s1, and the mismatch
    // BB then computed *s1 - *s1 = 0 (returning equal-on-mismatch).
    // Cycle cost: PULS B = 6 cy vs LEAS 1,$ss = 5 cy (+1 cy);
    // PULS D = 7 cy vs LEAS 2,$ss = 5 cy (+2 cy). Same byte count.
    Builder.buildInstr(MC6809::PULSs, {Src1Phys}, {});
    MI.eraseFromParent();
    return;
  }
  if (isSpillReg(Src1)) {
    // Src1 is the spill, Src2 is in physreg. Read Src1 directly from
    // its U-relative slot — flags = physreg(=Src2) - mem(=Src1).
    unsigned CmpOpc;
    if (isStaticSpillSlot(Src1, MF)) {
      pickCmpO8O16(Src2Phys, /*Offset=*/0, CmpOpc);
      Builder.buildInstr(getStaticStackOpcode(
                             CmpOpc, MF.getTarget().isPositionIndependent()))
          .addTargetIndex(MC6809::TI_STATIC_STACK, staticSpillOffset(Src1, MF));
    } else {
      int ByteOffset = computeSpillStackOffset(Src1, MF);
      pickCmpO8O16(Src2Phys, ByteOffset, CmpOpc);
      Builder.buildInstr(CmpOpc).addImm(ByteOffset).addReg(MC6809::SU);
    }
    MI.eraseFromParent();
    return;
  }
  // Both Src1 and Src2 already physical, no spill on either side.
  // HD6309: CMPRp regardless of whether Src1Phys == Src2Phys (regalloc
  // coalesced) or not.  Plain MC6809 (Bug #312): PSHS Src1Phys then
  // CMPx Src2 ,$ss++ (post-increment indexed — pops the byte/word
  // back off as part of the compare).  Same flag direction as CMPRp
  // (Src2 - Src1).  PULS isn't needed for value preservation: the
  // compare reads Src2Phys without writing it, and Src1Phys still
  // holds its value (PSHS doesn't clear it, just stores a copy).
  if (STI.has6309()) {
    Builder.buildInstr(MC6809::CMPRp).addUse(Src1Phys).addUse(Src2Phys);
    MI.eraseFromParent();
    return;
  }
  Builder.buildInstr(MC6809::PSHSs, {}, {Src1Phys});
  unsigned CmpIncOpc = 0;
  if (MIOpc == MC6809::Compare_i8_Reg) {
    CmpIncOpc = (Src2Phys == MC6809::AA) ? MC6809::CMPAi_Inc1
                                         : MC6809::CMPBi_Inc1;
  } else {
    if      (Src2Phys == MC6809::AD) CmpIncOpc = MC6809::CMPDi_Inc2;
    else if (Src2Phys == MC6809::IX) CmpIncOpc = MC6809::CMPXi_Inc2;
    else if (Src2Phys == MC6809::IY) CmpIncOpc = MC6809::CMPYi_Inc2;
    else                              CmpIncOpc = MC6809::CMPDi_Inc2;
  }
  Builder.buildInstr(CmpIncOpc).addReg(MC6809::SS);
  MI.eraseFromParent();
}





void MC6809InstrInfo::expandTestReg(MachineIRBuilder &Builder, MachineInstr &MI) const {
  assert(MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::CC && "The target of tests must be the CC register");
  assert((MI.getOperand(1).isImm() || MI.getOperand(1).isCImm()) && "The condition field of tests must be an immediate constant");
  assert(MI.getOperand(2).isReg() && "The source of register tests must be a register");

  auto SrcReg = MI.getOperand(2).getReg();
  if (needsMaterialization(SrcReg)) {
    MachineFunction &MF = *MI.getMF();
    Register IndexSrc = Register();
    // Optimization for spill registers: if the spill was stored from an INDEX
    // register (STX/STY), use CMPX/CMPY #0 directly. Avoids D clobber (bug #31).
    // (Imaginary registers are direct-page based, not stack-based, so skip scan.)
    if (isSpillReg(SrcReg)) {
      int SpillOffset = computeSpillStackOffset(SrcReg, MF);
      unsigned SpillSize = getSpillRegSize(SrcReg);
      MachineBasicBlock &MBB = *MI.getParent();
      for (auto It = MachineBasicBlock::reverse_iterator(MI.getIterator());
           It != MBB.rend(); ++It) {
        unsigned Opc = It->getOpcode();
        auto IsMatchingIndexStore = [&](unsigned O5, unsigned O8, unsigned O16) {
          return (Opc == O5 || Opc == O8 || Opc == O16) &&
                 It->getNumOperands() >= 2 &&
                 It->getOperand(0).isImm() &&
                 It->getOperand(0).getImm() == SpillOffset &&
                 It->getOperand(1).isReg() &&
                 It->getOperand(1).getReg() == MC6809::SU;
        };
        if (IsMatchingIndexStore(MC6809::STXi_o5, MC6809::STXi_o8, MC6809::STXi_o16)) {
          IndexSrc = MC6809::IX;
          break;
        }
        if (IsMatchingIndexStore(MC6809::STYi_o5, MC6809::STYi_o8, MC6809::STYi_o16)) {
          IndexSrc = MC6809::IY;
          break;
        }
        // Bail if an intervening store overwrote any byte of our spill slot —
        // the IX/IY value no longer mirrors the slot even if IX/IY itself is
        // still live (bug #125: SubSetCarry_i8_Reg writes spill_b/_a in place).
        if (storeOverlapsSpillSlot(*It, SpillOffset, SpillSize))
          break;
        // Bug #263: bail on any call. Calls clobber IX (caller-saved
        // per the MC6809 ABI) via the regmask, which
        // `definesRegister(_, TRI=nullptr)` doesn't consult.
        if (It->isCall())
          break;
        if (It->definesRegister(MC6809::IX, /*TRI=*/nullptr) ||
            It->definesRegister(MC6809::IY, /*TRI=*/nullptr))
          break;
      }
    }
    if (IndexSrc.isValid()) {
      // Use CMPX/CMPY #0 directly — preserves D.
      auto OpcodePair = CompareImmediateOpcode.find(IndexSrc);
      assert(OpcodePair != CompareImmediateOpcode.end());
      Builder.buildInstr(OpcodePair->getSecond()).addImm(0);
      MI.eraseFromParent();
      return;
    }
    // Fallback: materialize into real accumulator and test.
    SrcReg = materializeReg(Builder, SrcReg, MF);
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

void MC6809InstrInfo::expandTestMem(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Operand layout (see MC6809InstrFamilies.td defm Test_i8/_i16):
  //   op 0: CC (def, $dst)
  //   op 1: condition code (i8imm, unused by Test itself; artefact of fusion)
  //   op 2: INDEX16:$idx
  //   op 3: $offset (imm or reg)
  assert(MI.getOperand(2).isReg() && "Test_Mem: index must be a register");
  auto IndexOp = MI.getOperand(2);
  auto OffsetOp = MI.getOperand(3);
  int OffsetSize = offsetSizeInBits(OffsetOp);
  bool IsI16 = (MI.getOpcode() == MC6809::Test_i16_Mem);

  unsigned Opcode = 0;
  if (IsI16) {
    // No memory-form TSTD exists on MC6809 or HD6309 (TSTDa is accumulator-only).
    // Use LDD to set N/Z and clear V. AD is declared as a Def of the
    // Test_i16_Mem / TestBranch_i16_Mem pseudos so RA knows about the clobber.
    if (OffsetSize >= 0) {
      RegPlusOffsetLen Lookup{MC6809::AD, OffsetSize};
      auto OpcodePair = LoadIdxImmOpcode.find(Lookup);
      if (OpcodePair == LoadIdxImmOpcode.end())
        llvm_unreachable("Test_i16_Mem: unexpected immediate offset size.");
      Opcode = OpcodePair->getSecond();
    } else if (OffsetOp.isReg()) {
      RegPlusReg Lookup{MC6809::AD, OffsetOp.getReg()};
      auto OpcodePair = LoadIdxRegOpcode.find(Lookup);
      if (OpcodePair == LoadIdxRegOpcode.end())
        llvm_unreachable("Test_i16_Mem: unexpected register offset.");
      Opcode = OpcodePair->getSecond();
    } else
      llvm_unreachable("Test_i16_Mem: unknown offset type.");
  } else {
    // Test_i8_Mem: TST mem directly. TST sets N/Z, clears V; no register clobber.
    if (OffsetSize >= 0) {
      switch (OffsetSize) {
      case 0:  Opcode = MC6809::TSTi_o0;  break;
      case 5:  Opcode = MC6809::TSTi_o5;  break;
      case 8:  Opcode = MC6809::TSTi_o8;  break;
      case 16: Opcode = MC6809::TSTi_o16; break;
      default: llvm_unreachable("Test_i8_Mem: unexpected imm offset size");
      }
    } else if (OffsetOp.isReg()) {
      Register R = OffsetOp.getReg();
      if      (R == MC6809::AA) Opcode = MC6809::TSTi_oA;
      else if (R == MC6809::AB) Opcode = MC6809::TSTi_oB;
      else if (R == MC6809::AD) Opcode = MC6809::TSTi_oD;
      else if (R == MC6809::AE) Opcode = MC6809::TSTi_oE;
      else if (R == MC6809::AF) Opcode = MC6809::TSTi_oF;
      else if (R == MC6809::AW) Opcode = MC6809::TSTi_oW;
      else llvm_unreachable("Test_i8_Mem: unexpected reg offset");
    } else
      llvm_unreachable("Test_i8_Mem: unknown offset type.");
  }

  // Indexed-operand layout (see MC6809InstrFormats.td):
  //   _o0, _oA, _oB, _oD, _oE, _oF, _oW:  (ins INDEX16:$ireg)               — index only
  //   _o5, _o8, _o16:                     (ins offsetN:$offset, INDEX16:$ireg)
  auto NewMI = Builder.buildInstr(Opcode);
  if (OffsetSize > 0)
    NewMI.add(OffsetOp);
  NewMI.add(IndexOp);
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
  static const std::pair<unsigned, const char *> Flags[] = {
      {MC6809::MO_LO, "lo"},
      {MC6809::MO_HI, "hi"},
      {MC6809::MO_HI_JT, "hi-jt"},
      {MC6809::MO_OS9_DATA, "os9-data"},
      {MC6809::MO_OS9_BSS, "os9-bss"}};
  return Flags;
}

// Bug #387: lets the static-stack target index round-trip through MIR.
ArrayRef<std::pair<int, const char *>> MC6809InstrInfo::getSerializableTargetIndices() const {
  static const std::pair<int, const char *> Flags[] = {{MC6809::TI_STATIC_STACK, "mc6809-static-stack"}};
  return Flags;
}

void MC6809InstrInfo::expandFusedCompareBranch(MachineIRBuilder &Builder, MachineInstr &MI) const {
  // Fused compare-and-branch pseudos (bug #42). These were created as single
  // instructions so the register allocator couldn't insert CC-clobbering
  // reloads between the compare and branch. Now split them back.
  //
  // Operand layout:
  //   TestBranch_*_Reg:  (cc, src, tgt)
  //   TestBranch_*_Mem:  (cc, idx, offset, tgt)
  //   CompareBranch_*_Imm:  (cc, src, imm, tgt)
  //   CompareBranch_*_Reg:  (cc, src, src2, tgt)
  //   CompareBranch_*_Mem:  (cc, src, idx, offset, tgt)
  //
  // The last EXPLICIT operand is always the branch target MBB. (Use the
  // explicit-operand count so that implicit defs like TestBranch_i16_Mem's
  // AD clobber don't shift the target index.)

  unsigned Opc = MI.getOpcode();
  unsigned NumOps = MI.getNumExplicitOperands();
  MachineBasicBlock *TargetMBB = MI.getOperand(NumOps - 1).getMBB();
  unsigned CC = MI.getOperand(0).getImm();

  // Map fused opcode → original compare/test opcode.
  unsigned CmpOpc;
  switch (Opc) {
  case MC6809::TestBranch_i8_Reg:  CmpOpc = MC6809::Test_i8_Reg; break;
  case MC6809::TestBranch_i16_Reg: CmpOpc = MC6809::Test_i16_Reg; break;
  case MC6809::TestBranch_i8_Mem:  CmpOpc = MC6809::Test_i8_Mem; break;
  case MC6809::TestBranch_i16_Mem: CmpOpc = MC6809::Test_i16_Mem; break;
  case MC6809::CompareBranch_i8_Imm:  CmpOpc = MC6809::Compare_i8_Imm; break;
  case MC6809::CompareBranch_i16_Imm: CmpOpc = MC6809::Compare_i16_Imm; break;
  case MC6809::CompareBranch_i8_Reg:  CmpOpc = MC6809::Compare_i8_Reg; break;
  case MC6809::CompareBranch_i16_Reg: CmpOpc = MC6809::Compare_i16_Reg; break;
  case MC6809::CompareBranch_i8_Mem:  CmpOpc = MC6809::Compare_i8_Mem; break;
  case MC6809::CompareBranch_i16_Mem: CmpOpc = MC6809::Compare_i16_Mem; break;
  case MC6809::CompareBranch_i8_MemIndirect:  CmpOpc = MC6809::Compare_i8_MemIndirect; break;
  case MC6809::CompareBranch_i16_MemIndirect: CmpOpc = MC6809::Compare_i16_MemIndirect; break;
  case MC6809::CompareBranch_ptr_Imm:  CmpOpc = MC6809::Compare_ptr_Imm; break; // Bug #359
  case MC6809::CompareBranch_ptr_Reg:  CmpOpc = MC6809::Compare_ptr_Reg; break;
  case MC6809::CompareBranch_ptr_Mem:  CmpOpc = MC6809::Compare_ptr_Mem; break;
  default: llvm_unreachable("Unknown fused compare-branch opcode");
  }

  // Emit the compare/test — all operands except the last (branch target).
  // The compare defines CC as an implicit physical register.
  //
  // Bug #205: $cc is added as a LIVE def (not RegState::Dead). The CC
  // register has $n/$z/$v/$c as sub-registers (see MC6809RegisterInfo.td:
  // 'let CoveredBySubRegs = true'); dead-marking the super-reg propagates
  // to mark all sub-registers immediately dead. The LBlbc emitted just
  // below declares Uses=[N,Z,V,C], so the verifier reported each of those
  // 4 implicit-Uses as "Using an undefined physical register" on every
  // HD6309 long-conditional branch (1-24 errors per real picolibc TU under
  // -mllvm -verify-machineinstrs). Keeping the def live lets the verifier
  // see the sub-register defs as live; the LBlbc's killed-markers handle
  // liveness termination cleanly.
  auto CmpMI = Builder.buildInstr(CmpOpc);
  CmpMI.addDef(MC6809::CC);
  for (unsigned I = 0; I < NumOps - 1; ++I)
    CmpMI.add(MI.getOperand(I));

  // Emit the conditional branch — bug #206 picker selects LBlbc_NoC
  // when the cc doesn't actually consume $c, so the verifier doesn't
  // false-positive on TST-style predecessors.
  Builder.buildInstr(pickLBlbcVariant(CC))
      .addImm(CC)
      .addMBB(TargetMBB);

  // Bug #271 (cat-5): the fused pseudo had two CFG successors —
  // TargetMBB (the conditional target) and the implicit fallthrough.
  // After expansion the LBlbc above is a single-target conditional
  // branch whose not-taken arm falls through to layout-next. If the
  // CFG-fallthrough is a DIFFERENT MBB than layout-next, the BB now
  // has a CFG successor that no terminator reaches — verifier flags
  // it as 'MBB has unexpected successors'. Concrete site:
  // __ubsan_val_to_imax / __ubsan_val_to_umax in libc/ubsan/ at
  // -Og hd6309. Insert a LongBranchRelative to the CFG-fallthrough
  // when layout doesn't already provide it.
  //
  // Critical guard: if MI is NOT the last instruction in the BB,
  // there's already an unconditional branch (placed by insertBranch
  // when CFG-fallthrough != layout-next pre-expansion). Don't
  // duplicate it; insertBranch's LongBranchRelative will handle the
  // fallthrough. Adding another would produce dead duplicate branch
  // instructions (loop.ll regressed under that interpretation).
  if (MI.getNextNode() == nullptr) {
    MachineBasicBlock *MBB = MI.getParent();
    MachineBasicBlock *FallthroughMBB = nullptr;
    for (MachineBasicBlock *Succ : MBB->successors()) {
      if (Succ != TargetMBB) {
        FallthroughMBB = Succ;
        break;
      }
    }
    if (FallthroughMBB && FallthroughMBB != MBB->getNextNode())
      Builder.buildInstr(MC6809::LongBranchRelative).addMBB(FallthroughMBB);
  }

  MI.eraseFromParent();
}

//===----------------------------------------------------------------------===//
// Bug #202 — target-specific MIR verification
//
// Catches structural MIR shapes that round-17/18 of bug #161 hit four times
// in a row (commits 0b00909dd3b4, 9b925cc7ad67, 41a6ab7fddd1; the 4th —
// 5c2af2d12411 — was a semantic operand-order inversion not catchable by a
// structural verifier and is out of scope here).
//
// The check fires only on the actual page-3 reg-reg machine opcodes
// (ADDRp/ADCRp/SUBRp/SBCRp/ANDRp/ORRp/EORRp/CMPRp/TFRp), which are
// IsHD6309-only — there is nothing to verify on plain MC6809 codegen.
//===----------------------------------------------------------------------===//

namespace {
// The set of physical registers that the HD6309 page-3 reg-reg postbyte can
// name. If both source operands resolve to the same one of these, the
// postbyte degenerates to `op X,X`. Mirrors (and consolidates) the inline
// guards in `emitHD6309RegRegOp` and `expandCompareReg`'s collision
// fallback.
bool isHD6309RegRegPostbyteReg(Register R) {
  return R == MC6809::AA || R == MC6809::AB || R == MC6809::AD ||
         R == MC6809::AW || R == MC6809::AE || R == MC6809::AF ||
         R == MC6809::IX || R == MC6809::IY || R == MC6809::SU ||
         R == MC6809::SS;
}

// Byte-size of a physical register named by an HD6309 page-1/2/3 reg-reg
// postbyte. Used to flag size-mismatched TFRp which on HD6309 byte-replicates
// (0x002A → 0x2A2A) instead of zero/sign-extending.
unsigned regByteSize(Register R) {
  switch (R) {
  case MC6809::AA: case MC6809::AB:
  case MC6809::AE: case MC6809::AF:
    return 1;
  case MC6809::AD: case MC6809::AW:
  case MC6809::IX: case MC6809::IY:
  case MC6809::SU: case MC6809::SS:
  case MC6809::PC: case MC6809::AV:
    return 2;
  default:
    return 0; // unknown — verifier will not flag
  }
}
} // end anonymous namespace

bool MC6809InstrInfo::verifyInstruction(const MachineInstr &MI,
                                        StringRef &ErrInfo) const {
  unsigned Opc = MI.getOpcode();

  // Check A — same-physreg postbyte on non-commutative HD6309 page-3 ops.
  //
  // ADDRp/ADCRp/ANDRp/ORRp are commutative; `op X,X` is legitimate (e.g.
  // `ANDR X,X` is identity, `ADDR X,X` is `X<<1`) and is NOT flagged.
  //
  // Operand layout (per MC6809InstrFormats.td: RegisterPairArithmetic /
  // RegisterPairCompare):
  //   ADDRp/ADCRp/SUBRp/SBCRp/ANDRp/ORRp/EORRp:
  //     op0 = $dst (def, tied to $reg2), op1 = $reg1 (use), op2 = $reg2 (use)
  //   CMPRp:
  //     op0 = $reg1 (use), op1 = $reg2 (use)
  switch (Opc) {
  case MC6809::SUBRp:
  case MC6809::SBCRp:
  case MC6809::EORRp: {
    Register R1 = MI.getOperand(1).getReg();
    Register R2 = MI.getOperand(2).getReg();
    if (R1 == R2 && isHD6309RegRegPostbyteReg(R1)) {
      ErrInfo = "HD6309 non-commutative reg-reg op with degenerate "
                "same-reg postbyte (op X,X is always 0 / always equal)";
      return false;
    }
    break;
  }
  case MC6809::CMPRp: {
    Register R1 = MI.getOperand(0).getReg();
    Register R2 = MI.getOperand(1).getReg();
    if (R1 == R2 && isHD6309RegRegPostbyteReg(R1)) {
      ErrInfo = "HD6309 CMPR with degenerate same-reg postbyte "
                "(CMPR X,X is always equal — flags = 0,1,0,0)";
      return false;
    }
    break;
  }

  // Check B — TFRp with size-mismatched operands.
  //
  // HD6309 byte-replicates on size mismatch (e.g. `tfr B,W` with B=0x2A
  // gives W=0x2A2A, not W=0x002A). Bug #161 round 18 follow-up #2
  // (41a6ab7fddd1) fixed copyPhysReg to wrap with explicit zero-extend
  // (CLRA/CLRE) for ACC8↔ACC16 transfers. Any future regression that
  // re-emits a bare size-mismatched TFRp would be caught here.
  //
  // Operand layout: op0 = $reg2 (def), op1 = $reg1 (use). Both must be
  // physregs in the verifier's window (post-ISel onward).
  case MC6809::TFRp: {
    Register Dst = MI.getOperand(0).getReg();
    Register Src = MI.getOperand(1).getReg();
    if (Dst.isPhysical() && Src.isPhysical()) {
      unsigned DstBytes = regByteSize(Dst);
      unsigned SrcBytes = regByteSize(Src);
      if (DstBytes && SrcBytes && DstBytes != SrcBytes) {
        ErrInfo = "HD6309 TFR with size-mismatched operands "
                  "byte-replicates on hardware; wrap with explicit "
                  "zero-extend (TFR + CLRA/CLRE) or sign-extend";
        return false;
      }
    }
    break;
  }

  default:
    break;
  }

  return true;
}
