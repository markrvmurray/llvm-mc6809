//===-- MC6809MCInstrAnalysis.cpp - MC6809 instruction analysis -----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the declarations of the MC6809MCAsmInfo properties.
//
//===----------------------------------------------------------------------===//

#include "MC6809MCInstrAnalysis.h"
#include "MC6809MCTargetDesc.h"
#include "MC6809Subtarget.h"
#include "llvm/MC/MCInstrDesc.h"
#include "llvm/MC/MCSubtargetInfo.h"

namespace llvm {

bool MC6809MCInstrAnalysis::evaluateBranch(const MCInst &Inst, uint64_t Addr, uint64_t Size, uint64_t &Target) const {
  if ((!isBranch(Inst) && !isCall(Inst)) || isIndirectBranch(Inst))
    return false;
  unsigned NumOps = Inst.getNumOperands();
  if (NumOps == 0)
    return false;
  const auto &Op = Info->get(Inst.getOpcode()).operands()[NumOps - 1];
  switch (Op.OperandType) {
  case MC6809Op::OPERAND_ADDR16: {
    Target = (Addr & 0xFFFF0000) | (Inst.getOperand(NumOps - 1).getImm() & 0xFFFF);
    return true;
  }
  case MCOI::OPERAND_PCREL: {
    Target = Addr + Size + Inst.getOperand(NumOps - 1).getImm();
    return true;
  }
  }
  return false;
}

std::optional<uint64_t> MC6809MCInstrAnalysis::evaluateMemoryOperandAddress(const MCInst &Inst, const MCSubtargetInfo *STI, uint64_t Addr, uint64_t Size) const {
  uint64_t DpAddrOffset = static_cast<const MC6809Subtarget *>(STI)->getDirectPageOffset();
  uint64_t AbsAddrMask = 0xFFFF;

  unsigned NumOps = Inst.getNumOperands();
  // Assumption: Every opcode has only one memory operand.
  for (unsigned OpIdx = 0; OpIdx < NumOps; OpIdx++) {
    const auto &Op = Info->get(Inst.getOpcode()).operands()[OpIdx];
    switch (Op.OperandType) {
    case MC6809Op::OPERAND_ADDR8: {
      return (Addr & ~AbsAddrMask) | DpAddrOffset | (Inst.getOperand(OpIdx).getImm() & 0xFF);
    }
    case MC6809Op::OPERAND_ADDR16: {
      return (Addr & ~AbsAddrMask) | (Inst.getOperand(OpIdx).getImm() & 0xFFFF);
    }
    }
  }
  return std::nullopt;
}

} //  namespace llvm
