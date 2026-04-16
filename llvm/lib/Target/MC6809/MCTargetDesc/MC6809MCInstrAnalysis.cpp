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
  // Only inspect declared operands. Variadic instructions (e.g. PSHS/PULS via
  // `variable_ops`) can carry more MCInst operands than MCInstrDesc declares;
  // MCInstrDesc::operands() is sized to the declared count, so indexing
  // beyond that is OOB.
  auto Operands = Info->get(Inst.getOpcode()).operands();
  unsigned NumDeclared = Operands.size();
  if (NumDeclared == 0 || Inst.getNumOperands() < NumDeclared)
    return false;
  const auto &Op = Operands[NumDeclared - 1];
  switch (Op.OperandType) {
  case MC6809Op::OPERAND_ADDR16: {
    Target = (Addr & 0xFFFF0000) | (Inst.getOperand(NumDeclared - 1).getImm() & 0xFFFF);
    return true;
  }
  case MCOI::OPERAND_PCREL: {
    Target = Addr + Size + Inst.getOperand(NumDeclared - 1).getImm();
    return true;
  }
  }
  return false;
}

std::optional<uint64_t> MC6809MCInstrAnalysis::evaluateMemoryOperandAddress(const MCInst &Inst, const MCSubtargetInfo *STI, uint64_t Addr, uint64_t Size) const {
  uint64_t DpAddrOffset = static_cast<const MC6809Subtarget *>(STI)->getDirectPageOffset();
  uint64_t AbsAddrMask = 0xFFFF;

  // Only inspect declared operands. Variadic instructions (e.g. PSHS/PULS via
  // `variable_ops`) can carry more MCInst operands than MCInstrDesc declares;
  // MCInstrDesc::operands() is sized to the declared count, so indexing
  // beyond that is OOB.
  auto Operands = Info->get(Inst.getOpcode()).operands();
  unsigned Bound = std::min<unsigned>(Inst.getNumOperands(), Operands.size());
  // Assumption: Every opcode has only one memory operand.
  for (unsigned OpIdx = 0; OpIdx < Bound; OpIdx++) {
    const auto &Op = Operands[OpIdx];
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
