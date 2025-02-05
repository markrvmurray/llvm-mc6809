//===-- MC6809MCInstLower.h - Lower MachineInstr to MCInst ---------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
#ifndef LLVM_LIB_TARGET_MC6809_MC6809MCINSTLOWER_H
#define LLVM_LIB_TARGET_MC6809_MC6809MCINSTLOWER_H

#include "llvm/CodeGen/AsmPrinter.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/MC/MCContext.h"

namespace llvm {

class MC6809MCInstLower {
  MCContext &Ctx;
  const AsmPrinter &AP;

public:
  MC6809MCInstLower(MCContext &Ctx, const AsmPrinter &AP) : Ctx(Ctx), AP(AP) {}

  void lower(const MachineInstr *MI, MCInst &OutMI);
  bool lowerOperand(const MachineOperand &MO, MCOperand &MCOp);

private:
  MCOperand lowerSymbolOperand(const MachineOperand &MO, const MCSymbol *Sym);
};

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809MCINSTLOWER_H
