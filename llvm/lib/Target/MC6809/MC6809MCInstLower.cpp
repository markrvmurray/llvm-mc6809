//===-- MC6809MCInstLower.cpp - Convert MC6809 MachineInstr to an MCInst
//--------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains code to lower MC6809 MachineInstrs to their corresponding
// MCInst records.
//
//===----------------------------------------------------------------------===//
#include "MC6809MCInstLower.h"
#include "MC6809InstrInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809AsmBackend.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormatVariadic.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-mcinstlower"

void MC6809MCInstLower::lower(const MachineInstr *MI, MCInst &OutMI) {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MI = "; MI->dump(););
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : OutMI = "; OutMI.dump(););
  switch (MI->getOpcode()) {
  default:OutMI.setOpcode(MI->getOpcode());
    break;
  case MC6809::ReturnImplicit: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : ReturnImplicit\n";);
    OutMI.setOpcode(MC6809::RTSr);
    return;
  }
  case MC6809::ReturnIRQImplicit: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : ReturnIRQImplicit\n";);
    OutMI.setOpcode(MC6809::RTIr);
    return;
  }
  case MC6809::SEX16Implicit: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SEX16Implicit\n";);
    OutMI.setOpcode(MC6809::SEXx);
    return;
  }
  case MC6809::SEX32Implicit: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : SEX32Implicit\n";);
    OutMI.setOpcode(MC6809::SEXWx);
    return;
  }
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 2 OutMI = "; OutMI.dump(););

  // Handle any real instructions that weren't generated from a pseudo.
#ifndef NDEBUG
  if (MI->isPseudo()) {
    dbgs() << *MI;
    llvm_unreachable("Pseudoinstruction was never lowered.");
  }
#endif
  for (const MachineOperand &MO : MI->operands()) {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO = "; MO.dump(););
    MCOperand MCOp;
    if (lowerOperand(MO, MCOp))
      OutMI.addOperand(MCOp);
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 3 OutMI = "; OutMI.dump(););
}

bool MC6809MCInstLower::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  const MC6809RegisterInfo &TRI = *MO.getParent()->getMF()->getSubtarget<MC6809Subtarget>().getRegisterInfo();

  switch (MO.getType()) {
  default:
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : MO.getType() = " << (int)(MO.getType()) << "\n";);
    report_fatal_error("Operand type not implemented.");
  case MachineOperand::MO_RegisterMask:
    LLVM_DEBUG(dbgs() << "Operand MachineOperand::MO_RegisterMask not implemented\n";);
    return false;
  case MachineOperand::MO_BlockAddress:
    MCOp = lowerSymbolOperand(MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()));
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp = lowerSymbolOperand(MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()));
    break;
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MO.getGlobal();
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(GV));
    // This is the last chance to catch values that are attributed a direct-page
    // section. It is the user's responsibility to ensure the linker will
    // locate the symbol completely within the direct-page.
    if (MC6809AsmBackend::isBranchSectionName(GV->getSection())) {
      const MC6809MCExpr *Expr = MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_8, MCOp.getExpr(), /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
    }
    break;
  }
  case MachineOperand::MO_JumpTableIndex: {
    MCOp = lowerSymbolOperand(MO, AP.GetJTISymbol(MO.getIndex()));
    break;
  }
  case MachineOperand::MO_CImmediate:
    MCOp = MCOperand::createImm(MO.getCImm()->getLimitedValue());
    break;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = MCOperand::createExpr(MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
    break;
  case MachineOperand::MO_Register:
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    MCOp = MCOperand::createReg(MO.getReg());
    break;
  }
  return true;
}

MCOperand MC6809MCInstLower::lowerSymbolOperand(const MachineOperand &MO, const MCSymbol *Sym) {
  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);
  if (!MO.isJTI() && MO.getOffset() != 0)
    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
  switch (MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target operand flags.");
  case MC6809::MO_NO_FLAGS:
    break;
  case MC6809::MO_LO:
    Expr = MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_16, Expr, /*isNegated=*/false, Ctx);
    break;
  case MC6809::MO_HI:
    Expr = MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_16, Expr, /*isNegated=*/false, Ctx);
    break;
  case MC6809::MO_HI_JT: {
    // Jump tables are partitioned in two arrays: first all the low bytes,
    // then all the high bytes. This index referes to the high byte array, so
    // offset the appropriate amount into the overall array.
    assert(MO.isJTI());
    const MachineJumpTableInfo *JTI = MO.getParent()->getMF()->getJumpTableInfo();
    const auto &Table = JTI->getJumpTables()[MO.getIndex()];
    assert(Table.MBBs.size() < 256);
    Expr = MCBinaryExpr::createAdd(Expr, MCConstantExpr::create(Table.MBBs.size(), Ctx), Ctx);
    break;
  }
  }
  return MCOperand::createExpr(Expr);
}
