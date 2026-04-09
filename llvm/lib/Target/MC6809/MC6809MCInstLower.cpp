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
  // Handle any real instructions that weren't generated from a pseudo.
  if (MI->isPseudo()) {
    LLVM_DEBUG(dbgs() << "Pseudoinstruction was never lowered : "; MI->dump(););
    llvm_unreachable("Pseudoinstruction was never lowered.");
  }
  OutMI.setOpcode(MI->getOpcode());
  for (const MachineOperand &MO : MI->operands()) {
    MCOperand MCOp;
    if (lowerOperand(MO, MCOp))
      OutMI.addOperand(MCOp);
  }
}

bool MC6809MCInstLower::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  switch (MO.getType()) {
  default:
    report_fatal_error("Operand type not implemented.");
  case MachineOperand::MO_RegisterMask:
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
    const auto *GVar = dyn_cast<GlobalVariable>(GV->getAliaseeObject());
    if (MC6809::isDirectPageSectionName(GV->getSection()) ||
        (GVar && GVar->getAddressSpace() == MC6809::AS_DirectPage)) {
      const MC6809MCExpr *Expr = MC6809MCExpr::create(MC6809MCExpr::VK_ADDR8, MCOp.getExpr(), /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
    }
    break;
  }
  case MachineOperand::MO_JumpTableIndex: {
    MCOp = lowerSymbolOperand(MO, AP.GetJTISymbol(MO.getIndex()));
    break;
  }
  case MachineOperand::MO_CImmediate:
    // Use sign-extended value so negative offsets (e.g., i16 -1 from a
    // pre-decrement pointer) are emitted as -1, not as 65535.
    MCOp = MCOperand::createImm(MO.getCImm()->getSExtValue());
    break;
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = MCOperand::createExpr(MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
    break;
  case MachineOperand::MO_Register: {
    // Ignore all implicit register operands.
    if (MO.isImplicit())
      return false;
    Register Reg = MO.getReg();
    // Convert imaginary registers to symbol references (direct-page addresses).
    if (MC6809::Imag16RegClass.contains(Reg) ||
        MC6809::Imag8RegClass.contains(Reg)) {
      const auto &TRI =
          *MO.getParent()->getMF()->getSubtarget().getRegisterInfo();
      const MC6809RegisterInfo &MC6809TRI =
          static_cast<const MC6809RegisterInfo &>(TRI);
      const MCExpr *Expr = MCSymbolRefExpr::create(
          Ctx.getOrCreateSymbol(MC6809TRI.getImag8SymbolName(Reg)), Ctx);
      MCOp = MCOperand::createExpr(Expr);
    } else
      MCOp = MCOperand::createReg(Reg);
    break;
  }
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
