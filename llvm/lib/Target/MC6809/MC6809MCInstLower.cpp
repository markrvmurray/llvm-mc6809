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
  case MC6809::Load8Imm: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm8\n";);
    switch (MI->getOperand(0).getReg()) {
    default:llvm_unreachable("Unexpected register for LDImm8.");
    case MC6809::AA:OutMI.setOpcode(MC6809::LDAi8);
      break;
    case MC6809::AB:OutMI.setOpcode(MC6809::LDBi8);
      break;
    case MC6809::AE:OutMI.setOpcode(MC6809::LDEi8);
      break;
    case MC6809::AF:OutMI.setOpcode(MC6809::LDFi8);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::Load16Imm: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm16\n";);
    switch (MI->getOperand(0).getReg()) {
    default:llvm_unreachable("Unexpected register for LDImm16.");
    case MC6809::AD:OutMI.setOpcode(MC6809::LDDi16);
      break;
    case MC6809::AW:OutMI.setOpcode(MC6809::LDWi16);
      break;
    case MC6809::IX:OutMI.setOpcode(MC6809::LDXi16);
      break;
    case MC6809::IY:OutMI.setOpcode(MC6809::LDYi16);
      break;
    case MC6809::SU:OutMI.setOpcode(MC6809::LDUi16);
      break;
    case MC6809::SS:OutMI.setOpcode(MC6809::LDSi16);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::Load32Imm: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm32\n";);
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register for LDImm32.");
    case MC6809::AQ:OutMI.setOpcode(MC6809::LDQi32);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LEAPtrAddReg8: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LEAPtrAddReg8\n";);
    Register IndexReg = MI->getOperand(0).getReg();
    Register IndexOperand = MI->getOperand(1).getReg();
    Register OffsetReg = MI->getOperand(2).getReg();
    assert(IndexOperand == IndexReg && "Source and result index registers must be equal");
    switch (IndexReg) {
    case MC6809::IX:
      switch (OffsetReg) {
      case MC6809::AA: {
        OutMI.setOpcode(MC6809::LEAXi_oA);
        break;
      }
      case MC6809::AB: {
        OutMI.setOpcode(MC6809::LEAXi_oB);
        break;
      }
      case MC6809::AE: {
        OutMI.setOpcode(MC6809::LEAXi_oE);
        break;
      }
      case MC6809::AF: {
        OutMI.setOpcode(MC6809::LEAXi_oF);
        break;
      }
      default:
        llvm_unreachable("Illegal 8-bit offset register in LEAPtrAddReg8 (X)");
      }
      break;
    case MC6809::IY:
      switch (OffsetReg) {
      case MC6809::AA: {
        OutMI.setOpcode(MC6809::LEAYi_oA);
        break;
      }
      case MC6809::AB: {
        OutMI.setOpcode(MC6809::LEAYi_oB);
        break;
      }
      case MC6809::AE: {
        OutMI.setOpcode(MC6809::LEAYi_oE);
        break;
      }
      case MC6809::AF: {
        OutMI.setOpcode(MC6809::LEAYi_oF);
        break;
      }
      default:
        llvm_unreachable("Illegal 8-bit offset register in LEAPtrAddReg8 (Y)");
      }
      break;
    case MC6809::SU:
      switch (OffsetReg) {
      case MC6809::AA: {
        OutMI.setOpcode(MC6809::LEAUi_oA);
        break;
      }
      case MC6809::AB: {
        OutMI.setOpcode(MC6809::LEAUi_oB);
        break;
      }
      case MC6809::AE: {
        OutMI.setOpcode(MC6809::LEAUi_oE);
        break;
      }
      case MC6809::AF: {
        OutMI.setOpcode(MC6809::LEAUi_oF);
        break;
      }
      default:
        llvm_unreachable("Illegal 8-bit offset register in LEAPtrAddReg8 (U)");
      }
      break;
    case MC6809::SS:
      switch (OffsetReg) {
      case MC6809::AA: {
        OutMI.setOpcode(MC6809::LEASi_oA);
        break;
      }
      case MC6809::AB: {
        OutMI.setOpcode(MC6809::LEASi_oB);
        break;
      }
      case MC6809::AE: {
        OutMI.setOpcode(MC6809::LEASi_oE);
        break;
      }
      case MC6809::AF: {
        OutMI.setOpcode(MC6809::LEASi_oF);
        break;
      }
      default:
        llvm_unreachable("Illegal 8-bit offset register in LEAPtrAddReg8 (S)");
      }
      break;
    default:
       llvm_unreachable("Unknown pointer register in LEA");
    }
    MCOperand IndexDst, IndexSrc;
    if (!lowerOperand(MI->getOperand(0), IndexDst))
      llvm_unreachable("Failed to lower index destination");
    OutMI.addOperand(IndexDst);
    if (!lowerOperand(MI->getOperand(1), IndexSrc))
      llvm_unreachable("Failed to lower index source");
    OutMI.addOperand(IndexSrc);
    return;
  }
  case MC6809::LEAPtrAddReg16: {
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LEAPtrAddReg16\n";);
    Register IndexReg = MI->getOperand(0).getReg();
    Register IndexOperand = MI->getOperand(1).getReg();
    Register OffsetReg = MI->getOperand(2).getReg();
    assert(IndexOperand == IndexReg && "Source and result index registers must be equal");
    switch (IndexReg) {
    case MC6809::IX:
      switch (OffsetReg) {
      case MC6809::AD: {
        OutMI.setOpcode(MC6809::LEAXi_oD);
        break;
      }
      case MC6809::AW: {
        OutMI.setOpcode(MC6809::LEAXi_oW);
        break;
      }
      default:
        llvm_unreachable("Illegal 16-bit offset register in LEAPtrAddReg16 (X)");
      }
      break;
    case MC6809::IY:
      switch (OffsetReg) {
      case MC6809::AD: {
        OutMI.setOpcode(MC6809::LEAYi_oD);
        break;
      }
      case MC6809::AW: {
        OutMI.setOpcode(MC6809::LEAYi_oW);
        break;
      }
      default:
        llvm_unreachable("Illegal 16-bit offset register in LEAPtrAddReg16 (Y)");
      }
      break;
    case MC6809::SU:
      switch (OffsetReg) {
      case MC6809::AD: {
        OutMI.setOpcode(MC6809::LEAUi_oD);
        break;
      }
      case MC6809::AW: {
        OutMI.setOpcode(MC6809::LEAUi_oW);
        break;
      }
      default:
        llvm_unreachable("Illegal 16-bit offset register in LEAPtrAddReg16 (U)");
      }
      break;
    case MC6809::SS:
      switch (OffsetReg) {
      case MC6809::AD: {
        OutMI.setOpcode(MC6809::LEASi_oD);
        break;
      }
      case MC6809::AW: {
        OutMI.setOpcode(MC6809::LEASi_oW);
        break;
      }
      default:
        llvm_unreachable("Illegal 16-bit offset register in LEAPtrAddReg16 (S)");
      }
      break;
    default:
      llvm_unreachable("Unknown pointer register in LEA");
    }
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 0 OutMI = "; OutMI.dump(););
    MCOperand IndexDst, Offset, IndexSrc;
    if (!lowerOperand(MI->getOperand(0), IndexDst))
      llvm_unreachable("Failed to lower index destination");
    OutMI.addOperand(IndexDst);
    if (!lowerOperand(MI->getOperand(1), Offset))
      llvm_unreachable("Failed to lower offset");
    OutMI.addOperand(Offset);
    if (!lowerOperand(MI->getOperand(2), IndexSrc))
      llvm_unreachable("Failed to lower index source");
    OutMI.addOperand(IndexSrc);
    LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : 1 OutMI = "; OutMI.dump(););
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

bool MC6809MCInstLower::lowerOperand(const MachineOperand &MO,
                                     MCOperand &MCOp) {
  const MC6809RegisterInfo &TRI = *MO.getParent()
                                       ->getMF()
                                       ->getSubtarget<MC6809Subtarget>()
                                       .getRegisterInfo();

  switch (MO.getType()) {
  default:
    report_fatal_error("Operand type not implemented.");
  case MachineOperand::MO_RegisterMask:
    return false;
  case MachineOperand::MO_BlockAddress:
    MCOp =
        lowerSymbolOperand(MO, AP.GetBlockAddressSymbol(MO.getBlockAddress()));
    break;
  case MachineOperand::MO_ExternalSymbol:
    MCOp =
        lowerSymbolOperand(MO, AP.GetExternalSymbolSymbol(MO.getSymbolName()));
    break;
  case MachineOperand::MO_GlobalAddress: {
    const GlobalValue *GV = MO.getGlobal();
    MCOp = lowerSymbolOperand(MO, AP.getSymbol(GV));
    // This is the last chance to catch values that are attributed a direct-page
    // section. It is the user's responsibility to ensure the linker will
    // locate the symbol completely within the direct-page.
    if (MC6809AsmBackend::isBranchSectionName(GV->getSection())) {
      const MC6809MCExpr *Expr =
          MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_8, MCOp.getExpr(),
                               /*isNegated=*/false, Ctx);
      MCOp = MCOperand::createExpr(Expr);
    }
    break;
  }
  case MachineOperand::MO_JumpTableIndex: {
    MCOp = lowerSymbolOperand(MO, AP.GetJTISymbol(MO.getIndex()));
    break;
  }
  case MachineOperand::MO_Immediate:
    MCOp = MCOperand::createImm(MO.getImm());
    break;
  case MachineOperand::MO_MachineBasicBlock:
    MCOp = MCOperand::createExpr(
        MCSymbolRefExpr::create(MO.getMBB()->getSymbol(), Ctx));
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

MCOperand MC6809MCInstLower::lowerSymbolOperand(const MachineOperand &MO,
                                                const MCSymbol *Sym) {
  const MCExpr *Expr = MCSymbolRefExpr::create(Sym, Ctx);
  if (!MO.isJTI() && MO.getOffset() != 0)
    Expr = MCBinaryExpr::createAdd(
        Expr, MCConstantExpr::create(MO.getOffset(), Ctx), Ctx);
  switch (MO.getTargetFlags()) {
  default:
    llvm_unreachable("Invalid target operand flags.");
  case MC6809::MO_NO_FLAGS:
    break;
  case MC6809::MO_LO:
    Expr = MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_16, Expr,
                                /*isNegated=*/false, Ctx);
    break;
  case MC6809::MO_HI:
    Expr = MC6809MCExpr::create(MC6809MCExpr::VK_MC6809_ADDR_16, Expr,
                                /*isNegated=*/false, Ctx);
    break;
  case MC6809::MO_HI_JT: {
    // Jump tables are partitioned in two arrays: first all the low bytes,
    // then all the high bytes. This index referes to the high byte array, so
    // offset the appropriate amount into the overall array.
    assert(MO.isJTI());
    const MachineJumpTableInfo *JTI =
        MO.getParent()->getMF()->getJumpTableInfo();
    const auto &Table = JTI->getJumpTables()[MO.getIndex()];
    assert(Table.MBBs.size() < 256);
    Expr = MCBinaryExpr::createAdd(
        Expr, MCConstantExpr::create(Table.MBBs.size(), Ctx), Ctx);
    break;
  }
  }
  return MCOperand::createExpr(Expr);
}
