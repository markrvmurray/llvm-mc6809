//===-- MC6809MCInstLower.cpp - Convert MC6809 MachineInstr to an MCInst --------===//
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
#include "MCTargetDesc/MC6809AsmBackend.h"
#include "MCTargetDesc/MC6809MCExpr.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809InstrInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
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
  default:
    OutMI.setOpcode(MI->getOpcode());
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
  case MC6809::LDImm8: {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm8\n";);
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register for LDImm8.");
    case MC6809::AA:
      OutMI.setOpcode(MC6809::LDAi8);
      break;
    case MC6809::AB:
      OutMI.setOpcode(MC6809::LDBi8);
      break;
    case MC6809::AE:
      OutMI.setOpcode(MC6809::LDEi8);
      break;
    case MC6809::AF:
      OutMI.setOpcode(MC6809::LDFi8);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LDImm16: {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm16\n";);
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register for LDImm16.");
    case MC6809::AD:
      OutMI.setOpcode(MC6809::LDDi16);
      break;
    case MC6809::AW:
      OutMI.setOpcode(MC6809::LDWi16);
      break;
    case MC6809::IX:
      OutMI.setOpcode(MC6809::LDXi16);
      break;
    case MC6809::IY:
      OutMI.setOpcode(MC6809::LDYi16);
      break;
    case MC6809::SU:
      OutMI.setOpcode(MC6809::LDUi16);
      break;
    case MC6809::SS:
      OutMI.setOpcode(MC6809::LDSi16);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LDImm32: {
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : LDImm32\n";);
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register for LDImm32.");
    case MC6809::AQ:
      OutMI.setOpcode(MC6809::LDQi32);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
#if 0
  case MC6809::ADCAbsIdx:
  case MC6809::SBCAbsIdx: {
    switch (MI->getOpcode()) {
    case MC6809::ADCAbsIdx:
      switch (MI->getOperand(5).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::ADC_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::ADC_AbsoluteY);
        break;
      }
      break;
    case MC6809::SBCAbsIdx:
      switch (MI->getOperand(5).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::SBC_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::SBC_AbsoluteY);
        break;
      }
      break;
    }
    MCOperand Addr;
    if (!lowerOperand(MI->getOperand(4), Addr))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Addr);
    return;
  }
  case MC6809::ANDAbsIdx:
  case MC6809::EORAbsIdx:
  case MC6809::ORAAbsIdx: {
    switch (MI->getOpcode()) {
    case MC6809::ANDAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::AND_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::AND_AbsoluteY);
        break;
      }
      break;
    case MC6809::EORAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::EOR_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::EOR_AbsoluteY);
        break;
      }
      break;
    case MC6809::ORAAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::ORA_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::ORA_AbsoluteY);
        break;
      }
      break;
    }
    MCOperand Addr;
    if (!lowerOperand(MI->getOperand(2), Addr))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Addr);
    return;
  }
  case MC6809::ASL:
  case MC6809::LSR:
  case MC6809::ROL:
  case MC6809::ROR:
    switch (MI->getOperand(0).getReg()) {
    default: {
      assert(MC6809::Imag8RegClass.contains(MI->getOperand(0).getReg()));
      switch (MI->getOpcode()) {
      case MC6809::ASL:
        OutMI.setOpcode(MC6809::ASL_DirectPage);
        break;
      case MC6809::LSR:
        OutMI.setOpcode(MC6809::LSR_DirectPage);
        break;
      case MC6809::ROL:
        OutMI.setOpcode(MC6809::ROL_DirectPage);
        break;
      case MC6809::ROR:
        OutMI.setOpcode(MC6809::ROR_DirectPage);
        break;
      }
      MCOperand Addr;
      if (!lowerOperand(MI->getOperand(0), Addr))
        llvm_unreachable("Failed to lower operand");
      OutMI.addOperand(Addr);
      return;
    }
    case MC6809::A:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MC6809::ASL:
        OutMI.setOpcode(MC6809::ASL_Accumulator);
        return;
      case MC6809::LSR:
        OutMI.setOpcode(MC6809::LSR_Accumulator);
        return;
      case MC6809::ROL:
        OutMI.setOpcode(MC6809::ROL_Accumulator);
        return;
      case MC6809::ROR:
        OutMI.setOpcode(MC6809::ROR_Accumulator);
        return;
      }
    }
  case MC6809::BR: {
    Register Flag = MI->getOperand(1).getReg();
    int64_t Val = MI->getOperand(2).getImm();
    switch (Flag) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::C:
      OutMI.setOpcode(Val ? MC6809::BCS_Relative : MC6809::BCC_Relative);
      break;
    case MC6809::N:
      OutMI.setOpcode(Val ? MC6809::BMI_Relative : MC6809::BPL_Relative);
      break;
    case MC6809::V:
      OutMI.setOpcode(Val ? MC6809::BVS_Relative : MC6809::BVC_Relative);
      break;
    case MC6809::Z:
      OutMI.setOpcode(Val ? MC6809::BEQ_Relative : MC6809::BNE_Relative);
      break;
    }
    MCOperand Tgt;
    if (!lowerOperand(MI->getOperand(0), Tgt))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Tgt);
    return;
  }
  case MC6809::CMPImm:
  case MC6809::CMPImag8:
  case MC6809::CMPAbs:
  case MC6809::CMPAbsIdx: {
    switch (MI->getOpcode()) {
    case MC6809::CMPImm:
      switch (MI->getOperand(1).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::A:
        OutMI.setOpcode(MC6809::CMP_Immediate);
        break;
      case MC6809::X:
        OutMI.setOpcode(MC6809::CPX_Immediate);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::CPY_Immediate);
        break;
      }
      break;
    case MC6809::CMPImag8:
    case MC6809::CMPAbs:
      switch (MI->getOperand(1).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::A:
        OutMI.setOpcode(MC6809::CMP_DirectPage);
        break;
      case MC6809::X:
        OutMI.setOpcode(MC6809::CPX_DirectPage);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::CPY_DirectPage);
        break;
      }
      break;
    case MC6809::CMPAbsIdx:
      switch (MI->getOperand(3).getReg()) {
      default:
        llvm_unreachable("Unexpected register.");
      case MC6809::X:
        OutMI.setOpcode(MC6809::CMP_DirectPageX);
        break;
      case MC6809::Y:
        OutMI.setOpcode(MC6809::CMP_AbsoluteY);
        break;
      }
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(2), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LDImm8:
  case MC6809::LDAbs:
  case MC6809::LDImag8:
  case MC6809::STAbs: {
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::A:
      switch (MI->getOpcode()) {
      case MC6809::LDImm8:
        OutMI.setOpcode(MC6809::LDA_Immediate);
        break;
      case MC6809::LDAbs:
      case MC6809::LDImag8:
        OutMI.setOpcode(MC6809::LDA_DirectPage);
        break;
      case MC6809::STAbs:
        OutMI.setOpcode(MC6809::STA_DirectPage);
        break;
      }
      break;
    case MC6809::X:
      switch (MI->getOpcode()) {
      case MC6809::LDImm8:
        OutMI.setOpcode(MC6809::LDX_Immediate);
        break;
      case MC6809::LDAbs:
      case MC6809::LDImag8:
        OutMI.setOpcode(MC6809::LDX_DirectPage);
        break;
      case MC6809::STAbs:
        OutMI.setOpcode(MC6809::STX_DirectPage);
        break;
      }
      break;
    case MC6809::Y:
      switch (MI->getOpcode()) {
      case MC6809::LDImm8:
        OutMI.setOpcode(MC6809::LDY_Immediate);
        break;
      case MC6809::LDAbs:
      case MC6809::LDImag8:
        OutMI.setOpcode(MC6809::LDY_DirectPage);
        break;
      case MC6809::STAbs:
        OutMI.setOpcode(MC6809::STY_DirectPage);
        break;
      }
      break;
    }
    int64_t ImmIdx = MI->getOpcode() == MC6809::CMPImm ? 2 : 1;
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(ImmIdx), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LDAAbsIdx: {
    switch (MI->getOperand(2).getReg()) {
    default:
      llvm_unreachable("Unexpected LDAAbsIdx register.");
    case MC6809::X:
      OutMI.setOpcode(MC6809::LDA_DirectPageX);
      break;
    case MC6809::Y:
      OutMI.setOpcode(MC6809::LDA_AbsoluteY);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::LDCImm: {
    switch (MI->getOperand(1).getImm()) {
    default:
      llvm_unreachable("Unexpected LDCImm immediate.");
    case 0:
      OutMI.setOpcode(MC6809::CLC_Implied);
      return;
    case -1:
      OutMI.setOpcode(MC6809::SEC_Implied);
      return;
    }
  }
  case MC6809::DE:
  case MC6809::IN:
  case MC6809::TA:
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::X:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MC6809::DE:
        OutMI.setOpcode(MC6809::DEX_Implied);
        return;
      case MC6809::IN:
        OutMI.setOpcode(MC6809::INX_Implied);
        return;
      case MC6809::TA:
        OutMI.setOpcode(MC6809::TAX_Implied);
        return;
      }
    case MC6809::Y:
      switch (MI->getOpcode()) {
      default:
        llvm_unreachable("Inconsistent opcode.");
      case MC6809::DE:
        OutMI.setOpcode(MC6809::DEY_Implied);
        return;
      case MC6809::IN:
        OutMI.setOpcode(MC6809::INY_Implied);
        return;
      case MC6809::TA:
        OutMI.setOpcode(MC6809::TAY_Implied);
        return;
      }
    }
  case MC6809::PH:
  case MC6809::PL: {
    switch (MI->getOperand(0).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::A:
      OutMI.setOpcode(MI->getOpcode() == MC6809::PH ? MC6809::PHA_Implied
                                                 : MC6809::PLA_Implied);
      return;
    case MC6809::X:
      OutMI.setOpcode(MI->getOpcode() == MC6809::PH ? MC6809::PHX_Implied
                                                 : MC6809::PLX_Implied);
      return;
    case MC6809::Y:
      OutMI.setOpcode(MI->getOpcode() == MC6809::PH ? MC6809::PHY_Implied
                                                 : MC6809::PLY_Implied);
      return;
    case MC6809::P:
      OutMI.setOpcode(MI->getOpcode() == MC6809::PH ? MC6809::PHP_Implied
                                                 : MC6809::PLP_Implied);
      return;
    }
  }
  case MC6809::STAbsIdx: {
    switch (MI->getOperand(2).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::X:
      OutMI.setOpcode(MC6809::STA_DirectPageX);
      break;
    case MC6809::Y:
      OutMI.setOpcode(MC6809::STA_AbsoluteY);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(1), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::STImag8: {
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::A:
      OutMI.setOpcode(MC6809::STA_DirectPage);
      break;
    case MC6809::X:
      OutMI.setOpcode(MC6809::STX_DirectPage);
      break;
    case MC6809::Y:
      OutMI.setOpcode(MC6809::STY_DirectPage);
      break;
    }
    MCOperand Val;
    if (!lowerOperand(MI->getOperand(0), Val))
      llvm_unreachable("Failed to lower operand");
    OutMI.addOperand(Val);
    return;
  }
  case MC6809::T_A:
    switch (MI->getOperand(1).getReg()) {
    default:
      llvm_unreachable("Unexpected register.");
    case MC6809::X:
      OutMI.setOpcode(MC6809::TXA_Implied);
      return;
    case MC6809::Y:
      OutMI.setOpcode(MC6809::TYA_Implied);
      return;
    }
#endif
  }
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : OutMI = "; OutMI.dump(););

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
  LLVM_DEBUG(dbgs() << "OINQUE DEBUG " << __func__ << " : OutMI = "; OutMI.dump(););
}

bool MC6809MCInstLower::lowerOperand(const MachineOperand &MO, MCOperand &MCOp) {
  const MC6809RegisterInfo &TRI =
      *MO.getParent()->getMF()->getSubtarget<MC6809Subtarget>().getRegisterInfo();

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
