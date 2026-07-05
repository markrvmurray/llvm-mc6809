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
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineJumpTableInfo.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/MC/MCExpr.h"
#include "llvm/MC/MCInstrInfo.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Support/FormatVariadic.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-mcinstlower"

void MC6809MCInstLower::lower(const MachineInstr *MI, MCInst &OutMI) {
  // Handle any real instructions that weren't generated from a pseudo.
  if (MI->isPseudo()) {
    // Always print the opcode name and full dump unconditionally — this
    // failure mode is rare, developer-facing, and a silent llvm_unreachable
    // makes diagnosis unnecessarily hard. See bug #90 for a case where
    // this exact path fired on ``SEX8Implicit`` and the unreachable message
    // alone gave no hint which pseudo was missing.
    const TargetInstrInfo &TII = *MI->getMF()->getSubtarget().getInstrInfo();
    const MCInstrInfo *MCII = MI->getMF()->getTarget().getMCInstrInfo();
    StringRef OpName = MCII ? MCII->getName(MI->getOpcode()) : StringRef("<unknown>");
    (void)TII;
    errs() << "MC6809: pseudo instruction was never lowered: "
           << OpName << " (opcode=" << MI->getOpcode() << ")\n"
           << "  in function: "
           << MI->getMF()->getName() << "\n"
           << "  MI: ";
    MI->print(errs());
    errs() << "\nFix path:\n"
           << "  1. Add a case for ``" << OpName
           << "`` to ``MC6809InstrInfo::expandPostRAPseudo``.\n"
           << "  2. Or set ``PseudoInstExpansion<(...)>`` on the tablegen\n"
           << "     definition so ``lowerPseudoInstExpansion`` handles it.\n"
           << "  3. See ``memory/project_materialize_spills_liveness.md``\n"
           << "     for the MCInstrDesc invariant and the #57/#88/#89/#90\n"
           << "     worked examples.\n";
    llvm_unreachable("MC6809: pseudo instruction was never lowered.");
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
    // This is the last chance to catch values that are attributed to the direct
    // page and truncate the symbol reference to its 8-bit low byte (VK_ADDR8).
    // It is the user's responsibility to ensure the linker locates the symbol
    // completely within the direct page. There are two distinct cases:
    //
    //  * An addrspace(1) global is a genuine direct-page object: its pointer
    //    type is 8-bit (p1:8 in the data layout), so *every* reference to it is
    //    the in-page offset — truncate unconditionally.
    //
    //  * A `.dp` *section* on an ordinary (addrspace 0, 16-bit-pointer) global —
    //    e.g. a static-stack frame placed in the direct page — is different.
    //    Only an actual direct-page-addressing access (TSFlagDirectPageAddr)
    //    consumes the 8-bit form. An address-computing use of the same symbol —
    //    a PC-relative LEA taking the address of a frame local, an extended or
    //    indexed access — needs the full 16-bit reference; truncating there
    //    collapses e.g. `leay sym,pc` to its 8-bit form and computes a wrong
    //    pointer.
    const auto *GVar = dyn_cast<GlobalVariable>(GV->getAliaseeObject());
    bool Truncate = false;
    if (GVar && GVar->getAddressSpace() == MC6809::AS_DirectPage) {
      Truncate = true;
    } else if (MC6809::isDirectPageSectionName(GV->getSection())) {
      const MachineInstr *ParentMI = MO.getParent();
      Truncate = ParentMI &&
                 (ParentMI->getDesc().TSFlags & MC6809::TSFlagDirectPageAddr);
    }
    if (Truncate) {
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
    // Convert imaginary registers to symbol references (direct-page
    // addresses). Keyed off the symbol-name table rather than the Imag8/
    // Imag16 classes: the RS byte sub-registers (RS0HI..RS3LO) belong to
    // neither class but have direct-page symbols of their own, and an
    // allocated RS value accessed bytewise (EXTRACT_LO/HI on an RS-homed
    // i16) reaches here through them.
    const auto &TRI =
        *MO.getParent()->getMF()->getSubtarget().getRegisterInfo();
    const MC6809RegisterInfo &MC6809TRI =
        static_cast<const MC6809RegisterInfo &>(TRI);
    const char *SymName = MC6809TRI.getImag8SymbolName(Reg);
    if (SymName && SymName[0]) {
      const MCExpr *Expr = MCSymbolRefExpr::create(
          Ctx.getOrCreateSymbol(SymName), Ctx);
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
  case MC6809::MO_OS9_DATA:
  case MC6809::MO_OS9_BSS:
    Expr = MC6809MCExpr::create(MC6809MCExpr::VK_OS9_DATA_OFFSET, Expr,
                                /*IsNegated=*/false, Ctx);
    break;
  }
  return MCOperand::createExpr(Expr);
}
