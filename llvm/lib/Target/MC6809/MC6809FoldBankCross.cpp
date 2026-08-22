//===-- MC6809FoldBankCross.cpp - Fold INDEX->ACCUM copies into uses ------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Eliminate the bank-crossing copy a value picks up when it is produced in an
// index register (a pointer, an index computation, a call result returned in X)
// but consumed by an operation modelled in the accumulator bank.
//
//     %d:acc16 = COPY %x:index16      (tfr x,d)
//     Store_i16_Mem %d, %base, off    (std off,base)   -> Store_iPtr_Mem %x, ...   (stx off,base)
//     Compare_i16_Imm cc, %d, #imm    (cmpd #imm)      -> Compare_ptr_Imm cc, %x   (cmpx #imm)
//     Compare_i16_Mem cc, %d, %b, off (cmpd off,b)     -> Compare_ptr_Mem cc, %x   (cmpx off,b)
//     Add_i16_Reg %a, %d              (tfr x,d ...)    -> Add_i16_RegIdx %a, %x    (pshs x; addd ,s++)
//     (and Sub/AND/OR/XOR/carry-chain and Compare _Reg forms: the second
//     source is read from memory after a push either way)
//
// MC6809 has index-register store and compare forms (STX/STY/STU, CMPX/CMPY/
// CMPU), so the copy into D is pure overhead. Folding it pre-register-allocation
// means the value never occupies an accumulator at all — the allocator keeps it
// in one index register across the use, instead of pinning D for a copy that a
// later peephole would only undo locally.
//
// The Compare_i16/Compare_ptr and Store_i16/Store_iPtr pseudo families share an
// identical operand layout (only the value operand's register class differs),
// so the fold just retargets the opcode and the value operand. The
// register-register compare form (Compare_i16_Reg) is deliberately excluded:
// base 6809 has no register-register index compare, so that case stays in the
// accumulator bank. The _Imm and _Mem forms compare against an immediate or a
// memory operand and are a clean, copy-free win.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-fold-bank-cross"

using namespace llvm;

static cl::opt<bool> EnableFoldBankCross(
    "mc6809-enable-fold-bank-cross", cl::init(true), cl::Hidden,
    cl::desc("Fold an INDEX->ACCUM copy into the store/compare that uses it so "
             "the value stays in an index register (stx/cmpx, no tfr x,d)"));

namespace {

class MC6809FoldBankCross : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldBankCross() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FoldBankCrossPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold index->accumulator copy into store/compare";
  }
};

// For a foldable consumer of an accumulator value, return the index-bank
// opcode it becomes and the operand index that holds the value. Returns
// {0, -1} for anything that must stay in the accumulator.
struct FoldTarget {
  unsigned Opc;
  int ValueOp;
};
static FoldTarget indexConsumer(unsigned Opc) {
  switch (Opc) {
  case MC6809::Store_i16_Mem:         return {MC6809::Store_iPtr_Mem, 0};
  case MC6809::Store_i16_Sym:         return {MC6809::Store_iPtr_Sym, 0};
  case MC6809::Compare_i16_Imm:       return {MC6809::Compare_ptr_Imm, 2};
  case MC6809::Compare_i16_Mem:       return {MC6809::Compare_ptr_Mem, 2};
  case MC6809::Compare_i16_Sym:       return {MC6809::Compare_ptr_Sym, 2};
  case MC6809::CompareBranch_i16_Imm: return {MC6809::CompareBranch_ptr_Imm, 1};
  case MC6809::CompareBranch_i16_Mem: return {MC6809::CompareBranch_ptr_Mem, 1};
  case MC6809::CompareBranch_i16_Sym: return {MC6809::CompareBranch_ptr_Sym, 1};
  // The SECOND source of a 16-bit register-register arithmetic, bitwise or
  // compare pseudo may be an index register (the _RegIdx sibling): the
  // 6809 reads that operand from memory after a push either way, so a
  // value that arrived in X or Y stays there instead of being copied into
  // D and evicting the tied operand into an imaginary register.
  case MC6809::Add_i16_Reg:               return {MC6809::Add_i16_RegIdx, 2};
  case MC6809::Sub_i16_Reg:               return {MC6809::Sub_i16_RegIdx, 2};
  case MC6809::AddSetCarry_i16_Reg:       return {MC6809::AddSetCarry_i16_RegIdx, 2};
  case MC6809::SubSetCarry_i16_Reg:       return {MC6809::SubSetCarry_i16_RegIdx, 2};
  case MC6809::AddSetCarryUse_i16_Reg:    return {MC6809::AddSetCarryUse_i16_RegIdx, 2};
  case MC6809::SubSetCarryUse_i16_Reg:    return {MC6809::SubSetCarryUse_i16_RegIdx, 2};
  case MC6809::AddSetOverflow_i16_Reg:    return {MC6809::AddSetOverflow_i16_RegIdx, 2};
  case MC6809::SubSetOverflow_i16_Reg:    return {MC6809::SubSetOverflow_i16_RegIdx, 2};
  case MC6809::AddSetOverflowUse_i16_Reg: return {MC6809::AddSetOverflowUse_i16_RegIdx, 2};
  case MC6809::SubSetOverflowUse_i16_Reg: return {MC6809::SubSetOverflowUse_i16_RegIdx, 2};
  case MC6809::AND_i16_Reg:               return {MC6809::AND_i16_RegIdx, 2};
  case MC6809::OR_i16_Reg:                return {MC6809::OR_i16_RegIdx, 2};
  case MC6809::XOR_i16_Reg:               return {MC6809::XOR_i16_RegIdx, 2};
  case MC6809::Compare_i16_Reg:           return {MC6809::Compare_i16_RegIdx, 3};
  case MC6809::CompareBranch_i16_Reg:     return {MC6809::CompareBranch_i16_RegIdx, 2};
  default:                            return {0, -1};
  }
}

} // namespace

bool MC6809FoldBankCross::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldBankCross)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  bool Changed = false;

  // Collect the INDEX->ACCUM copies first; rewriting erases them.
  SmallVector<MachineInstr *, 8> Copies;
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB) {
      if (!MI.isCopy())
        continue;
      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();
      if (!Dst.isVirtual())
        continue;
      const TargetRegisterClass *DstRC = MRI.getRegClassOrNull(Dst);
      if (!DstRC || !MC6809::ACC16RegClass.hasSubClassEq(DstRC))
        continue;
      // The source is an index-bank virtual register, or one of the index
      // registers themselves (an incoming argument in X, a call result).
      if (Src.isVirtual()) {
        const TargetRegisterClass *SrcRC = MRI.getRegClassOrNull(Src);
        if (SrcRC && MC6809::INDEX16RegClass.hasSubClassEq(SrcRC))
          Copies.push_back(&MI);
      } else if (Src == MC6809::IX || Src == MC6809::IY) {
        Copies.push_back(&MI);
      }
    }
  for (MachineInstr *Copy : Copies) {
    Register AccReg = Copy->getOperand(0).getReg();
    Register IdxReg = Copy->getOperand(1).getReg();
    // A physical index source gets its own index-bank virtual register,
    // copied right where the accumulator copy was (the physical register is
    // known to hold the value there and nowhere else).
    if (IdxReg.isPhysical()) {
      bool AnyConsumer = false;
      for (MachineInstr &Use : MRI.use_nodbg_instructions(AccReg)) {
        FoldTarget FT = indexConsumer(Use.getOpcode());
        if (FT.Opc && Use.getOperand(FT.ValueOp).isReg() &&
            Use.getOperand(FT.ValueOp).getReg() == AccReg)
          AnyConsumer = true;
      }
      if (!AnyConsumer)
        continue;
      Register NewIdx = MRI.createVirtualRegister(&MC6809::INDEX16RegClass);
      BuildMI(*Copy->getParent(), std::next(Copy->getIterator()),
              Copy->getDebugLoc(), TII.get(TargetOpcode::COPY), NewIdx)
          .addReg(IdxReg);
      IdxReg = NewIdx;
    }

    // Retarget each store/compare that uses the accumulator value to read the
    // index register directly. Other uses keep the copy alive.
    for (MachineInstr &Use :
         llvm::make_early_inc_range(MRI.use_nodbg_instructions(AccReg))) {
      FoldTarget FT = indexConsumer(Use.getOpcode());
      if (!FT.Opc)
        continue;
      MachineOperand &ValMO = Use.getOperand(FT.ValueOp);
      if (!ValMO.isReg() || ValMO.getReg() != AccReg)
        continue;
      Use.setDesc(TII.get(FT.Opc));
      ValMO.setReg(IdxReg);
      Changed = true;
    }

    if (MRI.use_nodbg_empty(AccReg))
      Copy->eraseFromParent();
  }
  return Changed;
}

char MC6809FoldBankCross::ID = 0;

INITIALIZE_PASS(MC6809FoldBankCross, DEBUG_TYPE,
                "MC6809 fold index->accumulator copy into store/compare", false,
                false)

MachineFunctionPass *llvm::createMC6809FoldBankCrossPass() {
  return new MC6809FoldBankCross();
}
