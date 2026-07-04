//===-- MC6809FoldAddSub16.cpp - Refold i16 add/sub byte chains -----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Bug #360: recover native 16-bit ADDD/SUBD from byte-decomposed i16 adds.
//
// The legalizer narrows G_ADD/G_SUB s16 to an s8 carry chain
// (MC6809LegalizerInfo.cpp: "G_ADD/G_SUB use s8-only rules"), because an
// earlier attempt to keep i16 add native through the selector miscompiled
// atoi-like code at -O0 (a reg-bank / phi issue with INDEX-bank operands;
// see the NOTE in the legalizer). The selector therefore emits, for an
// i16 add of an immediate:
//
//     %lo  = EXTRACT_LO_i16 %src
//     %hi  = EXTRACT_HI_i16 %src
//     %rlo = AddSetCarry_i8_Imm    %lo, <loImm>   (defs carry %c)
//     %rhi = AddSetCarryUse_i8_Imm %hi, <hiImm>   (uses carry %c)
//     %dst = MERGE_LOHI_i16 %rlo, %rhi
//
// which lowers to `addb #lo; adca #hi` plus byte-spill staging, instead of
// the single `addd #imm` MC6809 has natively. The byte path also only sets
// per-byte flags, forcing a redundant 16-bit compare on the result.
//
// This pass runs on selected SSA MIR at -O1+ (addMachineSSAOptimization)
// and refolds that exact idiom back into the existing Add_i16_Imm pseudo
// (which expands to ADDDi16). It is safe with respect to the reverted i16
// selector work because:
//   * it never runs at -O0 (where #85b manifested), and
//   * it only ever matches a byte chain, which the selector emits solely
//     for ACCUM-bank i16 adds — pointer (INDEX-bank) arithmetic selects to
//     LEA and never produces this shape. So the refold is bank-safe by
//     construction; no INDEX-bank value is ever moved into ADDD here.
//
// Immediate form only for now; the reg/reg (SubSetCarry / AddSetCarry_Reg)
// forms are a follow-up.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-fold-addsub16"

using namespace llvm;

static cl::opt<bool> EnableFoldAddSub16(
    "mc6809-fold-addsub16", cl::init(true), cl::Hidden,
    cl::desc("Bug #360: refold byte-decomposed i16 add/sub of an immediate "
             "back into native ADDD/SUBD"));

namespace {

class MC6809FoldAddSub16 : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldAddSub16() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FoldAddSub16Pass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold i16 add/sub byte chains";
  }
};

} // namespace

/// The single virtual-register operand of MI matching the def/use predicate
/// (used to thread the imaginary carry register between the chain's halves).
static Register implicitVReg(const MachineInstr &MI, bool WantDef) {
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.isImplicit() || !MO.getReg().isVirtual())
      continue;
    if (MO.isDef() == WantDef)
      return MO.getReg();
  }
  return Register();
}

bool MC6809FoldAddSub16::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldAddSub16)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &TII = *MF.getSubtarget().getInstrInfo();
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  bool Changed = false;

  auto defOf = [&](Register R) -> MachineInstr * {
    return R.isVirtual() ? MRI.getVRegDef(R) : nullptr;
  };

  SmallVector<MachineInstr *, 8> ToErase;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      if (MI.getOpcode() != MC6809::MERGE_LOHI_i16)
        continue;
      // %dst = MERGE_LOHI_i16 %lo, %hi
      Register Dst = MI.getOperand(0).getReg();
      Register LoR = MI.getOperand(1).getReg();
      Register HiR = MI.getOperand(2).getReg();

      MachineInstr *LoDef = defOf(LoR);
      MachineInstr *HiDef = defOf(HiR);
      if (!LoDef || !HiDef)
        continue;

      // Classify the carry-chain family from the low-byte op. LoDef must be
      // the non-Use head of the chain (so it can't be a middle byte of a
      // wider add); HiDef must be the matching Use tail. `- C` reaches the
      // backend canonicalised to `+ (-C)`, so the Imm form is Add-only.
      enum Kind { Imm, Mem, Reg } Kind;
      unsigned HiOpc, ResultOpc;
      switch (LoDef->getOpcode()) {
      case MC6809::AddSetCarry_i8_Imm:
        HiOpc = MC6809::AddSetCarryUse_i8_Imm; ResultOpc = MC6809::Add_i16_Imm; Kind = Imm; break;
      case MC6809::AddSetCarry_i8_Mem:
        HiOpc = MC6809::AddSetCarryUse_i8_Mem; ResultOpc = MC6809::Add_i16_Mem; Kind = Mem; break;
      case MC6809::AddSetCarry_i8_Reg:
        HiOpc = MC6809::AddSetCarryUse_i8_Reg; ResultOpc = MC6809::Add_i16_Reg; Kind = Reg; break;
      case MC6809::SubSetCarry_i8_Mem:
        HiOpc = MC6809::SubSetCarryUse_i8_Mem; ResultOpc = MC6809::Sub_i16_Mem; Kind = Mem; break;
      case MC6809::SubSetCarry_i8_Reg:
        HiOpc = MC6809::SubSetCarryUse_i8_Reg; ResultOpc = MC6809::Sub_i16_Reg; Kind = Reg; break;
      default:
        continue;
      }
      if (HiDef->getOpcode() != HiOpc)
        continue;

      // Confirm a genuine low->high carry chain: LoDef's imaginary carry
      // def must be exactly the carry HiDef consumes.
      Register CarryOut = implicitVReg(*LoDef, /*WantDef=*/true);
      Register CarryIn = implicitVReg(*HiDef, /*WantDef=*/false);
      if (!CarryOut || CarryOut != CarryIn)
        continue;

      // The chain must END here: this is a complete, standalone i16 add/sub
      // only if HiDef's own carry-out is dead. If it is live, these two bytes
      // are the low half of a WIDER add (i32/i64) and the carry must keep
      // propagating — folding to ADDD/SUBD would drop the imaginary carry
      // vreg the next byte consumes.
      Register HiCarryOut = implicitVReg(*HiDef, /*WantDef=*/true);
      if (HiCarryOut && !MRI.use_nodbg_empty(HiCarryOut))
        continue;

      // The byte chain leaves CC ($nz/$v/$c) set at its tail; collapsing it to
      // one ADDD/SUBD moves that flag-set to the MERGE position. If anything
      // downstream reads those flags — e.g. usub-sat branches on the
      // subtract's borrow via `$c` between the chain and the MERGE — the fold
      // would reorder the flag-set past the reader and miscompile. Only fold
      // when the chain's CC defs are unused (the common case: the value is
      // consumed via MERGE and any test recomputes its own flags).
      MachineBasicBlock *ChainBB = HiDef->getParent();
      bool FlagsConsumed = false;
      for (auto It = std::next(HiDef->getIterator()); It != ChainBB->end(); ++It) {
        if (It->readsRegister(MC6809::C, &TRI) ||
            It->readsRegister(MC6809::NZ, &TRI) ||
            It->readsRegister(MC6809::V, &TRI)) {
          FlagsConsumed = true;
          break;
        }
        if (It->definesRegister(MC6809::C, &TRI) &&
            It->definesRegister(MC6809::NZ, &TRI) &&
            It->definesRegister(MC6809::V, &TRI))
          break; // all CC flags overwritten before any read — safe
      }
      if (!FlagsConsumed)
        for (MachineBasicBlock *Succ : ChainBB->successors())
          if (Succ->isLiveIn(MC6809::C) || Succ->isLiveIn(MC6809::NZ) ||
              Succ->isLiveIn(MC6809::V)) {
            FlagsConsumed = true;
            break;
          }
      if (FlagsConsumed)
        continue;

      // op0=dst, op1=src(tied minuend), then the subtrahend operand(s). The
      // minuend is normally the lo/hi halves of one i16 value (EXTRACT_LO/HI).
      // It may instead be a constant: `0 - x` (the negate) materialises the
      // minuend into the byte accumulators with Load_i8_Imm rather than an
      // EXTRACT, so it slipped through and stayed a byte chain. For the memory
      // subtrahend form we rematerialise the constant as one Load_i16_Imm and
      // let the native op subtract memory from it: `ldd #imm; subd mem`.
      Register LoSrc = LoDef->getOperand(1).getReg();
      Register HiSrc = HiDef->getOperand(1).getReg();
      MachineInstr *LoSrcDef = defOf(LoSrc);
      MachineInstr *HiSrcDef = defOf(HiSrc);
      if (!LoSrcDef || !HiSrcDef)
        continue;
      if (!MRI.hasOneNonDBGUse(LoR) || !MRI.hasOneNonDBGUse(HiR) ||
          !MRI.hasOneNonDBGUse(CarryOut))
        continue;

      Register Src;
      SmallVector<MachineInstr *, 2> SrcDefsToErase;
      bool MaterialiseConst = false;
      int16_t ConstImm = 0;
      if (LoSrcDef->getOpcode() == MC6809::EXTRACT_LO_i16 &&
          HiSrcDef->getOpcode() == MC6809::EXTRACT_HI_i16 &&
          LoSrcDef->getOperand(1).getReg() == HiSrcDef->getOperand(1).getReg() &&
          MRI.hasOneNonDBGUse(LoSrc) && MRI.hasOneNonDBGUse(HiSrc)) {
        // Extracted-halves minuend: the common `x +/- y`, `x - 1`, ... shape.
        Src = LoSrcDef->getOperand(1).getReg();
        SrcDefsToErase.push_back(LoSrcDef);
        SrcDefsToErase.push_back(HiSrcDef);
      } else if ((Kind == Mem || Kind == Reg) &&
                 LoSrcDef->getOpcode() == MC6809::Load_i8_Imm &&
                 HiSrcDef->getOpcode() == MC6809::Load_i8_Imm &&
                 LoSrcDef->getOperand(1).isImm() &&
                 HiSrcDef->getOperand(1).isImm()) {
        // Constant minuend (`0 - x` negate, `C - x`) against a memory or
        // register subtrahend. The memory form becomes `ldd #imm; subd mem`;
        // the register form becomes the native reg-reg subtract (SUBR on
        // HD6309, a push+SUBD idiom on plain 6809 via Sub_i16_Reg). One shared
        // Load_i8_Imm covers both halves when the bytes are equal (e.g. 0);
        // otherwise two distinct byte defs. Each must feed only this chain so
        // rematerialising the whole i16 minuend is safe.
        ConstImm = (int16_t)(((HiSrcDef->getOperand(1).getImm() & 0xFF) << 8) |
                             (LoSrcDef->getOperand(1).getImm() & 0xFF));
        // The register subtrahend form pays a constant materialisation plus,
        // on plain 6809, a subtrahend spill; that only wins for the true
        // negate (minuend 0 → `subr`/`ldd #0` with no extra load). A non-zero
        // constant minus a register is better left as the byte chain. The
        // memory form is always a win (`ldd #imm; subd mem`), so keep any C
        // there.
        if (Kind == Reg && ConstImm != 0)
          continue;
        if (LoSrc == HiSrc) {
          bool OnlyChainUses = true;
          for (MachineInstr &U : MRI.use_nodbg_instructions(LoSrc))
            if (&U != LoDef && &U != HiDef) {
              OnlyChainUses = false;
              break;
            }
          if (!OnlyChainUses)
            continue;
          SrcDefsToErase.push_back(LoSrcDef);
        } else {
          if (!MRI.hasOneNonDBGUse(LoSrc) || !MRI.hasOneNonDBGUse(HiSrc))
            continue;
          SrcDefsToErase.push_back(LoSrcDef);
          SrcDefsToErase.push_back(HiSrcDef);
        }
        MaterialiseConst = true;
      } else {
        continue;
      }

      if (MaterialiseConst) {
        Src = MRI.createVirtualRegister(MRI.getRegClass(Dst));
        BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(MC6809::Load_i16_Imm), Src)
            .addImm(ConstImm);
      }

      auto MIB = BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(ResultOpc), Dst)
                     .addReg(Src);

      if (Kind == Imm) {
        if (!LoDef->getOperand(2).isImm() || !HiDef->getOperand(2).isImm()) {
          MIB->eraseFromParent();
          continue;
        }
        // imm16 = hi:lo byte immediates of the original i16 addend.
        int16_t Imm16 = (int16_t)(((HiDef->getOperand(2).getImm() & 0xFF) << 8) |
                                  (LoDef->getOperand(2).getImm() & 0xFF));
        MIB.addImm(Imm16);
      } else if (Kind == Mem) {
        // op2=base (frame index or index reg), op3=byte offset. Big-endian:
        // hi byte at base+offHi, lo byte at base+offHi+1, so the addends must
        // be adjacent with LoOff == HiOff+1 and share a base. The 16-bit
        // ADDD/SUBD then reads base+offHi.
        const MachineOperand &LoBase = LoDef->getOperand(2);
        const MachineOperand &HiBase = HiDef->getOperand(2);
        bool SameBase =
            (LoBase.isFI() && HiBase.isFI() && LoBase.getIndex() == HiBase.getIndex()) ||
            (LoBase.isReg() && HiBase.isReg() && LoBase.getReg() == HiBase.getReg());
        if (!SameBase || !LoDef->getOperand(3).isImm() ||
            !HiDef->getOperand(3).isImm() ||
            LoDef->getOperand(3).getImm() != HiDef->getOperand(3).getImm() + 1) {
          MIB->eraseFromParent();
          continue;
        }
        MIB.add(HiDef->getOperand(2)).addImm(HiDef->getOperand(3).getImm());
      } else { // Reg
        // op2 of each byte op is a byte EXTRACTed from one i16 src2.
        Register LoS2 = LoDef->getOperand(2).getReg();
        Register HiS2 = HiDef->getOperand(2).getReg();
        MachineInstr *LoS2Def = defOf(LoS2);
        MachineInstr *HiS2Def = defOf(HiS2);
        if (!LoS2Def || LoS2Def->getOpcode() != MC6809::EXTRACT_LO_i16 ||
            !HiS2Def || HiS2Def->getOpcode() != MC6809::EXTRACT_HI_i16 ||
            LoS2Def->getOperand(1).getReg() != HiS2Def->getOperand(1).getReg() ||
            !MRI.hasOneNonDBGUse(LoS2) || !MRI.hasOneNonDBGUse(HiS2)) {
          MIB->eraseFromParent();
          continue;
        }
        MIB.addReg(LoS2Def->getOperand(1).getReg());
        ToErase.push_back(LoS2Def);
        ToErase.push_back(HiS2Def);
      }

      ToErase.push_back(&MI);
      ToErase.push_back(HiDef);
      ToErase.push_back(LoDef);
      for (MachineInstr *D : SrcDefsToErase)
        ToErase.push_back(D);
      Changed = true;
    }
  }

  for (MachineInstr *MI : ToErase)
    MI->eraseFromParent();

  return Changed;
}

char MC6809FoldAddSub16::ID = 0;

INITIALIZE_PASS(MC6809FoldAddSub16, DEBUG_TYPE,
                "MC6809 fold i16 add/sub byte chains", false, false)

MachineFunctionPass *llvm::createMC6809FoldAddSub16Pass() {
  return new MC6809FoldAddSub16();
}
