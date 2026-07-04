//===-- MC6809MaterializeSpills.cpp - Rewrite spill regs to real regs ------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Generic pass that materializes all spill pseudo-register operands into real
// registers by inserting loads before uses and stores after defs. After this
// pass, no instruction has a spill register operand — expansion functions
// only see real registers (AD, IX, IY etc.).
//
// ACC spills (SPILL_D/A/B) are materialized via AD/AA/AB.
// INDEX spills (SPILL_X) are materialized via IY.
//
// SPILL_A/B are sub-registers of SPILL_D and share the same 2-byte frame
// slot. Big-endian: A (high byte) at offset 0, B (low byte) at offset 1.
//
// D save/restore: only needed when D is live for a reason OTHER than being
// the container for spill register values. We use the same willClobberD
// logic as SpillDSaveRestore: if the instruction has spill operands that
// will cause D to be loaded/stored, AND D is live, save/restore it.
// COPYs between D and spills are exempt (D is the intended operand).
//
//===----------------------------------------------------------------------===//

#include "MC6809MaterializeSpills.h"

#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-materialize-spills"

using namespace llvm;

static bool isAccSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3:
  case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3:
  case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
  case MC6809::SPILL_D0: case MC6809::SPILL_D1: case MC6809::SPILL_D2: case MC6809::SPILL_D3:
  case MC6809::SPILL_D4: case MC6809::SPILL_D5: case MC6809::SPILL_D6: case MC6809::SPILL_D7:
    return true;
  default:
    return false;
  }
}

static bool isIndexSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_X0: case MC6809::SPILL_X1:
  case MC6809::SPILL_X2: case MC6809::SPILL_X3:
    return true;
  default:
    return false;
  }
}

static bool isAnySpillReg(Register Reg) {
  return isAccSpillReg(Reg) || isIndexSpillReg(Reg);
}

/// Bug #63 support: opcodes that need second-ACC-spill skipping so
/// their expansion's U-relative spill path can fire.
///
/// What this is for
/// ----------------
/// MaterializeSpills' default behavior for an ACC spill operand is
/// "load the spill slot into D and rewrite the operand to $ad". That
/// works fine for instructions with ONE spill operand. For an
/// instruction with TWO distinct ACC spills (e.g.
/// `AddSetCarry_i16_Reg $spill_d4, $aalsb, $spill_d4(tied), $spill_d1`)
/// the default would load both spills through D — and the second LDD
/// clobbers the first. The expansion would then see `(D, D)` and
/// compute `D op D` instead of `LHS op RHS`. That's bug #63 (the same
/// shape as #60, but for the ACC class instead of the INDEX class).
///
/// How we work around it
/// ---------------------
/// The expansion paths for these opcodes already have a U-relative
/// spill path that reads the spill slot directly without going through
/// D — `emit6809Reg{Byte,Pair}FromMem` for Add/Sub/Bitwise, the
/// page-1 CMP-from-spill fallback inside `expandCompareReg` for
/// Compare/CompareBranch. That path was previously dead code because
/// MaterializeSpills always rewrote spill operands BEFORE expansion
/// ran. We re-enable it by teaching MaterializeSpills to LEAVE THE
/// SECOND DISTINCT ACC SPILL alone for these specific opcodes — the
/// operand stays as a spill register, the expansion sees it as a
/// spill, and the U-relative path takes over naturally.
///
/// Tied operands stay together
/// ---------------------------
/// For `AddSetCarry_i16_Reg dst, carry, src, src2`, op0 (dst) and op2
/// (src) are tied — same physical register. The "second distinct ACC
/// spill" rule kicks in only when src2 (op3 for non-Use, op4 for Use)
/// references a DIFFERENT spill register from dst/src. The tied
/// dst+src pair both still materialize through D, together.
///
/// Why this list and not "all _Reg pseudos with two ACC operands"?
/// --------------------------------------------------------------
/// Only opcodes whose expansion path includes a U-relative spill
/// fallback can benefit. Instructions whose expansion has no such
/// fallback would need a separate fix. The list grew in round 18 to
/// cover Compare_*_Reg, CompareBranch_*_Reg, and the bitwise reg-reg
/// pseudos — all routed through the same MaterializeSpills skip plus
/// a matching expansion-time U-relative path.
static bool needsSecondAccSpillSkip(unsigned Opcode) {
  switch (Opcode) {
  case MC6809::Add_i8_Reg:
  case MC6809::Add_i16_Reg:
  case MC6809::Sub_i8_Reg:
  case MC6809::Sub_i16_Reg:
  case MC6809::AddSetCarry_i8_Reg:
  case MC6809::AddSetCarry_i16_Reg:
  case MC6809::SubSetCarry_i8_Reg:
  case MC6809::SubSetCarry_i16_Reg:
  case MC6809::AddSetCarryUse_i8_Reg:
  case MC6809::AddSetCarryUse_i16_Reg:
  case MC6809::SubSetCarryUse_i8_Reg:
  case MC6809::SubSetCarryUse_i16_Reg:
  // Bug #350: the SetOverflow / SetOverflowUse reg variants share the
  // AddSetCarry/SubSetCarry expansion (bug #147), so they have the same
  // U-relative spill fallback — but they were missing from this skip
  // list. Without the skip, MaterializeSpills loaded both distinct ACC
  // spill operands of the carry-in high byte into the same accumulator
  // (the second LDB clobbering the first), so e.g. the i16 add s = a + b
  // expanded to "b_hi + b_hi" instead of "a_hi + b_hi". Manifested as a
  // complex-divide miscompile (__divsc3/__divdc3) once scalbn fed a
  // negated value into an i16 saddo under spill pressure.
  case MC6809::AddSetOverflow_i8_Reg:
  case MC6809::AddSetOverflow_i16_Reg:
  case MC6809::SubSetOverflow_i8_Reg:
  case MC6809::SubSetOverflow_i16_Reg:
  case MC6809::AddSetOverflowUse_i8_Reg:
  case MC6809::AddSetOverflowUse_i16_Reg:
  case MC6809::SubSetOverflowUse_i8_Reg:
  case MC6809::SubSetOverflowUse_i16_Reg:
  // Bug #161 round 18: Compare_*_Reg has the same "two ACC spills
  // both want the same physical register" problem as the arithmetic
  // family. Without skip-second-spill, MaterializeSpills emits two
  // back-to-back LDDs (or LDB+LDB) into AD/AB, the second clobbering
  // the first, and the resulting CMPR/CMPRp degenerates to "X,X".
  // Routing the second operand through the U-relative spill slot via
  // expandCompareReg's collision fallback (which emits a direct
  // CMPB/CMPD n,U) avoids the clobber.
  case MC6809::Compare_i8_Reg:
  case MC6809::Compare_i16_Reg:
  case MC6809::Compare_ptr_Reg:
  // Fused compare-and-branch pseudos with two register operands —
  // get split back into Compare_*_Reg + LBlbc by expandPostRAPseudo,
  // but MaterializeSpills runs BEFORE that split, so the skip needs
  // to apply at the fused level too.
  case MC6809::CompareBranch_i8_Reg:
  case MC6809::CompareBranch_i16_Reg:
  // Bitwise reg-reg ops have the same shape as Add/Sub (in-place
  // dst==src1 plus a second source). Same skip logic, same payoff.
  case MC6809::AND_i8_Reg:
  case MC6809::AND_i16_Reg:
  case MC6809::OR_i8_Reg:
  case MC6809::OR_i16_Reg:
  case MC6809::XOR_i8_Reg:
  case MC6809::XOR_i16_Reg:
  // Bug #311 (2026-05-22): Build32_i16i16 takes two ADc operands
  // ($lo, $hi) and assembles them into AQ.  LDD <slot>,$su to load
  // a SPILL_D writes $ad — clobbering the OTHER operand if it was
  // also in $ad (or resolved to $ad from another spill).  Skip ACC
  // spills so they survive as SPILL_D into expandPostRAPseudo,
  // which loads them via dedicated sequences that don't lose data.
  //
  // PhysCollision detection at lines 1059-1070 handles the case
  // where $lo=$ad (non-spill) and $hi=$spill_d0: spill's RealReg
  // collides with $lo's physreg → skip first spill.  The 2nd-
  // distinct-spill skip handles the both-spilled case.
  case MC6809::Build32_i16i16:
    return true;
  default:
    return false;
  }
}

/// The INDEX analog of needsSecondAccSpillSkip. Reg-reg index compares whose
/// expansion (expandCompareReg) has a register-vs-spill fallback — PSHS one
/// operand + CMPx ,s++, or CMPx off,$su against the slot. For these, a second
/// distinct SPILL_X operand (or a SPILL_X that would materialise into IY while a
/// live IY operand of the same compare occupies it) must be LEFT as a spill
/// register so the expansion reads it from its slot. Otherwise both operands
/// stage through IY (getRealReg → IY for every SPILL_X) and the second load
/// clobbers the first, collapsing the compare to "$iy,$iy" (a value compared to
/// itself). The index-IMMEDIATE compares (Compare_ptr_Imm / CompareBranch_ptr_Imm)
/// are deliberately excluded: their expansion has no U-relative fallback
/// (Bug #359), so their single SPILL_X must materialise into IY.
static bool needsSecondIndexSpillSkip(unsigned Opcode) {
  switch (Opcode) {
  case MC6809::Compare_ptr_Reg:
  case MC6809::CompareBranch_ptr_Reg:
    return true;
  default:
    return false;
  }
}

/// Get the real register to use for materialization.
static Register getRealReg(Register SpillReg) {
  if (isIndexSpillReg(SpillReg))
    return MC6809::IY;
  switch (SpillReg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_A1: case MC6809::SPILL_A2: case MC6809::SPILL_A3:
  case MC6809::SPILL_A4: case MC6809::SPILL_A5: case MC6809::SPILL_A6: case MC6809::SPILL_A7:
    return MC6809::AA;
  case MC6809::SPILL_B0: case MC6809::SPILL_B1: case MC6809::SPILL_B2: case MC6809::SPILL_B3:
  case MC6809::SPILL_B4: case MC6809::SPILL_B5: case MC6809::SPILL_B6: case MC6809::SPILL_B7:
    return MC6809::AB;
  default:
    return MC6809::AD;
  }
}

/// Get the parent SPILL_D register for an 8-bit SPILL_A/B.
static Register getParentDSpill(Register SpillReg) {
  switch (SpillReg) {
  case MC6809::SPILL_A0: case MC6809::SPILL_B0: return MC6809::SPILL_D0;
  case MC6809::SPILL_A1: case MC6809::SPILL_B1: return MC6809::SPILL_D1;
  case MC6809::SPILL_A2: case MC6809::SPILL_B2: return MC6809::SPILL_D2;
  case MC6809::SPILL_A3: case MC6809::SPILL_B3: return MC6809::SPILL_D3;
  case MC6809::SPILL_A4: case MC6809::SPILL_B4: return MC6809::SPILL_D4;
  case MC6809::SPILL_A5: case MC6809::SPILL_B5: return MC6809::SPILL_D5;
  case MC6809::SPILL_A6: case MC6809::SPILL_B6: return MC6809::SPILL_D6;
  case MC6809::SPILL_A7: case MC6809::SPILL_B7: return MC6809::SPILL_D7;
  default: return SpillReg; // Already a SPILL_D or SPILL_X
  }
}

/// Get the frame index and byte offset for a spill register.
/// Big-endian: A=high byte (offset 0), B=low byte (offset 1).
static std::pair<int, int> getSpillSlot(Register SpillReg,
                                         MC6809FunctionInfo *FuncInfo) {
  if (isIndexSpillReg(SpillReg))
    return {FuncInfo->SpillRegFrameIndices[SpillReg], 0};

  Register ParentD = getParentDSpill(SpillReg);
  int FI = FuncInfo->SpillRegFrameIndices[ParentD];
  Register RealReg = getRealReg(SpillReg);
  int ByteOffset = (RealReg == MC6809::AB) ? 1 : 0;
  return {FI, ByteOffset};
}

/// Return true if SpillReg is a 16-bit ACC spill (SPILL_D0..D7), whose
/// materialization uses LDD and therefore clobbers both halves of D.
/// 8-bit ACC spills (SPILL_A0..7, SPILL_B0..7) use LDA/LDB for one half
/// and leave the other untouched.
static bool isWideAccSpillReg(Register Reg) {
  switch (Reg) {
  case MC6809::SPILL_D0: case MC6809::SPILL_D1:
  case MC6809::SPILL_D2: case MC6809::SPILL_D3:
  case MC6809::SPILL_D4: case MC6809::SPILL_D5:
  case MC6809::SPILL_D6: case MC6809::SPILL_D7:
    return true;
  default:
    return false;
  }
}

/// Bug #301 (2026-05-22): return true if Reg is a 32-bit ACC spill
/// (SPILL_Q0..Q31), whose materialization uses LDQ and therefore
/// clobbers the entire $aq register (= $ad + $aw + their sub-bytes).
/// Mirrors isWideAccSpillReg but for the wider $aq class.  Half-word
/// (SPILL_Q*HI/LO) and byte (SPILL_Q*HIHI..LOLO) sub-reg forms are NOT
/// included — those materialize via narrower loads (LDD/LDA/LDB) and
/// are handled by the existing NeedSaveD/A/B paths.
static bool isQSpillReg(Register Reg) {
  return Reg >= MC6809::SPILL_Q0 && Reg <= MC6809::SPILL_Q31;
}

/// Check if this instruction's spill operands will cause D to be clobbered
/// during materialization. Mirrors SpillDSaveRestore's willClobberD logic.
static bool willClobberD(const MachineInstr &MI,
                         const TargetRegisterInfo &TRI) {
  // Bug #118 Layer 1 (approach b): EXTRACT_{LO,HI}_i16 with a SPILL_D* src
  // is byte-folded in the forward pass — only the staging byte (AB for LO,
  // AA for HI) is touched, never LDD. Wrapping it with STD/LDD would UNDO
  // the extract (the restore clobbers the byte we just loaded). Let the
  // narrower NeedSaveA/NeedSaveB logic preserve the OTHER half if needed.
  if (MI.getOpcode() == MC6809::EXTRACT_LO_i16 ||
      MI.getOpcode() == MC6809::EXTRACT_HI_i16)
    return false;

  bool HasWideAccSpill = false;

  // Multi-INDEX-spill case: when 2 distinct SPILL_X operands appear, the
  // materializer routes one through D (because IY can only stage one).
  // That clobbers D, so we need the save/restore.
  llvm::SmallSet<Register, 4> UniqueIndexSpills;
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isIndexSpillReg(MO.getReg()))
      UniqueIndexSpills.insert(MO.getReg());
  }
  if (UniqueIndexSpills.size() >= 2)
    return true;

  // Only 16-bit ACC spills (SPILL_D*) clobber D — their materialization
  // uses LDD which loads both A and B. 8-bit ACC spills (SPILL_A*/SPILL_B*)
  // materialize via LDA/LDB which only touches one half of D, leaving the
  // other untouched. Bug #88: previously this was just `isAccSpillReg`,
  // which over-fired on byte-level pseudos like SubSetCarry_i8_Reg whose
  // 8-bit spill operand goes through path-(a) U-relative byte addressing
  // (no LDD at all) and whose tied DEF is one half of D — the over-eager
  // D-save then restored the OLD D after the instruction, clobbering the
  // freshly-computed result byte.
  //
  // Bug #94 analysis: the 16-bit save/restore IS correct because both the
  // save/restore (STD/LDD) and the expansion (LDD+op+STD) operate on the
  // full $ad — no partial-clobber mismatch. The dematerialize store-back
  // (in the SpillOps DEF loop below) runs BEFORE the D-restore (both use
  // the same After insertion point, but SpillOps fires first in the code
  // flow), so the result is safely in the spill slot before $ad is restored
  // to its pre-instruction value. The regalloc never reads $ad for the
  // spill-backed vreg; it rematerializes from the slot. Closed as not-a-bug.
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isWideAccSpillReg(MO.getReg()))
      HasWideAccSpill = true;
  }

  if (!HasWideAccSpill)
    return false;

  // COPYs: check for safe cases where D is the intended operand.
  // NOTE: Unlike SpillDSaveRestore (which leaves spill operands for later
  // expansion via STX/LDX), we rewrite spill operands to D — so INDEX↔SPILL
  // copies DO clobber D and are NOT exempt.
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    // D↔SPILL: D is the intended operand — materialization doesn't add
    // an extra D load/store, just a store-back to the spill slot.
    // The COPY becomes $ad = $ad (nop) + store, so D isn't clobbered.
    if (!isAccSpillReg(DstReg) && TRI.regsOverlap(DstReg, MC6809::AD))
      return false;
    if (!isAccSpillReg(SrcReg) && TRI.regsOverlap(SrcReg, MC6809::AD))
      return false;
  }

  return true;
}

/// Bug #301 (2026-05-22): check if this instruction's spill operands
/// will cause $aq to be clobbered during materialization.
///
/// SPILL_Q* operands materialize via LDQ to $aq, clobbering the entire
/// 32-bit accumulator (and all its sub-regs $ad, $aw, $aa, $ab, $ae,
/// $af).  When $aq has a live value at this MI's pre-state, the LDQ
/// destroys it — Bug #301's root cause for the gmtime/timegm/ftello
/// miscompiles.
///
/// COPYs to/from physical $aq don't add an extra LDQ (the COPY itself
/// IS the materialization), so they're exempt — same logic as
/// willClobberD's COPY carve-out.
static bool willClobberQ(const MachineInstr &MI,
                         const TargetRegisterInfo &TRI) {
  // FAKE_USE generates no code — it never materialises its $spill_q*
  // operand into the physical $aq, so it clobbers nothing. (Bug #344:
  // without this, the new sub-reg save path below wraps a FAKE_USE and
  // re-breaks the Bug #256 garbage-liveness test.)
  if (MI.getOpcode() == TargetOpcode::FAKE_USE)
    return false;

  bool HasQSpill = false;
  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isQSpillReg(MO.getReg()))
      HasQSpill = true;
  }
  if (!HasQSpill)
    return false;

  // COPYs: $aq↔SPILL_Q* — $aq IS the intended operand, the COPY's
  // materialization doesn't add an extra LDQ/STQ pair that wasn't
  // already needed by the COPY itself.
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (!isQSpillReg(DstReg) && TRI.regsOverlap(DstReg, MC6809::AQ))
      return false;
    if (!isQSpillReg(SrcReg) && TRI.regsOverlap(SrcReg, MC6809::AQ))
      return false;
  }

  return true;
}

/// Check if this instruction's INDEX spill operands will clobber IY.
static bool willClobberIY(const MachineInstr &MI,
                          const TargetRegisterInfo &TRI) {
  bool HasIndexSpill = false;
  for (const MachineOperand &MO : MI.operands()) {
    if (MO.isReg() && MO.getReg().isPhysical() && isIndexSpillReg(MO.getReg()))
      HasIndexSpill = true;
  }
  if (!HasIndexSpill)
    return false;

  // COPY IY↔SPILL_X: IY is the intended operand.
  if (MI.isCopy()) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    if (!isIndexSpillReg(DstReg) && TRI.regsOverlap(DstReg, MC6809::IY))
      return false;
    if (!isIndexSpillReg(SrcReg) && TRI.regsOverlap(SrcReg, MC6809::IY))
      return false;
  }

  return true;
}

/// Bug #127b: Check if this instruction will trigger the conflict-resolution
/// path that routes a spill operand through IX as the "alt" register (the
/// block near the `RealReg == MC6809::IY || RealReg == MC6809::IX` case
/// below). That path does `LDX SaveSlot` which clobbers IX. If IX is
/// externally live (e.g., holds a function argument) we must preserve it.
///
/// Triggered when: a spill operand with RealReg=IY conflicts with a
/// non-spill operand in the same MI that directly uses $iy. In that
/// situation the fixer saves IY, loads the saved value into IX, and
/// redirects the non-spill operand to IX.
static bool willClobberIX(const MachineInstr &MI,
                          const TargetRegisterInfo &TRI) {
  for (unsigned I = 0; I < MI.getNumOperands(); ++I) {
    const MachineOperand &SpillMO = MI.getOperand(I);
    if (!SpillMO.isReg() || !SpillMO.getReg().isPhysical()) continue;
    if (!isIndexSpillReg(SpillMO.getReg())) continue;
    if (!SpillMO.isUse() && !(SpillMO.isDef() && SpillMO.isTied())) continue;
    // This spill materializes through IY. A non-spill use of IY in the
    // same MI will force the conflict-resolution path → AltReg = IX.
    for (unsigned J = 0; J < MI.getNumOperands(); ++J) {
      if (J == I) continue;
      const MachineOperand &OtherMO = MI.getOperand(J);
      if (!OtherMO.isReg() || !OtherMO.isUse()) continue;
      if (!OtherMO.getReg().isPhysical()) continue;
      if (isAnySpillReg(OtherMO.getReg())) continue;
      if (OtherMO.getReg() != MC6809::IY) continue;
      // Tied conflicts take the SkipSpillLoad path (no IX load); only
      // untied conflicts emit `LDX SaveSlot`.
      if (SpillMO.isTied()) continue;
      return true;
    }
  }
  return false;
}

namespace {

class MC6809MaterializeSpills : public MachineFunctionPass {
public:
  static char ID;

  MC6809MaterializeSpills() : MachineFunctionPass(ID) {
    initializeMC6809MaterializeSpillsPass(*PassRegistry::getPassRegistry());
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

  StringRef getPassName() const override {
    return "MC6809 Materialize Spill Registers";
  }
};

bool MC6809MaterializeSpills::runOnMachineFunction(MachineFunction &MF) {
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  const auto &TII = *MF.getSubtarget().getInstrInfo();
  auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>();
  bool Changed = false;

  // Allocate frame slots for spill registers if not already done.
  // This normally happens in processFunctionBeforeFrameFinalized (during PEI),
  // but we run before PEI. Allocate them now so we can reference them.
  {
    MachineFrameInfo &MFI = MF.getFrameInfo();
    // Check if a SPILL_D register or any of its sub-registers (SPILL_A/B)
    // are used in any instruction.
    auto isSpillDUsedInFunction = [&](MCPhysReg SpillDReg) -> bool {
      for (const MachineBasicBlock &MBB : MF)
        for (const MachineInstr &MI : MBB)
          for (const MachineOperand &MO : MI.operands())
            if (MO.isReg() && !MO.isImplicit() && MO.getReg().isPhysical() &&
                isAccSpillReg(MO.getReg()) &&
                getParentDSpill(MO.getReg()) == SpillDReg)
              return true;
      return false;
    };
    auto isSpillXUsedInFunction = [&](MCPhysReg SpillReg) -> bool {
      for (const MachineBasicBlock &MBB : MF)
        for (const MachineInstr &MI : MBB)
          for (const MachineOperand &MO : MI.operands())
            if (MO.isReg() && !MO.isImplicit() && MO.getReg() == SpillReg)
              return true;
      return false;
    };
    static const MCPhysReg SpillDRegs[] = {
      MC6809::SPILL_D0, MC6809::SPILL_D1, MC6809::SPILL_D2, MC6809::SPILL_D3,
      MC6809::SPILL_D4, MC6809::SPILL_D5, MC6809::SPILL_D6, MC6809::SPILL_D7
    };
    bool AnySpillUsed = false;
    for (MCPhysReg Reg : SpillDRegs) {
      if (FuncInfo->SpillRegFrameIndices.count(Reg) == 0 &&
          isSpillDUsedInFunction(Reg)) {
        int FI = MFI.CreateStackObject(2, Align(1), false);
        FuncInfo->SpillRegFrameIndices[Reg] = FI;
        AnySpillUsed = true;
      }
    }
    static const MCPhysReg SpillXRegs[] = {
      MC6809::SPILL_X0, MC6809::SPILL_X1, MC6809::SPILL_X2, MC6809::SPILL_X3
    };
    for (MCPhysReg Reg : SpillXRegs) {
      if (FuncInfo->SpillRegFrameIndices.count(Reg) == 0 &&
          isSpillXUsedInFunction(Reg)) {
        int FI = MFI.CreateStackObject(2, Align(1), false);
        FuncInfo->SpillRegFrameIndices[Reg] = FI;
        AnySpillUsed = true;
      }
    }
    // Bug #316 (2026-05-21): also flag SPILL_Q*N usage.  The previous
    // version only checked SpillDRegs (i16-class) and SpillXRegs
    // (i16-pointer) and missed SPILL_Q[0..31] (i32-class spills).
    //
    // Without this check, a function that uses ONLY SPILL_Q (e.g.
    // picolibc's srandom, which does `_rand_next = seed` and needs an
    // i64-from-i16 zero-extension stash) leaves UsesSpillRegisters
    // false at determineCalleeSaves time.  hasFP then returns false
    // (no calls + no other trigger), so SavedRegs.set(SU) is NOT
    // called → PEI doesn't emit `pshs u`.  But emitPrologue later
    // sees hasFP=true (something else flips it post-PEI) and emits
    // `tfr s,u`, OVERWRITING caller's U without having saved it.
    //
    // The visible failure was Bug #315 (memcpy-1 SIGABRT at
    // -O2-hd6309-mame): main calls srand → srand calls srandom →
    // srandom corrupts U on return → cascades into main's U=0 →
    // all U-relative locals access garbage → memcpy ptr-check
    // fails → abort.
    //
    // SPILL_Q*N are 4 bytes each; the actual concrete frame slots
    // are created by the post-RA expansion of the spill itself.
    // Here we just need to flag SPILL_Q usage so UsesSpillRegisters
    // becomes true at PEI time.
    auto isSpillQUsedInFunction = [&]() -> bool {
      for (const MachineBasicBlock &MBB : MF)
        for (const MachineInstr &MI : MBB)
          for (const MachineOperand &MO : MI.operands())
            if (MO.isReg() && !MO.isImplicit() && MO.getReg().isPhysical()) {
              MCPhysReg R = MO.getReg();
              // 32-bit SPILL_Q full forms.
              if (R >= MC6809::SPILL_Q0 && R <= MC6809::SPILL_Q31)
                return true;
              // 16-bit SPILL_Q half-word sub-regs (high then low).
              if (R >= MC6809::SPILL_Q0HI && R <= MC6809::SPILL_Q31HI)
                return true;
              if (R >= MC6809::SPILL_Q0LO && R <= MC6809::SPILL_Q31LO)
                return true;
              // 8-bit SPILL_Q byte sub-regs (HI_HI .. LO_LO, 4 per slot).
              if (R >= MC6809::SPILL_Q0HIHI && R <= MC6809::SPILL_Q31LOLO)
                return true;
            }
      return false;
    };
    if (isSpillQUsedInFunction())
      AnySpillUsed = true;
    if (AnySpillUsed)
      FuncInfo->UsesSpillRegisters = true;
  }

  // Reusable save slots for preserving a live real register across a spill
  // materialisation. Each save is strictly bracketed (store before MI, reload
  // after MI, no nesting), so at most one save of each register-type is live at
  // any point — a single slot per type can be shared across the whole function.
  // Creating a fresh slot per materialisation (the old behaviour) bloated the
  // frame: these slots are minted after StackSlotColoring runs, so the coloring
  // pass never merges them (dozens of 2-byte slots for a hot function). This
  // matters especially for static-stack functions, where the frame lives in the
  // 64 KB global address space rather than on the runtime stack.
  // Only the D and Q save slots are shared. Their restores are always emitted
  // immediately after the (non-terminator) MI, so each save/restore is strictly
  // bracketed and no two are simultaneously live — one slot per type serves the
  // whole function. The A/B/IX/IY saves are NOT shared: their restores can be
  // deferred into successor blocks (a compare feeding a conditional branch, or a
  // terminator — see DeferByteRestore / insertPtr/ByteRestoreInSuccessors
  // below), so the save's live range spans BB boundaries and could overlap
  // another use of the same slot on a different path; those keep a fresh slot.
  int ReuseQSaveSlot = -1;
  int ReuseDSaveSlot = -1;
  auto getSaveSlot = [&](int &Cache, unsigned Size, bool CanReuse) -> int {
    if (!CanReuse)
      return MF.getFrameInfo().CreateStackObject(Size, Align(1), true);
    if (Cache == -1)
      Cache = MF.getFrameInfo().CreateStackObject(Size, Align(1), true);
    return Cache;
  };

  for (MachineBasicBlock &MBB : MF) {
    // Add spill pseudo-registers to liveins so the machine verifier doesn't
    // flag them as used-without-definition (bug #16). The value is defined
    // by a store in a predecessor block; the livein tells the verifier.
    for (const MachineInstr &MI : MBB)
      for (const MachineOperand &MO : MI.operands())
        if (MO.isReg() && MO.isUse() && MO.getReg().isPhysical() &&
            isAnySpillReg(MO.getReg()) && !MBB.isLiveIn(MO.getReg()))
          MBB.addLiveIn(MO.getReg());

    LivePhysRegs LPR(TRI);
    LPR.addLiveOuts(MBB);
    SmallVector<MachineInstr *, 4> ToErase;

    // Per-instruction: do we need D save/restore? IY save/restore?
    // NeedSaveA/NeedSaveB are narrower than NeedSaveD — they preserve only
    // one half of $ad around a pseudo whose expansion uses that half as a
    // scratch (bug #89). NeedSaveD saves both halves via STD/LDD.
    struct SpillInfo {
      bool NeedSaveD;
      bool NeedSaveIY;
      bool NeedSaveIX;
      bool NeedSaveA;
      bool NeedSaveB;
      // Bug #301 (2026-05-22): wider than NeedSaveD — saves the full
      // $aq via STQ/LDQ when SPILL_Q* materialization clobbers $aq.
      // NeedSaveD only preserves the AD sub-word (2 bytes); for SPILL_Q
      // we need all 4 bytes preserved (AD + AW).
      bool NeedSaveQ;
    };
    DenseMap<MachineInstr *, SpillInfo> NeedSave;

    for (MachineInstr &MI : llvm::reverse(MBB)) {
      LPR.stepBackward(MI);

      // Bug #91: BranchJumpTable requires its index operand (operand 0) in
      // $ad at the time the asm printer expands it (ASLB+ROLA+LEAX+LDD+JMP).
      // If regalloc placed the index in an imaginary register (RS0..RS3),
      // the isAnySpillReg gate below won't catch it and we'll silently skip
      // the instruction. Handle this before the HasSpill check: materialize
      // the index into $ad and rewrite the operand in place.
      if (MI.getOpcode() == MC6809::BranchJumpTable) {
        Register IdxReg = MI.getOperand(0).getReg();
        if (IdxReg != MC6809::AD) {
          DebugLoc DL = MI.getDebugLoc();
          if (isAccSpillReg(IdxReg)) {
            auto [FI, ByteOffset] = getSpillSlot(IdxReg, FuncInfo);
            BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Mem))
                .addReg(MC6809::AD, RegState::Define)
                .addFrameIndex(FI)
                .addImm(ByteOffset);
          } else if (IdxReg == MC6809::RS0 || IdxReg == MC6809::RS1 ||
                     IdxReg == MC6809::RS2 || IdxReg == MC6809::RS3) {
            BuildMI(MBB, MI, DL, TII.get(MC6809::LDDd))
                .addReg(IdxReg);
          } else {
            llvm_unreachable("BranchJumpTable: unexpected index register");
          }
          MI.getOperand(0).setReg(MC6809::AD);
          Changed = true;
        }
        // Don't fall through to the spill-operand loop; we've handled
        // BranchJumpTable completely. Mark any remaining spill operands as
        // processed (there shouldn't be any — BranchJumpTable has only
        // the index + a JTI operand).
        continue;
      }

      bool HasSpill = false;
      bool HasQSpillOnly = false;  // Bug #301
      for (const MachineOperand &MO : MI.operands()) {
        if (!MO.isReg() || !MO.getReg().isPhysical()) continue;
        if (isAnySpillReg(MO.getReg())) {
          HasSpill = true;
          break;
        }
        if (isQSpillReg(MO.getReg())) {
          HasQSpillOnly = true;
          // Don't break — keep scanning in case another operand is a
          // regular spill (more general HasSpill takes precedence).
        }
      }
      // Bug #301 (2026-05-22): MIs with ONLY SPILL_Q* operands (no
      // SPILL_A/B/D/X) aren't materialized by MaterializeSpills (the
      // post-isel expansion handles SPILL_Q* via LDQ/STQ to the
      // dedicated stack slot).  But the LDQ DOES clobber $aq — so we
      // still need the backward-pass NeedSaveQ analysis to fire for
      // these MIs.  Fall through to the rest of the loop for the save/
      // restore brackets; the materialization code (SpillOps loop) is
      // a no-op for SPILL_Q* operands since they don't match
      // isAccSpillReg.
      if (!HasSpill && !HasQSpillOnly)
        continue;

      // Handle COPY instructions specially to avoid going through D when
      // possible. INDEX↔ACC_SPILL copies use STX/LDX directly, preserving
      // the STX pattern needed by compare expansion backwards scan.
      if (MI.isCopy()) {
        Register DstReg = MI.getOperand(0).getReg();
        Register SrcReg = MI.getOperand(1).getReg();
        bool DstIsAccSpill = isAccSpillReg(DstReg);
        bool SrcIsAccSpill = isAccSpillReg(SrcReg);
        bool SrcIsIndex = (SrcReg == MC6809::IX || SrcReg == MC6809::IY);
        bool DstIsIndex = (DstReg == MC6809::IX || DstReg == MC6809::IY);

        if (DstIsAccSpill && SrcIsIndex) {
          // SPILL_D = COPY IX/IY → STX/STY spill_slot (no D clobber).
          auto [FI, ByteOffset] = getSpillSlot(DstReg, FuncInfo);
          BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(MC6809::Store_iPtr_Mem))
              .addReg(SrcReg)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
          ToErase.push_back(&MI);
          Changed = true;
          continue;
        }
        if (DstIsIndex && SrcIsAccSpill) {
          // IX/IY = COPY SPILL_D → LDX/LDY spill_slot (no D clobber).
          auto [FI, ByteOffset] = getSpillSlot(SrcReg, FuncInfo);
          BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(MC6809::Load_iPtr_Mem))
              .addReg(DstReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
          ToErase.push_back(&MI);
          Changed = true;
          continue;
        }
      }

      // Compare/test instructions with 16-bit spill operands (SPILL_D) can
      // sometimes be handled by the expansion's backwards scan (CMPX/CMPY),
      // which preserves D. BUT if D is live with a non-spill value, the
      // fallback materialization will clobber it. In that case, we MUST
      // process the instruction here so the D save/restore logic fires.
      //
      // BranchJumpTable is also isTerminator() but its expansion has NO
      // backwards-scan equivalent — it hardcodes ASLB+ROLA on D and just
      // assumes D already holds the index. So we must NEVER take the skip
      // path for it; the spill operand has to be materialized into D
      // before the asm printer's expansion runs. This was bug #68.
      if ((MI.isCompare() || MI.isTerminator()) &&
          MI.getOpcode() != MC6809::BranchJumpTable) {
        bool Has8BitSpill = false;
        for (const MachineOperand &MO : MI.operands()) {
          if (MO.isReg() && MO.getReg().isPhysical() && isAccSpillReg(MO.getReg())) {
            Register RealReg = getRealReg(MO.getReg());
            if (RealReg == MC6809::AA || RealReg == MC6809::AB)
              Has8BitSpill = true;
          }
        }
        // Bug #359: an INDEX spill operand (SPILL_X*) must NOT take the
        // D-only skip path below.  That path leaves the spill in place for
        // the expansion, which is sound for SPILL_D operands (the Bug #49
        // backward-scan rewrites the compare to CMPX/CMPY reading the STX'd
        // slot, preserving D).  But expandCompareImm has no U-relative
        // fallback for an index-immediate compare — it materialises the
        // SPILL_X into IX (getRealRegForSpill), silently clobbering whatever
        // lives in IX.  In __ascii_mbtowc the incoming pwc pointer lives in
        // IX across the s==NULL check; loading s into IX for `cmpx #0`
        // destroyed pwc, so the later `*pwc = c` store wrote through s and
        // the converted character was lost.  Falling through to the main
        // materialisation loop rewrites the operand to IY (leaving IX
        // untouched), with willClobberIX/IY guarding any genuinely-live
        // IX/IY.
        bool HasIndexSpill = false;
        for (const MachineOperand &MO : MI.operands())
          if (MO.isReg() && MO.getReg().isPhysical() &&
              isIndexSpillReg(MO.getReg()))
            HasIndexSpill = true;
        if (!Has8BitSpill && !HasIndexSpill) {
          // Check if D (or any sub-reg) is live with a value that would
          // be clobbered.  LivePhysRegs::contains(R) only matches if R
          // itself (or a reg that addReg(super) walked into) was
          // inserted, so an inserted sub-reg won't be found via the
          // super.  Walk sub-regs of AD explicitly to match the
          // NeedSaveD path's anySubRegLive.
          bool DLive = LPR.contains(MC6809::AD);
          if (!DLive)
            for (MCPhysReg Sub : TRI.subregs(MC6809::AD))
              if (LPR.contains(Sub)) { DLive = true; break; }
          if (!DLive)
            continue; // Safe to skip — D isn't live, expansion won't lose anything.
        }
      }

      bool NeedD = false, NeedIY = false, NeedIX = false;
      bool NeedA = false, NeedB = false;
      bool NeedQ = false;

      // LivePhysRegs::contains(R) returns true if R or any sub-reg was added
      // to the live set, but NOT if only a super-reg was added (addReg adds
      // sub-regs only). When a byte-level value (e.g. AALSB) is the only
      // thing live, we still need to treat its parent byte (AA) as live —
      // clobbering AA destroys AALSB. Walk sub-regs explicitly.
      auto anySubRegLive = [&](MCPhysReg Reg) {
        if (LPR.contains(Reg))
          return true;
        for (MCPhysReg Sub : TRI.subregs(Reg))
          if (LPR.contains(Sub))
            return true;
        return false;
      };

      // Bug #301 (2026-05-22): SPILL_Q* materialization clobbers
      // $aq via LDQ.  If $aq (or any of its 6 sub-regs $ad/$aw/$aa/
      // $ab/$ae/$af) is live at the pre-state of this MI, the LDQ
      // destroys the value — Bug #301's gmtime/timegm/ftello shape.
      // Detect and request NeedSaveQ; the forward pass emits STQ/LDQ
      // brackets around MI.
      //
      // NeedSaveQ subsumes NeedSaveD/A/B for this MI (the STQ/LDQ
      // covers all sub-regs).  Skip the NeedD/A/B paths when NeedQ
      // fires so we don't emit redundant save/restore pairs.
      if (willClobberQ(MI, TRI)) {
        // Discriminate against false-positive sub-reg liveness: the
        // canonical Bug #301 shape has \$aq loaded as a unit (e.g.
        // `\$aq = Load_i32_Mem ...`), so LPR.contains(AQ) is true at
        // the pre-state.  Sub-reg-only liveness (e.g. \$ab live for an
        // unrelated TestBranch consumer) shouldn't fire NeedQ because
        // either the MI's own implicit-defs already account for the
        // clobber (SpillLoad_i32_Mem), or the sub-reg's value is
        // Bug #256-style garbage from an upstream SpillLoad killed by
        // FAKE_USE.  Tightening to LPR.contains(AQ) avoids both.
        if (LPR.contains(MC6809::AQ)) {
          NeedQ = true;
        } else if (anySubRegLive(MC6809::AW) &&
                   !MI.modifiesRegister(MC6809::AQ, &TRI)) {
          // Bug #344 (2026-05-25): the full $aq isn't live, but a sub-reg
          // HALF holds a live value the Q-clobber (LDQ into $aq) would
          // destroy. The $ad half is preserved cheaply by the
          // willClobberD/NeedD path below (now enabled for Q-clobbers);
          // there is no narrow NeedSaveW path for the $aw half ($ae/$af),
          // so fall back to a full NeedSaveQ when $aw is live (also covers
          // the both-halves-live case). Manifested as strtol returning a
          // NEGATED positive value: the conditional negate
          // `if (flags&FLAG_NEG) val=-val` keeps `flags&1` in $ad while
          // Sub_i32_Reg(spilled) materialises -val into $aq, and the
          // TestBranch then reads the clobbered $ad.
          NeedQ = true;
        }
      }

      // Bug #242-style exemption: tied-def of $aq is the intended
      // output.  If $aq is a tied DEF on this MI, the new $aq value
      // IS the result the consumer wants — wrapping with STQ/LDQ
      // would restore the PRE-MI value and destroy the result.
      if (NeedQ) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isDef() || MO.isImplicit() || MO.isDead())
            continue;
          if (!MO.getReg().isPhysical())
            continue;
          if (!TRI.regsOverlap(MO.getReg(), MC6809::AQ))
            continue;
          if (!MO.isTied())
            continue;
          NeedQ = false;
          break;
        }
      }

      // Bug #344 (2026-05-25): a Q-clobbering MI (Sub_i32_Reg etc. on
      // spilled $spill_q* operands) materialises into the physical $aq,
      // which clobbers the $ad half too. When only the $ad half holds a
      // live value (full $aq not live, so NeedQ stayed false above), the
      // cheap byte-granular STD/LDD save below preserves it — so run the
      // D-save analysis for Q-clobbers as well as D-clobbers.
      //
      // The `!modifiesRegister(AQ)` gate is the discriminator vs. Bug
      // #256: a MI that DECLARES it defs the $aq sub-regs (e.g.
      // SpillLoad_i32_Mem's implicit-def $aa/$ab/$ad/$aw) has its clobber
      // already accounted for by regalloc, so any sub-reg "liveness" past
      // it is killed garbage — don't save. Sub_i32_Reg & friends do NOT
      // declare the AQ scratch clobber (the root of #344), so a live
      // sub-reg there is a genuine value that must be preserved.
      bool QScratchClobber =
          willClobberQ(MI, TRI) && !MI.modifiesRegister(MC6809::AQ, &TRI);
      if ((willClobberD(MI, TRI) || QScratchClobber) && !NeedQ) {
        // Bug #313 (2026-05-22): refine NeedD to byte-granular when
        // only one half of $ad is live.  Previously: any sub-reg of
        // $ad being live → NeedD (16-bit STD/LDD save+restore).  When
        // only $ab is live (with an 8-bit value) and $aa is dead, the
        // 16-bit save preserves garbage in $aa across the
        // save/restore — latent vulnerability documented in Bug #313
        // (i1 PHI → i16 store leaks $aa garbage to memory through the
        // preservation slot).
        //
        // Discriminate: if both $aa and $ab are in the live set, save
        // 16-bit (covers either two separate 8-bit values OR a 16-bit
        // value).  If only one half is live, save just that half
        // (NeedSaveA/NeedSaveB).  The byte-level save infrastructure
        // is already in place (Bug #89 for SPILL_A/B operands); this
        // change just routes NeedD's discovery into the same byte
        // path when appropriate.
        bool ALive = LPR.contains(MC6809::AA);
        bool BLive = LPR.contains(MC6809::AB);
        bool DLive = LPR.contains(MC6809::AD);
        if (DLive || (ALive && BLive)) {
          NeedD = true;
        } else if (ALive) {
          NeedA = true;
        } else if (BLive) {
          NeedB = true;
        }
      }

      // Bug #242: when MI has an explicit, non-dead, tied DEF of $ad
      // (e.g. AND_i16_Reg / OR_i16_Reg / XOR_i16_Reg / Add_i16_Reg /
      // Sub_i16_Reg / AddSetCarry*_i16_Reg with $ad as op0), the new
      // value of $ad IS the result we want propagated. Wrapping such
      // an MI with STD-before / LDD-after would unconditionally restore
      // the PRE-MI value of $ad, destroying the def. The "needs save"
      // intuition assumes a side-effect clobber; tied-defs are NOT
      // side-effect clobbers — they ARE the operation's intended output.
      //
      // Concrete bug repro at -Os HD6309 (LTO and non-LTO both):
      //   $ad = AND_i16_Reg killed $ad(tied-def 0), killed $spill_d0
      //   $ix = COPY $ad
      // MaterializeSpills inserted STD/LDD around the AND, leaving the
      // COPY reading the pre-AND value. Test-malloc-stress at
      // Os-lto-hd6309-mame returns LSB-set "unaligned" addresses for
      // every memalign call as a result.
      if (NeedD) {
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.isDef() || MO.isImplicit() || MO.isDead())
            continue;
          if (!MO.getReg().isPhysical())
            continue;
          if (!TRI.regsOverlap(MO.getReg(), MC6809::AD))
            continue;
          if (!MO.isTied())
            continue;
          NeedD = false;
          break;
        }
      }

      if (willClobberIY(MI, TRI)) {
        if (anySubRegLive(MC6809::IY))
          NeedIY = true;
      }

      // Bug #127b: same logic for IX, which the conflict-resolution path
      // uses as the alt register. Without this, an externally-live IX
      // (e.g., a function-arg pointer) is silently overwritten.
      if (willClobberIX(MI, TRI)) {
        if (anySubRegLive(MC6809::IX))
          NeedIX = true;
      }

      // Bug #89: an 8-bit ACC spill operand is materialized via LDA/LDB,
      // which uses $aa/$ab as a scratch register. If $aa/$ab is live with
      // a non-operand value, the expansion clobbers it — the register
      // allocator made its decisions without knowing the expansion would
      // touch $aa/$ab. Detect that case and preserve just the affected
      // half of $ad around the instruction.
      //
      // Two flavours to watch for:
      //
      //  (1) The spill operand's real reg X ($aa or $ab) is live with a
      //      non-operand value. Materialisation will overwrite X.
      //
      //  (2) The spill operand's real reg X matches a non-spill physical
      //      use of the SAME instruction (e.g., SubSetCarry_i8_Reg with
      //      op0 = $spill_b0 and op3 = $ab). Case 1 conflict resolution
      //      below saves the physical operand and redirects it to the
      //      ALT half — but that clobbers the alt half. If the alt half
      //      is live with a non-operand value, we must preserve it too.
      //
      // Wide D save/restore via NeedSaveD is wrong here: the restore
      // would overwrite the sub result that lives in $ab after the
      // expansion, causing bug #88.
      if (!NeedD) {
        bool SpillClobbersA = false, SpillClobbersB = false;
        bool Case1NeedsAltA = false, Case1NeedsAltB = false;
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.getReg().isPhysical()) continue;
          if (!isAccSpillReg(MO.getReg())) continue;
          Register RealReg = getRealReg(MO.getReg());
          if (RealReg == MC6809::AA) SpillClobbersA = true;
          if (RealReg == MC6809::AB) SpillClobbersB = true;

          // Detect Case 1: physical non-spill use on RealReg.
          for (const MachineOperand &RhsMO : MI.operands()) {
            if (!RhsMO.isReg() || !RhsMO.isUse()) continue;
            if (!RhsMO.getReg().isPhysical()) continue;
            if (isAnySpillReg(RhsMO.getReg())) continue;
            if (RhsMO.getReg() != RealReg) continue;
            // Case 1 will use the alt half as the redirect target.
            if (RealReg == MC6809::AB) Case1NeedsAltA = true;
            else Case1NeedsAltB = true;
            break;
          }
        }
        auto hasDirectOperand = [&](Register Half) {
          for (const MachineOperand &MO : MI.operands()) {
            if (!MO.isReg() || !MO.getReg().isPhysical()) continue;
            if (isAccSpillReg(MO.getReg())) continue;
            if (TRI.regsOverlap(MO.getReg(), Half))
              return true;
          }
          return false;
        };

        // Bug #256: when MI is a same-half SPILL→SPILL COPY (SPILL_B↔SPILL_B
        // or SPILL_A↔SPILL_A) and the "live" half-register value can be
        // proven garbage — coming from an upstream wide-acc-spill load
        // (`SpillLoad_i32_Mem`) whose super-register was subsequently killed
        // by a `FAKE_USE` (the `-fextend-lifetimes` pattern active at -Og)
        // — the pre-MI save/restore preserves nonsense. Worse, when this
        // wrap is nested inside an outer SaveD wrap (around an immediately-
        // preceding wide-acc-spill instruction in the same MBB), the inner
        // restore overwrites the outer materialisation's freshly-loaded
        // value, propagating garbage to the next consumer.
        //
        // Detect the exact pattern: scan backward in this MBB for the
        // closest def of the half-register. If that def is a
        // `SpillLoad_i32_Mem`-class implicit-def AND there is a
        // `FAKE_USE killed` of $aq (or a SPILL_Q*) between that def and
        // this MI, the live value at this MI's pre-state is regalloc-dead
        // garbage. Skip the save/restore in that case only — the COPY's
        // materialisation produces the value the consumer actually wants
        // (the loaded source-slot byte).
        auto isSameHalfSpillCopy = [&](Register Half) {
          if (!MI.isCopy()) return false;
          Register DstReg = MI.getOperand(0).getReg();
          Register SrcReg = MI.getOperand(1).getReg();
          if (!isAccSpillReg(DstReg) || !isAccSpillReg(SrcReg))
            return false;
          return getRealReg(DstReg) == Half && getRealReg(SrcReg) == Half;
        };
        auto preMIHalfIsGarbage = [&](Register Half) -> bool {
          // Walk backwards from MI in this MBB. We need the SEQUENCE:
          // SpillLoad_i32_Mem (implicit-def Half) ... FAKE_USE killed
          // $aq/$spill_q* ... [no other def of Half] ... MI.
          // If we find that shape, the live Half is garbage.
          bool SawFakeUseKill = false;
          MachineBasicBlock::iterator It(MI);
          while (It != MBB.begin()) {
            --It;
            const MachineInstr &Prev = *It;
            // FAKE_USE killed of $aq or SPILL_Q* (the wide-acc-spill super)?
            if (Prev.getOpcode() == TargetOpcode::FAKE_USE) {
              for (const MachineOperand &MO : Prev.operands()) {
                if (!MO.isReg() || !MO.isKill()) continue;
                if (!MO.getReg().isPhysical()) continue;
                Register R = MO.getReg();
                // Match $aq directly, or a SPILL_Q[0-3] 32-bit spill
                // placeholder (the slot-backed wide-acc-spill).
                // Phase A consolidation 2026-05-16: SPILL_Q[0..31]
                // enum values are consecutive (verified in
                // MC6809GenRegisterInfoEnums.inc: 381..412).
                if (R == MC6809::AQ ||
                    (R >= MC6809::SPILL_Q0 && R <= MC6809::SPILL_Q31)) {
                  SawFakeUseKill = true;
                  break;
                }
              }
              continue;
            }
            // Any other def of Half? If yes, the live value is NOT garbage
            // (it was defined after any FAKE_USE killed sequence).
            bool DefsHalf = false;
            bool IsSpillLoad32 =
                Prev.getOpcode() == MC6809::SpillLoad_i32_Mem;
            for (const MachineOperand &MO : Prev.operands()) {
              if (!MO.isReg() || !MO.isDef()) continue;
              if (!MO.getReg().isPhysical()) continue;
              if (!TRI.regsOverlap(MO.getReg(), Half)) continue;
              DefsHalf = true;
              break;
            }
            if (!DefsHalf) continue;
            // Found a def. If this is a SpillLoad_i32_Mem AND we already
            // saw a FAKE_USE kill of the super-reg, the value is garbage.
            if (SawFakeUseKill && IsSpillLoad32)
              return true;
            // Any other def shape: the live value is real, preserve it.
            return false;
          }
          return false;
        };

        if ((SpillClobbersB || Case1NeedsAltB) &&
            anySubRegLive(MC6809::AB) && !hasDirectOperand(MC6809::AB) &&
            !(isSameHalfSpillCopy(MC6809::AB) &&
              preMIHalfIsGarbage(MC6809::AB)))
          NeedB = true;
        if ((SpillClobbersA || Case1NeedsAltA) &&
            anySubRegLive(MC6809::AA) && !hasDirectOperand(MC6809::AA) &&
            !(isSameHalfSpillCopy(MC6809::AA) &&
              preMIHalfIsGarbage(MC6809::AA)))
          NeedA = true;
      }

      NeedSave[&MI] = {NeedD, NeedIY, NeedIX, NeedA, NeedB, NeedQ};
    }

    // Forward pass: materialize spill operands.
    MachineFrameInfo &MFI = MF.getFrameInfo();

    for (MachineInstr &MI : make_early_inc_range(MBB)) {
      auto It = NeedSave.find(&MI);
      if (It == NeedSave.end())
        continue;

      SpillInfo &Info = It->second;
      DebugLoc DL = MI.getDebugLoc();

      int IYSaveSlot = -1;
      int IXSaveSlot = -1;

      int DSaveSlot = -1;
      int ASaveSlot = -1;
      int BSaveSlot = -1;
      int QSaveSlot = -1;  // Bug #301

      // Bug #301 (2026-05-22): save Q (full $aq) if needed.  Emit
      // STQ $aq → 4-byte stack slot before MI.  NeedSaveQ fires
      // when MI's SPILL_Q* operand materialization (LDQ to $aq)
      // would clobber a live $aq value.  Save is wider than NeedD
      // — must be 4 bytes to capture all of $aq's sub-regs.
      if (Info.NeedSaveQ) {
        QSaveSlot = getSaveSlot(ReuseQSaveSlot, 4, !MI.isTerminator());
        // Same Undef rationale as NeedSaveD (Bug #275): if $aq isn't
        // directly live-in at MBB.begin(), the save reads bits that
        // are part of a super-reg's value but not directly defined.
        // Mark Undef so the verifier accepts the read.
        RegState SrcFlags = RegState(0);
        bool AqDirectLiveIn = MBB.isLiveIn(MC6809::AQ);
        for (MCPhysReg Sub : TRI.subregs(MC6809::AQ))
          AqDirectLiveIn = AqDirectLiveIn || MBB.isLiveIn(Sub);
        if (MachineBasicBlock::iterator(MI) == MBB.begin() && !AqDirectLiveIn)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i32_Mem))
            .addReg(MC6809::AQ, SrcFlags)
            .addFrameIndex(QSaveSlot)
            .addImm(0);
      }

      // Save D if needed.
      if (Info.NeedSaveD) {
        DSaveSlot = getSaveSlot(ReuseDSaveSlot, 2, !MI.isTerminator());
        // Bug #275: NeedSaveD's backward LPR analysis treats $ad as
        // live whenever any super-reg of $ad (e.g. $aq) is live —
        // because LivePhysRegs::addReg(super) adds all sub-regs into
        // the set. In that case the save preserves $ad's bits as part
        // of the super-reg's value, but $ad's value AT THE SAVE POINT
        // may not be reachable from a direct $ad def upstream. The
        // verifier then flags the save's $ad use as an undefined
        // physical register. Probe true liveness via
        // computeRegisterLiveness; if $ad isn't definitely live,
        // mark the save's $ad operand Undef so the bits are still
        // round-tripped (preserving the super-reg's value) without
        // tripping the verifier.
        // Use Undef when the save is at MBB.begin() and $ad isn't a
        // direct MBB live-in. Critical-edge splitter BBs created later
        // would otherwise need to propagate $ad's liveness through
        // synthetic predecessors that don't have it, and the verifier
        // is unforgiving about that. We still emit the STD so $ad's
        // bits are preserved across the upcoming clobber (super-reg
        // value-preservation semantics); the Undef flag merely says
        // "we know the read may be undef".
        RegState SrcFlags = RegState(0);
        bool AdDirectLiveIn = MBB.isLiveIn(MC6809::AD);
        for (MCPhysReg Sub : TRI.subregs(MC6809::AD))
          AdDirectLiveIn = AdDirectLiveIn || MBB.isLiveIn(Sub);
        if (MachineBasicBlock::iterator(MI) == MBB.begin() && !AdDirectLiveIn)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i16_Mem))
            .addReg(MC6809::AD, SrcFlags)
            .addFrameIndex(DSaveSlot)
            .addImm(0);
      }

      // Bug #89: save just $aa or $ab (not both) around an 8-bit spill-
      // operand instruction whose expansion uses that half as a scratch.
      //
      // Bug #272 Phase B Scope A fallout: when the accumulator-hierarchy
      // Defs lists were cleaned up to drop the AQ/AD/AW over-claim, the
      // regalloc gained byte-granular packing freedom and surfaced
      // pre-existing cases where $aa / $ab is reachable as part of a live
      // super-reg (e.g. $aq) but is NOT a direct live-in / live-here. The
      // reverse-pass anySubRegLive(AA) check still marks NeedSaveA in such
      // cases — and rightly so, the save preserves the byte's bits as part
      // of the super-reg's value — but at the forward-pass save site the
      // verifier sees `implicit $aa` reading an undefined physical reg and
      // trips. Same shape as the AD case in Bug #275 (see above), one
      // level finer.
      //
      // Fix: probe true liveness at the save insertion point via
      // MachineBasicBlock::computeRegisterLiveness. If $aa / $ab is
      // definitely dead, mark the save's source operand Undef. The bits
      // are still round-tripped (the STA writes whatever is currently in
      // $aa and the matching LDA later reads it back), so the super-reg's
      // logical value is preserved across the upcoming clobber, and the
      // verifier is satisfied.
      if (Info.NeedSaveA) {
        ASaveSlot = MFI.CreateStackObject(1, Align(1), true);
        RegState SrcFlags = RegState(0);
        if (MBB.computeRegisterLiveness(&TRI, MC6809::AA, MI) !=
            MachineBasicBlock::LQR_Live)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AA, SrcFlags)
            .addFrameIndex(ASaveSlot)
            .addImm(0);
      }
      if (Info.NeedSaveB) {
        BSaveSlot = MFI.CreateStackObject(1, Align(1), true);
        RegState SrcFlags = RegState(0);
        if (MBB.computeRegisterLiveness(&TRI, MC6809::AB, MI) !=
            MachineBasicBlock::LQR_Live)
          SrcFlags = RegState::Undef;
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AB, SrcFlags)
            .addFrameIndex(BSaveSlot)
            .addImm(0);
      }

      // Save IY if needed.
      if (Info.NeedSaveIY) {
        IYSaveSlot = MFI.CreateStackObject(2, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_iPtr_Mem))
            .addReg(MC6809::IY)
            .addFrameIndex(IYSaveSlot)
            .addImm(0);
      }

      // Bug #127b: Save IX if the conflict-resolution path will overwrite it.
      if (Info.NeedSaveIX) {
        IXSaveSlot = MFI.CreateStackObject(2, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_iPtr_Mem))
            .addReg(MC6809::IX)
            .addFrameIndex(IXSaveSlot)
            .addImm(0);
      }

      // Multi-INDEX-spill special case for Store_iPtr_Mem.
      //
      // When BOTH the source value and the base address are SPILL_X
      // registers (e.g., "store one pointer at another"), we can't stage
      // both through IY. Route the source through D and the base through
      // IY, then rewrite the opcode from Store_iPtr_Mem (INDEX16 source)
      // to Store_i16_Mem (ACC16 source). The expansion lookup table picks
      // up STD ,Y automatically.
      //
      // willClobberD already returned true for this case (2+ unique
      // SPILL_X operands), so D save/restore is handled at the top/end
      // of this iteration if D was live.
      if (MI.getOpcode() == MC6809::Store_iPtr_Mem &&
          MI.getOperand(0).isReg() && MI.getOperand(1).isReg()) {
        Register Op0 = MI.getOperand(0).getReg();
        Register Op1 = MI.getOperand(1).getReg();
        if (Op0.isPhysical() && Op1.isPhysical() &&
            isIndexSpillReg(Op0) && isIndexSpillReg(Op1) && Op0 != Op1) {
          auto [VFI, VOff] = getSpillSlot(Op0, FuncInfo);
          auto [AFI, AOff] = getSpillSlot(Op1, FuncInfo);
          (void)VOff; (void)AOff;
          // LDD value_slot — load source value into D
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(VFI)
              .addImm(0);
          // LDY base_slot — load base address into Y
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(MC6809::IY, RegState::Define)
              .addFrameIndex(AFI)
              .addImm(0);
          // Rewrite MI in place: Store_iPtr_Mem → Store_i16_Mem with
          // $ad as source and $iy as base. Same operand layout, just a
          // different source register class.
          MI.setDesc(TII.get(MC6809::Store_i16_Mem));
          MI.getOperand(0).setReg(MC6809::AD);
          MI.getOperand(1).setReg(MC6809::IY);
          // Operand 2 (offset immediate) unchanged.
          // No SPILL_X operands remain — the regular SpillOps loop below
          // will see nothing and skip materialization.
        }
      }

      // Collect spill operands into SpillOps for the materialization
      // loop below to process. Each entry is (operand index, spill reg).
      //
      // Bug #63 special-case
      // ====================
      // For Add/Sub/SetCarry binary _Reg pseudos that expand via
      // emit6809Reg{Byte,Pair}FromMem (see needsSecondAccSpillSkip), we
      // SKIP the SECOND-and-later distinct ACC spill operand here.
      // Those operands stay in the MI as spill regs, and the expansion
      // handles them via its U-relative spill path. See the long
      // comment on needsSecondAccSpillSkip for the full rationale.
      //
      // Tied dst+src referencing the SAME SPILL_D both stay in SpillOps
      // (because they hash to the same entry in SeenAccSpillsForSkip
      // and the size never grows past 1 from them). Only a NEW spill
      // register — the src2 — gets skipped.
      //
      // Worked example for `AddSetCarry_i16_Reg dst, carry, src, src2`
      // with dst = $spill_d4 (tied to src) and src2 = $spill_d1:
      //
      //   op0 dst   = $spill_d4 (def, tied to op2)
      //   op1 carry = $phantom_carry_N (def, PHANTOM_CARRY — not a spill)
      //   op2 src   = $spill_d4 (use, tied — same reg as op0)
      //   op3 src2  = $spill_d1 (use)
      //
      // Iteration:
      //   op0: $spill_d4 not seen → seen={$spill_d4}, size=1 → push
      //   op1: $phantom_carry_N not a spill → skip entirely
      //   op2: $spill_d4 already seen → push (still size 1)
      //   op3: $spill_d1 not seen → seen={d4,d1}, size=2 → SKIP push
      //
      // Result: SpillOps = [(0,d4), (2,d4)]. The materialization loop
      // rewrites op0 and op2 to $ad (loaded from spill_d4). op3 stays
      // as $spill_d1. The expansion's U-relative path then reads
      // spill_d1 as `adcb d1+1,u; adca d1+0,u`. No clash.
      SmallVector<std::pair<unsigned, Register>, 4> SpillOps;
      const bool NeedsAccSpillSkip = needsSecondAccSpillSkip(MI.getOpcode());
      const bool NeedsIndexSpillSkip = needsSecondIndexSpillSkip(MI.getOpcode());
      llvm::SmallSet<Register, 4> SeenAccSpillsForSkip;
      llvm::SmallSet<Register, 4> SeenIndexSpillsForSkip;
      for (unsigned I = 0; I < MI.getNumOperands(); ++I) {
        MachineOperand &MO = MI.getOperand(I);
        if (!MO.isReg() || !MO.getReg().isPhysical() ||
            !isAnySpillReg(MO.getReg()))
          continue;
        if (NeedsAccSpillSkip && isAccSpillReg(MO.getReg())) {
          // Track unique ACC spill registers seen so far. The first
          // unique one is added to SpillOps as normal. The second
          // (and beyond) are SKIPPED — they survive into expansion
          // as spill regs and the expansion's U-relative path
          // handles them. Same-spill repeats (tied dst+src) don't
          // count toward the limit because they don't grow the set.
          //
          // Bug #161 round 18: also skip if a NON-spill physical-reg
          // operand of this MI already occupies the same hardware
          // register that this spill would materialize into. Without
          // this, the spill load (LDD/LDB) clobbers the live phys-reg
          // operand; the expansion then produces a degenerate
          // `CMPR X,X` / `SUBR X,X` because both operands resolve to
          // the same physreg. Leaving the spill as-is lets the
          // expansion's U-relative path read it directly.
          Register RealReg = getRealReg(MO.getReg());
          bool PhysCollision = false;
          for (unsigned J = 0; J < MI.getNumOperands(); ++J) {
            if (J == I) continue;
            const MachineOperand &OtherMO = MI.getOperand(J);
            if (!OtherMO.isReg() || !OtherMO.getReg().isPhysical()) continue;
            if (isAnySpillReg(OtherMO.getReg())) continue;
            if (TRI.regsOverlap(OtherMO.getReg(), RealReg)) {
              PhysCollision = true;
              break;
            }
          }
          if (!SeenAccSpillsForSkip.contains(MO.getReg())) {
            SeenAccSpillsForSkip.insert(MO.getReg());
            if (SeenAccSpillsForSkip.size() > 1 || PhysCollision) {
              // 2nd+ unique ACC spill, or first spill colliding with a
              // physical operand — leave it for the expansion's
              // U-relative spill path.
              //
              // Bug #271 cat-1 residual: the SPILL_* operand survives
              // post-MaterializeSpills with no visible def (the def was
              // a Store_i8_Mem to the U-relative slot, not a phys-reg
              // def). The verifier flags this as "Using an undefined
              // physical register" because SPILL_* phys regs don't
              // model the slot-as-storage. Mark the use as Undef so
              // the verifier skips its liveness check; ExpandPostRAPseudo
              // still reads the SPILL_* reg identity to compute the
              // U-relative offset, so codegen is unaffected.
              if (MO.isUse())
                MO.setIsUndef(true);
              continue;
            }
          } else if (PhysCollision) {
            if (MO.isUse())
              MO.setIsUndef(true);
            continue;
          }
        }
        // INDEX analog: every SPILL_X materialises into IY, so a second
        // distinct SPILL_X — or one whose IY would clobber a live IY operand
        // of this same compare — is left for expandCompareReg's register-vs-
        // spill path instead of being staged into IY (which would collapse the
        // compare to "$iy,$iy"). See needsSecondIndexSpillSkip.
        if (NeedsIndexSpillSkip && isIndexSpillReg(MO.getReg())) {
          Register RealReg = getRealReg(MO.getReg()); // IY
          bool PhysCollision = false;
          for (unsigned J = 0; J < MI.getNumOperands(); ++J) {
            if (J == I) continue;
            const MachineOperand &OtherMO = MI.getOperand(J);
            if (!OtherMO.isReg() || !OtherMO.getReg().isPhysical()) continue;
            if (isAnySpillReg(OtherMO.getReg())) continue;
            if (TRI.regsOverlap(OtherMO.getReg(), RealReg)) {
              PhysCollision = true;
              break;
            }
          }
          if (!SeenIndexSpillsForSkip.contains(MO.getReg())) {
            SeenIndexSpillsForSkip.insert(MO.getReg());
            if (SeenIndexSpillsForSkip.size() > 1 || PhysCollision) {
              if (MO.isUse())
                MO.setIsUndef(true);
              continue;
            }
          } else if (PhysCollision) {
            if (MO.isUse())
              MO.setIsUndef(true);
            continue;
          }
        }
        SpillOps.push_back({I, MO.getReg()});
      }

      // Conflict detection for 8-bit spill loads that would clobber a
      // live value in the same register.
      //
      // Case 1 (LHS=spill, RHS=physical): the spill load clobbers the
      //   physical RHS. Fix: save RHS to a fresh frame slot, load into
      //   the OPPOSITE accumulator half, redirect RHS operand.
      //
      // Case 2 (LHS=physical/tied, RHS=spill): the spill load clobbers
      //   the tied LHS. Fix: DON'T load the spill — leave it as a
      //   spill register and let the expansion read it from its
      //   U-relative slot via path (a). This avoids the clobber entirely.
      SmallSet<unsigned, 4> SkipSpillLoad;
      for (auto &[OpIdx, SpillReg] : SpillOps) {
        MachineOperand &SpillMO = MI.getOperand(OpIdx);
        if (!SpillMO.isUse() && !(SpillMO.isDef() && SpillMO.isTied()))
          continue;
        Register RealReg = getRealReg(SpillReg);
        if (RealReg == MC6809::AD) continue;
        for (unsigned J = 0; J < MI.getNumOperands(); ++J) {
          if (J == (unsigned)OpIdx) continue;
          MachineOperand &OtherMO = MI.getOperand(J);
          if (!OtherMO.isReg() || !OtherMO.isUse()) continue;
          if (isAnySpillReg(OtherMO.getReg())) continue;
          if (OtherMO.getReg() != RealReg) continue;
          // Bug #116: this conflict-resolution block assumed RealReg is
          // an 8-bit accumulator (AA/AB). For INDEX spills (SPILL_X*),
          // getRealReg returns IY — a 16-bit register. The 8-bit save
          // path silently emitted ill-formed `Store_i8_Mem $iy, ...`
          // and ended up rewriting OtherMO to AB, which downstream
          // resolved to STBi_o0 (byte store) on what should have been
          // a `(store (p0))` (pointer store). Only the high byte of
          // the pointer got written; callees that followed the
          // resulting function pointer jumped into RAM.
          //
          // Handle the INDEX case explicitly: save as a 2-byte iPtr,
          // restore into the OTHER index register (IX↔IY), and
          // redirect OtherMO to that alt index.
          if (RealReg == MC6809::IY || RealReg == MC6809::IX) {
            int SaveFI = MF.getFrameInfo().CreateStackObject(
                2, Align(1), /*isSpillSlot=*/false);
            BuildMI(MBB, MI, DL, TII.get(MC6809::Store_iPtr_Mem))
                .addReg(RealReg)
                .addFrameIndex(SaveFI)
                .addImm(0);
            Register AltReg =
                (RealReg == MC6809::IY) ? MC6809::IX : MC6809::IY;
            if (OtherMO.isTied()) {
              // Tied case: defer the reload, leave the spill for the
              // expansion to read U-relative. Matches the 8-bit tied
              // behaviour below.
              SkipSpillLoad.insert(OpIdx);
            } else {
              BuildMI(MBB, MI, DL, TII.get(MC6809::Load_iPtr_Mem))
                  .addReg(AltReg, RegState::Define)
                  .addFrameIndex(SaveFI)
                  .addImm(0);
              OtherMO.setReg(AltReg);
            }
            break;
          }
          if (OtherMO.isTied()) {
            // Case 2: RHS spill would clobber tied LHS. Same fix as
            // Case 1: save the live value to a fresh slot, load into
            // the alt register. The tied DEF/USE pair keeps the
            // original register; only the spill USE gets the alt.
            int SaveFI = MF.getFrameInfo().CreateStackObject(
                2, Align(1), /*isSpillSlot=*/false);
            int SaveOff = (RealReg == MC6809::AB) ? 1 : 0;
            // Save the LHS (tied) value currently in the register.
            // Bug #272 Phase B Scope A fallout: probe true liveness — see
            // NeedSaveA/B notes above. Mark Undef when the byte is dead at
            // the save site.
            RegState SrcFlags = RegState(0);
            if (MBB.computeRegisterLiveness(&TRI, RealReg, MI) !=
                MachineBasicBlock::LQR_Live)
              SrcFlags = RegState::Undef;
            BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
                .addReg(RealReg, SrcFlags)
                .addFrameIndex(SaveFI)
                .addImm(SaveOff);
            // After the spill load clobbers the register with the RHS,
            // we need the LHS back. Load it into the alt register.
            Register AltReg = (RealReg == MC6809::AB) ? MC6809::AA : MC6809::AB;
            // Insert the reload AFTER all spill loads complete (deferred).
            // For now, change the tied operands to the alt register and
            // skip the spill load.
            SkipSpillLoad.insert(OpIdx);
          } else {
            // Case 1: LHS spill would clobber physical RHS. Save RealReg
            // (which currently holds OtherMO's value), redirect OtherMO
            // to the alt half via the saved value.
            //
            // Bug #184: previously this saved RealReg to a fresh stack
            // slot but never restored it after the MI ran. The spill
            // load + SUB then left the SUB result in RealReg, and any
            // later MI that expected RealReg to still hold the original
            // operand value (per regalloc's plan) read garbage. To fix,
            // route the save through ASaveSlot/BSaveSlot so the existing
            // post-MI restore mechanism (at the end of this iteration)
            // reloads RealReg with the saved value.
            Register AltReg = (RealReg == MC6809::AB) ? MC6809::AA : MC6809::AB;
            int &SaveSlotRef = (RealReg == MC6809::AB) ? BSaveSlot : ASaveSlot;
            if (SaveSlotRef < 0) {
              // No NeedSaveB/A was scheduled for this MI; allocate now.
              // Existing NeedSaveB save (if any) already stored RealReg
              // to BSaveSlot at the top of this iteration; in that case
              // we reuse it and skip our redundant save below.
              SaveSlotRef = MFI.CreateStackObject(
                  1, Align(1), /*isSpillSlot=*/true);
              // Bug #272 Phase B Scope A fallout — see NeedSaveA/B notes.
              RegState SrcFlags = RegState(0);
              if (MBB.computeRegisterLiveness(&TRI, RealReg, MI) ==
                  MachineBasicBlock::LQR_Dead)
                SrcFlags = RegState::Undef;
              BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
                  .addReg(RealReg, SrcFlags)
                  .addFrameIndex(SaveSlotRef)
                  .addImm(0);
            }
            BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(AltReg, RegState::Define)
                .addFrameIndex(SaveSlotRef)
                .addImm(0);
            OtherMO.setReg(AltReg);
          }
          break;
        }
      }

      // Load spill values into real registers for USE operands.
      for (auto &[OpIdx, SpillReg] : SpillOps) {
        // Case 2 skip: leave as a spill register for the expansion.
        if (SkipSpillLoad.count(OpIdx))
          continue;
        MachineOperand &MO = MI.getOperand(OpIdx);
        bool NeedLoad = MO.isUse() || (MO.isDef() && MO.isTied());
        if (!NeedLoad) {
          MO.setReg(getRealReg(SpillReg));
          continue;
        }

        Register RealReg = getRealReg(SpillReg);
        auto [FI, ByteOffset] = getSpillSlot(SpillReg, FuncInfo);

        // Bug #118 Layer 1 (approach b): EXTRACT_{LO,HI}_i16 with SPILL_D*
        // src. Default path would emit Load_i16_Mem → $ad (LDD), which
        // clobbers both halves of D and destroys any sibling extract
        // result already staged in the other byte. Load just the byte we
        // need — directly into the EXTRACT's destination register — and
        // erase the EXTRACT. Big-endian: HI at slot+0, LO at slot+1.
        //
        // Bug #267 (updated): the destination class is now ACC8_AB
        // ({AB, AA}), so the dst is always a real byte accumulator and
        // Load_i8_Mem can target it directly regardless of which half it
        // landed in; the old fixed-staging-byte scheme (stage into AB/AA
        // and rely on dst == staging to erase) left an ill-formed
        // `EXTRACT $aa, $ab` behind when regalloc chose the cross half.
        if ((MI.getOpcode() == MC6809::EXTRACT_LO_i16 ||
             MI.getOpcode() == MC6809::EXTRACT_HI_i16) &&
            isWideAccSpillReg(SpillReg)) {
          bool IsLo = (MI.getOpcode() == MC6809::EXTRACT_LO_i16);
          int ByteOff = ByteOffset + (IsLo ? 1 : 0);
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
              .addReg(MI.getOperand(0).getReg(), RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOff);
          // Defer erase to the end of BB processing so we don't invalidate
          // operand iterators or downstream loops in this MI.
          ToErase.push_back(&MI);
          continue;
        }

        // Sub-register extraction from a spilled i16: load only the
        // requested byte instead of the full i16. On big-endian MC6809:
        //   sub_hi_byte (MSB) = slot + 0, loaded into A
        //   sub_lo_byte (LSB) = slot + 1, loaded into B
        unsigned SubReg = MO.getSubReg();
        if (SubReg && RealReg == MC6809::AD) {
          bool IsLo = (SubReg == MC6809::sub_lo_byte);
          Register ByteReg = IsLo ? MC6809::AB : MC6809::AA;
          int SubOffset = ByteOffset + (IsLo ? 1 : 0);
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
              .addReg(ByteReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(SubOffset);
          MO.setReg(ByteReg);
          MO.setSubReg(0);
        } else if (isIndexSpillReg(SpillReg)) {
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(RealReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(0);
          MO.setReg(RealReg);
        } else if (RealReg == MC6809::AD) {
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
          MO.setReg(RealReg);
        } else {
          // 8-bit A or B: load individual byte.
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
              .addReg(RealReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
          MO.setReg(RealReg);
        }
      }

      // After the instruction: store back any DEFs to spill slots.
      auto After = std::next(MachineBasicBlock::iterator(MI));
      for (auto &[OpIdx, SpillReg] : SpillOps) {
        MachineOperand &MO = MI.getOperand(OpIdx);
        if (!MO.isDef())
          continue;
        Register RealReg = getRealReg(SpillReg);
        auto [FI, ByteOffset] = getSpillSlot(SpillReg, FuncInfo);

        // Bug #274: the DEF was inherited from the original spill-class
        // def (e.g. `dead $spill_b0 = EXTRACT_LO_i16 ...`). If it was
        // marked dead, the flag is stale once we emit a Store_i8_Mem
        // below that reads this def — clear it so the verifier sees a
        // live def → store edge.
        MO.setIsDead(false);

        // Sub-register store: write only the byte to the correct
        // position within the i16 spill slot.
        unsigned SubReg = MO.getSubReg();
        if (SubReg && RealReg == MC6809::AD) {
          bool IsLo = (SubReg == MC6809::sub_lo_byte);
          Register ByteReg = IsLo ? MC6809::AB : MC6809::AA;
          int SubOffset = ByteOffset + (IsLo ? 1 : 0);
          BuildMI(MBB, After, DL, TII.get(MC6809::Store_i8_Mem))
              .addReg(ByteReg)
              .addFrameIndex(FI)
              .addImm(SubOffset);
          MO.setSubReg(0);
        } else if (isIndexSpillReg(SpillReg)) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Store_iPtr_Mem))
              .addReg(RealReg)
              .addFrameIndex(FI)
              .addImm(0);
        } else if (RealReg == MC6809::AD) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Store_i16_Mem))
              .addReg(MC6809::AD)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
        } else {
          BuildMI(MBB, After, DL, TII.get(MC6809::Store_i8_Mem))
              .addReg(RealReg)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
        }
      }

      // Determine if A/B restores must be deferred to successors.
      // On the 6809, LDA/LDB set NZ flags. If we insert a byte restore
      // between a Compare (which sets CC) and its conditional branch
      // (which reads CC), the restore clobbers the comparison result.
      // This is the same CC-clobbering problem that led to the fused
      // CompareBranch pseudos (bugs #42, #59). The D and IY restores
      // (LDD, LDY) have the same NZ issue but are less common here.
      //
      // Detection: if the instruction defines CC and the NEXT instruction
      // is a conditional branch reading CC, defer A/B restores to the
      // branch's successor blocks (same path as terminators).
      bool DeferByteRestore = false;
      if (!MI.isTerminator() && (ASaveSlot >= 0 || BSaveSlot >= 0)) {
        if (MI.isCompare() ||
            MI.definesRegister(MC6809::CC, /*TRI=*/nullptr)) {
          auto NextIt = After;
          if (NextIt != MBB.end() && NextIt->isConditionalBranch())
            DeferByteRestore = true;
        }
      }

      if (!MI.isTerminator()) {
        // Restore saved registers after non-terminator instructions.
        if (IYSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(MC6809::IY, RegState::Define)
              .addFrameIndex(IYSaveSlot)
              .addImm(0);
        }
        if (IXSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(MC6809::IX, RegState::Define)
              .addFrameIndex(IXSaveSlot)
              .addImm(0);
        }
        // Bug #301 (2026-05-22): restore Q if saved.  LDQ slot → $aq
        // covers all sub-regs ($ad/$aw/$aa/$ab/$ae/$af).
        if (QSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_i32_Mem))
              .addReg(MC6809::AQ, RegState::Define)
              .addFrameIndex(QSaveSlot)
              .addImm(0);
        }
        if (DSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(DSaveSlot)
              .addImm(0);
        }
        if (!DeferByteRestore) {
          if (ASaveSlot >= 0) {
            BuildMI(MBB, After, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(MC6809::AA, RegState::Define)
                .addFrameIndex(ASaveSlot)
                .addImm(0);
          }
          if (BSaveSlot >= 0) {
            BuildMI(MBB, After, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(MC6809::AB, RegState::Define)
                .addFrameIndex(BSaveSlot)
                .addImm(0);
          }
        }
      }

      if (MI.isTerminator() || DeferByteRestore) {
        // For terminators (and deferred byte restores after compares):
        // insert the restore at the start of each successor block.
        //
        // D and IY restores go here ONLY for terminators. For deferred
        // byte restores (DeferByteRestore), only the A/B restores are
        // deferred — D and IY were already placed inline above.
        //
        // For multi-predecessor successors (critical edges), split the
        // edge by inserting a new block for the restore.
        // Bug #301 (2026-05-22): terminator-successor restore for Q.
        // Same structure as the D path above — single-predecessor
        // successors get the restore at the entry; critical-edge
        // successors get a new restore block on the edge.
        if (MI.isTerminator() && QSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i32_Mem))
                  .addReg(MC6809::AQ, RegState::Define)
                  .addFrameIndex(QSaveSlot)
                  .addImm(0);
            } else {
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i32_Mem))
                  .addReg(MC6809::AQ, RegState::Define)
                  .addFrameIndex(QSaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        }
        if (MI.isTerminator() && DSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i16_Mem))
                  .addReg(MC6809::AD, RegState::Define)
                  .addFrameIndex(DSaveSlot)
                  .addImm(0);
            } else {
              // Critical edge: split by inserting a new block.
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              // Update any branch targets in MBB that pointed to Succ.
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              // Transfer live-ins from the edge.
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              // Insert D restore and unconditional branch.
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i16_Mem))
                  .addReg(MC6809::AD, RegState::Define)
                  .addFrameIndex(DSaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        }
        // IY/IX restores into successors after a terminator. Symmetric
        // with the D-restore block above and the A/B-restore lambda
        // below: single-predecessor successors get the restore at the
        // entry; multi-predecessor (critical-edge) successors require
        // a new restore block on the edge so that the OTHER predecessors
        // don't inherit the restore unintentionally.
        //
        // Pre-2026-04-25 these two handlers were the ONLY ones missing
        // critical-edge handling — D had it (lines above), A/B had it
        // (commit a15e6822d693), but IY/IX silently skipped the
        // critical-edge case. Latent bug surfaced during rebase
        // investigation when looking for a related D-spill issue.
        auto insertPtrRestoreInSuccessors = [&](int SaveSlot, Register PtrReg) {
          if (SaveSlot < 0) return;
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_iPtr_Mem))
                  .addReg(PtrReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
            } else {
              // Critical edge: split by inserting a new block.
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_iPtr_Mem))
                  .addReg(PtrReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        };
        if (MI.isTerminator()) {
          insertPtrRestoreInSuccessors(IYSaveSlot, MC6809::IY);
          insertPtrRestoreInSuccessors(IXSaveSlot, MC6809::IX);
        }
        // Bug #89 fix completion: A/B saves also need restoration in
        // successors when the instruction is a terminator. Without this,
        // a TestBranch with a spill operand that clobbers $ab/$aa
        // saves the live value but never restores it on the path
        // through the successor blocks. Exposed by bug #86's ACC8
        // relaxation for TestBranch_i8_Reg.
        auto insertByteRestoreInSuccessors = [&](int SaveSlot, Register ByteReg) {
          if (SaveSlot < 0) return;
          for (MachineBasicBlock *Succ : llvm::to_vector(MBB.successors())) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i8_Mem))
                  .addReg(ByteReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
            } else {
              // Critical edge: split by inserting a new block.
              MachineBasicBlock *RestoreBB =
                  MF.CreateMachineBasicBlock(MBB.getBasicBlock());
              MF.insert(std::next(MBB.getIterator()), RestoreBB);
              RestoreBB->addSuccessor(Succ);
              MBB.replaceSuccessor(Succ, RestoreBB);
              for (MachineInstr &Term : MBB.terminators())
                for (MachineOperand &MO : Term.operands())
                  if (MO.isMBB() && MO.getMBB() == Succ)
                    MO.setMBB(RestoreBB);
              for (const auto &LI : Succ->liveins())
                if (!RestoreBB->isLiveIn(LI.PhysReg))
                  RestoreBB->addLiveIn(LI.PhysReg);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::Load_i8_Mem))
                  .addReg(ByteReg, RegState::Define)
                  .addFrameIndex(SaveSlot)
                  .addImm(0);
              BuildMI(*RestoreBB, RestoreBB->end(), DL,
                      TII.get(MC6809::LBRAlb))
                  .addMBB(Succ);
            }
          }
        };
        insertByteRestoreInSuccessors(ASaveSlot, MC6809::AA);
        insertByteRestoreInSuccessors(BSaveSlot, MC6809::AB);
      }

      Changed = true;
    }

    // Erase instructions that were replaced by direct load/store.
    for (MachineInstr *MI : ToErase)
      MI->eraseFromParent();
  }

  // Frame pointer ($su) liveins are added by emitPrologue (FrameLowering).
  // SU is unconditionally reserved (see MC6809RegisterInfo::getReservedRegs),
  // so the machine verifier accepts it as a livein without a def in this pass.

  return Changed;
}

char MC6809MaterializeSpills::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809MaterializeSpills, DEBUG_TYPE,
                "MC6809 Materialize Spill Registers", false, false)

MachineFunctionPass *llvm::createMC6809MaterializeSpillsPass() {
  return new MC6809MaterializeSpills();
}
