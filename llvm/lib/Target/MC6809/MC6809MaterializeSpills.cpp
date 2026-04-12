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

/// Bug #63 support: opcodes whose expansion runs through
/// emit6809Reg{Byte,Pair}FromMem (in MC6809InstrInfo.cpp).
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
/// `emit6809Reg{Byte,Pair}FromMem` already has a U-relative spill path
/// for the RHS that reads the spill slot directly without going through
/// D. That path was previously dead code because MaterializeSpills
/// always rewrote spill operands BEFORE expansion ran. We re-enable
/// it by teaching MaterializeSpills to LEAVE THE SECOND DISTINCT ACC
/// SPILL alone for these specific opcodes — the operand stays as a
/// spill register, the expansion sees it as a spill, and the
/// U-relative path takes over naturally.
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
/// Only opcodes whose expansion path includes the U-relative spill
/// fallback can benefit. emit6809Reg{Byte,Pair}FromMem is the helper
/// that has it; instructions whose expansion uses a different helper
/// would need a separate fix. The opcodes below all dispatch through
/// expandAddReg/expandSubReg/expand{Add,Sub}SetCarryReg/
/// expand{Add,Sub}SetCarryUseReg, which all call into the helper.
static bool isAddSubFamilyReg(unsigned Opcode) {
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

/// Check if this instruction's spill operands will cause D to be clobbered
/// during materialization. Mirrors SpillDSaveRestore's willClobberD logic.
static bool willClobberD(const MachineInstr &MI,
                         const TargetRegisterInfo &TRI) {
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
    if (AnySpillUsed)
      FuncInfo->UsesSpillRegisters = true;
  }

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
      bool NeedSaveA;
      bool NeedSaveB;
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
      for (const MachineOperand &MO : MI.operands()) {
        if (MO.isReg() && MO.getReg().isPhysical() && isAnySpillReg(MO.getReg())) {
          HasSpill = true;
          break;
        }
      }
      if (!HasSpill)
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
        if (!Has8BitSpill) {
          // Check if D is live with a value that would be clobbered.
          bool DLive = LPR.contains(MC6809::AD) ||
                       LPR.contains(MC6809::AA) ||
                       LPR.contains(MC6809::AB);
          if (!DLive)
            continue; // Safe to skip — D isn't live, expansion won't lose anything.
        }
      }

      bool NeedD = false, NeedIY = false;
      bool NeedA = false, NeedB = false;

      if (willClobberD(MI, TRI)) {
        bool DLive = LPR.contains(MC6809::AD) ||
                     LPR.contains(MC6809::AA) ||
                     LPR.contains(MC6809::AB);
        if (DLive)
          NeedD = true;
      }

      if (willClobberIY(MI, TRI)) {
        if (LPR.contains(MC6809::IY))
          NeedIY = true;
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
        if ((SpillClobbersB || Case1NeedsAltB) &&
            LPR.contains(MC6809::AB) && !hasDirectOperand(MC6809::AB))
          NeedB = true;
        if ((SpillClobbersA || Case1NeedsAltA) &&
            LPR.contains(MC6809::AA) && !hasDirectOperand(MC6809::AA))
          NeedA = true;
      }

      NeedSave[&MI] = {NeedD, NeedIY, NeedA, NeedB};
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

      int DSaveSlot = -1;
      int ASaveSlot = -1;
      int BSaveSlot = -1;

      // Save D if needed.
      if (Info.NeedSaveD) {
        DSaveSlot = MFI.CreateStackObject(2, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i16_Mem))
            .addReg(MC6809::AD)
            .addFrameIndex(DSaveSlot)
            .addImm(0);
      }

      // Bug #89: save just $aa or $ab (not both) around an 8-bit spill-
      // operand instruction whose expansion uses that half as a scratch.
      if (Info.NeedSaveA) {
        ASaveSlot = MFI.CreateStackObject(1, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AA)
            .addFrameIndex(ASaveSlot)
            .addImm(0);
      }
      if (Info.NeedSaveB) {
        BSaveSlot = MFI.CreateStackObject(1, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
            .addReg(MC6809::AB)
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
      if (MI.getOpcode() == MC6809::Store_iPtr_Mem) {
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
      // emit6809Reg{Byte,Pair}FromMem (see isAddSubFamilyReg), we
      // SKIP the SECOND-and-later distinct ACC spill operand here.
      // Those operands stay in the MI as spill regs, and the expansion
      // handles them via its U-relative spill path. See the long
      // comment on isAddSubFamilyReg for the full rationale.
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
      //   op1 carry = $aalsb       (def, BIT1 — not in any spill set)
      //   op2 src   = $spill_d4 (use, tied — same reg as op0)
      //   op3 src2  = $spill_d1 (use)
      //
      // Iteration:
      //   op0: $spill_d4 not seen → seen={$spill_d4}, size=1 → push
      //   op1: $aalsb not a spill → skip entirely
      //   op2: $spill_d4 already seen → push (still size 1)
      //   op3: $spill_d1 not seen → seen={d4,d1}, size=2 → SKIP push
      //
      // Result: SpillOps = [(0,d4), (2,d4)]. The materialization loop
      // rewrites op0 and op2 to $ad (loaded from spill_d4). op3 stays
      // as $spill_d1. The expansion's U-relative path then reads
      // spill_d1 as `adcb d1+1,u; adca d1+0,u`. No clash.
      SmallVector<std::pair<unsigned, Register>, 4> SpillOps;
      const bool IsAddSubReg = isAddSubFamilyReg(MI.getOpcode());
      llvm::SmallSet<Register, 4> SeenAccSpillsForSkip;
      for (unsigned I = 0; I < MI.getNumOperands(); ++I) {
        MachineOperand &MO = MI.getOperand(I);
        if (!MO.isReg() || !MO.getReg().isPhysical() ||
            !isAnySpillReg(MO.getReg()))
          continue;
        if (IsAddSubReg && isAccSpillReg(MO.getReg())) {
          // Track unique ACC spill registers seen so far. The first
          // unique one is added to SpillOps as normal. The second
          // (and beyond) are SKIPPED — they survive into expansion
          // as spill regs and the expansion's U-relative path
          // handles them. Same-spill repeats (tied dst+src) don't
          // count toward the limit because they don't grow the set.
          if (!SeenAccSpillsForSkip.contains(MO.getReg())) {
            SeenAccSpillsForSkip.insert(MO.getReg());
            if (SeenAccSpillsForSkip.size() > 1) {
              // 2nd+ unique ACC spill — leave it for the expansion's
              // U-relative spill path. See the worked example above.
              continue;
            }
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
          if (OtherMO.isTied()) {
            // Case 2: RHS spill would clobber tied LHS. Same fix as
            // Case 1: save the live value to a fresh slot, load into
            // the alt register. The tied DEF/USE pair keeps the
            // original register; only the spill USE gets the alt.
            int SaveFI = MF.getFrameInfo().CreateStackObject(
                2, Align(1), /*isSpillSlot=*/false);
            int SaveOff = (RealReg == MC6809::AB) ? 1 : 0;
            // Save the LHS (tied) value currently in the register.
            BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
                .addReg(RealReg)
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
            // Case 1: LHS spill would clobber physical RHS.
            int SaveFI = MF.getFrameInfo().CreateStackObject(
                2, Align(1), /*isSpillSlot=*/false);
            int SaveOff = (RealReg == MC6809::AB) ? 1 : 0;
            BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i8_Mem))
                .addReg(RealReg)
                .addFrameIndex(SaveFI)
                .addImm(SaveOff);
            Register AltReg = (RealReg == MC6809::AB) ? MC6809::AA : MC6809::AB;
            BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
                .addReg(AltReg, RegState::Define)
                .addFrameIndex(SaveFI)
                .addImm(SaveOff);
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

      if (!MI.isTerminator()) {
        // Restore saved registers after non-terminator instructions.
        if (IYSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(MC6809::IY, RegState::Define)
              .addFrameIndex(IYSaveSlot)
              .addImm(0);
        }
        if (DSaveSlot >= 0) {
          BuildMI(MBB, After, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(DSaveSlot)
              .addImm(0);
        }
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
      } else {
        // Terminator (fused compare-branch): can't place D restore after it.
        // Insert the D restore at the start of each successor. For
        // multi-predecessor successors (critical edges), split the edge
        // by inserting a new block for the restore.
        if (DSaveSlot >= 0) {
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
        if (IYSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : MBB.successors()) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_iPtr_Mem))
                  .addReg(MC6809::IY, RegState::Define)
                  .addFrameIndex(IYSaveSlot)
                  .addImm(0);
            }
          }
        }
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
