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

/// Check if this instruction's spill operands will cause D to be clobbered
/// during materialization. Mirrors SpillDSaveRestore's willClobberD logic.
static bool willClobberD(const MachineInstr &MI,
                         const TargetRegisterInfo &TRI) {
  bool HasAccSpill = false;
  bool DIsDirectOperand = false;

  for (const MachineOperand &MO : MI.operands()) {
    if (!MO.isReg() || !MO.getReg().isPhysical())
      continue;
    if (isAccSpillReg(MO.getReg()))
      HasAccSpill = true;
    if (TRI.regsOverlap(MO.getReg(), MC6809::AD) && !isAccSpillReg(MO.getReg()))
      DIsDirectOperand = true;
  }

  if (!HasAccSpill)
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
    if (AnySpillUsed) {
      FuncInfo->UsesSpillRegisters = true;
      // Reserve the frame pointer ($su) now that we know spill registers
      // are in use. The initial freezeReservedRegs() may have missed it
      // because hasFP() was false before spills were assigned (bug #16).
      MachineRegisterInfo &MRI = MF.getRegInfo();
      if (MRI.reservedRegsFrozen() && !MRI.isReserved(MC6809::SU)) {
        MRI.reserveReg(MC6809::SU, &TRI);
        // Clear 'renamable' flags on $su operands — they were set by the RA
        // before $su was reserved. The verifier flags these as errors.
        for (MachineBasicBlock &MBB : MF)
          for (MachineInstr &MI : MBB)
            for (MachineOperand &MO : MI.operands())
              if (MO.isReg() && MO.getReg() == MC6809::SU && MO.isRenamable())
                MO.setIsRenamable(false);
      }
    }
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
    struct SpillInfo {
      bool NeedSaveD;
      bool NeedSaveIY;
    };
    DenseMap<MachineInstr *, SpillInfo> NeedSave;

    for (MachineInstr &MI : llvm::reverse(MBB)) {
      LPR.stepBackward(MI);

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

      // Skip compare/test instructions and fused compare-branch terminators
      // that only have 16-bit spill operands (SPILL_D). Their expansion
      // functions handle these via backwards scan (CMPX/CMPY), preserving D.
      // Fused pseudos with 8-bit spill operands (SPILL_B) ARE processed
      // because the expansion can't avoid clobbering D for byte comparisons.
      if (MI.isCompare() || MI.isTerminator()) {
        // Check if any spill operand is an 8-bit byte spill that needs
        // materialization (the expansion can't handle these without D clobber).
        bool Has8BitSpill = false;
        for (const MachineOperand &MO : MI.operands()) {
          if (MO.isReg() && MO.getReg().isPhysical() && isAccSpillReg(MO.getReg())) {
            Register RealReg = getRealReg(MO.getReg());
            if (RealReg == MC6809::AA || RealReg == MC6809::AB) {
              Has8BitSpill = true;
              break;
            }
          }
        }
        if (!Has8BitSpill)
          continue;
      }

      bool NeedD = false, NeedIY = false;

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

      NeedSave[&MI] = {NeedD, NeedIY};
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

      // Save D if needed.
      if (Info.NeedSaveD) {
        DSaveSlot = MFI.CreateStackObject(2, Align(1), true);
        BuildMI(MBB, MI, DL, TII.get(MC6809::Store_i16_Mem))
            .addReg(MC6809::AD)
            .addFrameIndex(DSaveSlot)
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

      // Collect spill operands.
      SmallVector<std::pair<unsigned, Register>, 4> SpillOps;
      for (unsigned I = 0; I < MI.getNumOperands(); ++I) {
        MachineOperand &MO = MI.getOperand(I);
        if (MO.isReg() && MO.getReg().isPhysical() && isAnySpillReg(MO.getReg()))
          SpillOps.push_back({I, MO.getReg()});
      }

      // Load spill values into real registers for USE operands.
      for (auto &[OpIdx, SpillReg] : SpillOps) {
        MachineOperand &MO = MI.getOperand(OpIdx);
        bool NeedLoad = MO.isUse() || (MO.isDef() && MO.isTied());
        if (!NeedLoad) {
          MO.setReg(getRealReg(SpillReg));
          continue;
        }

        Register RealReg = getRealReg(SpillReg);
        auto [FI, ByteOffset] = getSpillSlot(SpillReg, FuncInfo);

        if (isIndexSpillReg(SpillReg)) {
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_iPtr_Mem))
              .addReg(RealReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(0);
        } else if (RealReg == MC6809::AD) {
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Mem))
              .addReg(MC6809::AD, RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
        } else {
          // 8-bit A or B: load individual byte.
          BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i8_Mem))
              .addReg(RealReg, RegState::Define)
              .addFrameIndex(FI)
              .addImm(ByteOffset);
        }

        MO.setReg(RealReg);
      }

      // After the instruction: store back any DEFs to spill slots.
      auto After = std::next(MachineBasicBlock::iterator(MI));
      for (auto &[OpIdx, SpillReg] : SpillOps) {
        MachineOperand &MO = MI.getOperand(OpIdx);
        if (!MO.isDef())
          continue;
        Register RealReg = getRealReg(SpillReg);
        auto [FI, ByteOffset] = getSpillSlot(SpillReg, FuncInfo);
        if (isIndexSpillReg(SpillReg)) {
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
      } else {
        // Terminator (fused compare-branch): can't place D restore after it.
        // Insert the D restore at the start of each single-predecessor
        // successor that expects D to be live.
        if (DSaveSlot >= 0) {
          for (MachineBasicBlock *Succ : MBB.successors()) {
            if (Succ->pred_size() == 1) {
              BuildMI(*Succ, Succ->begin(), DL, TII.get(MC6809::Load_i16_Mem))
                  .addReg(MC6809::AD, RegState::Define)
                  .addFrameIndex(DSaveSlot)
                  .addImm(0);
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

  // Add frame pointer ($su) to liveins of all blocks that use it.
  // MaterializeSpills inserts Store/Load instructions with $su as base;
  // without the livein, the machine verifier flags it as undefined (bug #16).
  const MC6809FrameLowering *TFI =
      static_cast<const MC6809FrameLowering *>(MF.getSubtarget().getFrameLowering());
  if (TFI->hasFP(MF)) {
    Register FPReg = MC6809::SU;
    for (MachineBasicBlock &MBB : MF)
      if (!MBB.isLiveIn(FPReg))
        MBB.addLiveIn(FPReg);
  }

  return Changed;
}

char MC6809MaterializeSpills::ID = 0;

} // end anonymous namespace

INITIALIZE_PASS(MC6809MaterializeSpills, DEBUG_TYPE,
                "MC6809 Materialize Spill Registers", false, false)

MachineFunctionPass *llvm::createMC6809MaterializeSpillsPass() {
  return new MC6809MaterializeSpills();
}
