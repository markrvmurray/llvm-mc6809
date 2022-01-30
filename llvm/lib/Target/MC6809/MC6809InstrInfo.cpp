//===-- MC6809InstrInfo.cpp - MC6809 Instruction Information --------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstrInfo.h"

#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809RegisterInfo.h"

#include "MC6809Subtarget.h"
#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/SparseBitVector.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include "llvm/Target/TargetMachine.h"

using namespace llvm;

#define DEBUG_TYPE "mc6809-instrinfo"

#define GET_INSTRINFO_CTOR_DTOR
#include "MC6809GenInstrInfo.inc"

MC6809InstrInfo::MC6809InstrInfo()
    : MC6809GenInstrInfo(/*CFSetupOpcode=*/MC6809::ADJCALLSTACKDOWN,
                      /*CFDestroyOpcode=*/MC6809::ADJCALLSTACKUP) {}

bool MC6809InstrInfo::isReallyTriviallyReMaterializable(const MachineInstr &MI,
                                                     AAResults *AA) const {
  switch (MI.getOpcode()) {
  default:
    return false;
  case MC6809::LDImm16:
    return true;
  }
}

unsigned MC6809InstrInfo::isLoadFromStackSlot(const MachineInstr &MI,
                                           int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MC6809::LDAbs:
    if (!MI.getOperand(0).isFI())
      return 0;
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  case MC6809::LDStk:
    FrameIndex = MI.getOperand(2).getIndex();
    return MI.getOperand(0).getReg();
  }
}

unsigned MC6809InstrInfo::isStoreToStackSlot(const MachineInstr &MI,
                                          int &FrameIndex) const {
  switch (MI.getOpcode()) {
  default:
    return 0;
  case MC6809::STAbs:
    if (!MI.getOperand(0).isFI())
      return 0;
    FrameIndex = MI.getOperand(1).getIndex();
    return MI.getOperand(0).getReg();
  case MC6809::STStk:
    FrameIndex = MI.getOperand(2).getIndex();
    return MI.getOperand(1).getReg();
  }
}

void MC6809InstrInfo::reMaterialize(MachineBasicBlock &MBB,
                                 MachineBasicBlock::iterator I,
                                 Register DestReg, unsigned SubIdx,
                                 const MachineInstr &Orig,
                                 const TargetRegisterInfo &TRI) const {
  if (Orig.getOpcode() == MC6809::LDImm16) {
    MachineInstr *MI = MBB.getParent()->CloneMachineInstr(&Orig);
    MI->RemoveOperand(1);
    MI->substituteRegister(MI->getOperand(0).getReg(), DestReg, SubIdx, TRI);
    MI->setDesc(get(MC6809::LDImm16Remat));
    MBB.insert(I, MI);
  } else {
    TargetInstrInfo::reMaterialize(MBB, I, DestReg, SubIdx, Orig, TRI);
  }
}

// The main difficulty in commuting 6502 instructions is that their register
// classes aren't symmetric. This routine determines whether or not the operands
// of an instruction can be commuted anyway, potentially rewriting the register
// classes of virtual registers to do so.
MachineInstr *MC6809InstrInfo::commuteInstructionImpl(MachineInstr &MI, bool NewMI,
                                                   unsigned Idx1,
                                                   unsigned Idx2) const {
  // NOTE: This doesn't seem to actually be used anywhere.
  if (NewMI)
    report_fatal_error("NewMI is not supported");

  MachineFunction &MF = *MI.getMF();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  LLVM_DEBUG(dbgs() << "Commute: " << MI);

  // Determines the register class for a given virtual register constrained by a
  // target register class and all uses outside this instruction. This
  // effectively removes the constraints due to just this instruction, then
  // tries to apply the constraint for the other operand.
  const auto NewRegClass =
      [&](Register Reg,
          const TargetRegisterClass *RC) -> const TargetRegisterClass * {
    for (MachineOperand &MO : MRI.reg_nodbg_operands(Reg)) {
      MachineInstr *UseMI = MO.getParent();
      if (UseMI == &MI)
        continue;
      unsigned OpNo = &MO - &UseMI->getOperand(0);
      RC = UseMI->getRegClassConstraintEffect(OpNo, RC, this, &TRI);
      if (!RC)
        return nullptr;
    }
    return RC;
  };

  const TargetRegisterClass *RegClass1 =
      getRegClass(MI.getDesc(), Idx1, &TRI, MF);
  const TargetRegisterClass *RegClass2 =
      getRegClass(MI.getDesc(), Idx2, &TRI, MF);
  Register Reg1 = MI.getOperand(Idx1).getReg();
  Register Reg2 = MI.getOperand(Idx2).getReg();

  // See if swapping the two operands are possible given their register classes.
  const TargetRegisterClass *Reg1Class = nullptr;
  const TargetRegisterClass *Reg2Class = nullptr;
  if (Reg1.isVirtual()) {
    Reg1Class = NewRegClass(Reg1, RegClass2);
    if (!Reg1Class)
      return nullptr;
  }
  if (Reg1.isPhysical() && !RegClass2->contains(Reg1))
    return nullptr;
  if (Reg2.isVirtual()) {
    Reg2Class = NewRegClass(Reg2, RegClass1);
    if (!Reg2Class)
      return nullptr;
  }
  if (Reg2.isPhysical() && !RegClass1->contains(Reg2))
    return nullptr;

  // If this fails, make sure to get it out of the way before rewriting reg
  // classes.
  MachineInstr *CommutedMI =
      TargetInstrInfo::commuteInstructionImpl(MI, NewMI, Idx1, Idx2);
  if (!CommutedMI)
    return nullptr;

  // PHI nodes keep the register classes of all their arguments. By the time the
  // two address instruction pass occurs, these phis have already been lowered
  // to copies. Changing register classes here can make those register classes
  // mismatch the new ones; to avoid this, we recompute the register classes for
  // any vregs copied into or out of a commuted vreg.
  const auto RecomputeCopyRC = [&](Register Reg) {
    for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
      if (!MI.isCopy())
        continue;
      Register Other = MI.getOperand(0).getReg() == Reg
                           ? MI.getOperand(1).getReg()
                           : MI.getOperand(0).getReg();
      if (!Other.isVirtual())
        continue;
      MRI.recomputeRegClass(Other);
    }
  };

  // Use the new register classes computed above, if any.
  if (Reg1Class) {
    MRI.setRegClass(Reg1, Reg1Class);
    RecomputeCopyRC(Reg1);
  }
  if (Reg2Class) {
    MRI.setRegClass(Reg2, Reg2Class);
    RecomputeCopyRC(Reg2);
  }
  return CommutedMI;
}

unsigned MC6809InstrInfo::getInstSizeInBytes(const MachineInstr &MI) const {
  // Overestimate the size of each instruction to guarantee that any necessary
  // branches are relaxed.
  return 3;
}

// 6809 instructions aren't as regular as most commutable instructions, so this
// routine determines the commutable operands manually.
bool MC6809InstrInfo::findCommutedOpIndices(const MachineInstr &MI,
                                         unsigned &SrcOpIdx1,
                                         unsigned &SrcOpIdx2) const {
  assert(!MI.isBundle() &&
         "MC6809InstrInfo::findCommutedOpIndices() can't handle bundles");

  // XXXX: FIXME: MarkM - Find and commute the 6809 instructions
  return false;
}

bool MC6809InstrInfo::isBranchOffsetInRange(unsigned BranchOpc,
                                         int64_t BrOffset) const {
  switch (BranchOpc) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::GBR:
  case MC6809::BR:
  case MC6809::BRA:
    // BR range is [-128,127] starting from the PC location after the
    // instruction, which is two bytes after the start of the instruction.
    return -126 <= BrOffset && BrOffset <= 129;
  case MC6809::JMP:
    return true;
  }
}

MachineBasicBlock *
MC6809InstrInfo::getBranchDestBlock(const MachineInstr &MI) const {
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Bad branch opcode");
  case MC6809::GBR:
  case MC6809::BR:
  case MC6809::BRA:
  case MC6809::JMP:
    return MI.getOperand(0).getMBB();
  case MC6809::JMPIndir:
    return nullptr;
  }
}

bool MC6809InstrInfo::analyzeBranch(MachineBasicBlock &MBB,
                                 MachineBasicBlock *&TBB,
                                 MachineBasicBlock *&FBB,
                                 SmallVectorImpl<MachineOperand> &Cond,
                                 bool AllowModify) const {
  auto I = MBB.getFirstTerminator();

  // Advance past any comparison terminators.
  while (I != MBB.end() && I->isCompare())
    ++I;

  // If no terminators, falls through.
  if (I == MBB.end())
    return false;

  // Non-branch terminators cannot be analyzed.
  if (!I->isBranch())
    return true;

  // Analyze first branch.
  auto FirstBR = I++;
  if (FirstBR->isPreISelOpcode())
    return true;
  // First branch always forms true edge, whether conditional or unconditional.
  TBB = getBranchDestBlock(*FirstBR);
  if (!TBB)
    return true;
  if (FirstBR->isConditionalBranch()) {
    Cond.push_back(FirstBR->getOperand(1));
    Cond.push_back(FirstBR->getOperand(2));
  }

  // If there's no second branch, done.
  if (I == MBB.end())
    return false;

  // Cannot analyze branch followed by non-branch.
  if (!I->isBranch())
    return true;

  auto SecondBR = I++;

  // If any instructions follow the second branch, cannot analyze.
  if (I != MBB.end())
    return true;

  // Exactly two branches present.

  // Can only analyze conditional branch followed by unconditional branch.
  if (!SecondBR->isUnconditionalBranch() || SecondBR->isPreISelOpcode())
    return true;

  // Second unconditional branch forms false edge.
  FBB = getBranchDestBlock(*SecondBR);
  if (!FBB)
    return true;
  return false;
}

unsigned MC6809InstrInfo::removeBranch(MachineBasicBlock &MBB,
                                    int *BytesRemoved) const {
  // Since analyzeBranch succeeded, we know that the only terminators are
  // comparisons and branches.

  auto Begin = MBB.getFirstTerminator();
  auto End = MBB.end();

  // Advance to first branch.
  while (Begin != End && Begin->isCompare())
    ++Begin;

  // Erase all remaining terminators.
  unsigned NumRemoved = std::distance(Begin, End);
  if (BytesRemoved) {
    *BytesRemoved = 0;
    for (const auto &I : make_range(Begin, End))
      *BytesRemoved += getInstSizeInBytes(I);
  }
  MBB.erase(Begin, End);
  return NumRemoved;
}

unsigned MC6809InstrInfo::insertBranch(MachineBasicBlock &MBB,
                                    MachineBasicBlock *TBB,
                                    MachineBasicBlock *FBB,
                                    ArrayRef<MachineOperand> Cond,
                                    const DebugLoc &DL, int *BytesAdded) const {
  // Since analyzeBranch succeeded and any existing branches were removed, the
  // only remaining terminators are comparisons.

  const MC6809Subtarget &STI = MBB.getParent()->getSubtarget<MC6809Subtarget>();

  MachineIRBuilder Builder(MBB, MBB.end());
  unsigned NumAdded = 0;
  if (BytesAdded)
    *BytesAdded = 0;

  // Unconditional branch target.
  auto *UBB = TBB;

  // Conditional branch.
  if (!Cond.empty()) {
    assert(TBB);
    // The condition stores the arguments for the BR instruction.
    assert(Cond.size() == 2);

    // The unconditional branch will be to the false branch (if any).
    UBB = FBB;

    // Add conditional branch.
    Register Reg = Cond[0].getReg();
    unsigned Opcode = Reg.isVirtual() || !MC6809::CCFlagRegClass.contains(Reg)
                          ? MC6809::GBR
                          : MC6809::BR;
    auto BR = Builder.buildInstr(Opcode).addMBB(TBB);
    for (const MachineOperand &Op : Cond)
      BR.add(Op);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*BR);
  }

  // Add unconditional branch if necessary.
  if (UBB) {
    // For 6809, assume BRA and relax into LBRA in insertIndirectBranch if
    // necessary.
    auto JMP = Builder.buildInstr(MC6809::BRA).addMBB(UBB);
    ++NumAdded;
    if (BytesAdded)
      *BytesAdded += getInstSizeInBytes(*JMP);
  }

  return NumAdded;
}

void MC6809InstrInfo::insertIndirectBranch(MachineBasicBlock &MBB,
                                        MachineBasicBlock &NewDestBB,
                                        MachineBasicBlock &RestoreBB,
                                        const DebugLoc &DL, int64_t BrOffset,
                                        RegScavenger *RS) const {
  // This method inserts a *direct* branch (JMP), despite its name.
  // LLVM calls this method to fixup unconditional branches; it never calls
  // insertBranch or some hypothetical "insertDirectBranch".
  // See lib/CodeGen/BranchRelaxation.cpp for details.
  // We end up here when a jump is too long for a BRA instruction.

  MachineIRBuilder Builder(MBB, MBB.end());
  Builder.setDebugLoc(DL);

  Builder.buildInstr(MC6809::JMP).addMBB(&NewDestBB);
}

void MC6809InstrInfo::copyPhysReg(MachineBasicBlock &MBB,
                               MachineBasicBlock::iterator MI,
                               const DebugLoc &DL, MCRegister DestReg,
                               MCRegister SrcReg, bool KillSrc) const {
  MachineIRBuilder Builder(MBB, MI);
  copyPhysRegImpl(Builder, DestReg, SrcReg);
}

static Register createVReg(MachineIRBuilder &Builder,
                           const TargetRegisterClass &RC) {
  Builder.getMF().getProperties().reset(
      MachineFunctionProperties::Property::NoVRegs);
  return Builder.getMRI()->createVirtualRegister(&RC);
}

bool MC6809InstrInfo::shouldOverlapInterval(const MachineInstr &MI) const {
  return MI.getOpcode() != MC6809::CMPTermZ;
}

void MC6809InstrInfo::copyPhysRegImpl(MachineIRBuilder &Builder, Register DestReg,
                                   Register SrcReg) const {
  if (DestReg == SrcReg)
    return;

  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  const auto &IsClass = [&](Register Reg, const TargetRegisterClass &RC) {
    if (Reg.isPhysical() && !RC.contains(Reg))
      return false;
    if (Reg.isVirtual() &&
        !Builder.getMRI()->getRegClass(Reg)->hasSuperClassEq(&RC))
      return false;
    return true;
  };

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return IsClass(DestReg, Dest) && IsClass(SrcReg, Src);
  };

  if (AreClasses(MC6809::ACCRegClass, MC6809::ACCRegClass)) {
    if (IsClass(SrcReg, MC6809::ACC8RegClass)) {
      assert(MC6809::INDEX16RegClass.contains(DestReg));
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
    } else if (IsClass(DestReg, MC6809::ACC8RegClass)) {
      assert(MC6809::INDEX16RegClass.contains(SrcReg));
      Builder.buildInstr(MC6809::TFRp).addDef(DestReg).addUse(SrcReg);
    } else {
      Register Tmp = createVReg(Builder, MC6809::ACC8RegClass);
      copyPhysRegImpl(Builder, Tmp, SrcReg);
      copyPhysRegImpl(Builder, DestReg, Tmp);
    }
  } else if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MC6809::sub_lo_byte),
                    TRI.getSubReg(SrcReg, MC6809::sub_lo_byte));
    copyPhysRegImpl(Builder, TRI.getSubReg(DestReg, MC6809::sub_hi_byte),
                    TRI.getSubReg(SrcReg, MC6809::sub_hi_byte));
  } else if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    assert(SrcReg.isPhysical() && DestReg.isPhysical());
    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 =
        TRI.getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        const MachineInstr &MI = *Builder.getInsertPt();
        // MC6809 defines LSB writes to write the whole 8-bit register, not just
        // part of it.
        assert(!MI.readsRegister(DestReg));

        copyPhysRegImpl(Builder, DestReg, SrcReg);
      } else {
        if (DestReg == MC6809::C) {
          if (!MC6809::ACCRegClass.contains(SrcReg)) {
            Register Tmp = createVReg(Builder, MC6809::ACCRegClass);
            copyPhysRegImpl(Builder, Tmp, SrcReg);
            SrcReg = Tmp;
          }
          // C = SrcReg >= 1
          Builder.buildInstr(MC6809::CMPImm, {MC6809::C}, {SrcReg, INT64_C(1)});
        } else {
          assert(DestReg == MC6809::V);
          const TargetRegisterClass &StackRegClass = MC6809::ACCRegClass;

          if (StackRegClass.contains(SrcReg)) {
            Builder.buildInstr(MC6809::PH, {}, {SrcReg});
            Builder.buildInstr(MC6809::PL, {SrcReg}, {})
                .addDef(MC6809::NZVC, RegState::Implicit);
            Builder.buildInstr(MC6809::SelectImm, {MC6809::V},
                               {Register(MC6809::Z), INT64_C(0), INT64_C(-1)});
          } else {
            Register Tmp = createVReg(Builder, StackRegClass);
            copyPhysRegImpl(Builder, Tmp, SrcReg);
            std::prev(Builder.getInsertPt())
                ->addOperand(MachineOperand::CreateReg(MC6809::NZVC,
                                                       /*isDef=*/true,
                                                       /*isImp=*/true));
            Builder.buildInstr(MC6809::SelectImm, {MC6809::V},
                               {Register(MC6809::Z), INT64_C(0), INT64_C(-1)});
          }
        }
      }
    } else {
      if (DestReg8) {
        DestReg = DestReg8;

        Register Tmp = DestReg;
        if (!MC6809::ACCRegClass.contains(Tmp))
          Tmp = createVReg(Builder, MC6809::ACCRegClass);
        Builder.buildInstr(MC6809::SelectImm, {Tmp},
                           {SrcReg, INT64_C(1), INT64_C(0)});
        if (Tmp != DestReg)
          copyPhysRegImpl(Builder, DestReg, Tmp);
      } else {
        Builder.buildInstr(MC6809::SelectImm, {DestReg},
                           {SrcReg, INT64_C(-1), INT64_C(0)});
      }
    }
  } else
    llvm_unreachable("Unexpected physical register copy.");
}

const TargetRegisterClass *MC6809InstrInfo::canFoldCopy(const MachineInstr &MI,
                                                     unsigned FoldIdx) const {
  if (!MI.getMF()->getFunction().doesNotRecurse())
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);

  Register FoldReg = MI.getOperand(FoldIdx).getReg();
  if (MC6809::ACCRegClass.contains(FoldReg) ||
      MC6809::BIT1RegClass.contains(FoldReg))
    return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  if (FoldReg.isVirtual()) {
    const auto *RC = MI.getMF()->getRegInfo().getRegClass(FoldReg);
    if (RC == &MC6809::ACCRegClass || RC == &MC6809::BIT1RegClass)
      return TargetInstrInfo::canFoldCopy(MI, FoldIdx);
  }
  return nullptr;
}

void MC6809InstrInfo::storeRegToStackSlot(MachineBasicBlock &MBB,
                                       MachineBasicBlock::iterator MI,
                                       Register SrcReg, bool isKill,
                                       int FrameIndex,
                                       const TargetRegisterClass *RC,
                                       const TargetRegisterInfo *TRI) const {
  loadStoreRegStackSlot(MBB, MI, SrcReg, isKill, FrameIndex, RC, TRI,
                        /*IsLoad=*/false);
}

void MC6809InstrInfo::loadRegFromStackSlot(MachineBasicBlock &MBB,
                                        MachineBasicBlock::iterator MI,
                                        Register DestReg, int FrameIndex,
                                        const TargetRegisterClass *RC,
                                        const TargetRegisterInfo *TRI) const {
  loadStoreRegStackSlot(MBB, MI, DestReg, false, FrameIndex, RC, TRI,
                        /*IsLoad=*/true);
}

// Load or store one byte from/to a location on the static stack.
static void loadStoreByteStaticStackSlot(MachineIRBuilder &Builder,
                                         MachineOperand MO, int FrameIndex,
                                         int64_t Offset,
                                         MachineMemOperand *MMO) {
  const MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Reg = MO.getReg();

  // Convert bit to byte if directly possible.
  if (Reg.isPhysical() && MC6809::BIT1RegClass.contains(Reg)) {
    Reg = TRI.getMatchingSuperReg(Reg, MC6809::sub_lsb, &MC6809::ACCRegClass);
    MO.setReg(Reg);
  } else if (Reg.isVirtual() &&
             MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACCRegClass) &&
             MO.getSubReg() == MC6809::sub_lsb) {
    MO.setSubReg(0);
  }

  // Emit directly through ACC if possible.
  if ((Reg.isPhysical() && MC6809::ACCRegClass.contains(Reg)) ||
      (Reg.isVirtual() &&
       MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACCRegClass) &&
       !MO.getSubReg())) {
    Builder.buildInstr(MO.isDef() ? MC6809::LDAbs : MC6809::STAbs)
        .add(MO)
        .addFrameIndex(FrameIndex, Offset)
        .addMemOperand(MMO);
    return;
  }

  // Emit via copy through ACC.
  bool IsBit = (Reg.isPhysical() && MC6809::BIT1RegClass.contains(Reg)) ||
               (Reg.isVirtual() &&
                (MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::BIT1RegClass) ||
                 MO.getSubReg() == MC6809::sub_lsb));
  MachineOperand Tmp = MachineOperand::CreateReg(
      Builder.getMRI()->createVirtualRegister(&MC6809::ACCRegClass), MO.isDef());
  if (Tmp.isUse()) {
    // Define the temporary register via copy from the MO.
    MachineOperand TmpDef = Tmp;
    TmpDef.setIsDef();
    if (IsBit) {
      TmpDef.setSubReg(MC6809::sub_lsb);
      TmpDef.setIsUndef();
    }
    Builder.buildInstr(MC6809::COPY).add(TmpDef).add(MO);

    loadStoreByteStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);
  } else {
    assert(Tmp.isDef());

    loadStoreByteStaticStackSlot(Builder, Tmp, FrameIndex, Offset, MMO);

    // Define the MO via copy from the temporary register.
    MachineOperand TmpUse = Tmp;
    TmpUse.setIsUse();
    if (IsBit)
      TmpUse.setSubReg(MC6809::sub_lsb);
    Builder.buildInstr(MC6809::COPY).add(MO).add(TmpUse);
  }
}

void MC6809InstrInfo::loadStoreRegStackSlot(
    MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg,
    bool IsKill, int FrameIndex, const TargetRegisterClass *RC,
    const TargetRegisterInfo *TRI, bool IsLoad) const {
  MachineFunction &MF = *MBB.getParent();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MachineRegisterInfo &MRI = MF.getRegInfo();

  MachinePointerInfo PtrInfo =
      MachinePointerInfo::getFixedStack(MF, FrameIndex);
  MachineMemOperand *MMO = MF.getMachineMemOperand(
      PtrInfo, IsLoad ? MachineMemOperand::MOLoad : MachineMemOperand::MOStore,
      MFI.getObjectSize(FrameIndex), MFI.getObjectAlign(FrameIndex));

  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);

  // If we're using the soft stack, since the offset is not yet known, it may be
  // either 8 or 16 bits. Emit a 16-bit pseudo to be lowered during frame index
  // elimination.
  if (!MF.getFunction().doesNotRecurse()) {
    Register Ptr = MRI.createVirtualRegister(&MC6809::ACC16RegClass);
    auto Instr = Builder.buildInstr(IsLoad ? MC6809::LDStk : MC6809::STStk);
    if (!IsLoad)
      Instr.addDef(Ptr, RegState::EarlyClobber);
    Instr.addReg(Reg, getDefRegState(IsLoad) | getKillRegState(IsKill));
    if (IsLoad)
      Instr.addDef(Ptr, RegState::EarlyClobber);
    Instr.addFrameIndex(FrameIndex).addImm(0).addMemOperand(MMO);
  } else {
    if ((Reg.isPhysical() && MC6809::ACC16RegClass.contains(Reg)) ||
        (Reg.isVirtual() &&
         MRI.getRegClass(Reg)->hasSuperClassEq(&MC6809::ACC16RegClass))) {
      MachineOperand Lo = MachineOperand::CreateReg(Reg, IsLoad);
      MachineOperand Hi = Lo;
      Register Tmp = Reg;
      if (Reg.isPhysical()) {
        Lo.setReg(TRI->getSubReg(Reg, MC6809::sub_lo_byte));
        Hi.setReg(TRI->getSubReg(Reg, MC6809::sub_hi_byte));
      } else {
        assert(Reg.isVirtual());
        // Live intervals for the original virtual register will already have
        // been computed by this point. Since this code introduces subregisters,
        // these must be using a new virtual register; otherwise there would be
        // no subregister live ranges for the new instructions. This can cause
        // VirtRegMap to fail.
        Tmp = MRI.createVirtualRegister(&MC6809::ACC16RegClass);
        Lo.setReg(Tmp);
        Lo.setSubReg(MC6809::sub_lo_byte);
        if (Lo.isDef())
          Lo.setIsUndef();
        Hi.setReg(Tmp);
        Hi.setSubReg(MC6809::sub_hi_byte);
      }
      if (!IsLoad) {
        if (Tmp != Reg)
          Builder.buildCopy(Tmp, Reg);

        // The register may not have been fully defined at this point. Adding a
        // KILL here makes the entire value alive, regardless of whether or not
        // it was prior to the store. We do this because this function does not
        // have access to the detailed liveness information about the virtual
        // register in use; if we did, we'd only need to store the portion of
        // the virtual register that is actually alive.
        Builder.buildInstr(MC6809::KILL, {Tmp}, {Tmp});
      }
      loadStoreByteStaticStackSlot(Builder, Lo, FrameIndex, 0,
                                   MF.getMachineMemOperand(MMO, 0, 1));
      loadStoreByteStaticStackSlot(Builder, Hi, FrameIndex, 1,
                                   MF.getMachineMemOperand(MMO, 1, 1));
      if (IsLoad && Tmp != Reg)
        Builder.buildCopy(Reg, Tmp);
    } else {
      loadStoreByteStaticStackSlot(
          Builder, MachineOperand::CreateReg(Reg, IsLoad), FrameIndex, 0, MMO);
    }
  }

  LLVM_DEBUG({
    dbgs() << "Inserted stack slot load/store:\n";
    for (const auto &MI : make_range(MIS.begin(), MIS.getInitial()))
      dbgs() << MI;
  });
}

bool MC6809InstrInfo::expandPostRAPseudo(MachineInstr &MI) const {
  MachineIRBuilder Builder(MI);

  bool Changed = true;
  switch (MI.getOpcode()) {
  default:
    Changed = false;
    break;
  // Post RA
  case MC6809::INC:
  case MC6809::DEC:
    expandIncDec(Builder);
    break;
  case MC6809::LDAbsIdx:
    expandLDIdx(Builder);
    break;
  case MC6809::LDImm1:
    expandLDImm1(Builder);
    break;
  case MC6809::LDImm16:
    expandLDImm16(Builder);
    break;
  case MC6809::LDImm16Remat:
    expandLDImm16Remat(Builder);
    break;
  case MC6809::LDZ:
    expandLDZ(Builder);
    break;
  case MC6809::CMPNZVCImm:
  case MC6809::CMPNZVCAbs:
  case MC6809::CMPNZVCAbsIdx:
  case MC6809::CMPNZVCIndirIdx:
  case MC6809::SBCNZVCImm:
  case MC6809::SBCNZVCAbs:
  case MC6809::SBCNZVCAbsIdx:
  case MC6809::SBCNZVCIndirIdx:
    expandNZ(Builder);
    break;
  case MC6809::CMPTermImm:
  case MC6809::CMPTermAbs:
  case MC6809::CMPTermIndir:
  case MC6809::CMPTermIdx:
    expandCMPTerm(Builder);
    break;

  // Control flow
  case MC6809::GBR:
    expandGBR(Builder);
    break;
  }

  return Changed;
}

//===---------------------------------------------------------------------===//
// Post RA pseudos
//===---------------------------------------------------------------------===//

void MC6809InstrInfo::expandLDIdx(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();

  // This occur when X or Y is both the destination and index register.
  // Since the 6502 has no instruction for this, use A as the destination
  // instead, then transfer to the real destination.
  if (MI.getOperand(0).getReg() == MI.getOperand(2).getReg()) {
    Register Tmp = createVReg(Builder, MC6809::ACC8RegClass);
    Builder.buildInstr(MC6809::LDAAbsIdx)
        .addDef(Tmp)
        .add(MI.getOperand(1))
        .add(MI.getOperand(2));
    Builder.buildInstr(MC6809::TFRp).add(MI.getOperand(0)).addUse(Tmp);
    MI.eraseFromParent();
    return;
  }

  unsigned Opcode;
  switch (MI.getOperand(0).getReg()) {
  default:
    llvm_unreachable("Bad destination for LDAbsIdx.");
  case MC6809::AA:
    Opcode = MC6809::LDAAbsIdx;
    break;
  case MC6809::IX:
    Opcode = MC6809::LDXAbsIdx;
    break;
  case MC6809::IY:
    Opcode = MC6809::LDYAbsIdx;
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
}

void MC6809InstrInfo::expandLDImm1(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();
  int64_t Val = MI.getOperand(1).getImm();

  unsigned Opcode;
  switch (DestReg) {
  default: {
    DestReg =
        Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(
            DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    assert(DestReg && "Unexpected destination for LDImm1");
    assert(MC6809::ACCRegClass.contains(DestReg));
    Opcode = MC6809::LDImm;
    MI.getOperand(0).setReg(DestReg);
    MI.getOperand(1).setImm(!!Val);
    break;
  }
  case MC6809::N:
    Opcode = MC6809::LDNImm;
    break;
  case MC6809::Z:
    Opcode = MC6809::LDZImm;
    break;
  case MC6809::V:
    Opcode = MC6809::LDVImm;
    break;
  case MC6809::C:
    Opcode = MC6809::LDCImm;
    break;
  }

  MI.setDesc(Builder.getTII().get(Opcode));
}

void MC6809InstrInfo::expandLDImm16(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  Register Dst = MI.getOperand(0).getReg();
  Register Tmp = MI.getOperand(1).getReg();
  MachineOperand Src = MI.getOperand(2);

  auto Lo = Builder.buildInstr(MC6809::LDImm, {Tmp}, {});
  if (Src.isImm()) {
    Lo.addImm(Src.getImm() & 0xff);
  } else {
    Lo.add(Src);
    Lo->getOperand(1).setTargetFlags(MC6809::MO_LO);
  }
  copyPhysRegImpl(Builder, TRI.getSubReg(Dst, MC6809::sub_lo_byte), Tmp);

  auto Hi = Builder.buildInstr(MC6809::LDImm, {Tmp}, {});
  if (Src.isImm()) {
    Hi.addImm(Src.getImm() >> 8);
  } else {
    Hi.add(Src);
    Hi->getOperand(1).setTargetFlags(MC6809::MO_HI);
  }
  // Appease the register scavenger by making this appear to be a redefinition.
  if (Tmp.isVirtual())
    Hi.addUse(Tmp, RegState::Implicit);
  copyPhysRegImpl(Builder, TRI.getSubReg(Dst, MC6809::sub_hi_byte), Tmp);

  MI.eraseFromParent();
}

void MC6809InstrInfo::expandLDImm16Remat(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  Register Scratch = createVReg(Builder, MC6809::ACCRegClass);
  auto Ld = Builder.buildInstr(MC6809::LDImm16, {MI.getOperand(0), Scratch}, {})
                .add(MI.getOperand(1));
  MI.eraseFromParent();
  Builder.setInsertPt(*Ld->getParent(), &*Ld);
  expandLDImm16(Builder);
}

void MC6809InstrInfo::expandLDZ(MachineIRBuilder &Builder) const {
  auto &MI = *Builder.getInsertPt();
  Register DestReg = MI.getOperand(0).getReg();

  if (MC6809::ACCRegClass.contains(DestReg)) {
    MI.setDesc(Builder.getTII().get(MC6809::LDImm));
    MI.addOperand(MachineOperand::CreateImm(0));
  } else {
    llvm_unreachable("Unexpected register class for LDZ.");
  }
}

void MC6809InstrInfo::expandIncDec(MachineIRBuilder &Builder) const {
  const auto &TII = Builder.getTII();

  auto &MI = *Builder.getInsertPt();
  Register R = MI.getOperand(0).getReg();
  bool IsInc = MI.getOpcode() == MC6809::INC;
  assert(IsInc || MI.getOpcode() == MC6809::DEC);

  switch (R) {
  case MC6809::AA:
  case MC6809::AB:
  case MC6809::AD:
  case MC6809::AE:
  case MC6809::AF:
  case MC6809::AW: {
    Register CC = createVReg(Builder, MC6809::CCFlagRegClass);
    Builder.buildInstr(MC6809::LDCImm)
        .addDef(CC, RegState::Undef, MC6809::sub_carry)
        .addImm(0);
    Builder.buildInstr(MC6809::ADCImm)
        .addDef(R)
        .addDef(CC, RegState::Dead, MC6809::sub_carry)
        .addDef(CC, RegState::Dead, MC6809::sub_overflow)
        .addUse(R, RegState::Kill)
        .addImm(IsInc ? 1 : 255)
        .addUse(CC, 0, MC6809::sub_carry);
    MI.eraseFromParent();
    break;
  }
#if 0
  // XXXX: FIXME: MarkM - use LEAX
  case MC6809::IX:
  case MC6809::IY:
  case MC6809::SU:
  case MC6809::SS:
    MI.setDesc(TII.get(IsInc ? MC6809::IN : MC6809::DE));
    break;
#endif
  default:
    llvm_unreachable("Unexpected register class for INC/DEC.");
    break;
  }
}

//===---------------------------------------------------------------------===//
// NZVC pseudos
//===---------------------------------------------------------------------===//

void MC6809InstrInfo::expandNZ(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  Register N;
  Register Z;
  switch (MI.getOpcode()) {
  case MC6809::CMPNZVCImm:
  case MC6809::CMPNZVCAbs:
  case MC6809::CMPNZVCAbsIdx:
  case MC6809::CMPNZVCIndirIdx:
    N = MI.getOperand(1).getReg();
    Z = MI.getOperand(2).getReg();
    break;
  case MC6809::SBCNZVCImm:
  case MC6809::SBCNZVCAbs:
  case MC6809::SBCNZVCAbsIdx:
  case MC6809::SBCNZVCIndirIdx:
    N = MI.getOperand(2).getReg();
    Z = MI.getOperand(4).getReg();
    break;
  }

  // The NZVC location to which to copy.
  Register NZOut = MC6809::NoRegister;
  // Which of N or Z to copy.
  Register NZIn = MC6809::NoRegister;
  if (N) {
    assert(!Z);
    NZOut = N;
    NZIn = MC6809::N;
  } else if (Z) {
    NZOut = Z;
    NZIn = MC6809::Z;
  }

  MachineInstrBuilder Op;
  switch (MI.getOpcode()) {
  case MC6809::CMPNZVCImm:
  case MC6809::CMPNZVCAbs:
  case MC6809::CMPNZVCAbsIdx:
  case MC6809::CMPNZVCIndirIdx: {
    unsigned Opcode;
    switch (MI.getOpcode()) {
    case MC6809::CMPNZVCImm:
      Opcode = MC6809::CMPImm;
      break;
    case MC6809::CMPNZVCAbs:
      Opcode = MC6809::CMPAbs;
      break;
    case MC6809::CMPNZVCAbsIdx:
      Opcode = MC6809::CMPAbsIdx;
      break;
    case MC6809::CMPNZVCIndirIdx:
      Opcode = MC6809::CMPIndirIdx;
      break;
    }
    Op = Builder.buildInstr(Opcode, {MI.getOperand(0)}, {});
    for (int Idx : seq(3u, MI.getNumOperands()))
      Op.add(MI.getOperand(Idx));
    break;
  }
  case MC6809::SBCNZVCImm:
  case MC6809::SBCNZVCAbs:
  case MC6809::SBCNZVCAbsIdx:
  case MC6809::SBCNZVCIndirIdx: {
    unsigned Opcode;
    switch (MI.getOpcode()) {
    case MC6809::SBCNZVCImm:
      Opcode = MC6809::SBCImm;
      break;
    case MC6809::SBCNZVCAbs:
      Opcode = MC6809::SBCAbs;
      break;
    case MC6809::SBCNZVCAbsIdx:
      Opcode = MC6809::SBCAbsIdx;
      break;
    case MC6809::SBCNZVCIndirIdx:
      Opcode = MC6809::SBCIndirIdx;
      break;
    }
    Op = Builder.buildInstr(
        Opcode, {MI.getOperand(0), MI.getOperand(1), MI.getOperand(3)}, {});
    for (int Idx : seq(5u, MI.getNumOperands()))
      Op.add(MI.getOperand(Idx));
    break;
  }
  }

  // Copy out N or Z to a BIT1 location if requested.
  if (NZOut) {
    assert(NZIn);
    Op.addDef(MC6809::NZVC, RegState::Implicit);
    Builder.buildInstr(MC6809::SelectImm, {NZOut},
                       {NZIn, INT64_C(-1), INT64_C(0)});
  }
  MI.eraseFromParent();
}

void MC6809InstrInfo::expandCMPTerm(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();
  switch (MI.getOpcode()) {
  case MC6809::CMPTermImm:
    MI.setDesc(Builder.getTII().get(MC6809::CMPImm));
    break;
  case MC6809::CMPTermAbs:
    MI.setDesc(Builder.getTII().get(MC6809::CMPAbs));
    break;
  case MC6809::CMPTermIdx:
    MI.setDesc(Builder.getTII().get(MC6809::CMPAbsIdx));
    break;
  case MC6809::CMPTermIndir:
    MI.setDesc(Builder.getTII().get(MC6809::CMPIndirIdx));
    break;
  }
}

//===---------------------------------------------------------------------===//
// Control flow pseudos
//===---------------------------------------------------------------------===//

void MC6809InstrInfo::expandGBR(MachineIRBuilder &Builder) const {
  MachineInstr &MI = *Builder.getInsertPt();

  MI.setDesc(Builder.getTII().get(MC6809::BR));

  Register Tst = MI.getOperand(1).getReg();
  switch (Tst) {
  case MC6809::C:
  case MC6809::V:
    return;
  default: {
    Register TstReg =
        Builder.getMF().getSubtarget().getRegisterInfo()->getMatchingSuperReg(
            Tst, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Builder.buildInstr(MC6809::CMPTermZ, {MC6809::C}, {TstReg})
        ->getOperand(0)
        .setIsDead();
  }
  }
  // Branch on zero flag, which is now the inverse of the test.
  MI.getOperand(1).setReg(MC6809::Z);
  MI.getOperand(2).setImm(MI.getOperand(2).getImm() ? 0 : 1);
}

bool MC6809InstrInfo::reverseBranchCondition(
    SmallVectorImpl<MachineOperand> &Cond) const {
  assert(Cond.size() == 2);
  auto &Val = Cond[1];
  Val.setImm(!Val.getImm());
  // Success.
  return false;
}

std::pair<unsigned, unsigned>
MC6809InstrInfo::decomposeMachineOperandsTargetFlags(unsigned TF) const {
  return std::make_pair(TF, 0u);
}

ArrayRef<std::pair<int, const char *>>
MC6809InstrInfo::getSerializableTargetIndices() const {
  static const std::pair<int, const char *> Flags[] = {
      {MC6809::TI_STATIC_STACK, "mc6809-static-stack"}};
  return Flags;
}

ArrayRef<std::pair<unsigned, const char *>>
MC6809InstrInfo::getSerializableDirectMachineOperandTargetFlags() const {
  static const std::pair<unsigned, const char *> Flags[] = {
      {MC6809::MO_LO, "lo"}, {MC6809::MO_HI, "hi"}, {MC6809::MO_HI_JT, "hi-jt"}};
  return Flags;
}
