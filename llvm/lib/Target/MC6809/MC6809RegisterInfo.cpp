//===-- MC6809RegisterInfo.cpp - MC6809 Register Information --------------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "MC6809RegisterInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "MC6809.h"
#include "MC6809FrameLowering.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MC6809GenRegisterInfo.inc"

using namespace llvm;

MC6809RegisterInfo::MC6809RegisterInfo()
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                         /*PC=*/0, /*HwMode=*/0) {
}

BitVector
 MC6809RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());
  const MC6809FrameLowering *TFI = getFrameLowering(MF);

  // Mark special registers as reserved.
  Reserved.set(MC6809::PC);
  Reserved.set(MC6809::SS);
  Reserved.set(MC6809::DP);
  Reserved.set(MC6809::CC);
  Reserved.set(MC6809::A0);

  // Mark frame pointer as reserved if needed.
  if (TFI->hasFP(MF))
    Reserved.set(MC6809::SU);

  return Reserved;
}

const MCPhysReg *
MC6809RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  const MC6809FrameLowering &TFI = *getFrameLowering(*MF);
  return TFI.isISR(*MF) ? MC6809_CSR_SaveList : MC6809_CSR_SaveList;
}

const uint32_t *
MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CallingConv) const {
  return MC6809_CSR_RegMask;
}

const TargetRegisterClass *
MC6809RegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &) const {
  if (RC->hasSuperClass(&MC6809::BIT1RegClass))
    return &MC6809::BIT1RegClass;
  if (RC->hasSuperClass(&MC6809::BIT8RegClass))
    return &MC6809::ACC8RegClass;
  return RC;
}

const TargetRegisterClass *
MC6809RegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MC6809::INDEX16RegClass)
    return &MC6809::ACC16RegClass;
  return RC;
}

// These values were chosen empirically based on the desired behavior of llc
// test cases. These values will likely need to be retuned as more examples come
// up.  Unfortunately, the way the register allocator actually uses this is very
// heuristic, and if tuning these params doesn't suffice, we'll need to build a
// more sophisticated analysis into the register allocator.
unsigned MC6809RegisterInfo::getCSRFirstUseCost(const MachineFunction &MF) const {
  if (MF.getFunction().doesNotRecurse()) {
    return 15 * 16384 / 10;
  }
  return 5 * 16384 / 10;
}

bool MC6809RegisterInfo::saveScavengerRegister(MachineBasicBlock &MBB,
                                            MachineBasicBlock::iterator I,
                                            MachineBasicBlock::iterator &UseMI,
                                            const TargetRegisterClass *RC,
                                            Register Reg) const {

  // Note: NZ cannot be live at this point, since it's only live in terminators,
  // and virtual registers are never inserted into terminators.

  // Consider the regions in a basic block where a physical register is live.
  // The register scavenger will select one of these regions to spill and mark
  // the physical register as available within that region. Such a region cannot
  // contain any calls, since the physical registers are clobbered by calls.
  // This means that a save/restore pair for that physical register cannot
  // overlap with any other save/restore pair for the same physical register.

  MachineIRBuilder Builder(MBB, I);
  const MC6809Subtarget &STI = Builder.getMF().getSubtarget<MC6809Subtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  switch (Reg) {
  default:
    errs() << "Register: " << getName(Reg) << "\n";
    report_fatal_error("Scavenger spill for register not yet implemented.");
  case MC6809::CC:
  case MC6809::AA:
  case MC6809::AB:
  case MC6809::AD:
  case MC6809::IX:
  case MC6809::IY:
  case MC6809::SU: {
    Builder.buildInstr(MC6809::PH, {}, {Reg});
    Builder.setInsertPt(MBB, UseMI);
    Builder.buildInstr(MC6809::PL, {Reg}, {});
    break;
  }
  }

  return true;
}

bool MC6809RegisterInfo::canSaveScavengerRegister(Register Reg) const {
  return true;
}

void MC6809RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator MI,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  MachineFunction &MF = *MI->getMF();
  const MachineFrameInfo &MFI = MF.getFrameInfo();

  assert(!SPAdj);

  int Idx = MI->getOperand(FIOperandNum).getIndex();
  int64_t Offset = MFI.getObjectOffset(Idx);
  if (FIOperandNum + 1 < MI->getNumOperands() &&
      MI->getOperand(FIOperandNum + 1).isImm())
    Offset += MI->getOperand(FIOperandNum + 1).getImm();
  else
    Offset += MI->getOperand(FIOperandNum).getOffset();

  if (MFI.getStackID(Idx) == TargetStackID::Default) {
    // All offsets are relative to the incoming SP
    // 1) Addr = Offset_SP + SP
    //
    // However, the incoming SP isn't available throughout the function; only
    // the frame pointer is. So we need to obtain the FP relative offset such
    // that:
    // 2) Addr = Offset_FP + FP
    //
    // Susbtituting (2) into (1) gives:
    // 3) Offset_FP = Offset_SP + SP - FP
    //
    // The frame pointer is:
    // 4) FP = SP - Stack_Size
    //
    // Substituting (4) into (3) gives:
    // 5) Offset_FP = Offset_SP + Stack_Size
    Offset += MFI.getStackSize();
  }

  switch (MI->getOpcode()) {
  default:
    MI->getOperand(FIOperandNum)
        .ChangeToTargetIndex(MC6809::TI_STATIC_STACK, Offset,
                             MI->getOperand(FIOperandNum).getTargetFlags());
    break;
  case MC6809::AddrLostk:
  case MC6809::AddrHistk:
  case MC6809::LDStk:
  case MC6809::STStk:
    MI->getOperand(FIOperandNum)
        .ChangeToRegister(getFrameRegister(MF), /*isDef=*/false);
    MI->getOperand(FIOperandNum + 1).setImm(Offset);
    break;
  }

  switch (MI->getOpcode()) {
  default:
    break;
  case MC6809::AddrLostk:
    expandAddrLostk(MI);
    break;
  case MC6809::AddrHistk:
    expandAddrHistk(MI);
    break;
  case MC6809::LDStk:
  case MC6809::STStk:
    expandLDSTStk(MI);
    break;
  }
}

void MC6809RegisterInfo::expandAddrLostk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI->getParent(), MI);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  const MachineOperand &Dst = MI->getOperand(0);
  Register Base = MI->getOperand(3).getReg();
  const MachineOperand &CDef = MI->getOperand(1);
  const MachineOperand &VDef = MI->getOperand(2);

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);
  Offset &= 0xFF;

  Register Src = TRI.getSubReg(Base, MC6809::sub_lo_byte);

  auto LDC = Builder.buildInstr(MC6809::LDCImm).add(CDef).addImm(0);
  if (LDC->getOperand(0).getSubReg())
    LDC->getOperand(0).setIsUndef();

  if (!Offset)
    Builder.buildInstr(MC6809::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MC6809::ACC8RegClass, Src).getReg(0);
    auto Instr = Builder.buildInstr(MC6809::ADCImm)
                     .addDef(A)
                     .add(CDef)
                     .add(VDef)
                     .addUse(A)
                     .addImm(Offset)
                     .addUse(CDef.getReg(), 0, CDef.getSubReg());
    Instr->getOperand(2).setIsDead();
    Builder.buildInstr(MC6809::COPY).add(Dst).addUse(A);
  }

  MI->eraseFromParent();
}

void MC6809RegisterInfo::expandAddrHistk(MachineBasicBlock::iterator MI) const {
  MachineIRBuilder Builder(*MI->getParent(), MI);
  const TargetRegisterInfo &TRI =
      *Builder.getMF().getSubtarget().getRegisterInfo();

  MachineOperand Dst = MI->getOperand(0);
  MachineOperand CDef = MI->getOperand(1);
  MachineOperand VDef = MI->getOperand(2);
  Register Base = MI->getOperand(3).getReg();

  int64_t OffsetImm = MI->getOperand(4).getImm();
  assert(0 <= OffsetImm && OffsetImm < 65536);
  auto Offset = static_cast<uint16_t>(OffsetImm);

  MachineOperand CUse = MI->getOperand(5);

  Register Src = TRI.getSubReg(Base, MC6809::sub_hi_byte);

  // Note: We can only elide the high byte of the address into a copy if the
  // whole offset is zero. There may be a carry from the low byte sum if only
  // the high byte is zero.
  if (!Offset)
    Builder.buildInstr(MC6809::COPY).add(Dst).addUse(Src);
  else {
    Register A = Builder.buildCopy(&MC6809::ACC8RegClass, Src).getReg(0);
    auto Instr = Builder.buildInstr(MC6809::ADCImm)
                     .addDef(A)
                     .add(CDef)
                     .add(VDef)
                     .addUse(A)
                     .addImm(Offset >> 8)
                     .add(CUse);
    Instr->getOperand(1).setIsDead();
    Instr->getOperand(2).setIsDead();
    Builder.buildInstr(MC6809::COPY).add(Dst).addUse(A);
  }

  MI->eraseFromParent();
}

void MC6809RegisterInfo::expandLDSTStk(MachineBasicBlock::iterator MI) const {
  MachineFunction &MF = *MI->getMF();
  MachineIRBuilder Builder(*MI->getParent(), MI);
  MachineRegisterInfo &MRI = *Builder.getMRI();
  const TargetRegisterInfo &TRI = *MRI.getTargetRegisterInfo();

  const bool IsLoad = MI->getOpcode() == MC6809::LDStk;

  Register Loc =
      IsLoad ? MI->getOperand(0).getReg() : MI->getOperand(1).getReg();
  int64_t Offset = MI->getOperand(3).getImm();

  if (Offset >= 256) {
    Register CC = MRI.createVirtualRegister(&MC6809::CCFlagRegClass);
    // Far stack accesses need a virtual base register, so materialize one here
    // using the pointer provided.
    Register NewBase =
        IsLoad ? MI->getOperand(1).getReg() : MI->getOperand(0).getReg();
    // We can't scavenge a 16-bit register, so this can't be virtual here (after
    // register allocation).
    assert(!NewBase.isVirtual() && "LDSTStk must not use a virtual base "
                                   "pointer after register allocation.");

    auto Lo = Builder.buildInstr(MC6809::AddrLostk)
                  .addDef(TRI.getSubReg(NewBase, MC6809::sub_lo_byte))
                  .addDef(CC, /*Flags=*/0, MC6809::sub_carry)
                  .addDef(CC, RegState::Dead, MC6809::sub_overflow)
                  .add(MI->getOperand(2))
                  .add(MI->getOperand(3));
    auto Hi = Builder.buildInstr(MC6809::AddrHistk)
                  .addDef(TRI.getSubReg(NewBase, MC6809::sub_hi_byte))
                  .addDef(CC, RegState::Dead, MC6809::sub_carry)
                  .addDef(CC, RegState::Dead, MC6809::sub_overflow)
                  .add(MI->getOperand(2))
                  .add(MI->getOperand(3))
                  .addUse(CC, /*Flags=*/0, MC6809::sub_carry)
                  .addUse(NewBase, RegState::Implicit);
    MI->getOperand(2).setReg(NewBase);
    MI->getOperand(3).setImm(0);

    expandAddrLostk(Lo);
    expandAddrHistk(Hi);
    expandLDSTStk(MI);
    return;
  }

  if (MC6809::ACC16RegClass.contains(Loc)) {
    if (!IsLoad) {
      // Loc may not be fully alive at this point, which would create uses of
      // undefined subregisters. Issuing a KILL here redefines the full 16-bit
      // register, making both halves alive, regardless of which parts of the
      // register were alive before.
      Builder.buildInstr(MC6809::KILL, {Loc}, {Loc});
    }
    Register Lo = TRI.getSubReg(Loc, MC6809::sub_lo_byte);
    Register Hi = TRI.getSubReg(Loc, MC6809::sub_hi_byte);
    auto LoInstr = Builder.buildInstr(MI->getOpcode());
    if (!IsLoad)
      LoInstr.add(MI->getOperand(0));
    LoInstr.addReg(Lo, getDefRegState(IsLoad));
    if (IsLoad)
      LoInstr.add(MI->getOperand(1));
    LoInstr.add(MI->getOperand(2))
        .add(MI->getOperand(3))
        .addMemOperand(MF.getMachineMemOperand(*MI->memoperands_begin(), 0, 1));
    auto HiInstr = Builder.buildInstr(MI->getOpcode());
    if (!IsLoad)
      HiInstr.add(MI->getOperand(0));
    HiInstr.addReg(Hi, getDefRegState(IsLoad));
    if (IsLoad)
      HiInstr.add(MI->getOperand(1));
    HiInstr.add(MI->getOperand(2))
        .addImm(MI->getOperand(3).getImm() + 1)
        .addMemOperand(MF.getMachineMemOperand(*MI->memoperands_begin(), 1, 1));
    MI->eraseFromParent();
    expandLDSTStk(LoInstr);
    expandLDSTStk(HiInstr);
    return;
  }

  Register Loc8 =
      TRI.getMatchingSuperReg(Loc, MC6809::sub_lsb, &MC6809::ACC8RegClass);
  if (Loc8)
    Loc = Loc8;

  assert(Loc == MC6809::C || Loc == MC6809::V || MC6809::ACC8RegClass.contains(Loc));

  // XXXX: FIXME: MarkM - the below may be *any* accumulator
  Register A = Loc;
  if (A != MC6809::AA)
    A = MRI.createVirtualRegister(&MC6809::ACC8RegClass);

  // Transfer the value to A to be stored (if applicable).
  if (!IsLoad && Loc != A) {
    if (Loc == MC6809::C || Loc == MC6809::V)
      Builder.buildInstr(MC6809::COPY)
          .addDef(A, RegState::Undef, MC6809::sub_lsb)
          .addUse(Loc);
    else {
      assert(MC6809::ACC8RegClass.contains(Loc));
      Builder.buildCopy(A, Loc);
    }
  }

  // This needs to occur after the above copy since the source may be Y.
  Register Y =
      Builder.buildInstr(MC6809::LDImm, {&MC6809::INDEX16RegClass}, {Offset}).getReg(0);

  Builder.buildInstr(IsLoad ? MC6809::LDIndirIdx : MC6809::STIndirIdx)
      .addReg(A, getDefRegState(IsLoad))
      .add(MI->getOperand(2))
      .addUse(Y)
      .addMemOperand(*MI->memoperands_begin());

  // Transfer the loaded value out of A (if applicable).
  if (IsLoad && Loc != A) {
    if (Loc == MC6809::C || Loc == MC6809::V)
      Builder.buildInstr(MC6809::COPY, {Loc}, {}).addUse(A, 0, MC6809::sub_lsb);
    else {
      assert(MC6809::ACC8RegClass.contains(Loc));
      Builder.buildCopy(Loc, A);
    }
  }

  MI->eraseFromParent();
  return;
}

Register MC6809RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MC6809::SU : MC6809::SS;
}

static bool referencedByShiftRotate(Register Reg, const MachineRegisterInfo &MRI) {
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MC6809::ASL:
    case MC6809::LSR:
    case MC6809::ROL:
    case MC6809::ROR:
      return true;
    }
  }
  return false;
}

static bool referencedByIncDec(Register Reg, const MachineRegisterInfo &MRI) {
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MC6809::INC:
    case MC6809::DEC:
      return true;
    }
  }
  return false;
}

// Returns whether there's exactly one RMW operation, and all of the other
// references are to the poorer regclass. In that case, it's better to do the
// operation in the poorer regclass then to copy into a better one then copy
// back out.
static bool isRmwPattern(Register Reg, const MachineRegisterInfo &MRI) {
  SmallVector<const MachineInstr *> RMW;
  const MachineInstr *Rmw = nullptr;
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(Reg)) {
    switch (MI.getOpcode()) {
    default:
      break;
    case MC6809::ASL:
    case MC6809::LSR:
    case MC6809::ROL:
    case MC6809::ROR:
    case MC6809::INC:
    case MC6809::DEC:
      if (Rmw && Rmw != &MI)
        return false;
      Rmw = &MI;
      continue;
    }

    if (!MI.isCopy())
      return false;

    Register Dst = MI.getOperand(0).getReg();
    Register Src = MI.getOperand(1).getReg();

    Register Other = Reg == Dst ? Src : Dst;
    assert(Other != Reg);

    if (Other.isPhysical())
      continue;

    const auto *OtherRC = MRI.getRegClass(Other);
    if (OtherRC != &MC6809::ACC8RegClass && OtherRC != &MC6809::ACC16RegClass)
      return false;
  }
  assert(Rmw);
  return true;
}

bool MC6809RegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
#if 0
  // Don't coalesce Imag8 and AImag8 registers together when used by shifts or
  // rotates.  This may cause expensive ASL zp's to be used when ASL A would
  // have sufficed. It's better to do arithmetic in A and then copy it out.
  // Same concerns apply to INC and DEC.
  if (NewRC == &MC6809::Imag8RegClass || NewRC == &MC6809::ACC16RegClass) {
    const auto &MRI = MI->getMF()->getRegInfo();
    if (DstRC == &MC6809::AImag8RegClass &&
        referencedByShiftRotate(MI->getOperand(0).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(0).getReg(), MRI))
      return false;
    if (SrcRC == &MC6809::AImag8RegClass &&
        referencedByShiftRotate(MI->getOperand(1).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(1).getReg(), MRI))
      return false;
    if (DstRC == &MC6809::ACC8RegClass &&
        referencedByIncDec(MI->getOperand(0).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(0).getReg(), MRI))
      return false;
    if (SrcRC == &MC6809::ACC8RegClass &&
        referencedByIncDec(MI->getOperand(1).getReg(), MRI) &&
        !isRmwPattern(MI->getOperand(1).getReg(), MRI))
      return false;
  }
  return true;
#else
  return false;
#endif
}

int copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) {
  const auto &TRI = *STI.getRegisterInfo();
  if (DestReg == SrcReg)
    return 0;

  const auto &AreClasses = [&](const TargetRegisterClass &Dest,
                               const TargetRegisterClass &Src) {
    return Dest.contains(DestReg) && Src.contains(SrcReg);
  };

  if (AreClasses(MC6809::ACCRegClass, MC6809::ACCRegClass)) {
    if (MC6809::ACC8RegClass.contains(SrcReg)) {
      assert(MC6809::ACC8RegClass.contains(DestReg));
      // TFRp
      return 1;
    }
    if (MC6809::ACC16RegClass.contains(SrcReg) || MC6809::INDEX16RegClass.contains(SrcReg)) {
      assert(MC6809::ACC16RegClass.contains(DestReg) || MC6809::INDEX16RegClass.contains(DestReg));
      // TFRp
      return 1;
    }
  }
  if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    Register SrcReg8 =
        TRI.getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 =
        TRI.getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);

    // XXXX: FIXME: MarkM - handle all the CC bits; the below is broken
    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MC6809::C) {
        // Cmp #1
        int Cost = 4;
        if (!MC6809::ACCRegClass.contains(SrcReg))
          Cost += copyCost(MC6809::AA, SrcReg, STI);
      }
    }
    if (DestReg8) {
      DestReg = DestReg8;
      Register Tmp = DestReg;
      if (!MC6809::ACCRegClass.contains(Tmp))
        Tmp = MC6809::AA;
      // TFR, AND, 
      int Cost = 13;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
    }
    return 20;
  }

  llvm_unreachable("Unexpected physical register copy.");
}

bool MC6809RegisterInfo::getRegAllocationHints(Register VirtReg,
                                            ArrayRef<MCPhysReg> Order,
                                            SmallVectorImpl<MCPhysReg> &Hints,
                                            const MachineFunction &MF,
                                            const VirtRegMap *VRM,
                                            const LiveRegMatrix *Matrix) const {
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  DenseMap<Register, int> RegScores;

  DenseMap<Register, int> OriginalIndex;
  for (const auto &R : enumerate(Order))
    OriginalIndex[R.value()] = R.index();

  SmallSet<const MachineInstr *, 32> Visited;
  for (MachineInstr &MI : MRI.reg_nodbg_instructions(VirtReg)) {
    if (!Visited.insert(&MI).second)
      continue;
    switch (MI.getOpcode()) {
    default:
      continue;
    case MC6809::COPY: {
      const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg
                                       ? MI.getOperand(0)
                                       : MI.getOperand(1);
      const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg
                                        ? MI.getOperand(1)
                                        : MI.getOperand(0);
      Register OtherReg = Other.getReg();
      if (OtherReg.isVirtual()) {
        if (!VRM->hasPhys(OtherReg))
          break;
        OtherReg = VRM->getPhys(OtherReg);
      }
      if (Other.getSubReg())
        OtherReg = TRI.getSubReg(OtherReg, Other.getSubReg());
      int WorstCost = 0;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        WorstCost = std::max(WorstCost, copyCost(SelfReg, OtherReg, STI));
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        int Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost < WorstCost)
          RegScores[R] += WorstCost - Cost;
      }
      break;
    }
    case MC6809::ASL:
    case MC6809::LSR:
    case MC6809::ROR:
    case MC6809::ROL:
      // ASL zp = 7
      // ASL a = 3
      if (is_contained(Order, MC6809::AA))
        RegScores[MC6809::AA] += 4;
      break;

    case MC6809::CMPTermZ:
      // CMPTermZ ACC best case: 0 (TAX)
      // CMPTermZ ACC worst case: 4 (CMP #0)
      // Splitting the difference: 2
      // CMPTermZ ZP best case: 0 (elided)
      // CMPTermZ ZP worst case: 14 (INC DEC)
      // Splitting the difference: 7
      if (is_contained(Order, MC6809::AA))
        RegScores[MC6809::AA] += 5;
      if (is_contained(Order, MC6809::IX))
        RegScores[MC6809::IX] += 5;
      if (is_contained(Order, MC6809::IY))
        RegScores[MC6809::IY] += 5;
      break;

    case MC6809::INC:
    case MC6809::DEC:
      // INC zp = (2 bytes + 5 cycles)
      // INXY = (1 bytes + 2 cycles)
      if (is_contained(Order, MC6809::IX))
        RegScores[MC6809::IX] += 4;
      if (is_contained(Order, MC6809::IY))
        RegScores[MC6809::IY] += 4;
      break;
    }
  }

  SmallVector<std::pair<Register, int>> RegsAndScores(RegScores.begin(),
                                                      RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, int> &A,
                          const std::pair<Register, int> &B) {
    if (A.second > B.second)
      return true;
    if (A.second < B.second)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}
