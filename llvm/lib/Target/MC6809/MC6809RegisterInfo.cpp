//===-- MC6809RegisterInfo.cpp - MC6809 Register Information --------------===//
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
#include "MC6809FrameLowering.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/ErrorHandling.h"
#include <algorithm>

#define DEBUG_TYPE "mc6809-reginfo"

#define GET_REGINFO_TARGET_DESC
#include "MC6809GenRegisterInfo.inc"

using namespace llvm;

/// Map from RS byte sub-reg to its parent RS register, or 0 if not a sub-reg.
static unsigned getImag16ParentForSymbol(unsigned Reg) {
  switch (Reg) {
  case MC6809::RS0HI: case MC6809::RS0LO: return MC6809::RS0;
  case MC6809::RS1HI: case MC6809::RS1LO: return MC6809::RS1;
  case MC6809::RS2HI: case MC6809::RS2LO: return MC6809::RS2;
  case MC6809::RS3HI: case MC6809::RS3LO: return MC6809::RS3;
  default: return 0;
  }
}

static void initImag8SymbolNames(const MC6809RegisterInfo &TRI,
                                  std::unique_ptr<std::string[]> &Names) {
  Names.reset(new std::string[TRI.getNumRegs()]);
  for (unsigned Reg : seq(0u, TRI.getNumRegs())) {
    // Generate `__<lowercase_name>` for every imaginary register
    // (RC0..RC7 in Imag8, RS0..RS3 in Imag16) and for the RS byte
    // sub-registers (RS0HI..RS3LO). The linker script assigns each
    // symbol a direct-page address. For the byte sub-regs, the
    // linker defines: __rs0hi = __rs0 (high byte at base address,
    // big-endian) and __rs0lo = __rs0 + 1.
    if (!MC6809::Imag16RegClass.contains(Reg) &&
        !MC6809::Imag8RegClass.contains(Reg) &&
        !getImag16ParentForSymbol(Reg))
      continue;
    std::string &Str = Names[Reg];
    Str = "__";
    Str += TRI.getName(Reg);
    std::transform(Str.begin(), Str.end(), Str.begin(), ::tolower);
  }
}

MC6809RegisterInfo::MC6809RegisterInfo()
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                            /*PC=*/0, /*HwMode=*/0) {
  initImag8SymbolNames(*this, Imag8SymbolNames);
}

MC6809RegisterInfo::MC6809RegisterInfo(const Triple &TT)
    : MC6809GenRegisterInfo(/*RA=*/0, /*DwarfFlavor=*/0, /*EHFlavor=*/0,
                            /*PC=*/0, /*HwMode=*/0) {
  initImag8SymbolNames(*this, Imag8SymbolNames);
}

BitVector MC6809RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Mark special registers as reserved.
  Reserved.set(MC6809::PC);
  Reserved.set(MC6809::SS);
  Reserved.set(MC6809::DP);
  Reserved.set(MC6809::A0);
  Reserved.set(MC6809::AV);
  Reserved.set(MC6809::MD);

  // SU is unconditionally reserved as the frame pointer. Even when a function
  // doesn't need a frame pointer at the start of compilation, the register
  // allocator may introduce stack-backed spill registers that require one.
  // Reserving SU upfront avoids the phase-ordering trap where hasFP() flips
  // mid-allocation (was bug #16).
  Reserved.set(MC6809::SU);

  // Imaginary registers are direct-page memory locations, not CPU registers.
  // Reserve them so the verifier doesn't track their liveness.
  for (MCPhysReg Reg : MC6809::Imag8RegClass)
    Reserved.set(Reg);
  for (MCPhysReg Reg : MC6809::Imag16RegClass)
    Reserved.set(Reg);

  // HD6309-only registers: reserve on standard 6809.
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  if (!STI.has6309()) {
    Reserved.set(MC6809::AE);
    Reserved.set(MC6809::AF);
    Reserved.set(MC6809::AW);
    Reserved.set(MC6809::AQ);
    Reserved.set(MC6809::AELSB);
    Reserved.set(MC6809::AFLSB);
    Reserved.set(MC6809::MD);
  }

  return Reserved;
}

const MCPhysReg *MC6809RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const { return MC6809_CSR_SaveList; }

const uint32_t *MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF, CallingConv::ID CallingConv) const {
  // Start with the TableGen-generated mask from CalleeSavedRegs.
  // Then mark spill pseudo-registers as call-preserved — they're backed by
  // stack memory so function calls can't clobber them.
  static uint32_t SpillPreservedMask[MC6809::NUM_TARGET_REGS / 32 + 1];
  static bool Initialized = false;
  if (!Initialized) {
    // Copy the base mask.
    unsigned MaskSize = (MC6809::NUM_TARGET_REGS + 31) / 32;
    memcpy(SpillPreservedMask, MC6809_CSR_RegMask, sizeof(uint32_t) * MaskSize);
    // Mark all spill registers as preserved across calls.
    for (MCPhysReg Reg : {MC6809::SPILL_D0, MC6809::SPILL_D1, MC6809::SPILL_D2, MC6809::SPILL_D3, MC6809::SPILL_D4, MC6809::SPILL_D5, MC6809::SPILL_D6, MC6809::SPILL_D7,
                          MC6809::SPILL_A0, MC6809::SPILL_A1, MC6809::SPILL_A2, MC6809::SPILL_A3, MC6809::SPILL_A4, MC6809::SPILL_A5, MC6809::SPILL_A6, MC6809::SPILL_A7,
                          MC6809::SPILL_B0, MC6809::SPILL_B1, MC6809::SPILL_B2, MC6809::SPILL_B3, MC6809::SPILL_B4, MC6809::SPILL_B5, MC6809::SPILL_B6, MC6809::SPILL_B7,
                          MC6809::SPILL_A0LSB, MC6809::SPILL_A1LSB, MC6809::SPILL_A2LSB, MC6809::SPILL_A3LSB, MC6809::SPILL_A4LSB, MC6809::SPILL_A5LSB, MC6809::SPILL_A6LSB, MC6809::SPILL_A7LSB,
                          MC6809::SPILL_B0LSB, MC6809::SPILL_B1LSB, MC6809::SPILL_B2LSB, MC6809::SPILL_B3LSB, MC6809::SPILL_B4LSB, MC6809::SPILL_B5LSB, MC6809::SPILL_B6LSB, MC6809::SPILL_B7LSB,
                          // Bug #85b: SPILL_X0-X3 are stack-backed INDEX16 spill
                          // registers, exactly like SPILL_D0-D7 for ACC16. They
                          // must be marked call-preserved so the allocator knows
                          // they survive function calls. Without this, INDEX16
                          // has only IX + IY across calls (2 registers), which is
                          // insufficient for any function with 3+ live pointers.
                          MC6809::SPILL_X0, MC6809::SPILL_X1, MC6809::SPILL_X2, MC6809::SPILL_X3,
                          // RS imaginary 16-bit regs and their byte sub-regs (memory-backed = call-preserved)
                          MC6809::RS0, MC6809::RS1, MC6809::RS2, MC6809::RS3,
                          MC6809::RS0HI, MC6809::RS0LO, MC6809::RS1HI, MC6809::RS1LO,
                          MC6809::RS2HI, MC6809::RS2LO, MC6809::RS3HI, MC6809::RS3LO}) {
      SpillPreservedMask[Reg / 32] |= (1u << (Reg % 32));
    }
    Initialized = true;
  }
  return SpillPreservedMask;
}

const TargetRegisterClass *
MC6809RegisterInfo::getSubClassWithSubReg(const TargetRegisterClass *RC,
                                           unsigned Idx) const {
  // Override tablegen's sub-class-with-subreg for ACC16 and ADc.
  // When RS imaginary byte sub-regs are in 8-bit allocatable classes
  // (AAc/ABc), tablegen's lattice algorithm tightens the sub_lo_byte
  // extraction from acc8 (26+ regs) to acc8_ab (2 regs), causing
  // cascading regalloc degradation. Fix: for ACC16/ADc, return the
  // class itself for byte-related sub-reg indices, preserving the
  // wide allocation pool.
  if (RC == &MC6809::ACC16RegClass || RC == &MC6809::ADcRegClass) {
    if (Idx == MC6809::sub_lo_byte || Idx == MC6809::sub_hi_byte ||
        Idx == MC6809::sub_lsb)
      return RC;
  }
  return MC6809GenRegisterInfo::getSubClassWithSubReg(RC, Idx);
}

const TargetRegisterClass *MC6809RegisterInfo::getCrossCopyRegClass(const TargetRegisterClass *RC) const {
  if (RC == &MC6809::INDEX16RegClass)
    return &MC6809::ACC16RegClass;
  else if (RC == &MC6809::CCFlagRegClass)
    return &MC6809::ACC8RegClass;
  return RC;
}

bool MC6809RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II, int SPAdj, unsigned FIOperandNum, RegScavenger *RS) const {
  assert(SPAdj == 0 && "Unexpected non-zero SPAdj");

  MachineInstr &MI = *II;
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  const MC6809FrameLowering *TFI = getFrameLowering(MF);
  DebugLoc dl = MI.getDebugLoc();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  unsigned BasePtr = (TFI->hasFP(MF) ? MC6809::SU : MC6809::SS);
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  int Offset = MFI.getObjectOffset(FrameIndex);

  // Fixed stack objects (args, positive offset) are above the return
  // address on the stack. Local objects (spills, negative offset) are
  // below it. Only args need the +2 PC skip.
  if (Offset >= 0)
    Offset += 2; // Skip the saved PC (return address)

  // Compute size of ALL callee-saved registers pushed onto the hard stack.
  // These sit between the stack allocation and the return address.
  // Count all CSRs regardless of isTargetSpilled() — the MC6809 backend
  // pushes all CSRs to the hard stack via PSHS (even those with frame indices).
  unsigned CalleeSavedSize = 0;
  for (const auto &CSI : MFI.getCalleeSavedInfo()) {
    const TargetRegisterInfo *TRI = MF.getRegInfo().getTargetRegisterInfo();
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(CSI.getReg());
    CalleeSavedSize += TRI->getSpillSize(*RC);
  }

  // The frame pointer (U) is set to S AFTER both stack allocation and
  // callee-saved pushes. So the offset from FP to args includes both.
  // Same formula for FP and non-FP — the base register differs (U vs S)
  // but the offset computation is identical.
  Offset += MFI.getStackSize() + CalleeSavedSize;

  // Fold imm into offset
  Offset += MI.getOperand(FIOperandNum + 1).getImm();
  MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
  MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Offset);

  // Ensure the base register is in the block's liveins so the machine
  // verifier doesn't flag it as undefined (bug #16).
  if (!MBB.isLiveIn(BasePtr))
    MBB.addLiveIn(BasePtr);

  return false;
}

Register MC6809RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  const TargetFrameLowering *TFI = getFrameLowering(MF);
  return TFI->hasFP(MF) ? MC6809::SU : MC6809::SS;
}

bool MC6809RegisterInfo::getRegAllocationHints(Register VirtReg, ArrayRef<MCPhysReg> Order, SmallVectorImpl<MCPhysReg> &Hints, const MachineFunction &MF, const VirtRegMap *VRM, const LiveRegMatrix *Matrix) const {
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const auto &TRI = *STI.getRegisterInfo();
  const MachineRegisterInfo &MRI = MF.getRegInfo();
  DenseMap<Register, MC6809InstrCost> RegScores;
  auto CostMode = MC6809InstrCost::getModeFor(MF);

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
      const MachineOperand &Self = MI.getOperand(0).getReg() == VirtReg ? MI.getOperand(0) : MI.getOperand(1);
      const MachineOperand &Other = MI.getOperand(0).getReg() == VirtReg ? MI.getOperand(1) : MI.getOperand(0);
      Register OtherReg = Other.getReg();
      if (OtherReg.isVirtual()) {
        if (!VRM->hasPhys(OtherReg))
          break;
        OtherReg = VRM->getPhys(OtherReg);
      }
      if (Other.getSubReg())
        OtherReg = TRI.getSubReg(OtherReg, Other.getSubReg());
      MC6809InstrCost WorstCost;
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MC6809InstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) > WorstCost.value(CostMode))
          WorstCost = Cost;
      }
      for (Register R : Order) {
        Register SelfReg = R;
        if (Self.getSubReg())
          SelfReg = TRI.getSubReg(SelfReg, Self.getSubReg());
        MC6809InstrCost Cost = copyCost(SelfReg, OtherReg, STI);
        if (Cost.value(CostMode) < WorstCost.value(CostMode))
          RegScores[R] += (WorstCost - Cost);
      }
      break;
    }
    }
  }

  SmallVector<std::pair<Register, MC6809InstrCost>> RegsAndScores(RegScores.begin(), RegScores.end());
  sort(RegsAndScores, [&](const std::pair<Register, MC6809InstrCost> &A, const std::pair<Register, MC6809InstrCost> &B) {
    auto AVal = A.second.value(CostMode);
    auto BVal = B.second.value(CostMode);
    if (AVal > BVal)
      return true;
    if (AVal < BVal)
      return false;
    return OriginalIndex[A.first] < OriginalIndex[B.first];
  });
  append_range(Hints, make_first_range(RegsAndScores));
  return false;
}

MC6809InstrCost MC6809RegisterInfo::copyCost(Register DestReg, Register SrcReg, const MC6809Subtarget &STI) const {
  if (DestReg == SrcReg)
    return MC6809InstrCost();

  const auto &AreClasses = [&](const TargetRegisterClass &Dest, const TargetRegisterClass &Src) { return Dest.contains(DestReg) && Src.contains(SrcReg); };

  auto TransferCost = MC6809InstrCost(1, 2);
  auto Push8Cost = MC6809InstrCost(1, 3);
  auto Push16Cost = MC6809InstrCost(1, 4);
  auto Pop8Cost = MC6809InstrCost(1, 3);
  auto Pop16Cost = MC6809InstrCost(1, 4);
  auto ClVCost = MC6809InstrCost(1, 2);
  auto JumpCost = MC6809InstrCost(3, 4);
  auto BranchCost = MC6809InstrCost(2, 4);
  auto LoadImm8Cost = MC6809InstrCost(2, 2);
  auto LoadImm16Cost = MC6809InstrCost(2, 3);
  auto AluImm8Cost = MC6809InstrCost(2, 2);
  auto AluImm16Cost = MC6809InstrCost(2, 3);
  auto ImpossibleCost = MC6809InstrCost(32768, 32768);

  // Spill register costs: load/store to stack via S-indexed addressing.
  auto SpillLoad8Cost = MC6809InstrCost(2, 4);   // LDA/LDB offset,S
  auto SpillStore8Cost = MC6809InstrCost(2, 4);   // STA/STB offset,S
  auto SpillLoad16Cost = MC6809InstrCost(2, 5);   // LDD offset,S
  auto SpillStore16Cost = MC6809InstrCost(2, 5);   // STD offset,S

  // Real ↔ 8-bit spill register
  if ((MC6809::ACC8RegClass.contains(DestReg) && MC6809::AAcRegClass.contains(SrcReg) && !MC6809::ACC8RegClass.contains(SrcReg)) ||
      (MC6809::ACC8RegClass.contains(DestReg) && MC6809::ABcRegClass.contains(SrcReg) && !MC6809::ACC8RegClass.contains(SrcReg)))
    return SpillLoad8Cost;
  if ((MC6809::AAcRegClass.contains(DestReg) && !MC6809::ACC8RegClass.contains(DestReg) && MC6809::ACC8RegClass.contains(SrcReg)) ||
      (MC6809::ABcRegClass.contains(DestReg) && !MC6809::ACC8RegClass.contains(DestReg) && MC6809::ACC8RegClass.contains(SrcReg)))
    return SpillStore8Cost;
  // Real ↔ 16-bit spill register
  if (MC6809::ACC16RegClass.contains(DestReg) && MC6809::ADcRegClass.contains(SrcReg) && !MC6809::ACC16RegClass.contains(SrcReg))
    return SpillLoad16Cost;
  if (MC6809::ADcRegClass.contains(DestReg) && !MC6809::ACC16RegClass.contains(DestReg) && MC6809::ACC16RegClass.contains(SrcReg))
    return SpillStore16Cost;
  // Spill ↔ spill (same size): load + store
  if (MC6809::AAcRegClass.contains(DestReg) && !MC6809::ACC8RegClass.contains(DestReg) &&
      MC6809::AAcRegClass.contains(SrcReg) && !MC6809::ACC8RegClass.contains(SrcReg))
    return SpillLoad8Cost + SpillStore8Cost;
  if (MC6809::ABcRegClass.contains(DestReg) && !MC6809::ACC8RegClass.contains(DestReg) &&
      MC6809::ABcRegClass.contains(SrcReg) && !MC6809::ACC8RegClass.contains(SrcReg))
    return SpillLoad8Cost + SpillStore8Cost;
  if (MC6809::ADcRegClass.contains(DestReg) && !MC6809::ACC16RegClass.contains(DestReg) &&
      MC6809::ADcRegClass.contains(SrcReg) && !MC6809::ACC16RegClass.contains(SrcReg))
    return SpillLoad16Cost + SpillStore16Cost;

  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC8RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC8RegClass)) {
    return TransferCost + TransferCost;
  }
  if (AreClasses(MC6809::ACC32RegClass, MC6809::ACC8RegClass)) {
    return TransferCost + TransferCost + TransferCost + TransferCost;
  }
  if (AreClasses(MC6809::Imag8RegClass, MC6809::ACC8RegClass)) {
    return MC6809InstrCost(2, 3);
  }
  if (AreClasses(MC6809::Imag16RegClass, MC6809::ACC16RegClass)) {
    return MC6809InstrCost(2, 4);
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::Imag8RegClass)) {
    return MC6809InstrCost(2, 3);
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::Imag16RegClass)) {
    return MC6809InstrCost(2, 4);
  }
  if (AreClasses(MC6809::Imag8RegClass, MC6809::Imag8RegClass)) {
    return Push8Cost + Pop8Cost + copyCost(DestReg, MC6809::AA, STI) + copyCost(MC6809::AA, SrcReg, STI);
  }
  if (AreClasses(MC6809::Imag16RegClass, MC6809::Imag16RegClass)) {
    return Push16Cost + Pop16Cost + copyCost(DestReg, MC6809::AD, STI) + copyCost(MC6809::AD, SrcReg, STI);
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::ACC16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC16RegClass, MC6809::INDEX16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::INDEX16RegClass, MC6809::ACC16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::INDEX16RegClass, MC6809::INDEX16RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::INDEX16RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC16RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::ACC32RegClass)) {
    return ImpossibleCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::CCFlagRegClass) || AreClasses(MC6809::CCFlagRegClass, MC6809::ACC8RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::ACC8RegClass, MC6809::CCondRegClass) || AreClasses(MC6809::CCondRegClass, MC6809::ACC8RegClass)) {
    return TransferCost;
  }
  if (AreClasses(MC6809::CCondRegClass, MC6809::CCondRegClass)) {
    return MC6809InstrCost(0, 0);
  }
  if (AreClasses(MC6809::BIT1RegClass, MC6809::BIT1RegClass)) {
    Register SrcReg8 = getMatchingSuperReg(SrcReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    Register DestReg8 = getMatchingSuperReg(DestReg, MC6809::sub_lsb, &MC6809::ACC8RegClass);
    auto BitCost = MC6809InstrCost(3, 4);

    if (SrcReg8) {
      SrcReg = SrcReg8;
      if (DestReg8) {
        DestReg = DestReg8;
        return copyCost(DestReg, SrcReg, STI);
      }
      if (DestReg == MC6809::C) {
        MC6809InstrCost Cost = AluImm8Cost;
        if (!MC6809::ACC8RegClass.contains(SrcReg))
          Cost += copyCost(MC6809::AA, SrcReg, STI);
        return Cost;
      }

      if (MC6809::ACC8RegClass.contains(SrcReg)) {
        return Push8Cost + Pop8Cost + BranchCost + BitCost + JumpCost + ClVCost;
      }
      return copyCost(MC6809::AA, SrcReg, STI) + BranchCost + BitCost + JumpCost + ClVCost;
    }
    if (DestReg8) {
      DestReg = DestReg8;

      Register Tmp = DestReg;
      if (!MC6809::ACC8RegClass.contains(Tmp))
        Tmp = MC6809::AA;
      MC6809InstrCost Cost = LoadImm8Cost * 2 + BranchCost;
      if (Tmp != DestReg)
        Cost += copyCost(DestReg, Tmp, STI);
      return Cost;
    }
    return (Push8Cost + Pop8Cost) * 3 + AluImm8Cost + BranchCost + ClVCost;
  }

  // Guard: any remaining pair is impossible (size mismatch, nonsensical).
  return ImpossibleCost;
}
