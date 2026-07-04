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
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
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

  // The RS 16-bit imaginaries are genuine allocation targets (the MOS
  // model): with the SPILL_D escape registers retired, they are what makes
  // ADc/ACC16 more than a single register on plain 6809 -- a reg-reg i16
  // compare needs BOTH operands placed somewhere at one instruction. Every
  // expansion already handles an RS operand (direct-page opcode forms,
  // materializeReg staging, the Imag16 spill-slot path).
  // They are deliberately NOT in the call-preserved mask: the direct-page
  // addresses are per-CPU, not per-frame, so values in them do not survive
  // calls (recursion included) and the allocator must keep their live
  // ranges call-free.

  // HD6309-only registers: reserve on standard 6809.
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  if (!STI.has6309()) {
    Reserved.set(MC6809::AE);
    Reserved.set(MC6809::AF);
    Reserved.set(MC6809::AW);
    Reserved.set(MC6809::AQ);
    Reserved.set(MC6809::MD);
  }

  // The RC byte imaginaries (direct-page memory bytes, same MOS model as
  // RS). On HD6309 they are genuine allocation targets: every real 8-bit
  // accumulator (AA/AB/AE/AF) aliases $aq, so an instruction defining a
  // byte alongside a live i32 — an i32 carry-out materialised to a byte is
  // the canonical case (lround) — has NO aliasing-free home without them;
  // with the SPILL_Q escape registers retired that is otherwise a
  // guaranteed allocation failure. CostPerUse biases regalloc back to the
  // real accumulators whenever one is free. On plain 6809 nothing aliases
  // the byte pool that way, so the S1-validated {AA, AB} + fold behaviour
  // is kept: RCs stay reserved as materialisation scratch there.
  if (!STI.has6309())
    for (MCPhysReg Reg : MC6809::Imag8RegClass)
      Reserved.set(Reg);

  return Reserved;
}

const MCPhysReg *MC6809RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const { return MC6809_CSR_SaveList; }

const uint32_t *MC6809RegisterInfo::getCallPreservedMask(const MachineFunction &MF, CallingConv::ID CallingConv) const {
  // The TableGen-generated mask from CalleeSavedRegs, unmodified. The
  // stack-backed spill pseudo-registers (SPILL_A/B/D/X/Q) that used to be
  // marked call-preserved here are gone — every class spills through stock
  // frame-index slots, which calls cannot clobber.
  //
  // The RS/RC imaginaries are deliberately NOT in this mask: unlike the
  // retired SPILL_* slots (U-relative, one per frame) they live at FIXED
  // direct-page addresses shared by every function, so a callee that
  // allocates the same imaginary clobbers the caller's value. Their live
  // ranges must stay call-free.
  return MC6809_CSR_RegMask;
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
  // Bug #311: sub_lsb retired.
  if (RC == &MC6809::ACC16RegClass || RC == &MC6809::ADcRegClass) {
    if (Idx == MC6809::sub_lo_byte || Idx == MC6809::sub_hi_byte)
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
  const MC6809InstrInfo &TII =
      *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());

  // Bug #387 (S2): a spill slot that escaped processFunctionBeforeFrameFinalized's
  // static marking — created later in PEI (a register-scavenging emergency slot,
  // or any spill/save slot the frame-finalisation scan didn't see) — still
  // belongs in the static frame. Once the dynamic frame has shrunk (the marked
  // spills moved to the static region), a remaining-dynamic slot's U/S-relative
  // offset is bogus. Mark it here, at its first access, so it flows into the
  // Mc6809Static path below; NextStaticStackOffset was seeded by
  // processFunctionBeforeFrameFinalized, which ran earlier in this same PEI pass.
  // Only when the accessing pseudo actually has a _Sym (extended) sibling,
  // AND the frame-finalisation scan didn't find a non-lowerable accessor of
  // the same slot (mixed-accessor slots must stay dynamic as a whole — see
  // MC6809FunctionInfo::StaticNotLowerable).
  if (TFI->usesStaticStack(MF) && MFI.isSpillSlotObjectIndex(FrameIndex) &&
      MFI.getStackID(FrameIndex) == TargetStackID::Default &&
      TII.getStaticSymOpcode(MI.getOpcode()) &&
      !MF.getInfo<MC6809FunctionInfo>()->StaticNotLowerable.count(FrameIndex))
    TFI->markSpillSlotStatic(MF, FrameIndex);

  // Bug #387 (S3/S4): a spill slot moved to the static stack is addressed
  // absolutely, not via U/S. Rewrite the Load/Store_*_Mem pseudo to its _Sym
  // sibling carrying a TI_STATIC_STACK target index (the per-function byte
  // offset assigned in processFunctionBeforeFrameFinalized). The post-RA
  // expander (expandLoadSym/expandStoreSym) emits the extended-mode access,
  // and MC6809StaticStackAlloc turns the target index into the real global.
  if (MFI.getStackID(FrameIndex) == TargetStackID::Mc6809Static) {
    int64_t StaticOff = MFI.getObjectOffset(FrameIndex) +
                        MI.getOperand(FIOperandNum).getOffset() +
                        MI.getOperand(FIOperandNum + 1).getImm();
    unsigned SymOpc = TII.getStaticSymOpcode(MI.getOpcode());
    assert(SymOpc && "static-stack frame index in a non-spill pseudo");
    // Copy every operand before the frame index — the value/dst register for
    // load/store/LEA, plus the tied source for the add/sub forms (whose
    // $dst=$src constraint MachineInstr::addOperand re-establishes from the
    // _Sym opcode's descriptor). The frame index and its companion immediate
    // (at FIOperandNum, FIOperandNum+1) collapse into the target index.
    // Explicit operands after the pair (a fused compare-branch's label)
    // follow it.
    auto MIB = BuildMI(MBB, MI, dl, TII.get(SymOpc));
    for (unsigned I = 0; I < FIOperandNum; ++I)
      MIB.add(MI.getOperand(I));
    MIB.addTargetIndex(MC6809::TI_STATIC_STACK, StaticOff);
    for (unsigned I = FIOperandNum + 2, E = MI.getNumExplicitOperands(); I < E;
         ++I)
      MIB.add(MI.getOperand(I));
    MIB.cloneMemRefs(MI);
    MI.eraseFromParent();
    return true;
  }
  // Bug #153/#154: include the FI operand's own byte offset.
  // foldMemoryOperandImpl uses `.addFrameIndex(FI, ByteOffset)` to encode
  // sub-slot byte addressing (e.g. +1 for the low half of a big-endian
  // i16 spill consumed by EXTRACT_LO_i16). Without this term, the +1
  // is silently dropped and the load fetches the high byte instead.
  int Offset = MFI.getObjectOffset(FrameIndex)
             + MI.getOperand(FIOperandNum).getOffset();

  // Fixed stack objects (args, positive offset) are above the return
  // address on the stack. Local objects (spills, negative offset) are
  // below it. Only args need the +2 PC skip.
  if (Offset >= 0)
    Offset += 2; // Skip the saved PC (return address)

  // Account for CSRs pushed on the hardware stack outside MFI's modeled stack.
  // Soft-stack CSR slots are already included in MFI.getStackSize().
  unsigned CalleeSavedSize = TFI->getHardStackCalleeSavedSize(MF);

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

// Refuse coalesces that would narrow a multi-register byte class down to the
// single-register AAc/ABc endpoint classes. The selector deliberately COPYs
// into those classes only at MERGE_LOHI/byte-pair endpoints; folding the copy
// away re-narrows the whole tied carry chain feeding it to one physical
// register, and two overlapping chains then present unspillable
// singleton-class ranges that greedy cannot allocate (strtoll's i64
// accumulation). Keeping the endpoint copy costs one TFR at worst.
bool MC6809RegisterInfo::shouldCoalesce(
    MachineInstr *MI, const TargetRegisterClass *SrcRC, unsigned SubReg,
    const TargetRegisterClass *DstRC, unsigned DstSubReg,
    const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
  auto IsByteSingleton = [](const TargetRegisterClass *RC) {
    return RC == &MC6809::AAcRegClass || RC == &MC6809::ABcRegClass;
  };
  if (IsByteSingleton(NewRC) && !IsByteSingleton(SrcRC) &&
      !IsByteSingleton(DstRC))
    return false;
  return true;
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
    case MC6809::MERGE_LOHI_i16: {
      // MERGE_LOHI_i16 assembles D = HI:LO, i.e. the high byte in AA and the
      // low byte in AB. Bias each byte operand toward the half it will occupy
      // so the merge collapses to zero instructions rather than an EXG/TFR
      // lane swap (the byte-wise 16-bit subtract `0 - x` otherwise lands its
      // low result in AA and high in AB and pays an `exg a,b`). Operand 1 is
      // the low byte, operand 2 the high byte.
      Register Preferred;
      if (MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == VirtReg)
        Preferred = MC6809::AB;
      else if (MI.getOperand(2).isReg() && MI.getOperand(2).getReg() == VirtReg)
        Preferred = MC6809::AA;
      else
        break;
      // Worth roughly the EXG the correct lane avoids.
      RegScores[Preferred] += MC6809InstrCost(2, 8);
      break;
    }
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
  if (AreClasses(MC6809::Imag16RegClass, MC6809::INDEX16RegClass) ||
      AreClasses(MC6809::INDEX16RegClass, MC6809::Imag16RegClass)) {
    // INDEX16 → Imag16: TFR IX/IY,D + STD dp  (or reverse: LDD dp + TFR D,IX/IY)
    return TransferCost + MC6809InstrCost(2, 4);
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
  // Guard: any remaining pair is impossible (size mismatch, nonsensical).
  return ImpossibleCost;
}
