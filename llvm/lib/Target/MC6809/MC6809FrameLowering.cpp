//===-- MC6809FrameLowering.cpp - MC6809 Frame Information ----------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of TargetFrameLowering class.
//
//===----------------------------------------------------------------------===//

#include "MC6809FrameLowering.h"

#include "MC6809.h"
#include "MC6809InstrBuilder.h"
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/CodeGen/RegisterScavenging.h"

#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GlobalISel/CallLowering.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/MCContext.h"
#include "llvm/MC/MCDwarf.h"
#include "llvm/MC/MCRegisterInfo.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"

#define DEBUG_TYPE "mc6809-framelowering"

using namespace llvm;

MC6809FrameLowering::MC6809FrameLowering()
    : TargetFrameLowering(StackGrowsDown, /*StackAlignment=*/Align(1),
                          /*LocalAreaOffset=*/0) {}

bool MC6809FrameLowering::assignCalleeSavedSpillSlots(MachineFunction &MF, const TargetRegisterInfo *TRI, std::vector<CalleeSavedInfo> &CSI) const {
  MachineFrameInfo &MFI = MF.getFrameInfo();
  const auto &MC6809FI = *MF.getInfo<MC6809FunctionInfo>();

  size_t HardStackRemaining = 4;
  for (CalleeSavedInfo &Info : CSI) {
    // Some CSRs may be rewritten to direct page locations.
    // These don't need to be spilled.
    auto It = MC6809FI.CSRDPOffsets.find(Info.getReg());
    if (It != MC6809FI.CSRDPOffsets.end()) {
      Info.setTargetSpilled();
      continue;
    }

    // We place the first four CSRs on the hard stack, which we don't
    // explicitly model in PEI.
    if (HardStackRemaining) {
      Info.setTargetSpilled();
      --HardStackRemaining;
    } else {
      Info.setFrameIdx(MFI.CreateSpillStackObject(1, Align()));
    }
  }

  return true;
}

uint64_t MC6809FrameLowering::getHardStackCalleeSavedSize(const MachineFunction &MF) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo *TRI = MF.getRegInfo().getTargetRegisterInfo();
  const auto &FuncInfo = *MF.getInfo<MC6809FunctionInfo>();

  uint64_t Size = 0;
  for (const CalleeSavedInfo &CSI : MFI.getCalleeSavedInfo()) {
    if (!CSI.isTargetSpilled())
      continue;
    if (FuncInfo.CSRDPOffsets.count(CSI.getReg()))
      continue;
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(CSI.getReg());
    Size += TRI->getSpillSize(*RC);
  }
  return Size;
}

bool MC6809FrameLowering::enableShrinkWrapping(const MachineFunction &MF) const {
  // Shrink-wrapping moves the prologue (LEAS -n,S; PSHS U; TFR S,U) past
  // early-exit blocks. On the MC6809, frame-relative accesses (n,U) require
  // U to be set up first. Shrink-wrapping can place these accesses before
  // the TFR S,U, causing incorrect addressing. Disable until the shrink-wrap
  // analysis understands U-relative addressing constraints.
  return false;
}

bool MC6809FrameLowering::spillCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, ArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  MachineIRBuilder Builder(MBB, MI);
  MachineInstrSpan MIS(MI, &MBB);
  const MC6809Subtarget &STI = MBB.getParent()->getSubtarget<MC6809Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  const auto &FuncInfo = MBB.getParent()->getInfo<MC6809FunctionInfo>();

  // There are intentionally very few CSRs, few enough to place on the hard
  // stack without much risk of overflow. This is the only across-calls way
  // the compiler uses the hard stack, since the free CSRs can then be used
  // with impunity. This is slightly more expensive than saving/resting values
  // directly on the hard stack, but it's significantly simpler.
  //
  // Non-INDEX16 non-AD/AW CSRs are routed through physreg $ad: COPY $ad, Reg
  // then PSHS D. We target $ad directly (not a virtual register in some
  // tightly-typed class) because (a) frame lowering runs post-regalloc so
  // physreg manipulation is the natural form and (b) avoiding a 1-register
  // vreg class here cooperates with the Layer-2 regalloc fix (bug #118):
  // no 1-register class exists for the coalescer to tighten i16 vregs to.
  //
  // Bug #175: bucket directly-pushable CSRs (INDEX16 + AD) into a SINGLE
  // fused PSHS — same encoding bytes either way (one opcode + one mask
  // byte regardless of how many regs are in the mask), but saves a whole
  // 2-byte PSHS per fused register. Hardware pushes high-mask-bit-first
  // (PC=0x80, S/U=0x40, Y=0x20, X=0x10, DP=0x08, B=0x04, A=0x02, CC=0x01;
  // AD = A+B = 0x06), so the fused encoding produces the SAME byte layout
  // as separate `pshs y; pshs x; pshs d` would have (Y at high addr, X
  // below, AD bytes at SP) — PEI-computed SP-relative offsets stay valid.
  // Non-fusible CSRs (those routed through AD, plus AW) emit individually.
  SmallVector<Register, 4> Fusible;
  SmallVector<Register, 4> AdRouted;
  bool HaveAW = false;
  for (const CalleeSavedInfo &CI : CSI) {
    Register Reg = CI.getReg();
    if (!CI.isTargetSpilled() || FuncInfo->CSRDPOffsets.count(Reg))
      continue;
    if (MC6809::INDEX16RegClass.contains(Reg) || Reg == MC6809::AD) {
      Fusible.push_back(Reg);
    } else if (Reg == MC6809::AW) {
      HaveAW = true;
    } else {
      AdRouted.push_back(Reg);
    }
  }

  // Sort Fusible by hardware-mask-descending so the source-list order
  // matches the order PSHS would push them. (Operand order in the MI
  // doesn't actually affect the encoded bitmask — the encoder just OR-s
  // bits — but keeping the source list sorted is good hygiene and
  // matches how the disassembler will print the fused form.)
  auto pshsMask = [](Register R) -> unsigned {
    switch (R) {
    case MC6809::SU: case MC6809::SS: return 0x40;  // S/U
    case MC6809::IY: return 0x20;
    case MC6809::IX: return 0x10;
    case MC6809::DP: return 0x08;
    case MC6809::AB: return 0x04;
    case MC6809::AA: return 0x02;
    case MC6809::AD: return 0x06;
    case MC6809::CC: return 0x01;
    default: return 0;
    }
  };
  llvm::sort(Fusible, [&](Register A, Register B) {
    return pshsMask(A) > pshsMask(B);
  });

  if (!Fusible.empty()) {
    auto MIB = Builder.buildInstr(MC6809::PSHSs);
    for (Register R : Fusible)
      MIB.addUse(R);
  }
  for (Register R : AdRouted) {
    Builder.buildCopy(Register(MC6809::AD), R);
    Builder.buildInstr(MC6809::PSHSs, {}, {Register(MC6809::AD)});
  }
  if (HaveAW)
    Builder.buildInstr(MC6809::PSHSWx, {}, {});

  // Record that the frame pointer is killed by these instructions.
  for (auto &MI : make_range(MIS.begin(), MIS.getInitial()))
    MI.setFlag(MachineInstr::FrameSetup);

  // The frame pointer will be generated after the last frame setup instruction.

  // Save registers to the soft stack afterwards, since this may require the
  // frame pointer.
  for (const CalleeSavedInfo &CI : CSI) {
    Register Reg = CI.getReg();
    if (CI.isTargetSpilled())
      continue;
    assert(!CI.isSpilledToReg());
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.storeRegToStackSlot(MBB, Builder.getInsertPt(), Reg, true, CI.getFrameIdx(), RC, Register{});
  }

  return true;
}

template <typename F, typename VisitSet> static void visitReturnBlocks(MachineBasicBlock *MBB, const F &Func, VisitSet &VisitedBBs) {
  if (!VisitedBBs.insert(MBB).second)
    return;
  if (MBB->isReturnBlock())
    Func(*MBB);

  // Follow branches in BB and look for returns
  for (MachineBasicBlock *Succ : MBB->successors())
    visitReturnBlocks(Succ, Func, VisitedBBs);
}

template <typename F> static void visitReturnBlocks(MachineBasicBlock *MBB, const F &Func) {
  SmallSet<MachineBasicBlock *, 32> VisitedBBs;
  visitReturnBlocks(MBB, Func, VisitedBBs);
}

bool MC6809FrameLowering::restoreCalleeSavedRegisters(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, MutableArrayRef<CalleeSavedInfo> CSI, const TargetRegisterInfo *TRI) const {
  MachineIRBuilder Builder(MBB, MI);
  const MC6809Subtarget &STI = MBB.getParent()->getSubtarget<MC6809Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  const auto &FuncInfo = MBB.getParent()->getInfo<MC6809FunctionInfo>();
  // Mirror of spillCalleeSavedRegisters: non-INDEX16 non-AW CSRs are
  // routed through physreg $ad (PULS D → COPY CSR, $ad). See the note
  // on the spill side for why we target $ad directly.

  for (const CalleeSavedInfo &CI : reverse(CSI)) {
    Register Reg = CI.getReg();
    if (CI.isTargetSpilled())
      continue;
    assert(!CI.isSpilledToReg());
    const TargetRegisterClass *RC = TRI->getMinimalPhysRegClass(Reg);
    TII.loadRegFromStackSlot(MBB, Builder.getInsertPt(), Reg, CI.getFrameIdx(), RC, Register{});
  }

  // Begin tracking the frame pointer exclusion region only after all soft stack
  // CSR restores are emitted.
  MachineInstrSpan MIS(MI, &MBB);

  // Bug #175: mirror of the spill-side fusion. Bucket the same way and
  // emit one fused PULS for the directly-pullable CSRs. The reverse
  // ordering (vs. the spill side) is purely cosmetic — PULS pops in
  // hardware-mask-ascending order regardless of source-list operand
  // order, so the encoded mask determines the layout. The original loop
  // walked `reverse(CSI)` so individual `puls reg` instructions came in
  // the reverse order of the spill `pshs reg` sequence; we preserve that
  // for the AdRouted (un-fusible) bucket so SP-relative offsets resolve
  // correctly during the unwind. The fused PULS comes out in one
  // instruction, after the AdRouted unwind completes.
  SmallVector<Register, 4> Fusible;
  SmallVector<Register, 4> AdRouted;
  bool HaveAW = false;
  for (const CalleeSavedInfo &CI : CSI) {
    Register Reg = CI.getReg();
    if (!CI.isTargetSpilled() || FuncInfo->CSRDPOffsets.count(Reg))
      continue;
    if (MC6809::INDEX16RegClass.contains(Reg) || Reg == MC6809::AD) {
      Fusible.push_back(Reg);
    } else if (Reg == MC6809::AW) {
      HaveAW = true;
    } else {
      AdRouted.push_back(Reg);
    }
  }

  // Restore order is the inverse of spill: AW (was last to push) first,
  // then AdRouted in REVERSE (each was a `copy ad,csr; pshs ad`, unwind
  // is `puls ad; copy csr,ad`), then the fused PULS for the
  // directly-pushable bucket (was first to push, so last to pop).
  if (HaveAW)
    Builder.buildInstr(MC6809::PULSWx, {}, {});
  for (Register R : reverse(AdRouted)) {
    Builder.buildInstr(MC6809::PULSs, {Register(MC6809::AD)}, {});
    Builder.buildCopy(R, Register(MC6809::AD));
  }
  if (!Fusible.empty()) {
    // Build the PULS with all Fusible regs as DEFs (unlike PSHS, which
    // takes them as USEs). The encoder iterates explicit operands and
    // OR-s bitmask bits regardless of def/use status, so source-list
    // order doesn't affect the encoded mask — but we keep the same
    // mask-descending sort as the spill side for output consistency.
    auto pshsMask = [](Register R) -> unsigned {
      switch (R) {
      case MC6809::SU: case MC6809::SS: return 0x40;
      case MC6809::IY: return 0x20;
      case MC6809::IX: return 0x10;
      case MC6809::DP: return 0x08;
      case MC6809::AB: return 0x04;
      case MC6809::AA: return 0x02;
      case MC6809::AD: return 0x06;
      case MC6809::CC: return 0x01;
      default: return 0;
      }
    };
    llvm::sort(Fusible, [&](Register A, Register B) {
      return pshsMask(A) > pshsMask(B);
    });
    auto MIB = Builder.buildInstr(MC6809::PULSs);
    for (Register R : Fusible)
      MIB.addDef(R);
  }

  // Mark the CSRs as used by the return to ensure Machine Copy Propagation
  // doesn't remove the copies that set them.
  visitReturnBlocks(&MBB, [&CSI](MachineBasicBlock &MBB) {
    assert(MBB.rbegin()->isReturn());
    const auto &FuncInfo = MBB.getParent()->getInfo<MC6809FunctionInfo>();
    for (const CalleeSavedInfo &CI : CSI) {
      if (FuncInfo->CSRDPOffsets.count(CI.getReg()))
        continue;
      MBB.rbegin()->addOperand(MachineOperand::CreateReg(CI.getReg(), /*isDef=*/false, /*isImp=*/true));
    }
  });

  // Record that the frame pointer is killed by these instructions.
  for (auto &MI : make_range(MIS.begin(), MIS.getInitial()))
    MI.setFlag(MachineInstr::FrameDestroy);

  return true;
}

bool MC6809FrameLowering::enableCalleeSaveSkip(const MachineFunction &MF) const {
  assert(MF.getFunction().hasFnAttribute(Attribute::NoReturn) && MF.getFunction().hasFnAttribute(Attribute::NoUnwind) && !MF.getFunction().hasFnAttribute(Attribute::UWTable));
  return true;
}

void MC6809FrameLowering::determineCalleeSaves(MachineFunction &MF, BitVector &SavedRegs, RegScavenger *RS) const {
  TargetFrameLowering::determineCalleeSaves(MF, SavedRegs, RS);

  // If we have a frame pointer, the frame register SU needs to be saved as
  // well, since the code that uses it hasn't yet been emitted.
  if (hasFP(MF))
    SavedRegs.set(MC6809::SU);

  if (isISR(MF)) {
    // We need A to save anything else. This may require in turn saving A.
    // Normally, this could be done with __save_A, but for ISRs, that location
    // must also be saved. So we have to save A as a CSR, not through the
    // scavenger. Luckily, due to the register ordering, we're ensured that A
    // is saved before any other register.
    if (!SavedRegs.none())
      SavedRegs.set(MC6809::AA);

    if (SavedRegs.size() > 4)
      SavedRegs.set(MC6809::IY);
  }
}

void MC6809FrameLowering::processFunctionBeforeFrameFinalized(MachineFunction &MF, RegScavenger *RS) const {
  // Always reserve an emergency spill slot for the register scavenger.
  // The 6809 has very few registers, so the scavenger frequently needs
  // to temporarily spill a register during frame index elimination.
  if (RS) {
    MachineFrameInfo &MFI = MF.getFrameInfo();
    int FI = MFI.CreateStackObject(2, Align(1), false);
    RS->addScavengingFrameIndex(FI);
  }

  MachineFrameInfo &MFI = MF.getFrameInfo();
  auto &FuncInfo = *MF.getInfo<MC6809FunctionInfo>();

  // Bug #387 (S2): in a static-stack function, move the spill slots off the
  // dynamic stack into static memory. This runs before calculateFrameObjectsOffsets,
  // which skips non-Default stack IDs — so the marked slots drop out of
  // StackSize (the dynamic frame shrinks) and keep the static offset assigned
  // here. The whole local frame is eligible (spill slots AND ordinary /
  // address-taken locals): every Default local object is reached through a
  // pseudo that eliminateFrameIndex rewrites to a _Sym (extended/absolute)
  // form — Load/Store for value access, Add/Sub for in-place i32 arithmetic,
  // LEA for an address-of. Only fixed objects (incoming args, negative index,
  // excluded by the seq starting at 0) and variable-sized objects stay dynamic.
  // MC6809StaticStackAlloc later lays these per-function regions out
  // non-overlappingly across the call graph.
  if (usesStaticStack(MF)) {
    const MC6809InstrInfo &TII =
        *static_cast<const MC6809InstrInfo *>(MF.getSubtarget().getInstrInfo());
    // A local can move to the static frame only if every instruction that
    // reaches it through a frame-index operand has a _Sym (extended-addressing)
    // lowering — otherwise eliminateFrameIndex could not rewrite that access
    // and would assert. Collect the locals with any non-lowerable accessor;
    // those stay on the dynamic frame.
    SmallSet<int, 8> NotLowerable;
    for (const MachineBasicBlock &MBB : MF)
      for (const MachineInstr &MI : MBB) {
        if (TII.getStaticSymOpcode(MI.getOpcode()))
          continue;
        for (const MachineOperand &MO : MI.operands())
          if (MO.isFI())
            NotLowerable.insert(MO.getIndex());
      }
    // Persist for eliminateFrameIndex's LATE spill-slot marking: a slot with
    // mixed accessors (one _Sym-capable, one not) stays dynamic and must not
    // be moved to the static frame when the _Sym-capable access is rewritten
    // first — the non-lowerable accessor would then hit the static assert.
    FuncInfo.StaticNotLowerable = NotLowerable;
    int64_t Offset = 0;
    for (int Idx : seq(0, MFI.getObjectIndexEnd())) {
      if (MFI.isDeadObjectIndex(Idx) || MFI.isVariableSizedObjectIndex(Idx) ||
          MFI.getStackID(Idx) != TargetStackID::Default ||
          NotLowerable.count(Idx))
        continue;
      MFI.setStackID(Idx, TargetStackID::Mc6809Static);
      MFI.setObjectOffset(Idx, Offset);
      Offset += MFI.getObjectSize(Idx); // Static stack grows up.
    }
    // Seed the running offset so slots created after frame finalisation
    // (markSpillSlotStatic from eliminateFrameIndex) land in the same
    // static region.
    FuncInfo.NextStaticStackOffset = Offset;
  }
}

// Bug #387: place a spill slot created after frame finalisation into the
// static-stack region, if this is a static-stack
// function. Mirrors the processFunctionBeforeFrameFinalized marking: static
// stack ID + a growing static offset. A no-op for ordinary dynamic frames, so
// the slot then keeps its normal U-relative placement.
void MC6809FrameLowering::markSpillSlotStatic(MachineFunction &MF,
                                              int FI) const {
  if (!usesStaticStack(MF))
    return;
  auto &FuncInfo = *MF.getInfo<MC6809FunctionInfo>();
  MachineFrameInfo &MFI = MF.getFrameInfo();
  MFI.setStackID(FI, TargetStackID::Mc6809Static);
  MFI.setObjectOffset(FI, FuncInfo.NextStaticStackOffset);
  FuncInfo.NextStaticStackOffset += MFI.getObjectSize(FI);
}

MachineBasicBlock::iterator MC6809FrameLowering::eliminateCallFramePseudoInstr(MachineFunction &MF, MachineBasicBlock &MBB, MachineBasicBlock::iterator MI) const {
  int64_t Offset = MI->getOperand(0).getImm();

  // If we've already reserved the outgoing call frame in the prolog/epilog, the
  // pseudo can be summarily removed.
  if (hasReservedCallFrame(MF) || !Offset)
    return MBB.erase(MI);

  // Increment/decrement the stack pointer to reserve space for the call frame.
  MachineIRBuilder Builder(MBB, MI);
  if (MI->getOpcode() == MF.getSubtarget().getInstrInfo()->getCallFrameSetupOpcode())
    Offset = -Offset;
  offsetSP(Builder, Offset);
  return MBB.erase(MI);
}

// Bug #164 Tier 2: emit a CFI_INSTRUCTION MI at position I in MBB.
// Mirrors the MSP430 / AArch64 pattern: add the MCCFIInstruction to the
// MachineFunction's frame-instruction table, then build a meta-MI that
// carries the index.  The base-class AsmPrinter::emitFunctionBody switch
// dispatches CFI_INSTRUCTION to AsmPrinter::emitCFIInstruction which
// calls OutStreamer->emitCFI*(…).
static void BuildCFI(MachineBasicBlock &MBB, MachineBasicBlock::iterator I,
                     const DebugLoc &DL, const MCCFIInstruction &CFIInst,
                     MachineInstr::MIFlag Flag = MachineInstr::FrameSetup) {
  MachineFunction &MF = *MBB.getParent();
  unsigned CFIIndex = MF.addFrameInst(CFIInst);
  BuildMI(MBB, I, DL,
          MF.getSubtarget().getInstrInfo()->get(TargetOpcode::CFI_INSTRUCTION))
      .addCFIIndex(CFIIndex)
      .setMIFlag(Flag);
}

void MC6809FrameLowering::emitPrologue(MachineFunction &MF, MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo &TRI = *MF.getRegInfo().getTargetRegisterInfo();
  const MCRegisterInfo *MRI = MF.getContext().getRegisterInfo();
  MachineIRBuilder Builder(MBB, MBB.begin());

  int64_t StackSize = MFI.getStackSize();
  // If the interrupted routine is in the middle of decrementing its stack
  // pointer, this routine may observe a stack pointer up to 255 bytes higher
  // than its atomic value.  Accordingly, summarily decrement the SP by a page.
  // Interrupts are rarer than the the routines they interrupt, so they pay the
  // cost of dealing with this atomicity problem.
  if (isISR(MF))
    StackSize += 256;

  // Bug #164 Tier 2: initial CFI.  At function entry (just after the caller's
  // JSR/BSR has pushed the 2-byte return address) the canonical frame address
  // (CFA) is S + 2.  The return PC lives at CFA - 2.
  // We emit these before any SP-adjusting instructions so the unwinder's PC
  // table row for address 0 (the function entry) is correct.
  unsigned SPDwarf = MRI->getDwarfRegNum(MC6809::SS, /*isEH=*/true);
  unsigned PCDwarf = MRI->getDwarfRegNum(MC6809::PC, /*isEH=*/true);
  auto EntryMBBI = MBB.begin();
  BuildCFI(MBB, EntryMBBI, {},
           MCCFIInstruction::cfiDefCfa(nullptr, SPDwarf, 2));
  BuildCFI(MBB, EntryMBBI, {},
           MCCFIInstruction::createOffset(nullptr, PCDwarf, -2));

  if (StackSize)
    offsetSP(Builder, -StackSize);

  if (!hasFP(MF))
    return;

  // Skip the callee-saved push instructions.
  auto MBBI = std::find_if_not(Builder.getInsertPt(), MBB.end(), [](const MachineInstr &MI) { return MI.getFlag(MachineInstr::FrameSetup); });

  // Set the frame pointer to the stack pointer.
  // Use TFRp directly instead of COPY to prevent Machine Copy Propagation
  // from eliminating it (PSHSs modifies SS, so SU != SS after the push).
  Builder.setInsertPt(MBB, MBBI);
  Builder.setDebugLoc({});
  Register FPReg = TRI.getFrameRegister(MF);
  Builder.buildInstr(MC6809::TFRp).addDef(FPReg).addUse(MC6809::SS);

  // Bug #164 Tier 2: after TFR S,U the frame pointer is valid.  Switch the
  // CFA to U-based.  Total offset from U to caller's SP:
  //   StackSize (locals) + callee-saved bytes + 2 (return address)
  // = exactly what MFI reports once all frame slots are laid out.
  // We compute CSR bytes from CalleeSavedInfo rather than relying on
  // MFI.getStackSize() which only covers locals.
  int64_t CSRBytes = getHardStackCalleeSavedSize(MF);
  int64_t TotalOffset = StackSize + CSRBytes + 2;

  unsigned FPDwarf = MRI->getDwarfRegNum(FPReg, /*isEH=*/true);
  // Insert CFI immediately before the first instruction that follows the TFR
  // we just built (= Builder.getInsertPt()).  Do NOT use std::next() here —
  // getInsertPt() already points one past the TFR; stepping again lands on
  // MBB.end() when TFR is the last non-terminator, causing BuildCFI to insert
  // the directive after the block's branch terminator.  That makes the
  // AsmPrinter skip the successor BB label, producing an "Undefined temporary
  // symbol" assembler error (fixed for bug #164 Tier 2 regression).
  auto AfterTFR = Builder.getInsertPt();
  BuildCFI(MBB, AfterTFR, {},
           MCCFIInstruction::cfiDefCfa(nullptr, FPDwarf, TotalOffset));

  // Add frame pointer to liveins of all blocks so the machine verifier
  // doesn't flag it as used-without-definition (bug #16).
  for (MachineBasicBlock &Block : MF)
    if (!Block.isLiveIn(FPReg))
      Block.addLiveIn(FPReg);
}

void MC6809FrameLowering::emitEpilogue(MachineFunction &MF, MachineBasicBlock &MBB) const {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  const TargetRegisterInfo &TRI = *MF.getRegInfo().getTargetRegisterInfo();
  const MCRegisterInfo *MRI = MF.getContext().getRegisterInfo();
  MachineIRBuilder Builder(MBB, MBB.getFirstTerminator());

  // Restore the stack pointer from the frame pointer.
  if (hasFP(MF)) {
    // Skip the callee-saved push instructions.
    auto MBBI = find_if_not(mbb_reverse(MBB.begin(), Builder.getInsertPt()), [](const MachineInstr &MI) { return MI.getFlag(MachineInstr::FrameDestroy); });
    Builder.setInsertPt(MBB, MachineBasicBlock::iterator(MBBI));

    // Set the stack pointer to the frame pointer.
    Builder.buildInstr(MC6809::TFRp).addDef(MC6809::SS).addUse(TRI.getFrameRegister(MF));
    Builder.setInsertPt(MBB, MBB.getFirstTerminator());
  }

  int64_t StackSize = MFI.getStackSize();

  if (isISR(MF))
    StackSize += 256;

  // If soft stack is used, increase the soft stack pointer SP.
  if (StackSize)
    offsetSP(Builder, StackSize);

  // Bug #164 Tier 2: restore SP-based CFA for the unwinder at the point of
  // the return instruction.  After the epilogue unwinds the frame, CFA is
  // back to SP + 2 (just the return address remains).
  if (hasFP(MF) || StackSize) {
    unsigned SPDwarf = MRI->getDwarfRegNum(MC6809::SS, /*isEH=*/true);
    BuildCFI(MBB, MBB.getFirstTerminator(), {},
             MCCFIInstruction::cfiDefCfa(nullptr, SPDwarf, 2),
             MachineInstr::FrameDestroy);
  }
}

bool MC6809FrameLowering::hasFP(const MachineFunction &MF) const {
  if (MF.getTarget().getTargetTriple().isOSOS9()) {
    if (MF.getFrameInfo().isFrameAddressTaken() ||
        MF.getFrameInfo().hasVarSizedObjects())
      report_fatal_error(
          "OS-9 MC6809 does not support a U-based frame pointer; U is the "
          "process data-area base");
    return false;
  }

  const MachineFrameInfo &MFI = MF.getFrameInfo();
  if (MFI.isFrameAddressTaken() || MFI.hasVarSizedObjects())
    return true;
  // Bug #387: a static-stack function lays its whole local frame — spill slots
  // included — in absolute (extended / PC-relative) memory, so it needs no U
  // frame pointer for those accesses. It still needs one when it makes calls
  // (a call's return-address push shifts S, so any S-relative access to the
  // remaining fixed args / dynamic slots would be unstable across the call).
  // A call-free static-stack function drops the frame pointer entirely: no
  // `pshs u; tfr s,u`, and U becomes available for allocation. hasFP must be
  // stable across passes (it is queried in determineCalleeSaves, before the
  // static marking runs), so gate only on inputs that don't depend on marking:
  // usesStaticStack and hasCalls.
  return MFI.hasCalls();
}

uint64_t MC6809FrameLowering::staticSize(const MachineFrameInfo &MFI) const {
  uint64_t Size = 0;
  for (int Idx : seq(0, MFI.getObjectIndexEnd()))
    if (MFI.getStackID(Idx) == TargetStackID::Mc6809Static)
      Size += MFI.getObjectSize(Idx);
  return Size;
}

bool MC6809FrameLowering::usesStaticStack(const MachineFunction &MF) const {
  // Bug #387: only for functions the inter-procedural MC6809NonReentrant
  // analysis proved single-activation, when the feature is enabled and we are
  // optimising. A genuinely re-entered function (recursive, interrupt-
  // reachable, or escaping through an external-call cycle) is never marked
  // "nonreentrant", so its frame stays dynamic.
  return MF.getSubtarget<MC6809Subtarget>().staticStack() &&
         !MF.getFunction().hasOptNone() &&
         MF.getFunction().hasFnAttribute("nonreentrant");
}

bool MC6809FrameLowering::isSupportedStackID(TargetStackID::Value ID) const {
  switch (ID) {
  case TargetStackID::Default:
  case TargetStackID::NoAlloc:
  case TargetStackID::Mc6809Static:
    return true;
  default:
    return false;
  }
}

bool MC6809FrameLowering::hasFPImpl(const MachineFunction &MF) const {
  return hasFP(MF);
}

void MC6809FrameLowering::offsetSP(MachineIRBuilder &Builder, int64_t Offset) const {
  assert(Offset);
  if (Offset < SHRT_MIN)
    report_fatal_error("Stack pointer decrement too large: " + Twine(-Offset));
  if (Offset > SHRT_MAX)
    report_fatal_error("Stack pointer increment too large: " + Twine(Offset));

  Builder.buildInstr(MC6809::LEAPtrAdd_Imm).addDef(MC6809::SS).addUse(MC6809::SS).addImm(Offset);
}

bool MC6809FrameLowering::isISR(const MachineFunction &MF) const {
  const Function &F = MF.getFunction();
  if (F.hasFnAttribute("no-isr"))
    return false;
  return F.hasFnAttribute("interrupt") || F.hasFnAttribute("interrupt-norecurse");
}
