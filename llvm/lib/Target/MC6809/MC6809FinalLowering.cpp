//===-- MC6809FinalLowering.cpp - MC6809 Final Lowering Pass --------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// MC6809 final-lowering pass (bug #149). See MC6809FinalLowering.h for the
// list of transform classes. This Commit 0 scaffold registers the pass,
// the per-class STATISTIC counters, and the per-class -O gating knobs;
// every transform is a stub returning false so the pass is codegen-neutral
// until subsequent commits land each class.
//
//===----------------------------------------------------------------------===//

#include "MC6809FinalLowering.h"

#include "MC6809.h"
#include "MC6809InstrInfo.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/Statistic.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "mc6809-final-lowering"

using namespace llvm;

STATISTIC(NumOffsetsRelaxed,    "Number of indexed offsets relaxed to a "
                                "narrower form (_o16->_o8->_o5/_o0)");
STATISTIC(NumLeafFramesElided,  "Number of leaf-function frame setup/teardown "
                                "pairs elided");
STATISTIC(NumAdjIncsFolded,     "Number of adjacent inc/add #1 sequences "
                                "folded into a single add #N");
STATISTIC(NumDupStoresElided,   "Number of dead duplicate stores to the same "
                                "slot elided");
STATISTIC(NumStoreReloadsElided,"Number of redundant reloads of unmodified "
                                "spill slots elided");
STATISTIC(NumBranchOverBranchFolded, "Number of branch-over-branch pairs "
                                     "(cond + uncond) folded into a single "
                                     "branch (bug #179)");

namespace {

// Each transform class has its own minimum -O level. Default is -O2
// (CodeGenOptLevel::Default). Set to 99 to disable a class entirely
// (useful when bisecting which class regressed).
//
// Naming convention: -mc6809-fl-<class>-min-O. The "fl" stands for
// "final lowering" and the leading "mc6809-" marks the flag as
// target-specific in `llc -help-hidden` output.
static cl::opt<int> RelaxOffsetsMinO(
    "mc6809-fl-offset-relax-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable indexed-offset relaxation "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> LeafFrameMinO(
    "mc6809-fl-leaf-frame-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable leaf-frame elision in "
             "MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> AdjIncMinO(
    "mc6809-fl-adj-inc-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable adjacent-increment folding "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> DupStoreMinO(
    "mc6809-fl-dup-store-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable duplicate-store elimination "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> StoreReloadMinO(
    "mc6809-fl-store-reload-min-O", cl::init(2), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable store+reload elimination "
             "in MC6809FinalLowering. 99 disables. Default: 2."));
static cl::opt<int> BranchOverBranchMinO(
    "mc6809-fl-branch-over-branch-min-O", cl::init(0), cl::Hidden,
    cl::desc("Minimum -O level (0..3) to enable branch-over-branch folding "
             "in MC6809FinalLowering (bug #179). 99 disables. Default: 0 "
             "(always on; transform is purely local and never enlarges code, "
             "and BranchFolderPass doesn't run at -O0 to do this work for "
             "us)."));

class MC6809FinalLowering : public MachineFunctionPass {
public:
  static char ID;

  MC6809FinalLowering() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FinalLoweringPass(
        *PassRegistry::getPassRegistry());
  }

  StringRef getPassName() const override {
    return "MC6809 Final Lowering";
  }

  bool runOnMachineFunction(MachineFunction &MF) override;

private:
  bool relaxOffsets(MachineFunction &MF);
  bool elideLeafFrame(MachineFunction &MF);
  bool foldAdjacentInc(MachineFunction &MF);
  bool elideDupStores(MachineFunction &MF);
  bool elideStoreReload(MachineFunction &MF);
  bool foldBranchOverBranch(MachineFunction &MF);

  // True if the current -O level (from MF.getTarget()) >= MinO.
  // MinO == 99 means "disabled by CL flag".
  bool enabled(const MachineFunction &MF, int MinO) const {
    if (MinO >= 99)
      return false;
    int CurO = static_cast<int>(MF.getTarget().getOptLevel());
    return CurO >= MinO;
  }
};

} // namespace

bool MC6809FinalLowering::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;

  // Class 6 (branch-over-branch fold) runs FIRST and is NOT gated by
  // skipFunction. Reason: at -O0, every function carries the `optnone`
  // attribute, which makes skipFunction return true and short-circuit
  // the whole pass. But Class 6's whole purpose is to backstop
  // BranchFolderPass at -O0 (where BranchFolderPass doesn't run); if we
  // skipped on optnone, Class 6 would never fire on its primary target.
  // The transform is purely local code-shape cleanup that never enlarges
  // code or changes semantics, so running it under optnone is safe.
  if (enabled(MF, BranchOverBranchMinO))
    Changed |= foldBranchOverBranch(MF);

  // Classes 1-5 are size/speed optimisations and respect optnone.
  if (skipFunction(MF.getFunction()))
    return Changed;

  if (enabled(MF, RelaxOffsetsMinO))
    Changed |= relaxOffsets(MF);
  if (enabled(MF, LeafFrameMinO))
    Changed |= elideLeafFrame(MF);
  if (enabled(MF, AdjIncMinO))
    Changed |= foldAdjacentInc(MF);
  if (enabled(MF, DupStoreMinO))
    Changed |= elideDupStores(MF);
  if (enabled(MF, StoreReloadMinO))
    Changed |= elideStoreReload(MF);
  return Changed;
}

// Commit 0 — every class is a stub. Subsequent commits replace the body of
// each function with the real transform and bump the matching STATISTIC.

bool MC6809FinalLowering::relaxOffsets(MachineFunction &MF) {
  // Walk every MI; for any indexed-immediate opcode whose offset value
  // fits a narrower encoding, switch to the narrower opcode and (when
  // shrinking to the _o0 form) drop the offset operand.
  //
  // Operand layout for indexed-immediate instructions in this backend
  // (post-ExpandPostRAPseudos; see InstrInfo's expandLoadIdx for the
  // ground truth on construction):
  //
  //   _o5/_o8/_o16: <maybe-implicit data reg> + offset imm + base index reg
  //   _o0:          <maybe-implicit data reg> + base index reg (no offset)
  //
  // The data register may be implicit-def (for loads/LEA), implicit-use
  // (for stores), or absent (for the LEA destination encoded in opcode).
  // To find the offset robustly, we just scan operands for the first
  // immediate operand and treat that as the offset. The opcode-table
  // lookup in getRelaxedIdxOpcode is the source of truth for whether
  // the instruction is actually an indexed-imm form.
  const auto *TII = static_cast<const MC6809InstrInfo *>(
      MF.getSubtarget().getInstrInfo());

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      // Find the first immediate operand (= offset for indexed-imm forms).
      int OffsetIdx = -1;
      for (unsigned I = 0, E = MI.getNumOperands(); I != E; ++I) {
        if (MI.getOperand(I).isImm()) {
          OffsetIdx = I;
          break;
        }
      }
      if (OffsetIdx < 0)
        continue;

      int64_t Offset = MI.getOperand(OffsetIdx).getImm();
      auto [NewOpc, NewLen] = TII->getRelaxedIdxOpcode(MI.getOpcode(),
                                                       Offset);
      if (NewOpc == 0)
        continue;

      MI.setDesc(TII->get(NewOpc));
      if (NewLen == 0) {
        // Shrinking to _o0 — drop the (now redundant) offset operand.
        MI.removeOperand(OffsetIdx);
      }
      ++NumOffsetsRelaxed;
      Changed = true;
    }
  }
  return Changed;
}

bool MC6809FinalLowering::elideLeafFrame(MachineFunction &MF) {
  // A function gets a U-based frame pointer (per MC6809FrameLowering::hasFP)
  // when ANY of: frame address taken, var-sized objects, spill pseudo-
  // registers used, or hasCalls(). For functions whose ONLY hasFP trigger
  // is hasCalls() AND that don't actually need U OR the stack-locals area,
  // the entire frame setup (LEAS -N,S; PSHSs U; TFRp $su=$ss) and matching
  // teardown (TFRp $ss=$su; PULSs U; LEAS +N,S) is dead weight: 12 bytes +
  // ~40 cycles of pure ABI ceremony per call to such a function.
  //
  // U is callee-saved per the MC6809 ABI. Eliding the save IS safe here
  // only because we're also eliding the write to U (the TFR setup), so
  // the caller's U is preserved by virtue of us never touching it.
  //
  // The eliding is pattern-driven: we match the exact 3-MI prologue
  // sequence at entry-block head and the exact 3-MI epilogue sequence
  // before each return-block terminator, so we don't accidentally
  // delete an outgoing-arg ADJCALLSTACK adjustment that has already
  // been lowered to a plain LEAS by this point in the pipeline.

  auto isLEASAdjust = [](const MachineInstr &MI) {
    // Post-ExpandPostRAPseudos / post-Class-1, S-pointer adjustments are
    // concrete LEASi_o{0,5,8,16} forms with operand layout
    // (imm offset, $ss base, implicit-def $ss). We must NOT match LEAS
    // forms whose base is X/Y/U — those are address arithmetic, not
    // stack adjustment.
    unsigned Opc = MI.getOpcode();
    if (Opc != MC6809::LEASi_o0 && Opc != MC6809::LEASi_o5 &&
        Opc != MC6809::LEASi_o8 && Opc != MC6809::LEASi_o16 &&
        Opc != MC6809::LEAPtrAdd_Imm)
      return false;
    // Find the base register in the explicit operands.
    for (const MachineOperand &MO : MI.operands()) {
      if (MO.isReg() && !MO.isImplicit() && MO.getReg() == MC6809::SS)
        return true;
    }
    return false;
  };
  auto isTFRSStoSU = [](const MachineInstr &MI) {
    return MI.getOpcode() == MC6809::TFRp && MI.getNumOperands() >= 2 &&
           MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::SU &&
           MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == MC6809::SS;
  };
  auto isTFRSUtoSS = [](const MachineInstr &MI) {
    return MI.getOpcode() == MC6809::TFRp && MI.getNumOperands() >= 2 &&
           MI.getOperand(0).isReg() && MI.getOperand(0).getReg() == MC6809::SS &&
           MI.getOperand(1).isReg() && MI.getOperand(1).getReg() == MC6809::SU;
  };
  auto pushesU = [](const MachineInstr &MI) {
    if (MI.getOpcode() != MC6809::PSHSs)
      return false;
    for (const MachineOperand &MO : MI.operands())
      if (MO.isReg() && MO.getReg() == MC6809::SU)
        return true;
    return false;
  };
  auto pullsU = [](const MachineInstr &MI) {
    if (MI.getOpcode() != MC6809::PULSs)
      return false;
    for (const MachineOperand &MO : MI.operands())
      if (MO.isReg() && MO.getReg() == MC6809::SU)
        return true;
    return false;
  };

  const MachineFrameInfo &MFI = MF.getFrameInfo();

  // Gate 1: must have a frame currently, and ONLY because of hasCalls().
  if (!MFI.hasCalls())
    return false;
  if (MFI.isFrameAddressTaken() || MFI.hasVarSizedObjects())
    return false;
  if (auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>())
    if (FuncInfo->UsesSpillRegisters)
      return false;

  // Gate 2: identify the entry-block prologue triple. Skip any LEAD-IN
  // debug values / non-real MIs.
  MachineBasicBlock &Entry = MF.front();
  auto It = Entry.SkipPHIsAndLabels(Entry.begin());
  while (It != Entry.end() && It->isDebugInstr())
    ++It;
  if (It == Entry.end() || !isLEASAdjust(*It))
    return false;
  MachineInstr *PrologLEAS = &*It;
  ++It;
  while (It != Entry.end() && It->isDebugInstr()) ++It;
  if (It == Entry.end() || !pushesU(*It))
    return false;
  MachineInstr *PrologPSHS = &*It;
  ++It;
  while (It != Entry.end() && It->isDebugInstr()) ++It;
  if (It == Entry.end() || !isTFRSStoSU(*It))
    return false;
  MachineInstr *PrologTFR = &*It;

  // Gate 3: scan body for any USE of U or any FI operand. The body is
  // every MI EXCEPT the prologue triple we just found and the epilogue
  // triples we'll find below. For the gate, treat the prologue triple
  // as "skip" and check everything else; epilogue MIs that touch U
  // pass the body-scan but will be erased below if the epilogue
  // pattern matches.
  for (const MachineBasicBlock &MBB : MF) {
    for (const MachineInstr &MI : MBB) {
      if (&MI == PrologLEAS || &MI == PrologPSHS || &MI == PrologTFR)
        continue;
      // Allow MIs that look like epilogue ones to slip past the
      // U-scan; the structural check below is the real authority.
      if (isTFRSUtoSS(MI) || pullsU(MI) || isLEASAdjust(MI))
        continue;
      for (const MachineOperand &MO : MI.operands()) {
        // Implicit register operands on calls/returns are liveness
        // markers, not actual reads/writes (e.g. RTSr lists `implicit $su`
        // because U was the FP, but RTS itself doesn't touch U). They
        // don't represent real use of the frame pointer for our purposes.
        if (MO.isReg() && !MO.isImplicit() && MO.getReg() == MC6809::SU)
          return false;
        if (MO.isFI())
          return false;
      }
    }
  }

  // Gate 4: every return block must end in the exact 3-MI epilogue
  // sequence (TFR $ss=$su; PULSs U; LEAS +N,S) immediately before the
  // terminator. Collect them; abort if any return block doesn't match.
  SmallVector<std::array<MachineInstr *, 3>, 4> Epilogues;
  for (MachineBasicBlock &MBB : MF) {
    if (!MBB.isReturnBlock())
      continue;
    auto T = MBB.getFirstTerminator();
    if (T == MBB.begin())
      return false;
    auto P3 = std::prev(T);
    while (P3 != MBB.begin() && P3->isDebugInstr())
      --P3;
    if (!isLEASAdjust(*P3) || P3 == MBB.begin())
      return false;
    auto P2 = std::prev(P3);
    while (P2 != MBB.begin() && P2->isDebugInstr())
      --P2;
    if (!pullsU(*P2) || P2 == MBB.begin())
      return false;
    auto P1 = std::prev(P2);
    while (P1 != MBB.begin() && P1->isDebugInstr())
      --P1;
    if (!isTFRSUtoSS(*P1))
      return false;
    Epilogues.push_back({&*P1, &*P2, &*P3});
  }
  if (Epilogues.empty())
    return false;

  // All gates passed — erase the prologue triple and every epilogue
  // triple. Count one elision per function (the unit of meaningful
  // work, regardless of how many return blocks).
  PrologLEAS->eraseFromParent();
  PrologPSHS->eraseFromParent();
  PrologTFR->eraseFromParent();
  for (auto &E : Epilogues) {
    E[0]->eraseFromParent();
    E[1]->eraseFromParent();
    E[2]->eraseFromParent();
  }
  ++NumLeafFramesElided;
  return true;
}

bool MC6809FinalLowering::foldAdjacentInc(MachineFunction &MF) {
  // Fold adjacent same-opcode immediate-arithmetic instructions:
  //
  //   ADDB #M
  //   ADDB #N      ->  ADDB #(M + N)
  //
  // Same for ADDA / ADDD / ADDE / ADDF / ADDW (and 6309 forms),
  // and SUBA / SUBB / SUBD.
  //
  // Conservative on CC: skip the fold when the combined immediate would
  // signed-overflow the operand width. Keeping the result in-range
  // preserves V exactly across the fold (V=0 in both the original
  // sequence's last add and the merged single add); other CC bits
  // (N/Z/C) are also identical because the final value is the same.
  //
  // We only consider strictly adjacent MIs (no intervening MI between
  // them) so the first MI's CC defs are immediately overwritten by the
  // second's — no liveness scan needed.
  //
  // We deliberately don't fold cross-opcode pairs (ADD then SUB, INC
  // then ADD, etc.) because their CC effects differ in subtle ways
  // (e.g. INC doesn't touch C, ADD does). Same-opcode + same dest is
  // the safe minimal cut.

  struct OpcodeInfo {
    unsigned Opc;
    unsigned BitWidth; // 8 or 16
    bool IsSub;        // true if SUB-family (immediate is subtracted)
  };
  static const OpcodeInfo Foldable[] = {
    {MC6809::ADDAi8,  8,  false}, {MC6809::ADDBi8,  8,  false},
    {MC6809::ADDEi8,  8,  false}, {MC6809::ADDFi8,  8,  false},
    {MC6809::ADDDi16, 16, false}, {MC6809::ADDWi16, 16, false},
    {MC6809::SUBAi8,  8,  true},  {MC6809::SUBBi8,  8,  true},
    {MC6809::SUBDi16, 16, false}, // SUBD's immediate adds for fold purposes
  };

  auto findInfo = [](unsigned Opc) -> const OpcodeInfo * {
    for (const OpcodeInfo &I : Foldable)
      if (I.Opc == Opc)
        return &I;
    return nullptr;
  };

  auto fitsSigned = [](int64_t V, unsigned Bits) {
    int64_t Lo = -(1LL << (Bits - 1));
    int64_t Hi = (1LL << (Bits - 1)) - 1;
    return V >= Lo && V <= Hi;
  };

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    auto It = MBB.begin();
    while (It != MBB.end()) {
      auto Next = std::next(It);
      if (Next == MBB.end()) break;
      const OpcodeInfo *Info = findInfo(It->getOpcode());
      if (!Info || It->getOpcode() != Next->getOpcode() ||
          It->getNumOperands() == 0 || Next->getNumOperands() == 0 ||
          !It->getOperand(0).isImm() || !Next->getOperand(0).isImm()) {
        ++It;
        continue;
      }
      int64_t M = It->getOperand(0).getImm();
      int64_t N = Next->getOperand(0).getImm();
      int64_t Combined = M + N;
      if (!fitsSigned(Combined, Info->BitWidth)) {
        ++It;
        continue;
      }
      // Fold: rewrite imm in It, erase Next. Don't advance It; another
      // adjacent MI of the same opcode might now be foldable.
      It->getOperand(0).setImm(Combined);
      Next->eraseFromParent();
      ++NumAdjIncsFolded;
      Changed = true;
    }
  }
  return Changed;
}

bool MC6809FinalLowering::elideDupStores(MachineFunction &MF) {
  // Eliminate dead duplicate stores to the same spill slot:
  //
  //   stb $21,u
  //   andb #$3        ; modifies B but doesn't touch memory
  //   stb $21,u       ; overwrites the first store, no intervening read
  //
  // Per-MBB linear scan: track the most recent store to each
  // (opcode, offset, base) triple. When another store to the same
  // triple is reached without an intervening MI that could read the
  // slot, the prior store is dead — erase it.
  //
  // Conservative invalidations (clear the entire tracking map):
  //   - any MI with mayLoad (loads might read the slot we wrote)
  //   - any MI with mayStore that we don't recognize as a tracked
  //     store opcode (it might write to overlapping memory)
  //   - any call (callees may read the caller's stack)
  //   - any inline-asm or barrier
  //
  // Intra-MBB only — cross-block tracking would require a forward
  // dataflow that's well outside the scope of a final-lowering pass.

  auto isTrackedStore = [](const MachineInstr &MI) {
    // Indexed-immediate store opcodes (post-expansion forms only —
    // _o0/_o5/_o8/_o16 across STA/STB/STD/STE/STF/STW/STQ/
    // STX/STY/STU/STS).
    static const unsigned Ops[] = {
      MC6809::STAi_o0, MC6809::STAi_o5, MC6809::STAi_o8, MC6809::STAi_o16,
      MC6809::STBi_o0, MC6809::STBi_o5, MC6809::STBi_o8, MC6809::STBi_o16,
      MC6809::STDi_o0, MC6809::STDi_o5, MC6809::STDi_o8, MC6809::STDi_o16,
      MC6809::STEi_o0, MC6809::STEi_o5, MC6809::STEi_o8, MC6809::STEi_o16,
      MC6809::STFi_o0, MC6809::STFi_o5, MC6809::STFi_o8, MC6809::STFi_o16,
      MC6809::STWi_o0, MC6809::STWi_o5, MC6809::STWi_o8, MC6809::STWi_o16,
      MC6809::STQi_o0, MC6809::STQi_o5, MC6809::STQi_o8, MC6809::STQi_o16,
      MC6809::STXi_o0, MC6809::STXi_o5, MC6809::STXi_o8, MC6809::STXi_o16,
      MC6809::STYi_o0, MC6809::STYi_o5, MC6809::STYi_o8, MC6809::STYi_o16,
      MC6809::STUi_o0, MC6809::STUi_o5, MC6809::STUi_o8, MC6809::STUi_o16,
      MC6809::STSi_o0, MC6809::STSi_o5, MC6809::STSi_o8, MC6809::STSi_o16,
    };
    unsigned Opc = MI.getOpcode();
    for (unsigned O : Ops)
      if (O == Opc)
        return true;
    return false;
  };

  // Slot key = (opcode, offset_imm, base_reg). A store with _o0 form
  // has offset=0 implicitly (no explicit operand), so we treat its
  // offset key as 0.
  struct SlotKey {
    unsigned Opcode;
    int64_t Offset;
    Register Base;
    bool operator==(const SlotKey &O) const {
      return Opcode == O.Opcode && Offset == O.Offset && Base == O.Base;
    }
  };

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    SmallVector<std::pair<SlotKey, MachineInstr *>, 8> RecentStores;

    auto invalidateAll = [&]() { RecentStores.clear(); };
    auto findStore = [&](const SlotKey &K) -> MachineInstr ** {
      for (auto &P : RecentStores)
        if (P.first == K)
          return &P.second;
      return nullptr;
    };
    auto recordStore = [&](const SlotKey &K, MachineInstr *MI) {
      if (auto **Slot = findStore(K)) {
        *Slot = MI;
        return;
      }
      RecentStores.push_back({K, MI});
    };

    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      // First: cheap structural classifications.
      if (MI.isCall() || MI.isInlineAsm() || MI.isReturn() ||
          MI.isBranch() || MI.isTerminator()) {
        invalidateAll();
        continue;
      }

      bool Tracked = isTrackedStore(MI);

      if (!Tracked) {
        // Untracked memory access invalidates everything.
        if (MI.mayLoad() || MI.mayStore())
          invalidateAll();
        continue;
      }

      // Build the slot key. For _o0 forms, no offset operand exists;
      // for _o5/_o8/_o16, operand[0] is the offset and operand[1] is
      // the base register (post-expansion shape).
      SlotKey K{MI.getOpcode(), 0, Register()};
      if (MI.getNumOperands() >= 2 && MI.getOperand(0).isImm() &&
          MI.getOperand(1).isReg()) {
        K.Offset = MI.getOperand(0).getImm();
        K.Base = MI.getOperand(1).getReg();
      } else if (MI.getNumOperands() >= 1 && MI.getOperand(0).isReg()) {
        // _o0 form: only the base register, no offset operand.
        K.Offset = 0;
        K.Base = MI.getOperand(0).getReg();
      } else {
        // Unexpected operand shape — be safe.
        invalidateAll();
        continue;
      }

      // If we already have a recorded store to this slot, the prior
      // one is dead.
      if (auto **Prior = findStore(K)) {
        if (*Prior) {
          (*Prior)->eraseFromParent();
          ++NumDupStoresElided;
          Changed = true;
        }
        *Prior = &MI;
      } else {
        recordStore(K, &MI);
      }
    }
  }
  return Changed;
}

bool MC6809FinalLowering::elideStoreReload(MachineFunction &MF) {
  // Eliminate redundant reloads of unmodified spill slots:
  //
  //   std 24,u      ; spill D to slot 24 (data register = $ad)
  //   ; ... no MI writes $ad and no MI writes slot 24 ...
  //   ldd 24,u      ; reload — but $ad still holds the same value.
  //
  // Erase the LDD (already-in-register).
  //
  // Per-MBB linear scan: track slotMap[(family, offset, base)] -> data
  // register currently holding the slot's value. On any clobber of a
  // data register OR a write to the slot, invalidate the corresponding
  // entries.
  //
  // Conservative invalidations:
  //   - any call: clear all entries (callees may clobber any caller-save
  //     reg AND may write through the stack)
  //   - inline-asm / barriers / branches: clear all
  //   - any unrecognised mayLoad / mayStore: clear all
  //
  // Family-based matching (STA<->LDA, STB<->LDB, STD<->LDD, etc.) means
  // we don't try to fold cross-width cases (e.g. STD followed by LDA of
  // the high byte). Strictly the safe MVP.

  // Map an opcode to (family-tag, data-register) iff it is a tracked
  // store or load. Family tag is the data-register itself (which is
  // unique per family).
  struct StoreOrLoad {
    bool IsStore;
    Register DataReg; // family tag + actual data reg
  };
  auto classify = [](unsigned Opc) -> std::optional<StoreOrLoad> {
#define STORE_FAMILY(R, A, B, C, D)                                            \
  if (Opc == MC6809::A || Opc == MC6809::B || Opc == MC6809::C ||              \
      Opc == MC6809::D)                                                        \
    return StoreOrLoad{true, MC6809::R};
#define LOAD_FAMILY(R, A, B, C, D)                                             \
  if (Opc == MC6809::A || Opc == MC6809::B || Opc == MC6809::C ||              \
      Opc == MC6809::D)                                                        \
    return StoreOrLoad{false, MC6809::R};
    STORE_FAMILY(AA, STAi_o0, STAi_o5, STAi_o8, STAi_o16)
    STORE_FAMILY(AB, STBi_o0, STBi_o5, STBi_o8, STBi_o16)
    STORE_FAMILY(AD, STDi_o0, STDi_o5, STDi_o8, STDi_o16)
    STORE_FAMILY(AE, STEi_o0, STEi_o5, STEi_o8, STEi_o16)
    STORE_FAMILY(AF, STFi_o0, STFi_o5, STFi_o8, STFi_o16)
    STORE_FAMILY(AW, STWi_o0, STWi_o5, STWi_o8, STWi_o16)
    STORE_FAMILY(AQ, STQi_o0, STQi_o5, STQi_o8, STQi_o16)
    STORE_FAMILY(IX, STXi_o0, STXi_o5, STXi_o8, STXi_o16)
    STORE_FAMILY(IY, STYi_o0, STYi_o5, STYi_o8, STYi_o16)
    STORE_FAMILY(SU, STUi_o0, STUi_o5, STUi_o8, STUi_o16)
    STORE_FAMILY(SS, STSi_o0, STSi_o5, STSi_o8, STSi_o16)
    LOAD_FAMILY(AA, LDAi_o0, LDAi_o5, LDAi_o8, LDAi_o16)
    LOAD_FAMILY(AB, LDBi_o0, LDBi_o5, LDBi_o8, LDBi_o16)
    LOAD_FAMILY(AD, LDDi_o0, LDDi_o5, LDDi_o8, LDDi_o16)
    LOAD_FAMILY(AE, LDEi_o0, LDEi_o5, LDEi_o8, LDEi_o16)
    LOAD_FAMILY(AF, LDFi_o0, LDFi_o5, LDFi_o8, LDFi_o16)
    LOAD_FAMILY(AW, LDWi_o0, LDWi_o5, LDWi_o8, LDWi_o16)
    LOAD_FAMILY(AQ, LDQi_o0, LDQi_o5, LDQi_o8, LDQi_o16)
    LOAD_FAMILY(IX, LDXi_o0, LDXi_o5, LDXi_o8, LDXi_o16)
    LOAD_FAMILY(IY, LDYi_o0, LDYi_o5, LDYi_o8, LDYi_o16)
    LOAD_FAMILY(SU, LDUi_o0, LDUi_o5, LDUi_o8, LDUi_o16)
    LOAD_FAMILY(SS, LDSi_o0, LDSi_o5, LDSi_o8, LDSi_o16)
#undef STORE_FAMILY
#undef LOAD_FAMILY
    return std::nullopt;
  };

  // Slot key: (data-reg-family-tag, offset, base-reg). Storing 0
  // implicit for the _o0 form.
  struct SlotKey {
    Register Family;
    int64_t Offset;
    Register Base;
    bool operator==(const SlotKey &O) const {
      return Family == O.Family && Offset == O.Offset && Base == O.Base;
    }
  };

  // For sub-register aliasing: AA/AB are sub-regs of AD; AE/AF of AW;
  // AA/AB/AE/AF/AW of AQ. A write to AA leaves AD's low byte (AB)
  // intact but invalidates the "AD as a whole holds slot X" tracking.
  // TargetRegisterInfo::regsOverlap captures this correctly across all
  // 8/16/32-bit family relationships including the 6309 wider forms.
  const TargetRegisterInfo *TRI = MF.getSubtarget().getRegisterInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    SmallVector<std::pair<SlotKey, Register>, 8> Slots;

    auto findSlot = [&](const SlotKey &K) -> Register * {
      for (auto &P : Slots)
        if (P.first == K)
          return &P.second;
      return nullptr;
    };
    auto recordSlot = [&](const SlotKey &K, Register DataReg) {
      if (auto *S = findSlot(K)) {
        *S = DataReg;
        return;
      }
      Slots.push_back({K, DataReg});
    };
    auto invalidateAll = [&]() { Slots.clear(); };
    auto invalidateReg = [&](Register R) {
      // Drop any slot whose tracked DATA reg OR BASE reg overlaps R.
      // The base-reg case matters for stack-pointer adjustments
      // (LEAS -2,S changes $ss, so any "slot N from $ss" tracked
      // before the LEAS now refers to a different memory location).
      // Frame-pointer ($su) base is stable within a function body
      // by construction, but we treat it uniformly for safety.
      Slots.erase(
          std::remove_if(Slots.begin(), Slots.end(),
                         [&, R](const std::pair<SlotKey, Register> &P) {
                           return TRI->regsOverlap(P.second, R) ||
                                  TRI->regsOverlap(P.first.Base, R);
                         }),
          Slots.end());
    };

    for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
      if (MI.isCall() || MI.isInlineAsm() || MI.isReturn() ||
          MI.isBranch() || MI.isTerminator()) {
        invalidateAll();
        continue;
      }

      auto Cls = classify(MI.getOpcode());
      if (!Cls) {
        // Untracked: invalidate everything if it could touch memory or
        // any tracked data register.
        if (MI.mayLoad() || MI.mayStore()) {
          invalidateAll();
          continue;
        }
        // Otherwise, invalidate only tracked-data-reg defs.
        for (const MachineOperand &MO : MI.operands()) {
          if (MO.isReg() && MO.isDef() && MO.getReg().isPhysical())
            invalidateReg(MO.getReg());
        }
        continue;
      }

      // Build slot key.
      SlotKey K{Cls->DataReg, 0, Register()};
      if (MI.getNumOperands() >= 2 && MI.getOperand(0).isImm() &&
          MI.getOperand(1).isReg()) {
        K.Offset = MI.getOperand(0).getImm();
        K.Base = MI.getOperand(1).getReg();
      } else if (MI.getNumOperands() >= 1 && MI.getOperand(0).isReg()) {
        K.Offset = 0;
        K.Base = MI.getOperand(0).getReg();
      } else {
        invalidateAll();
        continue;
      }

      // Only track loads/stores whose base is the canonical frame pointer
      // ($su) or stack pointer ($ss). Anything with a dynamic base
      // register (X/Y/U used as a generic pointer via LEA/etc.) might
      // alias an $su-relative slot — we can't prove non-aliasing without
      // pointer analysis, so a tracked store through such a base is
      // treated as an opaque mayStore (invalidateAll), and a tracked load
      // through it is treated as opaque mayLoad (invalidateAll). This
      // gives up some elision opportunities but is necessary for
      // correctness — see the qsort failure traced to STD ,Y aliasing
      // with STY $399,u in the swapfunc loop.
      if (K.Base != MC6809::SU && K.Base != MC6809::SS) {
        invalidateAll();
        continue;
      }

      // Conservative invariant: AT MOST ONE slot tracks a given data
      // register at any time. On every tracked store or load (whether
      // elided or not), invalidate any prior slot tracking this data
      // reg before recording the new slot. Reasoning: keeping multiple
      // slots tracked under the same data reg is correct in principle
      // (their contents are equal at the moment of recording), but
      // creates fragile state — any subsequent partial invalidation
      // (sub-reg aliasing, base-reg moves, dynamic-base writes) has
      // multiple slots to invalidate, each with its own subtle rules,
      // and we've already burned several bug-fix cycles on edge cases.
      // Single-slot tracking gives up some elision opportunities but
      // is dramatically simpler to reason about.
      if (Cls->IsStore) {
        invalidateReg(Cls->DataReg);
        recordSlot(K, Cls->DataReg);
      } else {
        // Load: if the slot is currently tracked AND already in DataReg,
        // the load is redundant.
        if (Register *Existing = findSlot(K)) {
          if (*Existing == Cls->DataReg) {
            MI.eraseFromParent();
            ++NumStoreReloadsElided;
            Changed = true;
            // Even though elided, normalize tracking to single-slot
            // invariant. The slot already tracks DataReg, so this is
            // mostly a no-op, but it ensures any earlier-recorded
            // duplicate-tracking (from the time before this fix) gets
            // cleaned up.
            invalidateReg(Cls->DataReg);
            recordSlot(K, Cls->DataReg);
            continue;
          }
        }
        // Otherwise the load brings a fresh value into DataReg.
        invalidateReg(Cls->DataReg);
        recordSlot(K, Cls->DataReg);
      }
    }
  }
  return Changed;
}

// Class 6 — Branch-over-branch fold (bug #179).
//
// Pattern in real picolibc output (especially at -O0 where
// BranchFolderPass doesn't run, but also a small residual at -O2+):
//
//     LBlbc cc, L1     ; conditional branch
//     LBRAlb L2        ; unconditional branch
//   <next_block>:
//
// If `next_block == L1` (the conditional target IS the layout
// successor), invert the condition and retarget to L2, then drop the
// unconditional. Result:
//
//     LBlbc !cc, L2
//   L1:                ; (fall through)
//
// If `next_block == L2` (the unconditional's target IS the layout
// successor), the unconditional is purely redundant — drop it. The
// conditional stays unchanged. Result:
//
//     LBlbc cc, L1
//   L2:                ; (fall through; the conditional falls through
//                       ; to L2 when condition is false)
//
// If neither is the layout successor, the pair genuinely needs both
// branches and we leave it alone. (Such cases at -O2+ are
// MachineBlockPlacement-fundamental — they'd require deeper layout
// intervention to address. Out of scope here.)
//
// LLVM-standard equivalent: lib/CodeGen/BranchFolding.cpp does this
// fold at -O>=1, AND the analyzeBranch hook at MC6809InstrInfo.cpp:662
// implements the same shape inside the analysis itself when called
// with AllowModify=true. Both paths skip -O0, hence the need for this
// safety net here. Per-class gating defaults to min-O=0 so this fires
// at every -O level — duplicating BranchFolder's work at -O>=1 is
// harmless (we just no-op when nothing's left to fold) and provides
// defence-in-depth.
bool MC6809FinalLowering::foldBranchOverBranch(MachineFunction &MF) {
  const auto *TII = static_cast<const MC6809InstrInfo *>(
      MF.getSubtarget().getInstrInfo());

  auto isCondBranch = [](unsigned Opc) {
    return Opc == MC6809::Bbc || Opc == MC6809::LBlbc ||
           Opc == MC6809::ConditionalBranchRelative ||
           Opc == MC6809::ConditionalLongBranchRelative;
  };
  auto isUncondBranch = [](unsigned Opc) {
    return Opc == MC6809::BRAb || Opc == MC6809::LBRAlb ||
           Opc == MC6809::BranchRelative ||
           Opc == MC6809::LongBranchRelative;
  };

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    // Find the last terminator (uncond branch) and the one before it
    // (must be a conditional branch). Skip blocks that don't end in
    // exactly this two-terminator shape.
    auto It = MBB.getLastNonDebugInstr();
    if (It == MBB.end() || !isUncondBranch(It->getOpcode()))
      continue;
    MachineBasicBlock::iterator UncondMI = It;

    // Walk back over any debug values to find the previous instruction.
    auto Prev = UncondMI;
    if (Prev == MBB.begin())
      continue;
    --Prev;
    while (Prev != MBB.begin() && Prev->isDebugInstr())
      --Prev;
    if (Prev->isDebugInstr() || !isCondBranch(Prev->getOpcode()))
      continue;
    MachineBasicBlock::iterator CondMI = Prev;

    // Operand layout (verified against analyzeBranch at line 602-613):
    //   Conditional: operand 0 = CC immediate, operand 1 = MBB target.
    //   Unconditional: operand 0 = MBB target.
    if (CondMI->getNumExplicitOperands() != 2 ||
        !CondMI->getOperand(0).isImm() || !CondMI->getOperand(1).isMBB() ||
        UncondMI->getNumExplicitOperands() != 1 ||
        !UncondMI->getOperand(0).isMBB())
      continue;

    MachineBasicBlock *L1 = CondMI->getOperand(1).getMBB();    // cond target
    MachineBasicBlock *L2 = UncondMI->getOperand(0).getMBB();  // uncond target
    MachineBasicBlock *LayoutNext = MBB.getNextNode();
    if (LayoutNext == nullptr)
      continue;

    if (LayoutNext == L2) {
      // Uncond goes to the next block — purely redundant. Drop it.
      UncondMI->eraseFromParent();
      ++NumBranchOverBranchFolded;
      Changed = true;
    } else if (LayoutNext == L1) {
      // Cond targets the next block — invert the CC, retarget the cond
      // to L2, drop the uncond. Use the existing getOppositeCondition
      // helper (see MC6809.h) which is already tested by analyzeBranch's
      // early-fold path.
      MC6809CC::CondCode CC =
          MC6809CC::CondCode(CondMI->getOperand(0).getImm());
      MC6809CC::CondCode Inverted = MC6809CC::getOppositeCondition(CC);
      // INVALID is the bail-out sentinel; if any branch we recognise
      // somehow has it, leave alone rather than miscompiling.
      if (Inverted == MC6809CC::INVALID)
        continue;
      CondMI->getOperand(0).setImm(Inverted);
      CondMI->getOperand(1).setMBB(L2);
      UncondMI->eraseFromParent();
      ++NumBranchOverBranchFolded;
      Changed = true;
    }
    // else: neither target is the layout successor — leave as-is.

    (void)TII;  // unused for now; held for future symmetry with other classes.
  }
  return Changed;
}

char MC6809FinalLowering::ID = 0;

INITIALIZE_PASS(MC6809FinalLowering, DEBUG_TYPE,
                "MC6809 Final Lowering", false, false)

MachineFunctionPass *llvm::createMC6809FinalLoweringPass() {
  return new MC6809FinalLowering();
}
