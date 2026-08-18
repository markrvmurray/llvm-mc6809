//===-- MC6809FoldBankCross.cpp - Fold INDEX->ACCUM copies into uses ------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Eliminate the bank-crossing copy a value picks up when it is produced in an
// index register (a pointer, an index computation, a call result returned in X)
// but consumed by an operation modelled in the accumulator bank.
//
//     %d:acc16 = COPY %x:index16      (tfr x,d)
//     Store_i16_Mem %d, %base, off    (std off,base)   -> Store_iPtr_Mem %x, ...   (stx off,base)
//     Compare_i16_Imm cc, %d, #imm    (cmpd #imm)      -> Compare_ptr_Imm cc, %x   (cmpx #imm)
//     Compare_i16_Mem cc, %d, %b, off (cmpd off,b)     -> Compare_ptr_Mem cc, %x   (cmpx off,b)
//     Add_i16_Reg %a, %d              (tfr x,d ...)    -> Add_i16_RegIdx %a, %x    (pshs x; addd ,s++)
//     (and Sub/AND/OR/XOR/carry-chain and Compare _Reg forms: the second
//     source is read from memory after a push either way)
//
// MC6809 has index-register store and compare forms (STX/STY/STU, CMPX/CMPY/
// CMPU), so the copy into D is pure overhead. Folding it pre-register-allocation
// means the value never occupies an accumulator at all — the allocator keeps it
// in one index register across the use, instead of pinning D for a copy that a
// later peephole would only undo locally.
//
// The Compare_i16/Compare_ptr and Store_i16/Store_iPtr pseudo families share an
// identical operand layout (only the value operand's register class differs),
// so the fold just retargets the opcode and the value operand. The
// register-register compare form (Compare_i16_Reg) is deliberately excluded:
// base 6809 has no register-register index compare, so that case stays in the
// accumulator bank. The _Imm and _Mem forms compare against an immediate or a
// memory operand and are a clean, copy-free win.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/SmallPtrSet.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-fold-bank-cross"

using namespace llvm;

static cl::opt<bool> EnableFoldBankCross(
    "mc6809-enable-fold-bank-cross", cl::init(true), cl::Hidden,
    cl::desc("Fold an INDEX->ACCUM copy into the store/compare that uses it so "
             "the value stays in an index register (stx/cmpx, no tfr x,d)"));

namespace {

class MC6809FoldBankCross : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldBankCross() : MachineFunctionPass(ID) {
    llvm::initializeMC6809FoldBankCrossPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold index->accumulator copy into store/compare";
  }
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineLoopInfoWrapperPass>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

// For a foldable consumer of an accumulator value, return the index-bank
// opcode it becomes and the operand index that holds the value. Returns
// {0, -1} for anything that must stay in the accumulator.
struct FoldTarget {
  unsigned Opc;
  int ValueOp;
};
static FoldTarget indexConsumer(unsigned Opc) {
  switch (Opc) {
  case MC6809::Store_i16_Mem:         return {MC6809::Store_iPtr_Mem, 0};
  case MC6809::Store_i16_Sym:         return {MC6809::Store_iPtr_Sym, 0};
  case MC6809::Compare_i16_Imm:       return {MC6809::Compare_ptr_Imm, 2};
  case MC6809::Compare_i16_Mem:       return {MC6809::Compare_ptr_Mem, 2};
  case MC6809::Compare_i16_Sym:       return {MC6809::Compare_ptr_Sym, 2};
  case MC6809::CompareBranch_i16_Imm: return {MC6809::CompareBranch_ptr_Imm, 1};
  case MC6809::CompareBranch_i16_Mem: return {MC6809::CompareBranch_ptr_Mem, 1};
  case MC6809::CompareBranch_i16_Sym: return {MC6809::CompareBranch_ptr_Sym, 1};
  // The SECOND source of a 16-bit register-register arithmetic, bitwise or
  // compare pseudo may be an index register (the _RegIdx sibling): the
  // 6809 reads that operand from memory after a push either way, so a
  // value that arrived in X or Y stays there instead of being copied into
  // D and evicting the tied operand into an imaginary register.
  case MC6809::Add_i16_Reg:               return {MC6809::Add_i16_RegIdx, 2};
  case MC6809::Sub_i16_Reg:               return {MC6809::Sub_i16_RegIdx, 2};
  case MC6809::AddSetCarry_i16_Reg:       return {MC6809::AddSetCarry_i16_RegIdx, 2};
  case MC6809::SubSetCarry_i16_Reg:       return {MC6809::SubSetCarry_i16_RegIdx, 2};
  case MC6809::AddSetCarryUse_i16_Reg:    return {MC6809::AddSetCarryUse_i16_RegIdx, 2};
  case MC6809::SubSetCarryUse_i16_Reg:    return {MC6809::SubSetCarryUse_i16_RegIdx, 2};
  case MC6809::AddSetOverflow_i16_Reg:    return {MC6809::AddSetOverflow_i16_RegIdx, 2};
  case MC6809::SubSetOverflow_i16_Reg:    return {MC6809::SubSetOverflow_i16_RegIdx, 2};
  case MC6809::AddSetOverflowUse_i16_Reg: return {MC6809::AddSetOverflowUse_i16_RegIdx, 2};
  case MC6809::SubSetOverflowUse_i16_Reg: return {MC6809::SubSetOverflowUse_i16_RegIdx, 2};
  case MC6809::AND_i16_Reg:               return {MC6809::AND_i16_RegIdx, 2};
  case MC6809::OR_i16_Reg:                return {MC6809::OR_i16_RegIdx, 2};
  case MC6809::XOR_i16_Reg:               return {MC6809::XOR_i16_RegIdx, 2};
  case MC6809::Compare_i16_Reg:           return {MC6809::Compare_i16_RegIdx, 3};
  case MC6809::CompareBranch_i16_Reg:     return {MC6809::CompareBranch_i16_RegIdx, 2};
  default:                            return {0, -1};
  }
}


// ---- Whole-value promotion into the index bank --------------------------
//
// The bank of a 16-bit integer is decided by its type: the accumulator
// bank. That bank has one real register (D); a second live 16-bit value
// lands in an imaginary (direct-page) register and every access stages
// through D. Many such values -- loop counters, lengths, values that are
// only loaded, stored, compared, stepped by a constant and moved -- would
// be just as happy in X or Y: LDX/STX/CMPX/LEAX exist for all of that.
//
// After selection every use of a value is a concrete pseudo, so it is easy
// to see which values touch nothing an index register cannot do: such a
// value (and everything it is copied to and from, or merged with in a phi)
// is re-classed INDEX16 and every instruction on it retargeted to its
// index-register sibling.

static cl::opt<bool> EnableIndexPromotion(
    "mc6809-enable-index-promotion", cl::init(true), cl::Hidden,
    cl::desc("Re-class 16-bit accumulator values whose every use has an "
             "index-register form into the index bank"));

// The index-bank sibling of an accumulator-bank pseudo for a promoted value
// in operand OpIdx. Returns 0 when the operand may not be promoted; appends
// to Linked the other values that must be promoted along (a phi's inputs
// and result, the other side of a copy, the tied source or result of an
// add-immediate). Rebuild says the instruction's operands change shape.
static constexpr unsigned KeepOpcode = ~0u;

static unsigned indexSibling(const MachineInstr &MI, unsigned OpIdx,
                             const MachineRegisterInfo &MRI,
                             SmallVectorImpl<Register> &Linked, bool &Rebuild) {
  Rebuild = false;
  unsigned Opc = MI.getOpcode();
  switch (Opc) {
  // Phis and copies keep their opcode (PHI's opcode number is 0, so a
  // sentinel stands for "unchanged").
  case TargetOpcode::PHI:
    for (unsigned I = 1; I < MI.getNumOperands(); I += 2)
      Linked.push_back(MI.getOperand(I).getReg());
    Linked.push_back(MI.getOperand(0).getReg());
    return KeepOpcode;
  case TargetOpcode::COPY: {
    Register Other = MI.getOperand(OpIdx == 0 ? 1 : 0).getReg();
    if (Other.isPhysical())
      return (Other == MC6809::IX || Other == MC6809::IY) ? KeepOpcode : 0;
    const TargetRegisterClass *RC = MRI.getRegClassOrNull(Other);
    if (RC && MC6809::INDEX16RegClass.hasSubClassEq(RC))
      return KeepOpcode;
    Linked.push_back(Other);
    return KeepOpcode;
  }
  case MC6809::Load_i16_Imm:  return OpIdx == 0 ? MC6809::Load_iPtr_Imm : 0;
  case MC6809::Load_i16_Mem:  return OpIdx == 0 ? MC6809::Load_iPtr_Mem : 0;
  case MC6809::Load_i16_MemIndirect:
    return OpIdx == 0 ? MC6809::Load_iPtr_MemIndirect : 0;
  case MC6809::Load_i16_Sym:  return OpIdx == 0 ? MC6809::Load_iPtr_Sym : 0;
  case MC6809::Store_i16_Mem: return OpIdx == 0 ? MC6809::Store_iPtr_Mem : 0;
  case MC6809::Store_i16_Sym: return OpIdx == 0 ? MC6809::Store_iPtr_Sym : 0;
  case MC6809::Compare_i16_Imm: return OpIdx == 2 ? MC6809::Compare_ptr_Imm : 0;
  case MC6809::Compare_i16_Mem: return OpIdx == 2 ? MC6809::Compare_ptr_Mem : 0;
  case MC6809::Compare_i16_Sym: return OpIdx == 2 ? MC6809::Compare_ptr_Sym : 0;
  case MC6809::CompareBranch_i16_Imm:
    return OpIdx == 1 ? MC6809::CompareBranch_ptr_Imm : 0;
  case MC6809::CompareBranch_i16_Mem:
    return OpIdx == 1 ? MC6809::CompareBranch_ptr_Mem : 0;
  case MC6809::CompareBranch_i16_Sym:
    return OpIdx == 1 ? MC6809::CompareBranch_ptr_Sym : 0;
  case MC6809::Test_i16_Reg:
    Rebuild = true;
    return OpIdx == 2 ? MC6809::Compare_ptr_Imm : 0;
  case MC6809::TestBranch_i16_Reg:
    Rebuild = true;
    return OpIdx == 1 ? MC6809::CompareBranch_ptr_Imm : 0;
  case MC6809::Add_i16_Imm:
  case MC6809::Sub_i16_Imm:
    // Result and tied source go together; the pair becomes a LEA.
    if (OpIdx > 1)
      return 0;
    Linked.push_back(MI.getOperand(OpIdx == 0 ? 1 : 0).getReg());
    Rebuild = true;
    return MC6809::LEAPtrAdd_Imm;
  default: {
    // The second source of a two-source arithmetic/bitwise/compare pseudo
    // (its _RegIdx sibling).
    FoldTarget FT = indexConsumer(Opc);
    if (FT.Opc && FT.ValueOp == int(OpIdx) && Opc != MC6809::Store_i16_Mem &&
        Opc != MC6809::Store_i16_Sym)
      return FT.Opc;
    return 0;
  }
  }
}

static bool promoteToIndexBank(MachineFunction &MF, const MachineLoopInfo &MLI) {
  if (!EnableIndexPromotion)
    return false;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();

  // Candidates: 16-bit accumulator-class virtual registers used whole.
  DenseSet<Register> Cand;
  for (unsigned I = 0, E = MRI.getNumVirtRegs(); I != E; ++I) {
    Register R = Register::index2VirtReg(I);
    if (MRI.reg_nodbg_empty(R))
      continue;
    const TargetRegisterClass *RC = MRI.getRegClassOrNull(R);
    if (!RC || !MC6809::ACC16RegClass.hasSubClassEq(RC))
      continue;
    bool Ok = true;
    for (const MachineOperand &MO : MRI.reg_nodbg_operands(R))
      if (MO.getSubReg()) {
        Ok = false;
        break;
      }
    if (Ok)
      Cand.insert(R);
  }
  // Fixed point: drop any candidate one of whose instructions has no index
  // sibling for it, or that is linked to a value that is neither an index
  // value nor a candidate.
  bool Shrunk = true;
  while (Shrunk) {
    Shrunk = false;
    for (Register R : SmallVector<Register, 32>(Cand.begin(), Cand.end())) {
      bool Ok = true;
      for (const MachineOperand &MO : MRI.reg_nodbg_operands(R)) {
        const MachineInstr &MI = *MO.getParent();
        SmallVector<Register, 4> Linked;
        bool Rebuild;
        if (!indexSibling(MI, MO.getOperandNo(), MRI, Linked, Rebuild)) {
          LLVM_DEBUG(dbgs() << "promote: " << printReg(R) << " blocked by " << MI);
          Ok = false;
          break;
        }
        for (Register L : Linked) {
          if (L == R || !L.isVirtual())
            continue;
          const TargetRegisterClass *LRC = MRI.getRegClassOrNull(L);
          if (LRC && MC6809::INDEX16RegClass.hasSubClassEq(LRC))
            continue;
          if (!Cand.count(L)) {
            LLVM_DEBUG(dbgs() << "promote: " << printReg(R) << " linked to non-candidate " << printReg(L) << " in " << MI);
            Ok = false;
            break;
          }
        }
        if (!Ok)
          break;
      }
      if (!Ok) {
        Cand.erase(R);
        Shrunk = true;
      }
    }
  }
  if (Cand.empty())
    return false;

  // Two index registers: a loop that already keeps two index values busy
  // has no room for a promoted one, and a spilled index counter is worse
  // than an imaginary accumulator one. Count, per loop, the index-class
  // values and candidates referenced inside it; where that exceeds two,
  // the candidates of that loop stay in the accumulator bank (and drag
  // their linked values along through the fixed point below).
  const unsigned IndexRegs = 2;
  bool Dropped = false;
  // Union-find over the function's values: a phi and its inputs, a copy's
  // two sides and a tied source that dies at its instruction (a stepped
  // pointer) are one value for register purposes.
  DenseMap<Register, Register> Parent;
  auto Find = [&](Register R) {
    Register Root = R;
    while (Parent.count(Root) && Parent[Root] != Root)
      Root = Parent[Root];
    while (Parent.count(R) && Parent[R] != Root) {
      Register N = Parent[R];
      Parent[R] = Root;
      R = N;
    }
    return Root;
  };
  auto Union = [&](Register A, Register B) {
    if (!Parent.count(A)) Parent[A] = A;
    if (!Parent.count(B)) Parent[B] = B;
    Register RA = Find(A), RB = Find(B);
    if (RA != RB) Parent[RA] = RB;
  };
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB) {
      if (MI.isPHI()) {
        // Inputs defined in one and the same block are live at once (a
        // lowered select): they cannot share a register, so they are not
        // one value here even though the phi joins them.
        SmallDenseSet<MachineBasicBlock *, 4> DefBlocks;
        bool Clash = false;
        for (unsigned I = 1; I < MI.getNumOperands(); I += 2) {
          Register In = MI.getOperand(I).getReg();
          if (!In.isVirtual())
            continue;
          if (MachineInstr *Def = MRI.getVRegDef(In))
            if (!DefBlocks.insert(Def->getParent()).second)
              Clash = true;
        }
        if (!Clash)
          for (unsigned I = 1; I < MI.getNumOperands(); I += 2)
            if (MI.getOperand(I).getReg().isVirtual())
              Union(MI.getOperand(0).getReg(), MI.getOperand(I).getReg());
        continue;
      }
      if (MI.isCopy() && MI.getOperand(0).getReg().isVirtual() &&
          MI.getOperand(1).getReg().isVirtual()) {
        Union(MI.getOperand(0).getReg(), MI.getOperand(1).getReg());
        continue;
      }
      for (unsigned I = 0; I < MI.getNumOperands(); ++I) {
        const MachineOperand &MO = MI.getOperand(I);
        unsigned Tied;
        if (MO.isReg() && MO.isDef() && MO.getReg().isVirtual() &&
            MI.isRegTiedToUseOperand(I, &Tied) &&
            MI.getOperand(Tied).getReg().isVirtual() &&
            MRI.hasOneNonDBGUse(MI.getOperand(Tied).getReg()))
          Union(MO.getReg(), MI.getOperand(Tied).getReg());
      }
    }
  // Regions to budget: every loop (its counter competes with the loop's
  // pointers for the whole trip) and every block (a straight-line stretch
  // holding three 16-bit values at once cannot keep them all in X and Y).
  SmallVector<SmallVector<MachineBasicBlock *, 8>, 16> Regions;
  for (MachineLoop *L : MLI.getLoopsInPreorder())
    Regions.emplace_back(L->getBlocks().begin(), L->getBlocks().end());
  for (MachineBasicBlock &MBB : MF)
    Regions.push_back({&MBB});
  for (auto &Blocks : Regions) {
    DenseSet<Register> IndexRoots, CandRoots, InRegion;
    for (MachineBasicBlock *MBB : Blocks)
      for (MachineInstr &MI : *MBB)
        for (const MachineOperand &MO : MI.operands()) {
          if (!MO.isReg() || !MO.getReg().isVirtual())
            continue;
          Register R = MO.getReg();
          if (Cand.count(R)) {
            InRegion.insert(R);
            CandRoots.insert(Find(R));
            continue;
          }
          const TargetRegisterClass *RC = MRI.getRegClassOrNull(R);
          if (RC && MC6809::INDEX16RegClass.hasSubClassEq(RC))
            IndexRoots.insert(Find(R));
        }
    if (InRegion.empty())
      continue;
    // A candidate group already merged with an index value (copied from a
    // pointer) costs nothing extra.
    unsigned Extra = 0;
    for (Register R : CandRoots)
      if (!IndexRoots.count(R))
        ++Extra;
    if (IndexRoots.size() + Extra > IndexRegs) {
      LLVM_DEBUG(dbgs() << "promote: region with " << IndexRoots.size()
                        << " index values and " << Extra
                        << " candidate groups -- keeping candidates in the accumulator\n");
      for (Register R : InRegion)
        Cand.erase(R);
      Dropped = true;
    }
  }
  if (Dropped) {
    // Re-run the fixed point so linked values follow.
    Shrunk = true;
    while (Shrunk) {
      Shrunk = false;
      for (Register R : SmallVector<Register, 32>(Cand.begin(), Cand.end())) {
        bool Ok = true;
        for (const MachineOperand &MO : MRI.reg_nodbg_operands(R)) {
          SmallVector<Register, 4> Linked;
          bool Rebuild;
          if (!indexSibling(*MO.getParent(), MO.getOperandNo(), MRI, Linked, Rebuild)) {
            Ok = false;
            break;
          }
          for (Register L : Linked) {
            if (L == R || !L.isVirtual())
              continue;
            const TargetRegisterClass *LRC = MRI.getRegClassOrNull(L);
            if (LRC && MC6809::INDEX16RegClass.hasSubClassEq(LRC))
              continue;
            if (!Cand.count(L)) {
              Ok = false;
              break;
            }
          }
          if (!Ok)
            break;
        }
        if (!Ok) {
          Cand.erase(R);
          Shrunk = true;
        }
      }
    }
    if (Cand.empty())
      return false;
  }

  // Apply: re-class, then retarget every instruction on a promoted value
  // (each instruction once; phis and copies need no new opcode).
  for (Register R : Cand)
    MRI.setRegClass(R, &MC6809::INDEX16RegClass);
  SmallPtrSet<MachineInstr *, 32> Done;
  SmallVector<MachineInstr *, 32> Work;
  for (Register R : Cand)
    for (MachineInstr &MI : MRI.reg_nodbg_instructions(R))
      if (Done.insert(&MI).second)
        Work.push_back(&MI);
  for (MachineInstr *MIP : Work) {
    MachineInstr &MI = *MIP;
    unsigned Opc = MI.getOpcode();
    if (Opc == TargetOpcode::PHI || Opc == TargetOpcode::COPY)
      continue;
    unsigned OpIdx = ~0u;
    for (unsigned I = 0; I < MI.getNumOperands(); ++I)
      if (MI.getOperand(I).isReg() && Cand.count(MI.getOperand(I).getReg())) {
        OpIdx = I;
        break;
      }
    SmallVector<Register, 4> Linked;
    bool Rebuild;
    unsigned NewOpc = indexSibling(MI, OpIdx, MRI, Linked, Rebuild);
    assert(NewOpc && "promoted value in an instruction without a sibling");
    if (!Rebuild) {
      MI.setDesc(TII.get(NewOpc));
      continue;
    }
    MachineBasicBlock &MBB = *MI.getParent();
    const DebugLoc &DL = MI.getDebugLoc();
    if (Opc == MC6809::Add_i16_Imm || Opc == MC6809::Sub_i16_Imm) {
      int64_t Imm = MI.getOperand(2).getImm();
      if (Opc == MC6809::Sub_i16_Imm)
        Imm = -Imm;
      Imm = (int16_t)Imm;
      BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Imm),
              MI.getOperand(0).getReg())
          .addReg(MI.getOperand(1).getReg())
          .addImm(Imm);
    } else if (Opc == MC6809::Test_i16_Reg) {
      BuildMI(MBB, MI, DL, TII.get(MC6809::Compare_ptr_Imm),
              MI.getOperand(0).getReg())
          .add(MI.getOperand(1))
          .add(MI.getOperand(2))
          .addImm(0);
    } else if (Opc == MC6809::TestBranch_i16_Reg) {
      BuildMI(MBB, MI, DL, TII.get(MC6809::CompareBranch_ptr_Imm))
          .add(MI.getOperand(0))
          .add(MI.getOperand(1))
          .addImm(0)
          .add(MI.getOperand(2));
    } else {
      llvm_unreachable("unexpected rebuild");
    }
    MI.eraseFromParent();
  }
  return true;
}

} // namespace

bool MC6809FoldBankCross::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldBankCross)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  bool Changed = promoteToIndexBank(
      MF, getAnalysis<MachineLoopInfoWrapperPass>().getLI());

  // Collect the INDEX->ACCUM copies first; rewriting erases them.
  SmallVector<MachineInstr *, 8> Copies;
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB) {
      if (!MI.isCopy())
        continue;
      Register Dst = MI.getOperand(0).getReg();
      Register Src = MI.getOperand(1).getReg();
      if (!Dst.isVirtual())
        continue;
      const TargetRegisterClass *DstRC = MRI.getRegClassOrNull(Dst);
      if (!DstRC || !MC6809::ACC16RegClass.hasSubClassEq(DstRC))
        continue;
      // The source is an index-bank virtual register, or one of the index
      // registers themselves (an incoming argument in X, a call result).
      if (Src.isVirtual()) {
        const TargetRegisterClass *SrcRC = MRI.getRegClassOrNull(Src);
        if (SrcRC && MC6809::INDEX16RegClass.hasSubClassEq(SrcRC))
          Copies.push_back(&MI);
      } else if (Src == MC6809::IX || Src == MC6809::IY) {
        Copies.push_back(&MI);
      }
    }
  for (MachineInstr *Copy : Copies) {
    Register AccReg = Copy->getOperand(0).getReg();
    Register IdxReg = Copy->getOperand(1).getReg();
    // A physical index source gets its own index-bank virtual register,
    // copied right where the accumulator copy was (the physical register is
    // known to hold the value there and nowhere else).
    if (IdxReg.isPhysical()) {
      bool AnyConsumer = false;
      for (MachineInstr &Use : MRI.use_nodbg_instructions(AccReg)) {
        FoldTarget FT = indexConsumer(Use.getOpcode());
        if (FT.Opc && Use.getOperand(FT.ValueOp).isReg() &&
            Use.getOperand(FT.ValueOp).getReg() == AccReg)
          AnyConsumer = true;
      }
      if (!AnyConsumer)
        continue;
      Register NewIdx = MRI.createVirtualRegister(&MC6809::INDEX16RegClass);
      BuildMI(*Copy->getParent(), std::next(Copy->getIterator()),
              Copy->getDebugLoc(), TII.get(TargetOpcode::COPY), NewIdx)
          .addReg(IdxReg);
      IdxReg = NewIdx;
    }

    // Retarget each store/compare that uses the accumulator value to read the
    // index register directly. Other uses keep the copy alive.
    for (MachineInstr &Use :
         llvm::make_early_inc_range(MRI.use_nodbg_instructions(AccReg))) {
      FoldTarget FT = indexConsumer(Use.getOpcode());
      if (!FT.Opc)
        continue;
      MachineOperand &ValMO = Use.getOperand(FT.ValueOp);
      if (!ValMO.isReg() || ValMO.getReg() != AccReg)
        continue;
      Use.setDesc(TII.get(FT.Opc));
      ValMO.setReg(IdxReg);
      Changed = true;
    }

    if (MRI.use_nodbg_empty(AccReg))
      Copy->eraseFromParent();
  }
  return Changed;
}

char MC6809FoldBankCross::ID = 0;

INITIALIZE_PASS_BEGIN(MC6809FoldBankCross, DEBUG_TYPE,
                      "MC6809 fold index->accumulator copy into store/compare",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfoWrapperPass)
INITIALIZE_PASS_END(MC6809FoldBankCross, DEBUG_TYPE,
                    "MC6809 fold index->accumulator copy into store/compare",
                    false, false)

MachineFunctionPass *llvm::createMC6809FoldBankCrossPass() {
  return new MC6809FoldBankCross();
}
