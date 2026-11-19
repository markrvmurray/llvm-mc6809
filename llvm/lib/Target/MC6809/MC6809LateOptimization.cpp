//===-- MC6809LateOptimization.cpp - MC6809 Late Optimization -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 late optimization pass.
//
// This pass performs simple optimizations once pseudo-instructions have been
// fully lowered. These optimizations might otherwise increase register pressure
// and cause spills, so they're done opportunistically at the very end.
//
//===----------------------------------------------------------------------===//

#include "MC6809LateOptimization.h"

#include "MC6809.h"
#include "MC6809InstrBuilder.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MC6809ValueTracker.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/Statistic.h"
#include "llvm/ADT/SmallSet.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/LivePhysRegs.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/Support/CommandLine.h"

#define DEBUG_TYPE "mc6809-late-opt"

STATISTIC(NumCompareZeroElided, "Redundant compares with zero removed");
STATISTIC(NumDPReloadsElided, "Redundant direct-page reloads removed");
STATISTIC(NumLEAIntoLoad, "lea r,d; ld ,r folded to ld d,b");
STATISTIC(NumMemoryRMW, "ld; op; st of one byte folded to a memory operation");
STATISTIC(NumPostIncLeaAfter, "op ,r; lea r k,r folded to op ,r+");
STATISTIC(NumWalkingCompare, "Successor pointer steps folded into a walking compare");
STATISTIC(NumWalkingCompensation, "Compensating lea inserted on the taken path of a walking compare");
STATISTIC(NumDiamondHoist, "Diamond false-arm loads hoisted");

using namespace llvm;

static cl::opt<bool> EnableDiamondFalseLoad(
    "mc6809-enable-diamond-false-load", cl::init(true), cl::Hidden,
    cl::desc("Load a compare result's false value ahead of the compare"));

namespace {

class MC6809LateOptimization : public MachineFunctionPass {
public:
  static char ID;

  MC6809LateOptimization() : MachineFunctionPass(ID) { llvm::initializeMC6809LateOptimizationPass(*PassRegistry::getPassRegistry()); }

  bool runOnMachineFunction(MachineFunction &MF) override;
  bool elideCompareZero(MachineBasicBlock &MBB) const;
  bool elideRedundantLoad(MachineBasicBlock &MBB) const;
  bool foldLEAIntoLoad(MachineBasicBlock &MBB) const;
  bool formPostIncrement(MachineBasicBlock &MBB) const;
  bool formWalkingCompareBranch(MachineBasicBlock &MBB) const;
  bool hoistDiamondFalseLoad(MachineBasicBlock &MBB) const;
  bool foldMemoryRMW(MachineBasicBlock &MBB) const;
};

bool MC6809LateOptimization::runOnMachineFunction(MachineFunction &MF) {
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    Changed |= elideCompareZero(MBB);
    Changed |= elideRedundantLoad(MBB);
    Changed |= foldLEAIntoLoad(MBB);
    Changed |= formPostIncrement(MBB);
    Changed |= formWalkingCompareBranch(MBB);
    Changed |= hoistDiamondFalseLoad(MBB);
    Changed |= foldMemoryRMW(MBB);
  }
  return Changed;
}



// A byte loaded, changed and stored back to the same place, with the
// accumulator dead after, is one memory operation:
//     ldb 4,x ; andb #$f0 ; stb 4,x   ->   aim #$f0,4,x     (6309: and/or/eor)
//     ldb <v  ; incb      ; stb <v    ->   inc <v           (inc/dec, either CPU)
// The memory forms set N, Z and V as the store did (and leave C, as the
// logical immediates and inc/dec do); `add #1`/`sub #1` set C where inc/dec
// do not, so those fold only where C is dead. Indexed, direct-page and
// extended forms; not a volatile access.
bool MC6809LateOptimization::foldMemoryRMW(MachineBasicBlock &MBB) const {
  MachineFunction &MF = *MBB.getParent();
  const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MF.getSubtarget().getInstrInfo();
  bool Is6309 = MF.getSubtarget<MC6809Subtarget>().has6309();
  if (!MF.getRegInfo().tracksLiveness())
    return false;
  enum class Op { None, And, Or, Eor, Inc, Dec };
  // The access's form: indexed (with its IdxForm), direct page or extended.
  enum class Kind { None, Idx, Dir, Ext };
  auto Access = [&](const MachineInstr &MI, bool Store, Register &R,
                    MC6809::IdxForm &F) {
    if (auto LS = MC6809InstrInfo::getIdxLoadStore(MI.getOpcode());
        LS && LS->IsStore == Store) {
      R = LS->Value;
      F = *MC6809InstrInfo::getIdxForm(MI.getOpcode());
      return Kind::Idx;
    }
    if (auto LS = MC6809InstrInfo::getDirLoadStore(MI.getOpcode());
        LS && LS->IsStore == Store) {
      R = LS->Value;
      return Kind::Dir;
    }
    switch (MI.getOpcode()) {
    case MC6809::LDAe: R = MC6809::AA; return Store ? Kind::None : Kind::Ext;
    case MC6809::LDBe: R = MC6809::AB; return Store ? Kind::None : Kind::Ext;
    case MC6809::STAe: R = MC6809::AA; return Store ? Kind::Ext : Kind::None;
    case MC6809::STBe: R = MC6809::AB; return Store ? Kind::Ext : Kind::None;
    default: return Kind::None;
    }
  };
  // The operation on R, its immediate for and/or/eor, and whether the
  // memory form sets C differently (add/sub #1 against inc/dec).
  auto Operation = [&](const MachineInstr &MI, Register R, int64_t &Imm,
                       bool &SetsC) {
    bool A = R == MC6809::AA;
    unsigned O = MI.getOpcode();
    SetsC = false;
    auto ImmIs = [&](int64_t V) {
      return MI.getOperand(0).isImm() && MI.getOperand(0).getImm() == V;
    };
    if (O == (A ? MC6809::INCAa : MC6809::INCBa))
      return Op::Inc;
    if (O == (A ? MC6809::DECAa : MC6809::DECBa))
      return Op::Dec;
    if (O == (A ? MC6809::ADDAi8 : MC6809::ADDBi8) && ImmIs(1)) {
      SetsC = true;
      return Op::Inc;
    }
    if (O == (A ? MC6809::SUBAi8 : MC6809::SUBBi8) && ImmIs(1)) {
      SetsC = true;
      return Op::Dec;
    }
    if (!Is6309 || !MI.getOperand(0).isImm())
      return Op::None;
    Imm = MI.getOperand(0).getImm();
    if (O == (A ? MC6809::ANDAi8 : MC6809::ANDBi8))
      return Op::And;
    if (O == (A ? MC6809::ORAi8 : MC6809::ORBi8))
      return Op::Or;
    if (O == (A ? MC6809::EORAi8 : MC6809::EORBi8))
      return Op::Eor;
    return Op::None;
  };
  auto NextReal = [&](MachineBasicBlock::iterator It) {
    for (++It; It != MBB.end() && It->isDebugInstr(); ++It)
      ;
    return It;
  };
  auto ReadLater = [&](MachineBasicBlock::iterator After, Register R) {
    LivePhysRegs Live(TRI);
    Live.addLiveOuts(MBB);
    for (auto It = MBB.end(); It != After;)
      Live.stepBackward(*--It);
    return !Live.available(MF.getRegInfo(), R);
  };

  bool Changed = false;
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &Ld = *It++;
    Register R;
    MC6809::IdxForm LF{}, SF{};
    Kind K = Access(Ld, false, R, LF);
    if (K == Kind::None || (R != MC6809::AA && R != MC6809::AB) ||
        MC6809InstrInfo::hasOrderedMemOperand(Ld))
      continue;
    auto OIt = NextReal(Ld.getIterator());
    if (OIt == MBB.end())
      continue;
    int64_t Imm = 0;
    bool SetsC;
    Op O = Operation(*OIt, R, Imm, SetsC);
    if (O == Op::None)
      continue;
    auto SIt = NextReal(OIt);
    if (SIt == MBB.end())
      continue;
    MachineInstr &St = *SIt;
    Register SR;
    if (Access(St, true, SR, SF) != K || SR != R ||
        (K == Kind::Idx && SF != LF) ||
        MC6809InstrInfo::hasOrderedMemOperand(St))
      continue;
    // The same address: the explicit operands of the load and the store
    // name it (for a direct-page register, as a plain and an undef use).
    unsigned NL = Ld.getNumExplicitOperands(), NS = St.getNumExplicitOperands();
    if (NL != NS)
      continue;
    bool Same = true;
    for (unsigned I = 0; I < NL && Same; ++I) {
      const MachineOperand &A = Ld.getOperand(I), &B = St.getOperand(I);
      if (A.isReg() && B.isReg())
        Same = A.getReg() == B.getReg();
      else
        Same = A.isIdenticalTo(B);
    }
    if (!Same)
      continue;
    auto After = std::next(SIt);
    if (ReadLater(After, R) || (SetsC && ReadLater(After, MC6809::C)))
      continue;
    unsigned NewOpc = 0;
    bool HasImm = O == Op::And || O == Op::Or || O == Op::Eor;
    unsigned IdxBase = O == Op::And   ? MC6809::AIMii_o0
                       : O == Op::Or  ? MC6809::OIMii_o0
                       : O == Op::Eor ? MC6809::EIMii_o0
                       : O == Op::Inc ? MC6809::INCi_o0
                                      : MC6809::DECi_o0;
    if (K == Kind::Idx)
      NewOpc = MC6809InstrInfo::getIdxSibling(IdxBase, LF);
    else if (K == Kind::Dir)
      NewOpc = O == Op::And   ? MC6809::AIMid
               : O == Op::Or  ? MC6809::OIMid
               : O == Op::Eor ? MC6809::EIMid
               : O == Op::Inc ? MC6809::INCd
                              : MC6809::DECd;
    else
      NewOpc = O == Op::And   ? MC6809::AIMie
               : O == Op::Or  ? MC6809::OIMie
               : O == Op::Eor ? MC6809::EIMie
               : O == Op::Inc ? MC6809::INCe
                              : MC6809::DECe;
    if (!NewOpc)
      continue;
    MachineInstrBuilder New =
        BuildMI(MBB, St, St.getDebugLoc(), TII.get(NewOpc));
    // The indexed forms take the immediate first (`aim #v,o,r`); the direct
    // and extended forms take it after the address (`(ins addr, val)`).
    if (HasImm && K == Kind::Idx)
      New.addImm(Imm & 0xFF);
    for (unsigned I = 0; I < NL; ++I)
      New.add(Ld.getOperand(I)); // the load's: a plain read of the address
    if (HasImm && K != Kind::Idx)
      New.addImm(Imm & 0xFF);
    for (const MachineOperand &MO : St.implicit_operands())
      if (MO.isReg() && MO.isDef() &&
          !TRI.regsOverlap(MO.getReg(), MC6809::CC))
        New.add(MO); // the direct-page register the store writes
    New.cloneMergedMemRefs({&Ld, &St});
    if (It == OIt || It == SIt)
      It = std::next(SIt);
    Ld.eraseFromParent();
    OIt->eraseFromParent();
    St.eraseFromParent();
    ++NumMemoryRMW;
    Changed = true;
  }
  return Changed;
}

// The register a LEA{X,Y,U} of the `k,r` (o5) form steps, and k, when it
// steps its own base register (`leax 1,x`); an invalid register otherwise.
static Register leaSelfStep(const MachineInstr &MI, int &K) {
  switch (MI.getOpcode()) {
  case MC6809::LEAXi_o5:
  case MC6809::LEAYi_o5:
  case MC6809::LEAUi_o5:
    break;
  default:
    return Register();
  }
  // Operands: (imm, base); the stepped register is the descriptor's def.
  const MachineOperand &Off = MI.getOperand(0);
  if (!(Off.isImm() || Off.isCImm()) || !MI.getOperand(1).isReg())
    return Register();
  Register Stepped = MI.getOpcode() == MC6809::LEAXi_o5   ? MC6809::IX
                     : MI.getOpcode() == MC6809::LEAYi_o5 ? MC6809::IY
                                                          : MC6809::SU;
  if (MI.getOperand(1).getReg() != Stepped)
    return Register();
  K = Off.isImm() ? Off.getImm() : Off.getCImm()->getSExtValue();
  return Stepped;
}

// An indexed access `op off,r` (o0, or o5 with an immediate): its base and
// offset.
static bool indexedBaseOffset(const MachineInstr &MI, Register &Base,
                              int &Off) {
  int64_t O;
  bool Found = MC6809InstrInfo::getIdxBaseOffset(MI, Base, O);
  Off = static_cast<int>(O);
  return Found;
}

// Fold a step of an index register into the access it belongs with:
//   op ,r ; lea r k,r      ->  op ,r+   (k = the access width)
// (The other order, `lea r k,r ; op -k,r`, never arose in the C library or
// its tests at any bench level, and its rewrite is gone.)
// The step's own Z (LEAX/LEAY set it) must be dead, and the access must
// neither write the register it steps (`ldx ,x+` is not `ldx ,x ; leax
// 2,x`) nor read its value as an operand: the 6809 steps the register
// before the operation, so `stx ,x++` stores the stepped X and `cmpx ,x++`
// compares it.
// Does MI read R other than as its indexed base -- as an implicit operand
// (the stored register of STX, the compared register of CMPX)?
static bool readsAsValue(const MachineInstr &MI, Register R) {
  for (const MachineOperand &MO : MI.implicit_operands())
    if (MO.isReg() && MO.isUse() && MO.getReg() == R)
      return true;
  // The index-register stores and compares read their register whether or
  // not an expander spelled that out.
  switch (MI.getOpcode()) {
  case MC6809::STXi_o0: case MC6809::STXi_o5:
  case MC6809::CMPXi_o0: case MC6809::CMPXi_o5:
    return R == MC6809::IX;
  case MC6809::STYi_o0: case MC6809::STYi_o5:
  case MC6809::CMPYi_o0: case MC6809::CMPYi_o5:
    return R == MC6809::IY;
  case MC6809::STUi_o0: case MC6809::STUi_o5:
  case MC6809::CMPUi_o0: case MC6809::CMPUi_o5:
    return R == MC6809::SU;
  default:
    return false;
  }
}

// A search loop compares through its pointer and steps it afterwards, on
// the way round:
//     ldb ,x ; cmpb ,y ; bne FOUND     (this block)
//     ... ; leax 1,x ; leay 1,y ; ...  (the sole fall-through successor)
// Each step folds into the access it belongs with (`ldb ,x+ ; cmpb ,y+`)
// when the successor is reached from here alone: the compare's step, and
// the steps of the accesses just before the compare (the loads that feed
// it), wherever the step sits among the successor's first instructions,
// provided nothing between the access and its step touches the register.
// On the branch-taken path the register is then one step ahead; if it is
// live there, that path gets `lea r -k,r` at its start -- on a block of its
// own when the taken block is reached from elsewhere too -- provided
// nothing there reads the flags before writing them (LEAX sets Z).
bool MC6809LateOptimization::formWalkingCompareBranch(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  auto FirstTerm = MBB.getFirstTerminator();
  if (FirstTerm == MBB.begin() || FirstTerm == MBB.end())
    return false;
  // Terminators: one conditional branch, then optionally an unconditional
  // one (or fall-through). Taken and not-taken successors.
  MachineInstr &Bcc = *FirstTerm;
  if (!Bcc.isConditionalBranch() || !Bcc.getOperand(1).isMBB())
    return false;
  MachineBasicBlock *Taken = Bcc.getOperand(1).getMBB();
  MachineBasicBlock *Through = nullptr;
  auto Next = std::next(FirstTerm);
  if (Next == MBB.end()) {
    // Fall-through: the layout successor.
    auto LI = std::next(MBB.getIterator());
    if (LI == MBB.getParent()->end() || !MBB.isSuccessor(&*LI))
      return false;
    Through = &*LI;
  } else if (Next->isUnconditionalBranch() && Next->getOperand(0).isMBB() &&
             std::next(Next) == MBB.end()) {
    Through = Next->getOperand(0).getMBB();
  } else {
    return false;
  }
  if (Through == Taken || Through->pred_size() != 1)
    return false;
  // The candidates: the compare (last non-terminator) and the indexed
  // accesses just before it, each through X, Y or U at offset 0, neither
  // writing nor reading its base as a value.
  SmallVector<MachineInstr *, 4> Cands;
  for (auto It = std::prev(FirstTerm);; --It) {
    Register Base;
    int Off;
    if (It->isDebugInstr()) {
      if (It == MBB.begin())
        break;
      continue;
    }
    if (!indexedBaseOffset(*It, Base, Off) || Off != 0 ||
        (Base != MC6809::IX && Base != MC6809::IY && Base != MC6809::SU) ||
        It->modifiesRegister(Base, &TRI) || readsAsValue(*It, Base))
      break;
    Cands.push_back(&*It);
    if (It == MBB.begin() || Cands.size() == 4)
      break;
  }
  if (Cands.empty())
    return false;
  // The successor's head: where the steps may sit.
  SmallVector<MachineInstr *, 8> Head;
  for (MachineInstr &MI : *Through) {
    if (MI.isDebugInstr())
      continue;
    if (MI.isTerminator() || MI.isCall() || Head.size() == 8)
      break;
    Head.push_back(&MI);
  }
  bool Changed = false;
  MachineBasicBlock *CompDest = nullptr; // where compensations go
  SmallVector<Register, 2> Done;
  for (MachineInstr *Acc : Cands) {
    Register Base;
    int Off;
    indexedBaseOffset(*Acc, Base, Off);
    // Not the stack pointer: moving a step of S changes the offset of every
    // S-relative access and the frame's CFI, which this rewrite does not
    // model.
    if (Base == MC6809::SS || llvm::is_contained(Done, Base))
      continue;
    // Its step in the head.
    MachineInstr *Step = nullptr;
    int K = 0;
    for (MachineInstr *MI : Head)
      if (leaSelfStep(*MI, K) == Base) {
        Step = MI;
        break;
      }
    if (!Step || K <= 0)
      continue;
    unsigned IncOpc = MC6809InstrInfo::getPostIncrementOpcode(Acc->getOpcode(), K);
    if (!IncOpc)
      continue;
    // Nothing between the access and its step may touch the register.
    bool Touched = false;
    for (auto It = std::next(Acc->getIterator()); It != MBB.end(); ++It)
      if (It->readsRegister(Base, &TRI) || It->modifiesRegister(Base, &TRI)) {
        Touched = true;
        break;
      }
    for (MachineInstr *MI : Head) {
      if (MI == Step)
        break;
      if (MI->readsRegister(Base, &TRI) || MI->modifiesRegister(Base, &TRI))
        Touched = true;
    }
    if (Touched)
      continue;
    // The step's own Z must be dead where it was.
    if (Through->computeRegisterLiveness(
            &TRI, MC6809::Z,
            MachineBasicBlock::const_iterator(std::next(Step->getIterator()))) !=
        MachineBasicBlock::LQR_Dead)
      continue;
    // The taken path: the register either dead there, or compensated at
    // its start.
    bool NeedCompensation = false;
    for (const auto &LI : Taken->liveins())
      if (TRI.regsOverlap(LI.PhysReg, Base))
        NeedCompensation = true;
    if (NeedCompensation) {
      // Nothing on the taken path may read a flag before it is written (the
      // compensating LEA sets Z); stop at the first flag def.
      bool FlagRead = false;
      for (const MachineInstr &MI : *(CompDest ? CompDest : Taken)) {
        if (MI.isDebugInstr())
          continue;
        for (Register F : {MC6809::N, MC6809::Z, MC6809::V, MC6809::C})
          if (MI.readsRegister(F, &TRI))
            FlagRead = true;
        if (FlagRead || MI.modifiesRegister(MC6809::Z, &TRI))
          break;
      }
      if (FlagRead)
        continue;
      // Compensating LEA at the head of the taken block, or on a block of
      // its own on the edge when the taken block is reached from elsewhere.
      if (!CompDest) {
        CompDest = Taken;
        if (Taken->pred_size() != 1) {
          CompDest = MBB.SplitCriticalEdge(
              Taken, *const_cast<MC6809LateOptimization *>(this));
          if (!CompDest)
            return Changed;
        }
      }
      unsigned LeaOpc;
      switch (Base) {
      case MC6809::IX: LeaOpc = MC6809::LEAXi_o5; break;
      case MC6809::IY: LeaOpc = MC6809::LEAYi_o5; break;
      case MC6809::SU: LeaOpc = MC6809::LEAUi_o5; break;
      default: llvm_unreachable("walking access through a non-index register");
      }
      auto At = CompDest->begin();
      while (At != CompDest->end() && At->isDebugInstr())
        ++At;
      ++NumWalkingCompensation;
      BuildMI(*CompDest, At, Bcc.getDebugLoc(), TII.get(LeaOpc))
          .addImm(-K)
          .addReg(Base);
    }
    // Rewrite the access and drop the step.
    for (unsigned I = 0; I < Acc->getNumOperands(); ++I)
      if (Acc->getOperand(I).isImm() || Acc->getOperand(I).isCImm()) {
        Acc->removeOperand(I);
        break;
      }
    Acc->setDesc(TII.get(IncOpc));
    llvm::erase(Head, Step);
    Step->eraseFromParent();
    Done.push_back(Base);
    ++NumWalkingCompare;
    Changed = true;
  }
  return Changed;
}

// A materialised compare result is a diamond of two immediate loads:
//     cmpx #n ; beq T      (this block)
//     ldb #0 ; bra J       (F: the fall-through, reached from here alone)
//   T: ldb #1              (reached from here alone, falls into J)
//   J:
// The false load moves ahead of the flag producer, whose flags overwrite
// the load's, and the branch inverts over the true load:
//     ldb #0 ; cmpx #n ; bne J ; ldb #1 ; J:
// One conditional branch instead of a conditional and an unconditional,
// and a block fewer. Only when the register the loads write is free across
// the producer: not read or written by it, and not live before it -- known
// here, after allocation (a byte compare `cmpb` uses B itself; the value
// would then have to go elsewhere).
bool MC6809LateOptimization::hoistDiamondFalseLoad(MachineBasicBlock &MBB) const {
  if (!EnableDiamondFalseLoad)
    return false;
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  auto ImmLoadReg = [](const MachineInstr &MI) -> Register {
    switch (MI.getOpcode()) {
    case MC6809::LDAi8: return MC6809::AA;
    case MC6809::LDBi8: return MC6809::AB;
    case MC6809::LDEi8: return MC6809::AE;
    case MC6809::LDFi8: return MC6809::AF;
    default: return Register();
    }
  };
  // This block: flag producer, then one conditional branch, no unconditional.
  auto Term = MBB.getFirstTerminator();
  if (Term == MBB.end() || Term == MBB.begin() ||
      !Term->isConditionalBranch() || std::next(Term) != MBB.end() ||
      !Term->getOperand(1).isMBB())
    return false;
  MachineInstr &Bcc = *Term;
  MachineInstr &Producer = *std::prev(Term);
  // The producer must define every flag the branch could read (a load's
  // own N/Z/V must not leak through) -- a compare or test, not a LEA.
  for (Register F : {MC6809::N, MC6809::Z, MC6809::V})
    if (!Producer.modifiesRegister(F, &TRI))
      return false;
  MachineBasicBlock *T = Bcc.getOperand(1).getMBB();
  MachineBasicBlock *F = MBB.getNextNode();
  if (!F || F == T || !MBB.isSuccessor(F) || F->pred_size() != 1 ||
      T->pred_size() != 1 || F->getNextNode() != T)
    return false;
  MachineBasicBlock *J = T->getNextNode();
  if (!J || T->succ_size() != 1 || !T->isSuccessor(J) || F->succ_size() != 1 ||
      !F->isSuccessor(J))
    return false;
  // F: `ld r #k1 ; bra J`. T: `ld r #k2` (falls through).
  auto FI = F->begin(), TI = T->begin();
  while (FI != F->end() && FI->isDebugInstr()) ++FI;
  while (TI != T->end() && TI->isDebugInstr()) ++TI;
  if (FI == F->end() || TI == T->end())
    return false;
  MachineInstr &FalseLoad = *FI, &TrueLoad = *TI;
  Register R = ImmLoadReg(FalseLoad);
  if (!R || ImmLoadReg(TrueLoad) != R)
    return false;
  auto FNext = std::next(FI);
  while (FNext != F->end() && FNext->isDebugInstr()) ++FNext;
  if (FNext == F->end() || !FNext->isUnconditionalBranch() ||
      !FNext->getOperand(0).isMBB() || FNext->getOperand(0).getMBB() != J ||
      std::next(FNext) != F->end())
    return false;
  auto TNext = std::next(TI);
  while (TNext != T->end() && TNext->isDebugInstr()) ++TNext;
  if (TNext != T->end())
    return false;
  // R free across the producer: it neither reads nor writes R, and R is not
  // live into the producer (nothing after the branch reads it before the
  // loads: T and F both write it first, so only the producer matters).
  if (Producer.readsRegister(R, &TRI) || Producer.modifiesRegister(R, &TRI))
    return false;
  if (MBB.computeRegisterLiveness(&TRI, R, MachineBasicBlock::const_iterator(Producer.getIterator())) !=
      MachineBasicBlock::LQR_Dead)
    return false;
  MC6809CC::CondCode Inv = MC6809CC::getOppositeCondition(
      MC6809CC::CondCode(Bcc.getOperand(0).getImm()));
  if (Inv == MC6809CC::INVALID)
    return false;
  // Move the false load ahead of the producer (its flags now dead), invert
  // the branch to J, drop F, and let T fall through into J.
  FalseLoad.removeFromParent();
  MBB.insert(Producer.getIterator(), &FalseLoad);
  for (MachineOperand &MO : FalseLoad.operands())
    if (MO.isReg() && MO.isDef() && MO.isImplicit() &&
        (MO.getReg() == MC6809::NZ || MO.getReg() == MC6809::N ||
         MO.getReg() == MC6809::Z || MO.getReg() == MC6809::V))
      MO.setIsDead();
  Bcc.getOperand(0).setImm(Inv);
  Bcc.getOperand(1).setMBB(J);
  auto PT = MBB.getSuccProbability(llvm::find(MBB.successors(), T));
  auto PF = MBB.getSuccProbability(llvm::find(MBB.successors(), F));
  MBB.removeSuccessor(F);
  MBB.removeSuccessor(T);
  MBB.addSuccessor(J, PF);
  MBB.addSuccessor(T, PT);
  F->removeSuccessor(J);
  F->eraseFromParent();
  // The register the loads write is live out of MBB into J now (through
  // the false load); T's live-ins already exclude it (T defines it).
  if (!J->isLiveIn(R))
    J->addLiveIn(R);
  (void)TII;
  ++NumDiamondHoist;
  return true;
}

bool MC6809LateOptimization::formPostIncrement(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  bool Changed = false;
  auto NextReal = [&](MachineBasicBlock::iterator It) {
    ++It;
    while (It != MBB.end() && It->isDebugInstr())
      ++It;
    return It;
  };
  auto Rewrite = [&](MachineInstr &Access, MachineInstr &Lea, unsigned IncOpc) {
    // The Inc form's explicit operand is the register alone: drop the
    // offset immediate of an o5 form.
    for (unsigned I = 0; I < Access.getNumOperands(); ++I)
      if (Access.getOperand(I).isImm() || Access.getOperand(I).isCImm()) {
        Access.removeOperand(I);
        break;
      }
    Access.setDesc(TII.get(IncOpc));
    Lea.eraseFromParent();
    Changed = true;
  };
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &MI = *It++;
    Register Base;
    int Off, K;
    // op ,r ; lea r k,r
    if (!indexedBaseOffset(MI, Base, Off) || Off != 0)
      continue;
    auto NIt = NextReal(MI.getIterator());
    if (NIt == MBB.end())
      continue;
    MachineInstr &Lea = *NIt;
    if (leaSelfStep(Lea, K) != Base || K <= 0 ||
        MI.modifiesRegister(Base, &TRI) || readsAsValue(MI, Base))
      continue;
    unsigned IncOpc = MC6809InstrInfo::getPostIncrementOpcode(MI.getOpcode(), K);
    if (!IncOpc)
      continue;
    if (Lea.modifiesRegister(MC6809::Z, &TRI) &&
        MBB.computeRegisterLiveness(&TRI, MC6809::Z, NextReal(NIt)) !=
            MachineBasicBlock::LQR_Dead)
      continue;
    It = NextReal(NIt);
    ++NumPostIncLeaAfter;
    Rewrite(MI, Lea, IncOpc);
  }
  return Changed;
}

// Bug #360: drop a redundant `CMPD/CMPW #0` when the immediately preceding
// instruction already produced the compared register and set its N/Z flags
// (e.g. `addd #-1; cmpd #0; bne` from a decrement loop — the ADDD's Z is
// exactly what the compare recomputes).
//
// Flag semantics gate: a compare-with-0 sets Z and N from the value and
// clears V and C. A value-producing op (ADDD/SUBD/LDD/…) sets the same Z/N
// but leaves V/C reflecting the operation, not 0. So the compare is only
// redundant for branches that test Z and/or N alone — EQ/NE (Z) and MI/PL
// (N). Conditions that consult V (signed GE/LT/GT/LE, VS/VC) or C
// (HI/LS/CC/CS) are NOT safe and are left untouched.
bool MC6809LateOptimization::elideCompareZero(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  bool Changed = false;
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &MI = *It++;
    // The redundant flag-setter: a CMPr #0 (immediate compare against zero,
    // ACC or index) or a TSTr (an inherent compare-with-zero of its
    // register). CmpReg is the register whose value the flags reflect.
    Register CmpReg;
    bool IsTst = false;
    switch (MI.getOpcode()) {
    case MC6809::CMPDi16: CmpReg = MC6809::AD; break;
    case MC6809::CMPWi16: CmpReg = MC6809::AW; break;
    case MC6809::CMPXi16: CmpReg = MC6809::IX; break;
    case MC6809::CMPYi16: CmpReg = MC6809::IY; break;
    case MC6809::TSTAa: CmpReg = MC6809::AA; IsTst = true; break;
    case MC6809::TSTBa: CmpReg = MC6809::AB; IsTst = true; break;
    case MC6809::TSTDa: CmpReg = MC6809::AD; IsTst = true; break;
    case MC6809::TSTWa: CmpReg = MC6809::AW; IsTst = true; break;
    case MC6809::TSTEa: CmpReg = MC6809::AE; IsTst = true; break;
    case MC6809::TSTFa: CmpReg = MC6809::AF; IsTst = true; break;
    default: continue;
    }
    if (!IsTst &&
        (!MI.getOperand(0).isImm() || MI.getOperand(0).getImm() != 0))
      continue;
    if (MI.getIterator() == MBB.begin())
      continue;

    auto ModifiesFlags = [&](const MachineInstr &I) {
      return I.modifiesRegister(MC6809::N, &TRI) ||
             I.modifiesRegister(MC6809::Z, &TRI) ||
             I.modifiesRegister(MC6809::V, &TRI);
    };

    // Producer search: walk back over instructions that leave the flags AND
    // CmpReg untouched, stopping at the first instruction that sets the flags
    // — that is the producer whose N/Z the compare would recompute. The skip
    // lets a flag-transparent instruction sit between producer and compare:
    //   addd #-1 ; tfr y,x ; tstd   (the tfr touches neither flags nor AD).
    // A skipped instruction that rewrites CmpReg would leave the producer's
    // flags stale for the value the compare tests, so bail in that case.
    MachineInstr *Producer = nullptr;
    for (auto PIt = MI.getIterator(); PIt != MBB.begin();) {
      MachineInstr &P = *--PIt;
      if (ModifiesFlags(P)) {
        Producer = &P;
        break;
      }
      if (P.modifiesRegister(CmpReg, &TRI))
        break;
    }
    if (!Producer)
      continue;

    // The producer's N/Z must describe EXACTLY the compared register, at its
    // width. A super-register def is unsafe: `ldd <16-bit>; tstb` defines
    // AD ⊇ AB but LDD's flags are the 16-bit D's, not the 8-bit B's that TSTB
    // tests (D=0x0100 -> D!=0 yet B==0).
    //
    // Defining the register is not enough for an instruction that reaches
    // memory, because an index register it addresses through is not what its
    // flags describe:
    //  - a store sets N/Z from the register it stores (`stb ,x+` sets Z when
    //    B is zero), so it produces flags for that register and no other --
    //    not for the base of `stb ,x`, and not for the X that `stb ,x+` steps;
    //  - a load or an operation that steps its index register (`ldb ,x+`,
    //    `addb ,-x`) defines the index register too, but its flags are the
    //    loaded or computed value's. `ldx ,x` still qualifies: X is its value.
    // The value register of these instructions is named by the opcode, so it
    // is an implicit operand; the explicit register operands are the address.
    auto IsValueReg = [&](const MachineInstr &P) {
      if (P.mayStore() && !P.mayLoad()) {
        for (const MachineOperand &MO : P.implicit_operands())
          if (MO.isReg() && MO.isUse() && MO.getReg() == CmpReg)
            return true;
        return false;
      }
      if (P.mayLoad() && MC6809::stepsIndexRegister(P.getDesc().TSFlags))
        for (const MachineOperand &MO : P.explicit_operands())
          if (MO.isReg() && MO.getReg() == CmpReg)
            return false;
      for (const MachineOperand &MO : P.operands())
        if (MO.isReg() && MO.isDef() && MO.getReg() == CmpReg)
          return true;
      return false;
    };
    if (!IsValueReg(*Producer))
      continue;

    // Consumer search: walk forward over flag-transparent instructions to the
    // first that touches the flags. A flag reader there is the consuming
    // branch (its flags come from the producer once the compare is gone); a
    // flag writer means a different producer feeds the branch, so bail. The
    // skip lets flag-transparent instructions sit between compare and branch:
    //   ldd <rs0> ; tstd ; puls d ; puls d ; beq   (the puls preserve CC).
    // CmpReg being rewritten in this span is fine — the branch reads flags,
    // not CmpReg, and the producer's flags already captured its value.
    auto ConsIt = std::next(MI.getIterator());
    // Stop at the first instruction that touches ANY flag, whether it reads or
    // writes it. Dropping the compare makes the producer's N/Z/V (and, for a
    // CMPr#0, C) reach past this point instead of the compare's, so a skipped
    // instruction may only be one that consults no flag at all — a reader of V
    // or C, not just N/Z, would see the changed value.
    while (ConsIt != MBB.end() && !ModifiesFlags(*ConsIt) &&
           !ConsIt->readsRegister(MC6809::N, &TRI) &&
           !ConsIt->readsRegister(MC6809::Z, &TRI) &&
           !ConsIt->readsRegister(MC6809::V, &TRI) &&
           !ConsIt->readsRegister(MC6809::C, &TRI))
      ++ConsIt;
    if (ConsIt == MBB.end() || ModifiesFlags(*ConsIt))
      continue;
    MachineInstr &Cons = *ConsIt;

    // The consumer must be a conditional branch on an N/Z-only condition
    // (EQ/NE test Z, MI/PL test N — exactly the flags a value op sets the same
    // way a compare-with-0 would; V and C differ, so signed and unsigned
    // conditions are excluded).
    if (Cons.getNumOperands() == 0 || !Cons.getOperand(0).isImm())
      continue;
    int64_t CC = Cons.getOperand(0).getImm();
    if (CC != MC6809CC::EQ && CC != MC6809CC::NE && CC != MC6809CC::MI &&
        CC != MC6809CC::PL)
      continue;
    if (Cons.readsRegister(MC6809::C, &TRI))
      continue;

    // Every flag the condition actually consults (Z for EQ/NE, N for
    // MI/PL) must be (re)defined by the producer, so dropping the compare
    // leaves that flag reflecting the compared value. The branch's operand
    // list may name more flags than the condition reads (a Bcc reads N/Z/V
    // as a group); those may come from wherever they were last set -- a
    // Z-only producer such as LEAX/LEAY feeds an EQ/NE branch just fine.
    // Every flag the branch names must be (re)defined by the producer, so
    // dropping the compare leaves no flag read undefined. An EQ/NE branch
    // names only Z (the OnlyZ variant), so a Z-only producer such as
    // LEAX/LEAY covers it.
    bool ProducerCoversFlags = true;
    bool ReadsNZ = false;
    for (const MachineOperand &MO : Cons.operands()) {
      if (!MO.isReg() || !MO.isUse())
        continue;
      Register R = MO.getReg();
      if (R == MC6809::N || R == MC6809::Z)
        ReadsNZ = true;
      if ((R == MC6809::N || R == MC6809::Z || R == MC6809::V) &&
          !Producer->definesRegister(R, &TRI)) {
        ProducerCoversFlags = false;
        break;
      }
    }
    if (!ProducerCoversFlags || !ReadsNZ)
      continue;

    // The branch will now read the producer's flags; clear any stale dead
    // markers (regalloc may have flagged them when the compare was the only
    // flag consumer).
    for (MachineOperand &MO : Producer->operands())
      if (MO.isReg() && MO.isDef() &&
          (MO.getReg() == MC6809::N || MO.getReg() == MC6809::Z ||
           MO.getReg() == MC6809::NZ || MO.getReg() == MC6809::V))
        MO.setIsDead(false);

    MI.eraseFromParent();
    ++NumCompareZeroElided;
    Changed = true;
  }
  return Changed;
}

// The index-LEA addressing suffix we can fold into a load. Constant (o5/o8/o16)
// carry an explicit (offset, base) operand pair; accumulator (oA/oB/oD and the
// HD6309 oE/oF/oW) carry only the base, with the accumulator as an implicit use.
// PC-relative (o8PC/o16PC) is intentionally absent: a global addressed
// PC-relatively is folded into the load/store at instruction selection now (the
// deferred _Sym pseudo + its post-RA expander), so no `leax sym,pc; ld ,x` pair
// survives to here. Indirect (`…I`), auto inc/dec, and LEAS/LEAU are not
// foldable. Enum values index the LoadTab columns below.
enum class IdxSuf : int {
  o5 = 0, o8, o16, oA, oB, oD, oE, oF, oW, None = -1
};

static IdxSuf leaIdxSuffix(unsigned LeaOpc) {
  switch (LeaOpc) {
  case MC6809::LEAXi_o5:    case MC6809::LEAYi_o5:    return IdxSuf::o5;
  case MC6809::LEAXi_o8:    case MC6809::LEAYi_o8:    return IdxSuf::o8;
  case MC6809::LEAXi_o16:   case MC6809::LEAYi_o16:   return IdxSuf::o16;
  case MC6809::LEAXi_oA:    case MC6809::LEAYi_oA:    return IdxSuf::oA;
  case MC6809::LEAXi_oB:    case MC6809::LEAYi_oB:    return IdxSuf::oB;
  case MC6809::LEAXi_oD:    case MC6809::LEAYi_oD:    return IdxSuf::oD;
  case MC6809::LEAXi_oE:    case MC6809::LEAYi_oE:    return IdxSuf::oE;
  case MC6809::LEAXi_oF:    case MC6809::LEAYi_oF:    return IdxSuf::oF;
  case MC6809::LEAXi_oW:    case MC6809::LEAYi_oW:    return IdxSuf::oW;
  default: return IdxSuf::None;
  }
}

// The value register a zero-offset register-indirect load (`ld ,r`) loads into.
static Register loadO0ValueReg(unsigned LoadOpc) {
  auto LS = MC6809InstrInfo::getIdxLoadStore(LoadOpc);
  if (!LS || LS->IsStore ||
      MC6809InstrInfo::getIdxForm(LoadOpc) != MC6809::IdxForm::O0)
    return MC6809::NoRegister;
  // The registers whose `ld ,r` the LEA fold takes (not E, F, W, Q or S).
  switch (LS->Value) {
  case MC6809::AA: case MC6809::AB: case MC6809::AD:
  case MC6809::IX: case MC6809::IY: case MC6809::SU:
    return LS->Value;
  default:
    return MC6809::NoRegister;
  }
}

// The load opcode that loads ValueReg using addressing suffix Suf. Rows are the
// value-register families; columns are the IdxSuf order {o5,o8,o16,oA,oB,oD,oE,
// oF,oW}. (HD6309 value families E/F/W are not folded — they fall to 0.)
static unsigned indexedLoadOpcode(Register ValueReg, IdxSuf Suf) {
  if (Suf == IdxSuf::None)
    return 0;
  static const unsigned LoadTab[6][9] = {
    /* AA */ {MC6809::LDAi_o5, MC6809::LDAi_o8, MC6809::LDAi_o16, MC6809::LDAi_oA, MC6809::LDAi_oB, MC6809::LDAi_oD, MC6809::LDAi_oE, MC6809::LDAi_oF, MC6809::LDAi_oW},
    /* AB */ {MC6809::LDBi_o5, MC6809::LDBi_o8, MC6809::LDBi_o16, MC6809::LDBi_oA, MC6809::LDBi_oB, MC6809::LDBi_oD, MC6809::LDBi_oE, MC6809::LDBi_oF, MC6809::LDBi_oW},
    /* AD */ {MC6809::LDDi_o5, MC6809::LDDi_o8, MC6809::LDDi_o16, MC6809::LDDi_oA, MC6809::LDDi_oB, MC6809::LDDi_oD, MC6809::LDDi_oE, MC6809::LDDi_oF, MC6809::LDDi_oW},
    /* IX */ {MC6809::LDXi_o5, MC6809::LDXi_o8, MC6809::LDXi_o16, MC6809::LDXi_oA, MC6809::LDXi_oB, MC6809::LDXi_oD, MC6809::LDXi_oE, MC6809::LDXi_oF, MC6809::LDXi_oW},
    /* IY */ {MC6809::LDYi_o5, MC6809::LDYi_o8, MC6809::LDYi_o16, MC6809::LDYi_oA, MC6809::LDYi_oB, MC6809::LDYi_oD, MC6809::LDYi_oE, MC6809::LDYi_oF, MC6809::LDYi_oW},
    /* SU */ {MC6809::LDUi_o5, MC6809::LDUi_o8, MC6809::LDUi_o16, MC6809::LDUi_oA, MC6809::LDUi_oB, MC6809::LDUi_oD, MC6809::LDUi_oE, MC6809::LDUi_oF, MC6809::LDUi_oW},
  };
  int Row;
  switch (ValueReg) {
  case MC6809::AA: Row = 0; break;
  case MC6809::AB: Row = 1; break;
  case MC6809::AD: Row = 2; break;
  case MC6809::IX: Row = 3; break;
  case MC6809::IY: Row = 4; break;
  case MC6809::SU: Row = 5; break;
  default: return 0;
  }
  return LoadTab[Row][static_cast<int>(Suf)];
}

// Bug #361: fold an address-generating index-LEA into the register-indirect load
// that immediately dereferences it:
//     leax d,x ; ldx ,x   ->   ldx d,x      (dst == base, always safe)
//     leax 4,u ; ldx ,x   ->   ldx 4,u      (base is the LEA's input, not X)
//     leax d,x ; ldy ,x   ->   ldy d,x      (dst != base — only when X dies here)
// The folded load addresses memory the way the LEA did, loading the o0 load's
// value register. Two safety regimes:
//   * dst == base (value reg == index reg == the load's base): the load
// The register a direct-page load fills, or a direct-page store writes to
// its slot (operand 0, an imaginary register in the cases handled here).
static Register dpLoadValueReg(unsigned Opc) {
  auto LS = MC6809InstrInfo::getDirLoadStore(Opc);
  return LS && !LS->IsStore ? LS->Value : Register();
}
static Register dpStoreValueReg(unsigned Opc) {
  auto LS = MC6809InstrInfo::getDirLoadStore(Opc);
  return LS && LS->IsStore ? LS->Value : Register();
}

// Drop a direct-page load whose value register still provably holds the same
// slot's contents, from an earlier identical load or from the store that
// filled the slot:
//     ldd <__rs0> ; std <__rs1> ; ldd <__rs0>   ->   ldd <__rs0> ; std <__rs1>
//     addd #-1 ; std <__rs0> ; ldd <__rs0>      ->   addd #-1 ; std <__rs0>
// The store copies D out without disturbing D, so the reload is pure waste.
// Which register holds which slot is MC6809ValueTracker's: a store naming an
// overlapping imaginary register, or one whose footprint is not known to be
// private, drops the mirror, as does a def of the holder; calls and unmodelled
// side effects drop everything. The dropped load's N/Z/V must be dead, or
// still exactly what the access that set up the mirror left (both loads and
// stores set N and Z from the value and clear V) with nothing in between
// rewriting them.
bool MC6809LateOptimization::elideRedundantLoad(MachineBasicBlock &MBB) const {
  MachineFunction &MF = *MBB.getParent();
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  if (!MF.getRegInfo().tracksLiveness())
    return false;
  bool Changed = false;

  auto ModifiesFlags = [&](const MachineInstr &I) {
    return I.modifiesRegister(MC6809::N, &TRI) ||
           I.modifiesRegister(MC6809::Z, &TRI) ||
           I.modifiesRegister(MC6809::V, &TRI);
  };
  auto ReadsFlags = [&](const MachineInstr &I) {
    return I.readsRegister(MC6809::N, &TRI) ||
           I.readsRegister(MC6809::Z, &TRI) ||
           I.readsRegister(MC6809::V, &TRI);
  };
  // The flags MI leaves are dead: the first later flag access writes them,
  // or none reads them before the block ends and they are not live out.
  auto FlagsDeadAfter = [&](MachineInstr &MI) {
    for (auto FIt = std::next(MI.getIterator()); FIt != MBB.end(); ++FIt) {
      if (ReadsFlags(*FIt))
        return false;
      if (ModifiesFlags(*FIt))
        return true;
    }
    LivePhysRegs LiveOut(TRI);
    LiveOut.addLiveOuts(MBB);
    return !LiveOut.contains(MC6809::N) && !LiveOut.contains(MC6809::Z) &&
           !LiveOut.contains(MC6809::V) && !LiveOut.contains(MC6809::NZ);
  };

  MC6809ValueTracker T(MF, TRI);
  for (MachineInstr &MI : llvm::make_early_inc_range(MBB)) {
    if (MI.isDebugInstr())
      continue;
    Register LoadReg = dpLoadValueReg(MI.getOpcode());
    Register StoreReg = dpStoreValueReg(MI.getOpcode());
    Register Slot;
    if ((LoadReg || StoreReg) && MI.getOperand(0).isReg() &&
        MI.getOperand(0).getReg().isPhysical())
      Slot = MI.getOperand(0).getReg();
    else
      LoadReg = StoreReg = Register();

    if (MI.isCall() || MI.isInlineAsm() ||
        (!Slot && MI.hasUnmodeledSideEffects())) {
      T.clear();
      continue;
    }

    if (LoadReg) {
      MC6809ValueTracker::Key K{Slot, 0};
      MC6809ValueTracker::Mirror *M = T.find(K);
      if (M && M->Reg == LoadReg && M->Est) {
        MachineInstr &Prior = *M->Est;
        bool Ok = FlagsDeadAfter(MI);
        if (!Ok) {
          // Live, but the same as the prior access left them?
          Ok = true;
          for (auto BIt = std::next(Prior.getIterator());
               BIt != MI.getIterator(); ++BIt)
            if (ModifiesFlags(*BIt) ||
                BIt->modifiesRegister(MC6809::NZ, &TRI))
              Ok = false;
          if (Ok)
            for (Register F : {MC6809::N, MC6809::Z, MC6809::V, MC6809::NZ})
              Prior.clearRegisterDeads(F);
        }
        if (Ok) {
          MC6809ValueTracker::clearKillsToPriorDef(MI, LoadReg, TRI);
          MI.eraseFromParent();
          ++NumDPReloadsElided;
          Changed = true;
          continue;
        }
      }
    }

    if (MI.mayStore())
      T.storeAlias(MI, false, Register());
    for (const MachineOperand &MO : MI.operands())
      if (MO.isReg() && MO.isDef() && MO.getReg().isPhysical()) {
        T.clearHolder(MO.getReg());
        T.clearBase(MO.getReg());
      }
    if (LoadReg)
      T.set({Slot, 0}, LoadReg, MI);
    else if (StoreReg)
      T.set({Slot, 0}, StoreReg, MI);
  }
  return Changed;
}

//     overwrites the computed address, so dropping the LEA's advance is
//     unobservable — unconditionally safe.
//   * dst != base (the base survives the load): folding drops the base's new
//     value, so it is valid ONLY if that base register is dead after the load.
//     The counter-example is a walking pointer (`leax -2,x ; ldy ,x` advancing X
//     for the next iteration) — there X is live and must NOT fold.
// Requires adjacency (load immediately follows the LEA): the load redefines
// N/Z/V, so the LEA's dropped Z def is dead and needs no preservation.
bool MC6809LateOptimization::foldLEAIntoLoad(MachineBasicBlock &MBB) const {
  const auto &TRI = *MBB.getParent()->getSubtarget().getRegisterInfo();
  const TargetInstrInfo &TII = *MBB.getParent()->getSubtarget().getInstrInfo();
  bool Changed = false;
  for (auto It = MBB.begin(); It != MBB.end();) {
    MachineInstr &Lea = *It++;
    IdxSuf Suf = leaIdxSuffix(Lea.getOpcode());
    if (Suf == IdxSuf::None)
      continue;

    // The index register the LEA computes (X or Y) — its single INDEX16 def.
    Register IndexReg;
    for (const MachineOperand &MO : Lea.operands())
      if (MO.isReg() && MO.isDef() &&
          MC6809::INDEX16RegClass.contains(MO.getReg())) {
        IndexReg = MO.getReg();
        break;
      }
    if (!IndexReg)
      continue;

    // Consumer: the immediately-following (non-debug) zero-offset load whose
    // base is the LEA's output (it dereferences the computed address).
    auto NextIt = std::next(Lea.getIterator());
    while (NextIt != MBB.end() && NextIt->isDebugInstr())
      ++NextIt;
    if (NextIt == MBB.end())
      continue;
    MachineInstr &Load = *NextIt;
    Register ValueReg = loadO0ValueReg(Load.getOpcode());
    if (!ValueReg || Load.getNumOperands() == 0 ||
        !Load.getOperand(0).isReg() || Load.getOperand(0).getReg() != IndexReg)
      continue;

    // Safety. dst == base is always safe. dst != base requires the base (the
    // LEA's output) to be dead after the load: a `killed` flag on the load's
    // base use, or a definite dead liveness result.
    if (ValueReg != IndexReg) {
      bool BaseDead = Load.getOperand(0).isKill() ||
                      MBB.computeRegisterLiveness(
                          &TRI, IndexReg,
                          std::next(MachineBasicBlock::iterator(Load))) ==
                          MachineBasicBlock::LQR_Dead;
      if (!BaseDead)
        continue;
    }

    unsigned FoldedOpc = indexedLoadOpcode(ValueReg, Suf);
    if (!FoldedOpc)
      continue;

    // Build the folded load: copy the LEA's explicit operands verbatim — they
    // are exactly the load's addressing operands for the same suffix
    // ((offset imm,) base). The base is the LEA's INPUT base (e.g. U in
    // `leax 4,u`), not the index register. The value-reg def, N/Z/V defs, and
    // any accumulator-offset implicit use come from the new opcode's descriptor.
    MachineInstrBuilder MIB =
        BuildMI(MBB, Load, Load.getDebugLoc(), TII.get(FoldedOpc));
    for (const MachineOperand &MO : Lea.explicit_operands())
      MIB.add(MO);

    Lea.eraseFromParent();
    Load.eraseFromParent();
    It = std::next(MIB->getIterator());
    ++NumLEAIntoLoad;
    Changed = true;
  }
  return Changed;
}

} // namespace

char MC6809LateOptimization::ID = 0;

INITIALIZE_PASS(MC6809LateOptimization, DEBUG_TYPE, "MC6809 Late Optimizations", false, false)

MachineFunctionPass *llvm::createMC6809LateOptimizationPass() { return new MC6809LateOptimization; }
