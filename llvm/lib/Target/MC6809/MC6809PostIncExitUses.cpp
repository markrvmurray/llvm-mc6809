//===-- MC6809PostIncExitUses.cpp - Exit uses through the stepped pointer -===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// A pointer that walks through memory in a loop -- `while (*p) p++` -- is one
// index register with its step folded into the access (`ldb ,x+`) as long as
// the pointer's only uses inside the loop are that access and the step. When
// the value before the step is also wanted after the loop (`p - start` in
// strlen), the pre-step pointer stays live across the access, which costs a
// copy per iteration and unfuses the step. Those outside uses are rewritten
// to read the stepped pointer minus the step, computed once where they are.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineDominators.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineLoopInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-postinc-exit-uses"

using namespace llvm;

static cl::opt<bool> EnablePostIncExitUses(
    "mc6809-enable-postinc-exit-uses", cl::init(true), cl::Hidden,
    cl::desc("Rewrite uses outside a loop of a stepped pointer's pre-step "
             "value to the stepped value minus the step"));

namespace {

class MC6809PostIncExitUses : public MachineFunctionPass {
public:
  static char ID;
  MC6809PostIncExitUses() : MachineFunctionPass(ID) {
    initializeMC6809PostIncExitUsesPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 loop exit uses through the stepped pointer";
  }
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<MachineDominatorTreeWrapperPass>();
    AU.addRequired<MachineLoopInfoWrapperPass>();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

} // namespace

char MC6809PostIncExitUses::ID = 0;

INITIALIZE_PASS_BEGIN(MC6809PostIncExitUses, DEBUG_TYPE,
                      "MC6809 loop exit uses through the stepped pointer",
                      false, false)
INITIALIZE_PASS_DEPENDENCY(MachineDominatorTreeWrapperPass)
INITIALIZE_PASS_DEPENDENCY(MachineLoopInfoWrapperPass)
INITIALIZE_PASS_END(MC6809PostIncExitUses, DEBUG_TYPE,
                    "MC6809 loop exit uses through the stepped pointer", false,
                    false)

MachineFunctionPass *llvm::createMC6809PostIncExitUsesPass() {
  return new MC6809PostIncExitUses();
}

// Put a new block on the edge MBB -> Succ (generic MIR: the branch is a
// G_BR). Succ's phis are rewired to the new block.
// Only a block whose terminators are plain G_BR / G_BRCOND can have an
// edge split this way (a jump table names its targets in the table).
static bool hasSimpleTerminators(const MachineBasicBlock &MBB) {
  for (const MachineInstr &MI : MBB.terminators())
    if (MI.getOpcode() != TargetOpcode::G_BR &&
        MI.getOpcode() != TargetOpcode::G_BRCOND)
      return false;
  return true;
}

static MachineBasicBlock *splitEdge(MachineBasicBlock &MBB,
                                    MachineBasicBlock &Succ) {
  MachineFunction &MF = *MBB.getParent();
  if (!hasSimpleTerminators(MBB))
    return nullptr;
  if (MBB.empty() || std::prev(MBB.end())->getOpcode() != TargetOpcode::G_BR) {
    // The block falls through to its layout successor: make that explicit
    // first, so that the new block can be laid out right after this one
    // whichever edge it splits (the fall-through one or a G_BRCOND target).
    auto LI = std::next(MBB.getIterator());
    if (LI != MF.end() && MBB.isSuccessor(&*LI)) {
      MachineIRBuilder MIB(MBB, MBB.end());
      MIB.buildBr(*LI);
    }
  }
  MachineBasicBlock *Split = MF.CreateMachineBasicBlock(MBB.getBasicBlock());
  MachineIRBuilder MIB(*Split, Split->begin());
  MIB.buildBr(Succ);
  Split->addSuccessor(&Succ);
  MF.insert(std::next(MBB.getIterator()), Split);
  MBB.ReplaceUsesOfBlockWith(&Succ, Split);
  Succ.replacePhiUsesWith(&MBB, Split);
  return Split;
}

static cl::opt<bool> EnableSinkExitPhiConstants(
    "mc6809-enable-sink-exit-phi-constants", cl::init(true), cl::Hidden,
    cl::desc("Materialise a constant that only feeds a phi across a loop's "
             "exit edge on that edge instead of inside the loop"));

// A phi in a loop's exit block whose input from inside the loop is a
// constant (`return NULL` after a search loop, say) is fed by a copy at the
// end of the exiting block -- inside the loop, every iteration, in a
// register the loop could use (an index register for a pointer result,
// which is exactly what a search loop's counter would want). Give the
// constant its own block on the exit edge, where it is executed once.
static bool sinkConstantExitPhiInputs(MachineFunction &MF,
                                      const MachineLoopInfo &MLI) {
  if (!EnableSinkExitPhiConstants)
    return false;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &Phi : make_early_inc_range(MBB.phis())) {
      for (unsigned I = 1; I < Phi.getNumOperands(); I += 2) {
        Register V = Phi.getOperand(I).getReg();
        MachineBasicBlock *PB = Phi.getOperand(I + 1).getMBB();
        MachineLoop *L = MLI.getLoopFor(PB);
        if (!L || L->contains(&MBB) || PB->succ_size() < 2)
          continue;
        // Wherever the constant is defined now, the localizer will bring it
        // next to its use -- the end of the exiting block, inside the loop.
        MachineInstr *Def = MRI.getVRegDef(V);
        if (!Def)
          continue;
        switch (Def->getOpcode()) {
        case TargetOpcode::G_CONSTANT:
        case TargetOpcode::G_FCONSTANT:
        case TargetOpcode::G_GLOBAL_VALUE:
        case TargetOpcode::G_FRAME_INDEX:
          break;
        default:
          continue;
        }
        MachineBasicBlock *NB = splitEdge(*PB, MBB);
        if (!NB)
          continue;
        // The split rewired this phi's incoming block to NB; a fresh copy of
        // the constant there feeds it (the original keeps its other uses).
        MachineInstr *Clone = MF.CloneMachineInstr(Def);
        Register NV = MRI.cloneVirtualRegister(V);
        Clone->getOperand(0).setReg(NV);
        NB->insert(NB->begin(), Clone);
        Phi.getOperand(I).setReg(NV);
        Changed = true;
      }
    }
  }
  return Changed;
}

bool MC6809PostIncExitUses::runOnMachineFunction(MachineFunction &MF) {
  if (!EnablePostIncExitUses)
    return false;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  auto &MDT = getAnalysis<MachineDominatorTreeWrapperPass>().getDomTree();
  auto &MLI = getAnalysis<MachineLoopInfoWrapperPass>().getLI();
  bool Changed = sinkConstantExitPhiInputs(MF, MLI);
  if (Changed) {
    // New blocks on exit edges: the dominator tree and loop structure
    // below must know them.
    MDT.recalculate(MF);
    MLI.releaseMemory();
    MLI.analyze(MDT);
  }

  for (MachineLoop *L : MLI.getLoopsInPreorder()) {
    MachineBasicBlock *Header = L->getHeader();
    for (MachineInstr &Phi : Header->phis()) {
      Register P = Phi.getOperand(0).getReg();
      if (!MRI.getType(P).isPointer())
        continue;
      // The step: a G_PTR_ADD of P by a small constant, inside the loop,
      // feeding the phi back; the access: a load or store through P at
      // offset 0 of the step's size, in the step's block. Nothing else in
      // the loop may read P (or the fusion could not happen anyway).
      MachineInstr *Step = nullptr, *Access = nullptr;
      int64_t C = 0;
      bool Other = false;
      SmallVector<MachineOperand *, 4> Outside;
      for (MachineOperand &Use : MRI.use_nodbg_operands(P)) {
        MachineInstr &U = *Use.getParent();
        if (!L->contains(U.getParent())) {
          Outside.push_back(&Use);
          continue;
        }
        if (&U == &Phi) {
          Other = true; // a self loop of one block? leave it
          break;
        }
        if (U.getOpcode() == TargetOpcode::G_PTR_ADD && !Step &&
            U.getOperand(1).getReg() == P) {
          auto K = getIConstantVRegValWithLookThrough(U.getOperand(2).getReg(),
                                                      MRI);
          if (K && (K->Value.abs() == 1 || K->Value.abs() == 2)) {
            Step = &U;
            C = K->Value.getSExtValue();
            continue;
          }
        }
        if ((U.getOpcode() == TargetOpcode::G_LOAD ||
             U.getOpcode() == TargetOpcode::G_STORE) &&
            !Access && U.getOperand(1).getReg() == P) {
          Access = &U;
          continue;
        }
        Other = true;
        break;
      }
      if (Other || !Step || !Access)
        continue;
      if (Access->memoperands_empty() ||
          !(*Access->memoperands_begin())->getSize().hasValue())
        continue;
      // The access must be the step's size and (a post-step access is
      // `*p++`, a pre-step one `*--p`) sit on the right side of the step.
      unsigned Size = (*Access->memoperands_begin())->getSize().getValue();
      if (int64_t(Size) != std::abs(C))
        continue;
      // The step's result must be what the phi takes back round the loop.
      Register Stepped = Step->getOperand(0).getReg();
      bool FeedsPhi = false;
      for (unsigned I = 1; I < Phi.getNumOperands(); I += 2)
        if (Phi.getOperand(I).getReg() == Stepped &&
            L->contains(Phi.getOperand(I + 1).getMBB()))
          FeedsPhi = true;
      if (!FeedsPhi)
        continue;
      // A forward step that sits after the access -- in a later block (past
      // the loop's exit test, say) or further down the same one -- is pure
      // arithmetic on a value the access already has: bring it right up to
      // the access, where the selector can fold it in. Only a step the
      // access dominates can move; its result keeps dominating its uses. (A
      // step already before the access is fused as it stands.)
      bool StepAfterInBlock = false;
      if (Step->getParent() == Access->getParent())
        for (auto It = std::next(Access->getIterator());
             It != Access->getParent()->end(); ++It)
          if (&*It == Step) {
            StepAfterInBlock = true;
            break;
          }
      // Not worth it for a load whose one consumer will read the memory
      // itself (a compare selected as CompareBranch_*_Mem, a 16-bit
      // arithmetic op that folds the load): the load never becomes a
      // fused access, and a step ahead of the consumer only keeps both
      // the old and the new pointer live across it. (A compare-and-branch
      // gets its walking form after allocation instead.)
      auto ConsumerFoldsLoad = [&](const MachineInstr &Ld) {
        if (Ld.getOpcode() != TargetOpcode::G_LOAD ||
            !MRI.hasOneNonDBGUse(Ld.getOperand(0).getReg()))
          return false;
        const MachineInstr &U = *MRI.use_instr_nodbg_begin(Ld.getOperand(0).getReg());
        switch (U.getOpcode()) {
        case TargetOpcode::G_ICMP: {
          // A test against zero is the load's own flags (`ldb ,x+ ; bne`);
          // a compare against anything else reads the memory itself.
          Register Other = U.getOperand(2).getReg() == Ld.getOperand(0).getReg()
                               ? U.getOperand(3).getReg()
                               : U.getOperand(2).getReg();
          auto K = getIConstantVRegValWithLookThrough(Other, MRI);
          return !(K && K->Value.isZero());
        }
        case TargetOpcode::G_ADD: case TargetOpcode::G_SUB:
        case TargetOpcode::G_AND: case TargetOpcode::G_OR: case TargetOpcode::G_XOR:
        case TargetOpcode::G_UADDO: case TargetOpcode::G_UADDE:
        case TargetOpcode::G_USUBO: case TargetOpcode::G_USUBE:
          return MRI.getType(Ld.getOperand(0).getReg()) == LLT::scalar(16);
        default:
          return false;
        }
      };
      if (C > 0 && (Step->getParent() != Access->getParent() ||
                    (StepAfterInBlock &&
                     std::next(Access->getIterator()) != Step->getIterator()))) {
        if (!MDT.dominates(Access, Step) || ConsumerFoldsLoad(*Access))
          continue;
        MachineIRBuilder B(*Access->getParent(),
                           std::next(Access->getIterator()));
        B.setDebugLoc(Step->getDebugLoc());
        LLT PtrTy = MRI.getType(P);
        auto Off = B.buildConstant(LLT::scalar(PtrTy.getSizeInBits()), C);
        Register NewStepped = B.buildPtrAdd(PtrTy, P, Off).getReg(0);
        MRI.replaceRegWith(Stepped, NewStepped);
        Step->eraseFromParent();
        Step = MRI.getVRegDef(NewStepped);
        Stepped = NewStepped;
        Changed = true;
      }
      if (Access->getParent() != Step->getParent())
        continue;
      bool AccessFirst = false;
      for (auto It = Access->getIterator(); It != Access->getParent()->end();
           ++It)
        if (&*It == Step) {
          AccessFirst = true;
          break;
        }
      if (AccessFirst != (C > 0))
        continue;
      if (Outside.empty())
        continue;
      // Every outside use must be dominated by the step: a plain
      // instruction directly, a phi through the in-loop predecessor its
      // incoming value comes from (the rewritten value is computed on that
      // exit edge, in a block of its own so the loop does not pay for it).
      bool AllDominated = true;
      for (MachineOperand *Use : Outside) {
        MachineInstr &U = *Use->getParent();
        if (U.isPHI()) {
          // The value is needed at the end of the incoming block: fine if
          // that block is outside the loop and the step dominates it, or
          // an exit edge that can take a block of its own.
          unsigned Idx = Use->getOperandNo();
          MachineBasicBlock *PB = U.getOperand(Idx + 1).getMBB();
          if (!MDT.dominates(Step->getParent(), PB) ||
              (L->contains(PB) &&
               (PB->succ_size() < 2 || !hasSimpleTerminators(*PB)))) {
            AllDominated = false;
            break;
          }
        } else if (!MDT.dominates(Step, &U)) {
          AllDominated = false;
          break;
        }
      }
      if (!AllDominated)
        continue;

      // Rewrite: one `stepped - C` per using block, at its first non-phi;
      // for a phi, in a new block on the exit edge.
      LLT PtrTy = MRI.getType(P);
      LLT OffTy = LLT::scalar(PtrTy.getSizeInBits());
      DenseMap<MachineBasicBlock *, Register> PerBlock;
      auto Materialise = [&](MachineBasicBlock &UB, MachineBasicBlock::iterator At,
                             const DebugLoc &DL) {
        Register &R = PerBlock[&UB];
        if (!R) {
          MachineIRBuilder B(UB, At);
          B.setDebugLoc(DL);
          auto Off = B.buildConstant(OffTy, -C);
          R = B.buildPtrAdd(PtrTy, Stepped, Off).getReg(0);
        }
        return R;
      };
      for (MachineOperand *Use : Outside) {
        MachineInstr &U = *Use->getParent();
        if (U.isPHI()) {
          unsigned Idx = Use->getOperandNo();
          MachineBasicBlock *PB = U.getOperand(Idx + 1).getMBB();
          if (!L->contains(PB)) {
            Use->setReg(Materialise(*PB, PB->getFirstTerminator(), U.getDebugLoc()));
            continue;
          }
          MachineBasicBlock *NB = splitEdge(*PB, *U.getParent());
          // The split rewired the phi's incoming block to NB.
          Use->setReg(Materialise(*NB, NB->begin(), U.getDebugLoc()));
          continue;
        }
        MachineBasicBlock *UB = U.getParent();
        Use->setReg(Materialise(*UB, UB->getFirstNonPHI(), U.getDebugLoc()));
      }
      LLVM_DEBUG(dbgs() << "Rewrote outside uses of " << Phi
                        << "  through the stepped " << *Step);
      Changed = true;
    }
  }
  return Changed;
}
