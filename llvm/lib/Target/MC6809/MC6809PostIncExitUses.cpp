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
static MachineBasicBlock *splitEdge(MachineBasicBlock &MBB,
                                    MachineBasicBlock &Succ) {
  MachineFunction &MF = *MBB.getParent();
  if (MBB.empty() || std::prev(MBB.end())->getOpcode() != TargetOpcode::G_BR) {
    // The edge is the fall-through: make it an explicit branch first so
    // that the new block can be inserted anywhere.
    MachineIRBuilder MIB(MBB, MBB.end());
    MIB.buildBr(Succ);
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

bool MC6809PostIncExitUses::runOnMachineFunction(MachineFunction &MF) {
  if (!EnablePostIncExitUses)
    return false;
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &MDT = getAnalysis<MachineDominatorTreeWrapperPass>().getDomTree();
  const auto &MLI = getAnalysis<MachineLoopInfoWrapperPass>().getLI();
  bool Changed = false;

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
      if (C > 0 && (Step->getParent() != Access->getParent() ||
                    (StepAfterInBlock &&
                     std::next(Access->getIterator()) != Step->getIterator()))) {
        if (!MDT.dominates(Access, Step))
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
          unsigned Idx = Use->getOperandNo();
          MachineBasicBlock *PB = U.getOperand(Idx + 1).getMBB();
          if (!L->contains(PB) || !MDT.dominates(Step->getParent(), PB) ||
              PB->succ_size() < 2) {
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
