//===-- MC6809FoldLoadIntoConsumer.cpp - Fold loads into their consumers --===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Every MC6809 arithmetic, bitwise and compare instruction takes its second
// operand from memory in any addressing mode, so a value that is loaded and
// then used once by such an instruction never needs a register of its own:
// `d += v` is `addd d; std d`, not `ldd d; addd ...`. Selection folds a load
// into its consumer only when the load is the consumer's second operand and
// the address is register-based; this pass, on selected SSA MIR, catches the
// rest:
//
//   * a load from a symbol (`Load_*_Sym`: a global, direct-page or not,
//     absolute or PC-relative) feeding the second source of a pseudo that has
//     a _Mem form becomes the _Sym form of that pseudo -- `addd sym`;
//   * a register-based or frame-based load (`Load_*_Mem`) that escaped the
//     patterns becomes the _Mem form;
//   * a load feeding the FIRST (tied) source of a commutative pseudo has the
//     sources exchanged first, so `mem + reg` folds like `reg + mem`.
//
// The pay-off is largest for 16-bit values: an i16 that must sit in D at the
// same time as another i16 forces one of them into an imaginary register and
// its staging traffic (the shape Bug #421 reports); folding the loaded one
// into the ADDD memory operand needs no second register at all.
//
// A fold moves the memory read from the load to the consumer, so nothing
// between them may store to (or otherwise disturb) that memory: same block,
// no call, no ordered access, no possibly-aliasing store in between.
//
//===----------------------------------------------------------------------===//

#include "MC6809.h"
#include "MC6809InstrInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Debug.h"

#define DEBUG_TYPE "mc6809-fold-load-into-consumer"

using namespace llvm;

static cl::opt<bool> EnableFoldLoadIntoConsumer(
    "mc6809-enable-fold-load-into-consumer", cl::init(true), cl::Hidden,
    cl::desc("Fold single-use loads into the arithmetic, bitwise or compare "
             "pseudo that consumes them"));

static cl::opt<unsigned> FoldNearRegLoadDistance(
    "mc6809-fold-near-reg-load-distance", cl::init(3), cl::Hidden,
    cl::desc("Also fold a register-based load into a consumer at most this "
             "many instructions away when its base is not live past the "
             "consumer (0: only when the base is live past)"));

namespace {

class MC6809FoldLoadIntoConsumer : public MachineFunctionPass {
public:
  static char ID;
  MC6809FoldLoadIntoConsumer() : MachineFunctionPass(ID) {
    initializeMC6809FoldLoadIntoConsumerPass(*PassRegistry::getPassRegistry());
  }
  bool runOnMachineFunction(MachineFunction &MF) override;
  StringRef getPassName() const override {
    return "MC6809 fold load into consumer";
  }
  void getAnalysisUsage(AnalysisUsage &AU) const override {
    AU.addRequired<AAResultsWrapperPass>();
    AU.setPreservesCFG();
    MachineFunctionPass::getAnalysisUsage(AU);
  }
};

} // namespace

char MC6809FoldLoadIntoConsumer::ID = 0;

INITIALIZE_PASS_BEGIN(MC6809FoldLoadIntoConsumer, DEBUG_TYPE,
                      "MC6809 fold load into consumer", false, false)
INITIALIZE_PASS_DEPENDENCY(AAResultsWrapperPass)
INITIALIZE_PASS_END(MC6809FoldLoadIntoConsumer, DEBUG_TYPE,
                    "MC6809 fold load into consumer", false, false)

MachineFunctionPass *llvm::createMC6809FoldLoadIntoConsumerPass() {
  return new MC6809FoldLoadIntoConsumer();
}

// The loads this pass folds, and whether each is the symbol form.
static bool isFoldableLoad(unsigned Opc, bool &IsSym) {
  switch (Opc) {
  case MC6809::Load_i8_Sym:
  case MC6809::Load_i16_Sym:
  case MC6809::Load_iPtr_Sym:
    IsSym = true;
    return true;
  case MC6809::Load_i8_Mem:
  case MC6809::Load_i16_Mem:
  case MC6809::Load_iPtr_Mem:
    IsSym = false;
    return true;
  default:
    return false;
  }
}

// Nothing between the load and its consumer may disturb the loaded memory
// (the consumer will read it instead of the load).
static bool safeToMoveLoad(const MachineInstr &Load, const MachineInstr &User,
                           AAResults *AA) {
  if (Load.getParent() != User.getParent())
    return false;
  if (Load.hasOrderedMemoryRef())
    return false;
  for (const MachineInstr &I :
       make_range(std::next(Load.getIterator()), User.getIterator())) {
    if (I.isCall() || I.hasUnmodeledSideEffects())
      return false;
    if (I.mayStore() || I.hasOrderedMemoryRef()) {
      if (I.hasOrderedMemoryRef())
        return false;
      if (Load.memoperands_empty() || I.memoperands_empty())
        return false;
      if (I.mayAlias(AA, Load, /*UseTBAA=*/true))
        return false;
    }
  }
  return true;
}

// The walking form of an op that reads memory through a pointer: its
// `,r+` / `,-r` sibling (and the 2-byte forms), where the pointer operand
// sits, the access size, and the accumulator class the sibling needs (none
// for a compare).
struct StepForm {
  unsigned Opc;
  unsigned PtrIdx;
  unsigned Size;
  bool NeedsHD6309;
  const TargetRegisterClass *AccRC;
  // A plain load's walking forms are the post-modify pseudos, one per
  // direction and without a step operand (the selector fuses these itself
  // at the access; this catches a load it left for a fold that did not
  // happen).
  unsigned DecOpc = 0;
};

static std::optional<StepForm> stepFormOf(unsigned Opc) {
  // ADD/SUB step through the HD6309 W as well; the carry and bitwise word
  // ops through D only. Bytes stay on A/B like the _Mem forms.
  const TargetRegisterClass *Byte = &MC6809::ACC8_AB_SPRegClass;
  const TargetRegisterClass *AnyWord = &MC6809::ACC16RegClass;
  const TargetRegisterClass *Word = &MC6809::ADcRegClass;
  switch (Opc) {
  case MC6809::Load_i8_Mem:
    return StepForm{MC6809::Load_i8_PostInc, 1, 1, false, nullptr,
                    MC6809::Load_i8_PreDec};
  case MC6809::Load_i16_Mem:
    return StepForm{MC6809::Load_i16_PostInc, 1, 2, false, nullptr,
                    MC6809::Load_i16_PreDec};
  case MC6809::Compare_i8_Mem:
    return StepForm{MC6809::Compare_i8_MemStep, 3, 1, false, nullptr};
  case MC6809::Compare_i16_Mem:
    return StepForm{MC6809::Compare_i16_MemStep, 3, 2, false, nullptr};
  // (Not the fused compare-and-branch forms: a terminator must not define
  // an allocatable register -- the allocator cannot spill a value after a
  // terminator. Those walking compares are formed after allocation, in
  // MC6809LateOptimization.)
#define ARITH(NAME, SIZE, HD, RC)                                             \
  case MC6809::NAME##_Mem:                                                    \
    return StepForm{MC6809::NAME##_MemStep, 2, SIZE, HD, RC};
  ARITH(Add_i8, 1, false, Byte)
  ARITH(Add_i16, 2, false, AnyWord)
  ARITH(Sub_i8, 1, false, Byte)
  ARITH(Sub_i16, 2, false, AnyWord)
  ARITH(AddSetCarry_i8, 1, false, Byte)
  ARITH(AddSetCarry_i16, 2, false, AnyWord)
  ARITH(SubSetCarry_i8, 1, false, Byte)
  ARITH(SubSetCarry_i16, 2, false, AnyWord)
  ARITH(AddSetOverflow_i8, 1, false, Byte)
  ARITH(AddSetOverflow_i16, 2, false, AnyWord)
  ARITH(SubSetOverflow_i8, 1, false, Byte)
  ARITH(SubSetOverflow_i16, 2, false, AnyWord)
  ARITH(AddSetCarryUse_i8, 1, false, Byte)
  ARITH(SubSetCarryUse_i8, 1, false, Byte)
  ARITH(AddSetOverflowUse_i8, 1, false, Byte)
  ARITH(SubSetOverflowUse_i8, 1, false, Byte)
  ARITH(AND_i8, 1, false, Byte)
  ARITH(OR_i8, 1, false, Byte)
  ARITH(XOR_i8, 1, false, Byte)
  // The 6809 has no 16-bit bitwise op: ANDD/ORD/EORD are HD6309.
  ARITH(AND_i16, 2, true, Word)
  ARITH(OR_i16, 2, true, Word)
  ARITH(XOR_i16, 2, true, Word)
#undef ARITH
  default:
    return std::nullopt;
  }
}

// An op that reads memory through a pointer at offset 0, and the pointer's
// step by the access size with no other use of the pointer, become one
// walking op (`cmpb ,x+`, `addd ,x++`, `ldb ,-x`-style `addb ,-x`): the
// step's result is defined by the op (tied to the pointer). A step forward
// (`*p++`) may sit before or after the op, which reads the unstepped
// pointer; a step back (`*--p`) sits before the op, which reads the stepped
// one. Either way nothing else reads the pointer the op consumes.
static bool formStepMemOps(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &STI = MF.getSubtarget<MC6809Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &Op : make_early_inc_range(MBB)) {
      std::optional<StepForm> F = stepFormOf(Op.getOpcode());
      if (!F || (F->NeedsHD6309 && !STI.has6309()))
        continue;
      const MachineOperand &PtrMO = Op.getOperand(F->PtrIdx);
      const MachineOperand &OffMO = Op.getOperand(F->PtrIdx + 1);
      if (!PtrMO.isReg() || !PtrMO.getReg().isVirtual())
        continue;
      int64_t Off = OffMO.isImm() ? OffMO.getImm()
                    : OffMO.isCImm() ? OffMO.getCImm()->getSExtValue() : 1;
      const int64_t Size = F->Size;
      // Reading `0,p` with p stepping forward: `,p+`. Reading `-size,p` with
      // p stepping back: `,-p`. (Reading `0,q` where q is p stepped back is
      // the same walk expressed on the stepped pointer; see below.)
      if (Off != 0 && Off != -Size)
        continue;
      Register Read = PtrMO.getReg(); // the pointer the op reads through
      MachineInstr *Step = nullptr;   // the LEAPtrAdd_Imm absorbed
      Register PtrIn, PtrOut;         // the walking op's tied pair
      int64_t StepBy = Off == 0 ? Size : -Size;
      auto IsStep = [&](const MachineInstr &U, int64_t By) {
        return U.getOpcode() == MC6809::LEAPtrAdd_Imm && U.getParent() == &MBB &&
               U.getOperand(2).isImm() && U.getOperand(2).getImm() == By;
      };
      // The read pointer's only other user is its step (before or after
      // the op).
      bool Other = false;
      for (MachineInstr &U : MRI.use_nodbg_instructions(Read)) {
        if (&U == &Op)
          continue;
        if (!Step && IsStep(U, StepBy) && U.getOperand(1).getReg() == Read) {
          Step = &U;
          continue;
        }
        Other = true;
        break;
      }
      if (!Other && Step) {
        PtrIn = Read;
        PtrOut = Step->getOperand(0).getReg();
        // If the step comes first, nothing between it and the op may read
        // its result (it will be defined at the op instead).
        bool StepFirst = false;
        for (auto It = Step->getIterator(); It != MBB.end(); ++It)
          if (&*It == &Op) {
            StepFirst = true;
            break;
          }
        if (StepFirst) {
          bool UsedBetween = false;
          for (auto It = std::next(Step->getIterator()); &*It != &Op; ++It)
            if (It->readsRegister(PtrOut, nullptr)) {
              UsedBetween = true;
              break;
            }
          if (UsedBetween)
            continue;
        }
      } else if (Off == 0) {
        // The read pointer is itself a step back of a pointer with no other
        // use, made in this block, and nothing between the step and the op
        // reads it (the op will define it): `,-p`.
        MachineInstr *Def = MRI.getVRegDef(Read);
        if (!Def || !IsStep(*Def, -Size) ||
            !MRI.hasOneNonDBGUse(Def->getOperand(1).getReg()))
          continue;
        bool UsedBetween = false;
        for (auto It = std::next(Def->getIterator()); &*It != &Op; ++It)
          if (It->readsRegister(Read, nullptr)) {
            UsedBetween = true;
            break;
          }
        if (UsedBetween)
          continue;
        Step = Def;
        PtrIn = Def->getOperand(1).getReg();
        PtrOut = Read;
        StepBy = -Size;
      } else {
        continue;
      }
      // The walking form keeps its accumulator off HD6309 W/E/F.
      if (F->AccRC) {
        Register Dst = Op.getOperand(0).getReg(), Src = Op.getOperand(1).getReg();
        if (!Dst.isVirtual() || !Src.isVirtual() ||
            !MRI.constrainRegClass(Dst, F->AccRC) ||
            !MRI.constrainRegClass(Src, F->AccRC))
          continue;
      }
      const bool IsLoad = F->DecOpc != 0;
      MachineInstrBuilder MIB = BuildMI(
          MBB, Op, Op.getDebugLoc(),
          TII.get(IsLoad && StepBy < 0 ? F->DecOpc : F->Opc));
      MIB.add(Op.getOperand(0)); // CC, accumulator or loaded value def
      MIB.addDef(PtrOut);
      for (unsigned I = 1; I < F->PtrIdx; ++I)
        MIB.add(Op.getOperand(I)); // cc + src, or the tied src
      MIB.addReg(PtrIn);
      if (!IsLoad)
        MIB.addImm(StepBy);
      // Implicit operands the selector appended (flag defs) stay with the
      // descriptor; carry any virtual ones over.
      for (unsigned I = Op.getNumExplicitOperands(), E = Op.getNumOperands();
           I != E; ++I)
        if (Op.getOperand(I).isReg() && Op.getOperand(I).getReg().isVirtual())
          MIB.add(Op.getOperand(I));
      MIB.setMemRefs(Op.memoperands());
      MRI.setRegClass(PtrOut, &MC6809::INDEX16RegClass);
      MRI.setRegClass(PtrIn, &MC6809::INDEX16RegClass);
      Op.eraseFromParent();
      Step->eraseFromParent();
      Changed = true;
    }
  }
  return Changed;
}

bool MC6809FoldLoadIntoConsumer::runOnMachineFunction(MachineFunction &MF) {
  if (!EnableFoldLoadIntoConsumer)
    return false;

  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &TII = *MF.getSubtarget<MC6809Subtarget>().getInstrInfo();
  const auto &TRI = *MF.getSubtarget().getRegisterInfo();
  AAResults *AA = &getAnalysis<AAResultsWrapperPass>().getAAResults();
  bool Changed = false;

  for (MachineBasicBlock &MBB : MF) {
    // Collect first: a fold erases the consumer, which may be the next
    // instruction in the block.
    SmallVector<MachineInstr *, 16> Loads;
    for (MachineInstr &MI : MBB) {
      bool IsSym;
      if (isFoldableLoad(MI.getOpcode(), IsSym))
        Loads.push_back(&MI);
    }
    for (MachineInstr *LoadPtr : Loads) {
      MachineInstr &Load = *LoadPtr;
      bool IsSym;
      isFoldableLoad(Load.getOpcode(), IsSym);
      Register Val = Load.getOperand(0).getReg();
      if (!Val.isVirtual() || !MRI.hasOneNonDBGUse(Val))
        continue;
      // A register-based load must have a constant offset: an accumulator
      // offset (`ldb d,y`) would keep D busy across the consumer, which for a
      // byte op is the accumulator the op needs, and the expander would have
      // to form the address in an index register anyway.
      if (!IsSym && !Load.getOperand(2).isImm() && !Load.getOperand(2).isCImm())
        continue;
      MachineInstr *UserPtr = &*MRI.use_instr_nodbg_begin(Val);
      // A pointer-typed value that crosses to the accumulator bank (an
      // integer use of a loaded pointer, or of a pointer argument arriving
      // through the stack) reaches its consumer through a bank COPY. The
      // consumer reads memory the same way whichever bank the value was
      // headed for: fold through the copy and drop it.
      MachineInstr *Copy = nullptr;
      if (UserPtr->isCopy() && UserPtr->getOperand(0).getReg().isVirtual() &&
          !UserPtr->getOperand(0).getSubReg() &&
          !UserPtr->getOperand(1).getSubReg() &&
          MRI.hasOneNonDBGUse(UserPtr->getOperand(0).getReg())) {
        Copy = UserPtr;
        Val = Copy->getOperand(0).getReg();
        UserPtr = &*MRI.use_instr_nodbg_begin(Val);
      }
      MachineInstr &User = *UserPtr;
      auto Fold = MC6809InstrInfo::getMemFoldSibling(User.getOpcode());
      if (!Fold)
        continue;
      unsigned MemOpc = Fold->MemOpc;
      if (IsSym) {
        MemOpc = TII.getStaticSymOpcode(MemOpc);
        if (!MemOpc)
          continue;
      }
      // Which source is the load? The fold slot directly, or -- for a
      // commutative op -- the tied first source, exchanged with a register
      // second source.
      unsigned UseIdx = ~0u;
      for (unsigned I = 0, E = User.getNumExplicitOperands(); I != E; ++I) {
        const MachineOperand &MO = User.getOperand(I);
        if (MO.isReg() && MO.isUse() && MO.getReg() == Val) {
          UseIdx = I;
          break;
        }
      }
      if (UseIdx == ~0u)
        continue;
      // A compare's operands can always be exchanged if its condition is
      // swapped with them; the condition immediate sits just before the
      // first compared operand.
      int CCIdx = -1;
      switch (User.getOpcode()) {
      case MC6809::Compare_i8_Reg:
      case MC6809::Compare_i16_Reg:
        CCIdx = 1;
        break;
      case MC6809::CompareBranch_i8_Reg:
      case MC6809::CompareBranch_i16_Reg:
        CCIdx = 0;
        break;
      default:
        break;
      }
      bool Swap = false;
      if (UseIdx != Fold->FoldIdx) {
        // The tied (or first compared) source is the operand right after the
        // condition, or operand 1; the other source must be a plain register.
        unsigned FirstSrc = CCIdx >= 0 ? CCIdx + 1 : 1;
        if (UseIdx != FirstSrc ||
            !(CCIdx >= 0 ||
              MC6809InstrInfo::isCommutableTwoSource(User.getOpcode())) ||
            !User.getOperand(Fold->FoldIdx).isReg())
          continue;
        if (CCIdx >= 0 &&
            getSwappedCondition(
                (MC6809CC::CondCode)User.getOperand(CCIdx).getImm()) ==
                MC6809CC::RA)
          continue;
        Swap = true;
      }
      if (!safeToMoveLoad(Load, User, AA))
        continue;
      // A register-based fold keeps the base index register live up to the
      // consumer, and the scheduler may then move the address computation
      // further away. With two index registers that can cost an index
      // register (and a two-byte spill of the address) at exactly the point
      // the loaded byte was supposed to be cheap. Fold only when the base is
      // live past the consumer anyway -- then nothing is extended -- or when
      // the consumer follows within a few instructions, so there is next to
      // nothing to extend across (a pointer step between them, say).
      if (!IsSym) {
        const MachineOperand &Base = Load.getOperand(1);
        unsigned Distance = 0;
        for (auto It = std::next(Load.getIterator());
             It != MBB.end() && &*It != &User && Distance <= FoldNearRegLoadDistance;
             ++It)
          if (!It->isDebugInstr())
            ++Distance;
        if (Base.isReg() && Base.getReg().isVirtual() &&
            Distance > FoldNearRegLoadDistance) {
          bool LivePast = false;
          for (const MachineInstr &Use :
               MRI.use_nodbg_instructions(Base.getReg())) {
            if (&Use == &Load)
              continue;
            if (Use.getParent() != &MBB) {
              LivePast = true;
              break;
            }
            // Same block: is the use at or after the consumer?
            for (auto It = User.getIterator(); It != MBB.end(); ++It)
              if (&*It == &Use) {
                LivePast = true;
                break;
              }
            if (LivePast)
              break;
          }
          if (!LivePast)
            continue;
        }
      }

      const MCInstrDesc &MemDesc = TII.get(MemOpc);
      // The register operands that stay must satisfy the memory form's
      // operand classes (see foldMemoryOperandImpl); constrain them. On the
      // HD6309 the byte memory forms exist only for A and B, so a fold would
      // take E and F away from a value that may live there today: not a
      // free trade, skip it unless the value is already confined to the
      // memory form's class. (On the 6809 the classes allocate the same
      // registers and the constraint costs nothing.)
      const bool Has6309 = MF.getSubtarget<MC6809Subtarget>().has6309();
      unsigned NumExplicit = User.getNumExplicitOperands();
      bool CanConstrain = true;
      SmallVector<MachineOperand, 6> Kept;
      for (unsigned I = 0; I < NumExplicit; ++I)
        Kept.push_back(User.getOperand(I));
      if (Swap) {
        // The load's value takes the fold slot; the register that was there
        // becomes the first source (for an arithmetic op the tied one, and
        // through the tie the destination); a compare gets the swapped
        // condition.
        unsigned FirstSrc = CCIdx >= 0 ? CCIdx + 1 : 1;
        Kept[FirstSrc] = User.getOperand(Fold->FoldIdx);
        Kept[Fold->FoldIdx] = User.getOperand(FirstSrc);
        if (CCIdx >= 0)
          Kept[CCIdx] = MachineOperand::CreateImm(getSwappedCondition(
              (MC6809CC::CondCode)User.getOperand(CCIdx).getImm()));
      }
      unsigned NewOpIdx = 0;
      for (unsigned I = 0; I < NumExplicit; ++I) {
        if (I == Fold->FoldIdx) {
          NewOpIdx += IsSym ? 1 : 2;
          continue;
        }
        const MachineOperand &MO = Kept[I];
        if (MO.isReg() && MO.getReg().isVirtual() &&
            NewOpIdx < MemDesc.getNumOperands()) {
          if (const TargetRegisterClass *RC =
                  TII.getRegClass(MemDesc, NewOpIdx)) {
            const TargetRegisterClass *Cur = MRI.getRegClassOrNull(MO.getReg());
            if (Has6309 && Cur && Cur->contains(MC6809::AE) &&
                !RC->contains(MC6809::AE)) {
              CanConstrain = false;
              break;
            }
            if (!MRI.constrainRegClass(MO.getReg(), RC)) {
              CanConstrain = false;
              break;
            }
          }
        }
        ++NewOpIdx;
      }
      if (!CanConstrain)
        continue;

      MachineInstrBuilder MIB =
          BuildMI(MBB, User, User.getDebugLoc(), MemDesc);
      for (unsigned I = 0; I < NumExplicit; ++I) {
        if (I == Fold->FoldIdx) {
          if (IsSym) {
            MIB.add(Load.getOperand(1));
          } else {
            MIB.add(Load.getOperand(1)); // base: index register or frame index
            MIB.add(Load.getOperand(2)); // offset
          }
          continue;
        }
        MachineOperand MO = Kept[I];
        if (MO.isReg() && MO.isUse())
          MO.setIsKill(false);
        MIB.add(MO);
      }
      // Carry over the selector-appended virtual implicit operands (the
      // phantom carry of the SetCarry/SetOverflow family); the physical CC
      // defs come from the memory form's descriptor.
      for (unsigned I = NumExplicit, E = User.getNumOperands(); I != E; ++I) {
        const MachineOperand &MO = User.getOperand(I);
        if (MO.isReg() && MO.getReg().isVirtual())
          MIB.add(MO);
      }
      MIB.setMemRefs(Load.memoperands());

      LLVM_DEBUG(dbgs() << "Folded " << Load << "  into " << User
                        << "  giving " << *MIB);
      User.eraseFromParent();
      if (Copy)
        Copy->eraseFromParent();
      Load.eraseFromParent();
      Changed = true;
    }
  }
  (void)TRI;
  Changed |= formStepMemOps(MF);
  return Changed;
}
