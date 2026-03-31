//===-- MC6809DirectPageAlloc.cpp - MC6809 Direct Page Allocation ---------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
///
/// \file
/// This file defines the MC6809 direct page allocation pass.
///
//===----------------------------------------------------------------------===//

#include "MC6809DirectPageAlloc.h"

#include "MC6809.h"
#include "MC6809CallGraphUtils.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/SCCIterator.h"
#include "llvm/Analysis/BasicAliasAnalysis.h"
#include "llvm/Analysis/BlockFrequencyInfo.h"
#include "llvm/Analysis/CallGraph.h"
#include "llvm/Analysis/DominanceFrontier.h"
#include "llvm/Analysis/GlobalsModRef.h"
#include "llvm/Analysis/IVUsers.h"
#include "llvm/Analysis/LoopInfo.h"
#include "llvm/Analysis/MemoryDependenceAnalysis.h"
#include "llvm/Analysis/ScalarEvolution.h"
#include "llvm/Analysis/ScalarEvolutionAliasAnalysis.h"
#include "llvm/CodeGen/LazyMachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineModuleInfo.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/Dominators.h"
#include "llvm/Support/BlockFrequency.h"
#include "llvm/Support/Casting.h"
#include <algorithm>
#include <memory>
#include <utility>

#define DEBUG_TYPE "mc6809-direct-page-alloc"

using namespace llvm;

namespace {

cl::opt<uint64_t> DPAvail("dp-avail",
                          cl::desc("Number of bytes of direct page available for "
                                   "the compiler in the current TU"),
                          cl::value_desc("bytes"));

struct SCC;

struct Candidate {
  MachineFunction *MF = nullptr;
  size_t Size;

  // One of
  Register CSR = 0;
  GlobalVariable *GV = nullptr;
  int FI = -1;

  // The number of DP locations tentatively assigned to this candidate. When
  // this reaches Size, the assignment is final.
  unsigned AssignedSize = 0;

  SCC *Comp = nullptr;
};

struct LocalCandidate {
  Candidate *Cand;
  // Benefit relative to local function entry.
  float Benefit;
};

struct EntryCandidate {
  LocalCandidate *LC;
  float Benefit;
};

} // namespace

#ifndef NDEBUG
raw_ostream &operator<<(raw_ostream &OS, const Candidate &C) {
  if (C.MF)
    OS << C.MF->getName() << ", ";
  if (C.CSR)
    OS << "CSR " << printReg(C.CSR, C.MF->getSubtarget().getRegisterInfo());
  else if (C.GV)
    OS << "Global " << *C.GV;
  else
    OS << "Frame Index " << C.FI;
  OS << ", Size " << C.Size;
  if (C.AssignedSize) {
    if (C.AssignedSize == C.Size)
      OS << ", Assigned";
    else
      OS << ", Partially assigned " << C.AssignedSize;
  }
  return OS;
}

raw_ostream &operator<<(raw_ostream &OS, const LocalCandidate &EC) {
  OS << *EC.Cand << ", Benefit " << EC.Benefit;
  return OS;
}

raw_ostream &operator<<(raw_ostream &OS, const EntryCandidate &EC) {
  OS << *EC.LC << ", Global benefit " << EC.Benefit;
  return OS;
}
#endif

namespace {

// A strongly connected component in the call graph. These SCCs themselves form
// a DAG used throughout this algorithm.
struct SCC {
  SmallVector<Function *> Funcs;
  SmallVector<SCC *> Callees;
  SmallVector<SCC *> Callers;
  SmallVector<LocalCandidate> Candidates;

  // Offset of the direct page area for this SCC from the start of the entry
  // graph. If this is within an interrupt-norecurse call, relative to the start
  // of the call.
  size_t DPOffset = 0;

  // Current size of the direct page area of this SCC, in bytes.
  size_t DPSize = 0;

  // The maximum amount of DP used by any path that includes this SCC. If this
  // is within an interrupt-norecurse call, only considers the contents of the
  // call.
  size_t MaxDPSize = 0;
};

// A view of the SCC graph rooted at an externally callable node. Since we can't
// generally reason about the relative frequencies of calls from outside the TU,
// instead DP's are assigned equally to EntryGraphs round-robin.
struct EntryGraph {
  SCC *Entry;

  // Candidates scored for this entry point, ordered best first.
  std::vector<EntryCandidate> Candidates = {};

  bool IsINR = false;

  size_t NextCand = 0;
};

struct SCCGraph {
  std::vector<SCC> SCCs;
  DenseMap<const Function *, SCC *> FunctionSCCs;
  // Corresponds to the external calling sentinel call graph node.
  SCC *ExternalCallingSCC;
  std::vector<std::unique_ptr<Candidate>> Candidates;
  size_t DPSize = 0;
  size_t GlobalDPSize = 0;
  size_t InterruptDPSize = 0;
  size_t RegularDPSize = 0;
  DenseMap<const MachineFunction *, size_t> MFDPSizes = DenseMap<const MachineFunction *, size_t>();
};

} // namespace

template <> struct llvm::GraphTraits<EntryGraph> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const EntryGraph &EG) { return EG.Entry; }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

template <> struct llvm::GraphTraits<SCC> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const SCC &SCC) { return const_cast<struct SCC *>(&SCC); }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

template <> struct llvm::GraphTraits<SCCGraph> {
  using NodeRef = SCC *;
  using ChildIteratorType = SmallVector<SCC *>::iterator;

  static NodeRef getEntryNode(const SCCGraph &G) { return G.ExternalCallingSCC; }
  static ChildIteratorType child_begin(NodeRef N) { return N->Callees.begin(); }
  static ChildIteratorType child_end(NodeRef N) { return N->Callees.end(); }
};

namespace {

class MC6809DirectPageAlloc : public ModulePass {
public:
  static char ID;

  MC6809DirectPageAlloc() : ModulePass(ID) { llvm::initializeMC6809DirectPageAllocPass(*PassRegistry::getPassRegistry()); }

  bool runOnModule(Module &M) override;
  void getAnalysisUsage(AnalysisUsage &AU) const override;

private:
  MachineModuleInfo *MMI;
  unsigned ModuleDPAvail;
  SCCGraph buildSCCGraph(Module &M);

  void collectCandidates(MachineFunction &MF, std::vector<std::unique_ptr<Candidate>> &Candidates, SmallVectorImpl<LocalCandidate> &LocalCandidates, DenseMap<GlobalVariable *, Candidate *> &GVCandidates);

  std::vector<EntryGraph> buildEntryGraphs(Module &M, SCCGraph &SCCGraph);
  bool assignDPs(SCCGraph &SCCGraph, std::vector<EntryGraph>::iterator Begin, std::vector<EntryGraph>::iterator End);
  bool assignDP(SCCGraph &SCCGraph, EntryGraph &EG);
};

void MC6809DirectPageAlloc::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  AU.addRequired<MachineModuleInfoWrapperPass>();
  AU.addPreserved<MachineModuleInfoWrapperPass>();
  AU.addRequired<BlockFrequencyInfoWrapperPass>();
  AU.addRequired<CallGraphWrapperPass>();

  AU.addPreserved<BasicAAWrapperPass>();
  AU.addPreserved<DominanceFrontierWrapperPass>();
  AU.addPreserved<DominatorTreeWrapperPass>();
  AU.addPreserved<AAResultsWrapperPass>();
  AU.addPreserved<GlobalsAAWrapperPass>();
  AU.addPreserved<IVUsersWrapperPass>();
  AU.addPreserved<LoopInfoWrapperPass>();
  AU.addPreserved<MemoryDependenceWrapperPass>();
  AU.addPreserved<ScalarEvolutionWrapperPass>();
  AU.addPreserved<SCEVAAWrapperPass>();
}

} // namespace

static float getFreq(const BlockFrequencyInfo &BFI, MachineBasicBlock &MBB);

bool MC6809DirectPageAlloc::runOnModule(Module &M) {
  if (!DPAvail)
    return false;

  // The frontend should report this error on the corresponding option.
  assert(DPAvail <= 256 - 32 && "There must be room for the imaginary registers.");

  ModuleDPAvail = DPAvail;
  for (GlobalVariable &GV : M.globals()) {
    StringRef SecName = GV.getSection();
    if (MC6809::isDirectPageSectionName(SecName) || GV.getAddressSpace() == MC6809::AS_DirectPage) {
      size_t Size = (GV.getParent()->getDataLayout().getTypeSizeInBits(GV.getValueType()) + 7) / 8;
      if (Size >= ModuleDPAvail)
        return false;
      ModuleDPAvail -= Size;
    }
  }

  MMI = &getAnalysis<MachineModuleInfoWrapperPass>().getMMI();

  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");
  LLVM_DEBUG(dbgs() << "** MC6809 Direct Page Allocation\n");
  LLVM_DEBUG(dbgs() << "*******************************************************"
                       "*************************\n");

  SCCGraph SCCGraph = buildSCCGraph(M);

  std::vector<EntryGraph> EntryGraphs = buildEntryGraphs(M, SCCGraph);

  // Interrupt-norecurse functions get absolute priority, since they're almost
  // always time-sensitive, and they have an abnormally high number of CSRs.
  const auto RegularEGBegin = partition(EntryGraphs, [](EntryGraph &EG) { return !EG.IsINR; });

  // Assign DP locations to entry graphs round-robin until no candidates remain.
  LLVM_DEBUG(dbgs() << "Assigning DP to candidates:\n");
  while (assignDPs(SCCGraph, EntryGraphs.begin(), RegularEGBegin))
    ;
  while (assignDPs(SCCGraph, RegularEGBegin, EntryGraphs.end()))
    ;

  // Move the offsets of the interrupts after everything else and after one
  // another.
  size_t InterruptOffset = SCCGraph.GlobalDPSize + SCCGraph.RegularDPSize;
  for (EntryGraph &EG : EntryGraphs) {
    if (!EG.IsINR)
      continue;
    EG.Entry->DPOffset = InterruptOffset;
    InterruptOffset += EG.Entry->MaxDPSize;
    for (SCC *Comp : ReversePostOrderTraversal<SCC>(*EG.Entry)) {
      size_t EndOffset = Comp->DPOffset + Comp->DPSize;
      for (struct SCC *Callee : Comp->Callees)
        Callee->DPOffset = std::max(Callee->DPOffset, EndOffset);
    }
  }

  LLVM_DEBUG(dbgs() << "Enacting assignments:\n");

  // Create a global variable for the static stack as a whole.
  size_t StackSize = SCCGraph.InterruptDPSize + SCCGraph.RegularDPSize;
  GlobalVariable *Stack;
  if (StackSize) {
    Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), StackSize);
    Stack = new GlobalVariable(M, Typ, /*IsConstant=*/false, GlobalValue::PrivateLinkage, UndefValue::get(Typ), "dp_stack",
                               /*InsertBefore=*/nullptr, GlobalValue::NotThreadLocal, MC6809::AS_DirectPage);
    LLVM_DEBUG(dbgs() << "  " << *Stack << '\n');
  }

  bool Changed = false;
  DenseMap<const MachineFunction *, size_t> NextOffsets;
  for (std::unique_ptr<Candidate> &Cand : SCCGraph.Candidates) {
    if (Cand->AssignedSize < Cand->Size)
      continue;
    Changed = true;
    if (Cand->GV) {
      // The dance here with Tmp avoids an infinite recursion in
      // replaceAllUsesWith().
      auto *Tmp = new GlobalVariable(M, Cand->GV->getValueType(), Cand->GV->isConstant(), Cand->GV->getLinkage(), Cand->GV->getInitializer());
      Cand->GV->replaceAllUsesWith(Tmp);
      Cand->GV->mutateType(PointerType::get(M.getContext(), MC6809::AS_DirectPage));
      Tmp->replaceAllUsesWith(ConstantExpr::getAddrSpaceCast(Cand->GV, PointerType::get(M.getContext(), 0)));
      Tmp->eraseFromParent();
      LLVM_DEBUG(dbgs() << "  " << *Cand->GV << '\n');
    } else {
      MachineFunction &MF = *Cand->MF;
      MachineFrameInfo &MFI = MF.getFrameInfo();
      auto Res = NextOffsets.try_emplace(&MF, 0);
      size_t &Offset = Res.first->second;
      if (Res.second) {
        Constant *Aliasee = Stack;
        if (Cand->Comp->DPOffset) {
          Type *I16 = Type::getInt16Ty(Stack->getContext());
          Aliasee = ConstantExpr::getGetElementPtr(Stack->getValueType(), Stack, SmallVector<Constant *>{ConstantInt::get(I16, 0), ConstantInt::get(I16, Cand->Comp->DPOffset)}, /*InBounds=*/true);
        }
        Cand->Comp->DPOffset += SCCGraph.MFDPSizes[&MF];
        Type *Typ = ArrayType::get(Type::getInt8Ty(M.getContext()), Cand->Comp->DPSize);
        auto *Alias = GlobalAlias::create(Typ, Stack->getAddressSpace(), Stack->getLinkage(), Twine(MF.getName()) + "_dp_stk", Aliasee, Stack->getParent());
        LLVM_DEBUG(dbgs() << "  " << *Alias);
      }
      LLVM_DEBUG(dbgs() << "  " << *Cand << ", Offset " << Offset << '\n');

      if (Cand->CSR) {
        DenseMap<Register, size_t> &CSRDPOffsets = MF.getInfo<MC6809FunctionInfo>()->CSRDPOffsets;
        const TargetRegisterInfo &TRI = *MF.getSubtarget().getRegisterInfo();
        if (MC6809::Imag16RegClass.contains(Cand->CSR)) {
          CSRDPOffsets[Cand->CSR] = CSRDPOffsets[TRI.getSubReg(Cand->CSR, MC6809::sub_lo_byte)] = Offset++;
          CSRDPOffsets[TRI.getSubReg(Cand->CSR, MC6809::sub_hi_byte)] = Offset++;
        } else {
          CSRDPOffsets[Cand->CSR] = Offset++;
        }
      } else {
        assert(Cand->FI >= 0);
        MFI.setStackID(Cand->FI, TargetStackID::Mc6809DirectPage);
        MFI.setObjectOffset(Cand->FI, Offset);
        Offset += Cand->Size;
      }
    }
  }

  return Changed;
}

// Contract the call graph into its strongly-connect components, then build a
// SCC DAG out of the results.
SCCGraph MC6809DirectPageAlloc::buildSCCGraph(Module &M) {
  auto &CG = getAnalysis<CallGraphWrapperPass>().getCallGraph();

  mc6809::addLibcallEdges(CG, *MMI);
  mc6809::addExternalEdges(CG);
  LLVM_DEBUG(CG.dump());

  std::vector<SCC> SCCs;
  std::vector<SmallSet<const CallGraphNode *, 4>> SCCCallees;
  DenseMap<const CallGraphNode *, size_t> SCCIdx;
  std::vector<std::unique_ptr<Candidate>> Candidates;
  DenseMap<GlobalVariable *, Candidate *> GVCandidates;
  for (auto I = scc_begin(&CG), E = scc_end(&CG); I != E; ++I) {
    SCCs.emplace_back();
    SCC &SCC = SCCs.back();
    SCCCallees.emplace_back();
    for (const CallGraphNode *N : *I) {
      SCCIdx[N] = SCCs.size() - 1;
      auto &Callees = SCCCallees.back();
      for (const auto &KV : *N)
        Callees.insert(KV.second);

      Function *F = N->getFunction();
      if (F)
        SCC.Funcs.push_back(F);
    }
    for (Function *F : SCC.Funcs) {
      MachineFunction *MF = MMI->getMachineFunction(*F);
      if (!MF)
        continue;
      collectCandidates(*MF, Candidates, SCC.Candidates, GVCandidates);
    }
  }
  for (const auto &KV : enumerate(SCCCallees)) {
    SmallVector<SCC *> &Callees = SCCs[KV.index()].Callees;
    for (const CallGraphNode *Callee : KV.value()) {
      size_t CalleeSCCIdx = SCCIdx[Callee];
      if (CalleeSCCIdx != KV.index())
        Callees.push_back(&SCCs[CalleeSCCIdx]);
    }
  }
  DenseMap<const Function *, SCC *> FunctionSCCs;

  SCC *ExternalCallingSCC = &SCCs[SCCIdx[CG.getExternalCallingNode()]];
  for (SCC &Component : SCCs) {
    for (LocalCandidate &LC : Component.Candidates)
      if (!LC.Cand->GV)
        LC.Cand->Comp = &Component;
    for (const Function *F : Component.Funcs)
      FunctionSCCs[F] = &Component;
    for (SCC *Callee : Component.Callees)
      Callee->Callers.push_back(&Component);
  }

  LLVM_DEBUG({
    dbgs() << "Candidates:\n";
    for (SCC &Component : SCCs)
      for (const LocalCandidate &LC : Component.Candidates)
        dbgs() << "  " << LC << '\n';
    dbgs() << '\n';
  });

  return {std::move(SCCs), std::move(FunctionSCCs), ExternalCallingSCC, std::move(Candidates)};
}

// For each SCC, find all of the local options for DP allocation and score
// them relative to the function's entry frequency.
void MC6809DirectPageAlloc::collectCandidates(MachineFunction &MF, std::vector<std::unique_ptr<Candidate>> &Candidates, SmallVectorImpl<LocalCandidate> &LocalCandidates, DenseMap<GlobalVariable *, Candidate *> &GVCandidates) {
  const MachineFrameInfo &MFI = MF.getFrameInfo();
  auto &BFI = getAnalysis<BlockFrequencyInfoWrapperPass>(MF.getFunction()).getBFI();

  DenseMap<GlobalVariable *, float> GlobalBenefit;
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      for (const MachineOperand &MO : MI.operands()) {
        if (MI.mayLoadOrStore() && MO.isGlobal()) {
          const GlobalValue *GVal = MO.getGlobal();
          const GlobalObject *GO = GVal->getAliaseeObject();
          if (!GO)
            continue;
          const auto *GV = dyn_cast<GlobalVariable>(GO);
          if (!GV || GV->isDeclaration() || GV->getAlign().valueOrOne() != 1 || GV->hasSection() || GV->hasImplicitSection() || GV->getAddressSpace() == MC6809::AS_DirectPage)
            continue;

          // Generally moving an absolute reference to the direct page saves one
          // cycle and one byte.
          GlobalBenefit[const_cast<GlobalVariable *>(GV)] += 2 * getFreq(BFI, MBB);
        }
      }
    }
  }

  for (const auto &KV : GlobalBenefit) {
    GlobalVariable *GV = KV.first;
    auto It = GVCandidates.find(GV);
    if (It == GVCandidates.end()) {
      size_t Size = (GV->getParent()->getDataLayout().getTypeSizeInBits(GV->getValueType()) + 7) / 8;
      if (!Size)
        continue;
      Candidates.push_back(std::make_unique<Candidate>(Candidate{/*MF=*/nullptr, Size, /*CSR=*/0, GV}));
      It = GVCandidates.try_emplace(GV, Candidates.back().get()).first;
    }
    assert(It != GVCandidates.end());
    Candidate *Cand = It->second;
    float Benefit = KV.second;
    Benefit /= Cand->Size;
    LocalCandidates.push_back(LocalCandidate{Cand, Benefit});
  }

  const MC6809FrameLowering &TFL = *MF.getSubtarget<MC6809Subtarget>().getFrameLowering();

  float SaveFreq = 0;
  float RestoreFreq = 0;
  if (MFI.getSavePoints().empty()) {
    SaveFreq = getFreq(BFI, *MF.begin());
    for (MachineBasicBlock &MBB : MF) {
      if (MBB.isReturnBlock())
        RestoreFreq += getFreq(BFI, MBB);
    }
  } else {
    for (const auto &[SavePoint, _] : MFI.getSavePoints())
      SaveFreq += getFreq(BFI, *SavePoint);
    for (const auto &[RestorePoint, _] : MFI.getRestorePoints()) {
      // If RestorePoint does not have any successor and is not a return block
      // then the end point is unreachable and we do not need to insert any
      // epilogue.
      if (RestorePoint->succ_empty() && !RestorePoint->isReturnBlock())
        continue;
      RestoreFreq += getFreq(BFI, *RestorePoint);
    }
  }

  // Compute the benefit for moving each CSR to a static DP location.
  BitVector SavedRegs;
  TFL.determineCalleeSaves(MF, SavedRegs, /*RS=*/nullptr);
  unsigned Idx = 0;
  DenseSet<Register> Imag16Regs;
  for (Register Reg : SavedRegs.set_bits()) {
    if (!MC6809::Imag8RegClass.contains(Reg))
      continue;

    // If a call occurs with a different calling convention, it may clobber
    // callee-saved registers in a way that can't be rewritten by this pass.
    // These need to be saved and restored like normal.
    if (MF.getRegInfo().getUsedPhysRegsMask().test(Reg))
      continue;

    size_t Size = 1;
    float Benefit = 0;
    if (Idx++ < 4) {
      Benefit = 9 * SaveFreq;      // LDA ZP,PHA
      Benefit += 10 * RestoreFreq; // PLA,STA ZP
    } else {
      Benefit = 12 * SaveFreq;     // LDA ZP,STA ABS
      Benefit += 12 * RestoreFreq; // LDA ABS,STA ZP
    }

    // If the CSR is used as a 16-bit pointer, then the two halves cannot be
    // assigned independently. Thus, instead of two size-1 candidates, one
    // size-2 candidate is used.
    Register Imag16 = *MF.getSubtarget().getRegisterInfo()->superregs(Reg).begin();
    assert(MC6809::Imag16RegClass.contains(Imag16));
    if (!MF.getRegInfo().reg_nodbg_empty(Imag16)) {
      // Don't create the Imag16 candidate twice.
      if (Imag16Regs.contains(Imag16))
        continue;

      Reg = Imag16;
      Imag16Regs.insert(Reg);

      // Account for the second byte.
      if (Idx++ < 4) {
        Benefit = 9 * SaveFreq;      // LDA ZP,PHA
        Benefit += 10 * RestoreFreq; // PLA,STA ZP
      } else {
        Benefit = 12 * SaveFreq;     // LDA ZP,STA ABS
        Benefit += 12 * RestoreFreq; // LDA ABS,STA ZP
      }
      ++Size;
      Benefit /= Size;
    }

    Candidates.push_back(std::make_unique<Candidate>(Candidate{&MF, Size, /*CSR=*/Reg}));
    LocalCandidates.push_back(LocalCandidate{Candidates.back().get(), Benefit});
  }

  std::vector<SmallVector<MachineInstr *>> FIMIs(MFI.getObjectIndexEnd());
  for (MachineBasicBlock &MBB : MF)
    for (MachineInstr &MI : MBB)
      for (const MachineOperand &MO : MI.operands())
        if (MO.isFI() && MO.getIndex() >= 0)
          FIMIs[MO.getIndex()].push_back(&MI);

  // Compute the benefit for moving each stack object to the DP.
  for (int I = 0, E = MFI.getObjectIndexEnd(); I != E; ++I) {
    if (MFI.isDeadObjectIndex(I) || MFI.isVariableSizedObjectIndex(I))
      continue;

    float Benefit = 0;
    for (MachineInstr *MI : FIMIs[I]) {
      // Generally moving an absolute reference to the direct page saves one
      // cycle and one byte.
      Benefit += 2 * getFreq(BFI, *MI->getParent());
    }
    auto Size = static_cast<size_t>(MFI.getObjectSize(I));
    Benefit /= Size;
    Candidates.push_back(std::make_unique<Candidate>(Candidate{&MF, Size, /*CSR=*/0, /*GV=*/nullptr, /*FI=*/I}));
    LocalCandidates.push_back(LocalCandidate{Candidates.back().get(), Benefit});
  }
}

static bool isUndef(const Constant *C) {
  if (isa<UndefValue>(C))
    return true;
  if (!isa<ConstantAggregate>(C))
    return false;
  for (auto Operand : C->operand_values()) {
    if (!isUndef(cast<Constant>(Operand)))
      return false;
  }
  return true;
}

static bool isSuitableForNoInit(const GlobalVariable *GV) {
  const Constant *C = GV->getInitializer();

  // Must have an undef initializer.
  if (!isUndef(C))
    return false;

  // If the global has an explicit section specified, don't put it in BSS.
  if (GV->hasSection())
    return false;

  // Otherwise, put it in NoInit!
  return true;
}

// For each globally-callable entry point, trace the SCCs transitively callable
// from that entry, and assign each their relative frequency to the entry point.
// From these frequences, an ordered set of candidates is derived for each entry
// point.
std::vector<EntryGraph> MC6809DirectPageAlloc::buildEntryGraphs(Module &M, SCCGraph &SCCGraph) {
  std::vector<EntryGraph> EntryGraphs;
  for (SCC *Entry : SCCGraph.ExternalCallingSCC->Callees) {
    EntryGraphs.push_back(EntryGraph{Entry});
    EntryGraphs.back().IsINR = any_of(Entry->Funcs, [](Function *F) { return F->hasFnAttribute("interrupt-norecurse"); });
  }
  for (EntryGraph &EG : EntryGraphs) {
    LLVM_DEBUG({
      dbgs() << "Entry SCC\n";
      for (const Function *F : EG.Entry->Funcs)
        dbgs() << "  " << F->getName() << "\n";
      dbgs() << '\n';
    });

    DenseMap<const SCC *, float> EntryFreqs;
    EntryFreqs[EG.Entry] = 1;

    // Callers are traversed before callees.
    for (SCC *Component : ReversePostOrderTraversal<EntryGraph>(EG)) {
      // Keep track of the original entry frequency of the SCC. Recursive calls
      // within the SCC should increase the entry frequency, but this shouldn't
      // compound, so they're scaled to the original entry frequency, not the
      // increased one.
      float OldEntryFreq = EntryFreqs[Component];

      // This is the current entry frequency of the SCC, based on SCC callers
      // and recursive calls seen so far.
      float EntryFreq = OldEntryFreq;
      LLVM_DEBUG(dbgs() << "  SCC " << EntryFreq << "\n");

      // Find all calls within the SCC and propagate entry frequencies across
      // the edges.
      DenseMap<const Function *, float> CalleeFreqs;
      for (Function *F : Component->Funcs) {
        LLVM_DEBUG(dbgs() << "    " << F->getName() << "\n");
        MachineFunction *MF = MMI->getMachineFunction(*F);
        if (!MF)
          continue;

        auto &BFI = getAnalysis<BlockFrequencyInfoWrapperPass>(*F).getBFI();
        for (MachineBasicBlock &MBB : *MF) {
          for (const MachineInstr &MI : MBB) {
            if (!MI.isCall())
              continue;
            for (const MachineOperand &MO : MI.operands()) {
              const Function *Callee = nullptr;
              if (MO.isGlobal())
                Callee = dyn_cast<Function>(MO.getGlobal());
              else if (MO.isSymbol())
                Callee = mc6809::getSymbolFunction(M, MO.getSymbolName());
              if (!Callee)
                continue;
              float Freq = getFreq(BFI, MBB);
              if (is_contained(Component->Funcs, Callee)) {
                LLVM_DEBUG(dbgs() << "      Recursively calls " << Callee->getName() << " " << Freq << '\n');
                // Recursive calls are another way to enter the given SCC, so
                // increase the Entry frequency. Don't compound the increases
                // though; it's not worth risking overflowing the entry counts,
                // especially since possible recursion paths may be accidental.
                EntryFreq += OldEntryFreq * Freq;
                LLVM_DEBUG(dbgs() << "    SCC freq += " << OldEntryFreq * Freq << '\n');
              }
              // Defer handling normal calls until after the loop; the final
              // entry frequency won't be known until afterwards.
              LLVM_DEBUG(dbgs() << "      Calls " << Callee->getName() << ' ' << Freq << '\n');
              CalleeFreqs[Callee] += Freq;
            }
          }
        }
      }
      // Now that recursion has been handled, the final entry frequency is known
      // for the component.

      // Apply the final entry frequency to the candidates.
      for (LocalCandidate &LC : Component->Candidates) {
        EntryCandidate EC{&LC, EntryFreq * LC.Benefit};
        if (LC.Cand->GV && LC.Cand->GV->hasInitializer() && !isSuitableForNoInit(LC.Cand->GV)) {
          // Pessimistically assume that initializing the global variable costs
          // takes a LDA #imm, STA zp, and that the entry function is only
          // called once.
          if (EC.Benefit < 9) {
            LLVM_DEBUG(dbgs() << "GV not worth initializing to DP; skipping: " << EC << '\n');
            continue;
          }
        }

        EG.Candidates.push_back(EC);
      }

      // Apply the final entry frequency to each outgoing call from the SCC and
      // propagate the resulting entry frequencies to callee SCCs.
      for (const auto &KV : CalleeFreqs) {
        float Freq = EntryFreq * KV.second;
        LLVM_DEBUG(dbgs() << "    " << KV.first->getName() << " += " << Freq << '\n');
        EntryFreqs[SCCGraph.FunctionSCCs[KV.first]] += Freq;
      }
    }

    stable_sort(EG.Candidates, [](const EntryCandidate &A, const EntryCandidate &B) { return A.Benefit > B.Benefit; });
    LLVM_DEBUG({
      dbgs() << "\n  Candidates:\n";
      for (EntryCandidate &Cand : EG.Candidates)
        dbgs() << "    " << Cand << '\n';
      dbgs() << '\n';
    });
  }
  return EntryGraphs;
}

bool MC6809DirectPageAlloc::assignDPs(SCCGraph &SCCGraph, std::vector<EntryGraph>::iterator Begin, std::vector<EntryGraph>::iterator End) {
  bool AssignedAny = false;
  for (auto I = Begin; I != End; ++I)
    AssignedAny |= assignDP(SCCGraph, *I);
  return AssignedAny;
}

bool MC6809DirectPageAlloc::assignDP(SCCGraph &SCCGraph, EntryGraph &EG) {
  const auto NewDPSize = [&](Candidate &Cand, size_t Size) {
    size_t NewGlobalSize = SCCGraph.GlobalDPSize;
    size_t NewRegularSize = SCCGraph.RegularDPSize;
    size_t NewInterruptSize = SCCGraph.InterruptDPSize;
    if (Cand.GV) {
      NewGlobalSize += Size;
    } else {
      if (EG.IsINR) {
        NewInterruptSize -= EG.Entry->MaxDPSize;
        NewInterruptSize += std::max(EG.Entry->MaxDPSize, Size + Cand.Comp->MaxDPSize);
      } else {
        NewRegularSize = std::max(NewRegularSize, Size + Cand.Comp->MaxDPSize);
      }
    }
    return NewGlobalSize + NewRegularSize + NewInterruptSize;
  };

  // Advance to first unassigned candidate.
  for (;; ++EG.NextCand) {
    if (EG.NextCand == EG.Candidates.size())
      return false;
    EntryCandidate &EC = EG.Candidates[EG.NextCand];
    Candidate &Cand = *EC.LC->Cand;
    // Another entry path may have already assigned the candidate.
    if (Cand.AssignedSize == Cand.Size)
      continue;
    // If the candidate is too big to fit, no reason to start allocating bytes
    // to it.
    if (!Cand.AssignedSize && NewDPSize(Cand, Cand.Size) > ModuleDPAvail)
      continue;
    break;
  }
  EntryCandidate &EC = EG.Candidates[EG.NextCand];
  Candidate &Cand = *EC.LC->Cand;

  ++Cand.AssignedSize;

  LLVM_DEBUG(dbgs() << "Entry " << EG.Entry->Funcs.front()->getName() << ", Func " << Cand << '\n';);

  if (NewDPSize(Cand, Cand.AssignedSize) > ModuleDPAvail) {
    LLVM_DEBUG(dbgs() << "No longer fits; unassigning.\n");
    size_t NumToReassign = Cand.AssignedSize;
    Cand.AssignedSize = 0;
    bool AssignedAny = false;
    for (size_t I = 0; I < NumToReassign; ++I)
      AssignedAny |= assignDP(SCCGraph, EG);
    return AssignedAny;
  }

  if (Cand.AssignedSize < Cand.Size)
    return true;

  // Update the whole-graph sizes.
  if (Cand.GV) {
    SCCGraph.GlobalDPSize += Cand.Size;
    return true;
  }

  Cand.Comp->DPSize += Cand.AssignedSize;
  SCCGraph.MFDPSizes[Cand.MF] += Cand.AssignedSize;

  if (EG.IsINR) {
    SCCGraph.InterruptDPSize -= EG.Entry->MaxDPSize;
    SCCGraph.InterruptDPSize += std::max(EG.Entry->MaxDPSize, Cand.Size + Cand.Comp->MaxDPSize);
  } else {
    SCCGraph.RegularDPSize = std::max(SCCGraph.RegularDPSize, Cand.Size + Cand.Comp->MaxDPSize);
  }

  // Allocating a DP can change the offsets of transitive callees, so propagate
  // those downward.
  for (SCC *Comp : ReversePostOrderTraversal<SCC>(*Cand.Comp)) {
    size_t EndOffset = Comp->DPOffset + Comp->DPSize;
    for (struct SCC *Callee : Comp->Callees)
      Callee->DPOffset = std::max(Callee->DPOffset, EndOffset);
  }

  // From the new offsets, the new max DP sizes for paths through each node can
  // be computed. They're trivial at the leaves and computable upward.
  for (SCC *Comp : post_order(*EG.Entry)) {
    if (Comp->Callees.empty()) {
      Comp->MaxDPSize = Comp->DPOffset + Comp->DPSize;
      continue;
    }

    Comp->MaxDPSize = 0;
    for (struct SCC *Callee : Comp->Callees)
      Comp->MaxDPSize = std::max(Comp->MaxDPSize, Callee->MaxDPSize);
  }

  return true;
}

// We can't use machine block frequency due to a pass scheduling SNAFU, so
// approximate with the IR block frequencies.
static float getFreq(const BlockFrequencyInfo &BFI, MachineBasicBlock &MBB) {
  if (!MBB.getBasicBlock())
    return 1;
  return (float)BFI.getBlockFreq(MBB.getBasicBlock()).getFrequency() / (float)BFI.getEntryFreq().getFrequency();
}

char MC6809DirectPageAlloc::ID = 0;

INITIALIZE_PASS(MC6809DirectPageAlloc, DEBUG_TYPE, "Allocate direct page", false, false)

ModulePass *llvm::createMC6809DirectPageAllocPass() { return new MC6809DirectPageAlloc(); }
