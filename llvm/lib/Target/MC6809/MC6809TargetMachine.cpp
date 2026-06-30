//===-- MC6809TargetMachine.cpp - Define TargetMachine for MC6809 ---------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 specific subclass of TargetMachine.
//
//===----------------------------------------------------------------------===//

#include "MC6809TargetMachine.h"

#include "llvm/CodeGen/CodeGenTargetMachineImpl.h"
#include "llvm/CodeGen/GlobalISel/CSEInfo.h"
#include "llvm/CodeGen/GlobalISel/IRTranslator.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelect.h"
#include "llvm/CodeGen/GlobalISel/Legalizer.h"
#include "llvm/CodeGen/GlobalISel/Localizer.h"
#include "llvm/CodeGen/GlobalISel/RegBankSelect.h"
#include "llvm/CodeGen/MachineBlockFrequencyInfo.h"
#include "llvm/CodeGen/Passes.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/InitializePasses.h"
#include "llvm/MC/TargetRegistry.h"
#include "llvm/Passes/PassBuilder.h"
#include "llvm/Support/CodeGen.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar/IndVarSimplify.h"
#include "llvm/Transforms/Utils.h"

#include "GISel/MC6809Combiner.h"
#include "MC6809.h"
#include "MC6809CopyOpt.h"
#include "MC6809DirectPageAlloc.h"
#include "MC6809StaticStackAlloc.h"
#include "MC6809FinalLowering.h"
#include "MC6809IncDecPhi.h"
#include "MC6809IndexIV.h"
#include "MC6809InsertCopies.h"
#include "MC6809SanitiseDebugInfo.h"
#include "MC6809Internalize.h"
#include "MC6809BundleCC.h"
#include "MC6809LateOptimization.h"
#include "MC6809PostRASpillOpt.h"
#include "MC6809PreCGPFreeze.h"
#include "MC6809LowerSelect.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809MachineScheduler.h"
#include "MC6809NonReentrant.h"
#include "MC6809PostRAScavenging.h"
#include "MC6809MaterializeSpills.h"
#include "MC6809ShiftRotateChain.h"
#include "MC6809TargetObjectFile.h"
#include "MC6809TargetTransformInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

using namespace llvm;

extern "C" LLVM_EXTERNAL_VISIBILITY void LLVMInitializeMC6809Target() {
  // Register the target.
  RegisterTargetMachine<MC6809TargetMachine> X(getTheMC6809Target());

  PassRegistry &PR = *PassRegistry::getPassRegistry();
  initializeGlobalISel(PR);
  initializeMC6809BundleCCPass(PR);
  initializeMC6809CombinerPass(PR);
  initializeMC6809CopyOptPass(PR);
  initializeMC6809FinalLoweringPass(PR);
  initializeMC6809IncDecPhiPass(PR);
  initializeMC6809InsertCopiesPass(PR);
  initializeMC6809InternalizePass(PR);
  initializeMC6809LateOptimizationPass(PR);
  initializeMC6809LowerSelectPass(PR);
  initializeMC6809NonReentrantPass(PR);
  initializeMC6809PhantomCarryGuardPass(PR);
  initializeMC6809PostRAScavengingPass(PR);
  initializeMC6809PostRASpillOptPass(PR);
  initializeMC6809PreCGPFreezePass(PR);
  initializeMC6809MaterializeSpillsPass(PR);
  initializeMC6809SanitiseDebugInfoPass(PR);
  initializeMC6809ShiftRotateChainPass(PR);
  initializeMC6809DirectPageAllocPass(PR);
  initializeMC6809StaticStackAllocPass(PR);
  initializeMC6809PreferPage1IndexPass(PR);
  initializeMC6809FoldAddSub16Pass(PR);
  initializeMC6809FoldCallThroughMemPass(PR);
  initializeMC6809FoldBankCrossPass(PR);
}

// MC6809 is big-endian (Motorola byte order).
// Address space 1 is the direct page ($0000-$00FF): 8-bit pointers,
// since the high byte is implicit in the DP register. Bug #178 user
// `__attribute__((directpage))` globals live here.
static const char *MC6809DataLayout = "E-p:16:8-p1:8:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) { return (CPU.empty() || CPU == "generic") ? "mc6809" : CPU; }

static Reloc::Model getEffectiveRelocModel(std::optional<Reloc::Model> RM) { return RM ? *RM : Reloc::Static; }

MC6809TargetMachine::MC6809TargetMachine(const Target &T, const Triple &TT, StringRef CPU, StringRef FS, const TargetOptions &Options, std::optional<Reloc::Model> RM, std::optional<CodeModel::Model> CM, CodeGenOptLevel OL, bool JIT)
    : CodeGenTargetMachineImpl(T, MC6809DataLayout, TT, getCPU(CPU), FS, Options, getEffectiveRelocModel(RM), getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, getCPU(CPU).str(), FS.str(), *this) {
  this->TLOF = std::make_unique<MC6809TargetObjectFile>();

  initAsmInfo();

  // MC6809 has no FP registers — always soft-float.
  if (this->Options.FloatABIType == FloatABI::Default)
    this->Options.FloatABIType = FloatABI::Soft;

  setGlobalISel(true);
  // Prevents fallback to SelectionDAG by allowing direct aborts.
  setGlobalISelAbort(GlobalISelAbortMode::Enable);
}

const MC6809Subtarget *MC6809TargetMachine::getSubtargetImpl(const Function &F) const {
  Attribute CPUAttr = F.getFnAttribute("target-cpu");
  Attribute FSAttr = F.getFnAttribute("target-features");

  auto CPU = CPUAttr.isValid() ? CPUAttr.getValueAsString().str() : TargetCPU;
  auto FS = FSAttr.isValid() ? FSAttr.getValueAsString().str() : TargetFS;

  auto &I = SubtargetMap[CPU + FS];
  if (!I) {
    // This needs to be done before we create a new subtarget since any
    // creation will depend on the TM and the code generation flags on the
    // function that reside in TargetOptions.
    resetTargetOptions(F);
    I = std::make_unique<MC6809Subtarget>(TargetTriple, CPU, FS, *this);
  }
  return I.get();
}

TargetTransformInfo MC6809TargetMachine::getTargetTransformInfo(const Function &F) const { return TargetTransformInfo(std::make_unique<MC6809TTIImpl>(this, F)); }

void MC6809TargetMachine::registerPassBuilderCallbacks(PassBuilder &PB) {
  PB.registerPipelineParsingCallback([](StringRef Name, LoopPassManager &PM, ArrayRef<PassBuilder::PipelineElement>) {
    if (Name == "mc6809-indexiv") {
      // Rewrite pointer arithmetic in loops to use 8-bit IV offsets.
      PM.addPass(MC6809IndexIV());
      return true;
    }
    return false;
  });

  PB.registerPipelineParsingCallback([](StringRef Name, ModulePassManager &PM, ArrayRef<PassBuilder::PipelineElement>) {
    if (Name == "mc6809-nonreentrant") {
      PM.addPass(MC6809NonReentrantPass());
      return true;
    }
    return false;
  });

  PB.registerFullLinkTimeOptimizationLastEPCallback(
      [](ModulePassManager &MPM, OptimizationLevel Level) {
        MPM.addPass(MC6809NonReentrantPass());
      });

  PB.registerLateLoopOptimizationsEPCallback([](LoopPassManager &PM, OptimizationLevel Level) {
    if (Level != OptimizationLevel::O0) {
      PM.addPass(MC6809IndexIV());

      // New induction variables may have been added.
      PM.addPass(IndVarSimplifyPass());
    }
  });
}

StringRef MC6809TargetMachine::getSectionPrefix(const GlobalObject *GO) const { return GO->getAddressSpace() == MC6809::AS_DirectPage ? ".dp" : ""; }

MachineFunctionInfo *MC6809TargetMachine::createMachineFunctionInfo(BumpPtrAllocator &Allocator, const Function &F, const TargetSubtargetInfo *STI) const {

  return MC6809FunctionInfo::create<MC6809FunctionInfo>(Allocator, F, static_cast<const MC6809Subtarget *>(STI));
}

//===----------------------------------------------------------------------===//
// Pass Pipeline Configuration
//===----------------------------------------------------------------------===//

namespace {
/// MC6809 Code Generator Pass Configuration Options.
class MC6809PassConfig : public TargetPassConfig {
public:
  MC6809PassConfig(MC6809TargetMachine &TM, PassManagerBase &PM) : TargetPassConfig(TM, PM) {}

  MC6809TargetMachine &getMC6809TargetMachine() const { return getTM<MC6809TargetMachine>(); }

  void addIRPasses() override;
  bool addPreISel() override;
  bool addIRTranslator() override;
  void addPreLegalizeMachineIR() override;
  bool addLegalizeMachineIR() override;
  void addPreRegBankSelect() override;
  bool addRegBankSelect() override;
  void addPreGlobalInstructionSelect() override;
  bool addGlobalInstructionSelect() override;

  // Register pressure is too high around calls to work without detailed
  // scheduling.
  bool alwaysRequiresMachineScheduler() const override { return true; }

  void addMachineSSAOptimization() override;

  // Register pressure is too high to work without optimized register
  // allocation.
  void addFastRegAlloc() override { addOptimizedRegAlloc(); }
  void addOptimizedRegAlloc() override;

  void addMachineLateOptimization() override;
  void addPrePEI() override;
  void addPreSched2() override;
  void addPreEmitPass() override;

  std::unique_ptr<CSEConfigBase> getCSEConfig() const override;
};
} // namespace

TargetPassConfig *MC6809TargetMachine::createPassConfig(PassManagerBase &PM) { return new MC6809PassConfig(*this, PM); }

void MC6809PassConfig::addIRPasses() {
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809NonReentrantPass());

  // Bug #214 / Bug #217: LSR was previously force-disabled here
  // (commit d357062e38a9, bug #209 Os-lto leg) to dodge a byte-pair
  // miscompile in `sub i16 0, %iv` paired with `add i16 %base, %iv`.
  // The real fix retired the workaround:
  //
  //   * Bug #214 made the AccReg-clobber explicit on byte-arithmetic
  //     _Reg pseudos via TableGen Defs.
  //   * Bug #217 routed dst through ABc (B-half only) so the
  //     Defs declaration shrinks to `Defs += AB` — regalloc commits
  //     to B-half pre-RA, sees precise per-half pressure, and can
  //     absorb the clobber without the union-claim that broke
  //     non-LTO opt levels.
  //
  // LSR runs at all opt levels.
  TargetPassConfig::addIRPasses();
  // Clean up after LSR in particular.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createInstructionCombiningPass());
  // Pre-freeze icmps that have multiple select users so CodeGenPrepare's
  // per-select freeze expansion does not introduce independent poison
  // resolutions (bug #136). Must be the last IR-level MC6809 pass so it
  // runs immediately before addCodeGenPrepare().
  addPass(createMC6809PreCGPFreezePass());
}

bool MC6809PassConfig::addPreISel() { return false; }

bool MC6809PassConfig::addIRTranslator() {
  addPass(new IRTranslator(getOptLevel()));
  return false;
}

void MC6809PassConfig::addPreLegalizeMachineIR() {
  if (getOptLevel() != CodeGenOptLevel::None) {
    addPass(createMC6809Combiner());
    addPass(createMC6809IncDecPhiPass());
    addPass(createMC6809ShiftRotateChainPass());
  }
}

bool MC6809PassConfig::addLegalizeMachineIR() {
  addPass(new Legalizer());
  addPass(createMC6809InternalizePass());
  return false;
}

void MC6809PassConfig::addPreRegBankSelect() {
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809Combiner());
  addPass(createMC6809LowerSelectPass());
}

bool MC6809PassConfig::addRegBankSelect() {
  addPass(new RegBankSelect());
  return false;
}

void MC6809PassConfig::addPreGlobalInstructionSelect() {
  // This pass helps reduce the live ranges of constants to within a basic
  // block, which can greatly improve machine scheduling, as they can now be
  // moved around to keep register pressure low.
  addPass(new Localizer());
}

bool MC6809PassConfig::addGlobalInstructionSelect() {
  addPass(new InstructionSelect());
  // Post-ISel cleanup (STACK16 vreg → $ss). Must run before RA at all opt
  // levels — see MC6809InsertCopies.cpp.
  addPass(createMC6809InsertCopiesPass());
  // Bug #156: drop dangling DBG_VALUE register operands (vregs with no
  // producer, e.g. clang's widened-i32 debug refs for i16 rsize_t locals).
  // Otherwise LiveVariables asserts at CodeGen/LiveVariables.cpp:159. Must
  // run before LiveVariables (which is scheduled in addOptimizedRegAlloc).
  addPass(createMC6809SanitiseDebugInfoPass());
  return false;
}

void MC6809PassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();
  // Bug #360: refold byte-decomposed i16 add/sub of an immediate back into
  // native ADDD/SUBD. Runs on selected SSA MIR (only at -O1+), after the
  // generic SSA opts and before register allocation.
  addPass(createMC6809FoldAddSub16Pass());
  // Fold an invariant function-pointer load into the indirect call that uses it
  // (jsr [n,r]), freeing the index register the pointer would otherwise occupy
  // across the call. Runs on selected SSA MIR before register allocation.
  addPass(createMC6809FoldCallThroughMemPass());
  // Fold an INDEX->ACCUM copy into the store/compare that consumes it, keeping
  // the value in an index register (stx/cmpx) instead of transferring it to D.
  addPass(createMC6809FoldBankCrossPass());
}

void MC6809PassConfig::addOptimizedRegAlloc() {
  if (getOptLevel() != CodeGenOptLevel::None) {
    // Run the coalescer twice to coalesce RMW patterns revealed by the first
    // coalesce.
    insertPass(&llvm::TwoAddressInstructionPassID, &llvm::RegisterCoalescerID);

    // Re-run Live Intervals after coalescing to renumber the contained values.
    // This can allow constant rematerialization after aggressive coalescing.
    insertPass(&llvm::MachineSchedulerID, &llvm::LiveIntervalsID);
  }
  TargetPassConfig::addOptimizedRegAlloc();
}

void MC6809PassConfig::addMachineLateOptimization() {
  TargetPassConfig::addMachineLateOptimization();
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809CopyOptPass());
}

void MC6809PassConfig::addPrePEI() {
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809DirectPageAllocPass());
  addPass(createMC6809MaterializeSpillsPass());
}

void MC6809PassConfig::addPreSched2() {
  addPass(createMC6809PostRAScavengingPass());
  // Lower control flow pseudos.
  addPass(&FinalizeISelID);
  // Lower pseudos produced by control flow pseudos.
  addPass(&ExpandPostRAPseudosID);
  addPass(createMC6809PostRAScavengingPass());

  // Eliminate redundant spill loads/stores from post-RA expansion.
  //
  // Pipeline-ordering note (bug #165 phase B1): if/when a post-RA
  // MachineScheduler is enabled here (deferred Phase D — see plan), it
  // MUST be added AFTER this SpillOpt pass, not before. SpillOpt is a
  // linear-scan slot-tracker (MC6809PostRASpillOpt.cpp:241-369) that
  // walks each MBB once and deletes redundant load/store pairs whose
  // slots can be proven equivalent — its analysis is invariant to
  // scheduler-induced reordering of *unrelated* loads/stores within the
  // block, but a scheduler running BEFORE it can interleave new
  // load/store pairs that defeat its slot-equivalence detection. Run
  // SpillOpt first (smaller / safer instruction stream), then schedule.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809PostRASpillOptPass());

  // Re-home call-free IY live-ranges to the cheaper page-1 IX where IX is free.
  // Runs after SpillOpt so it sees the final concrete index ops, and before
  // BranchRelaxation since it shrinks page-2 ops by one byte each.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809PreferPage1IndexPass());
}

void MC6809PassConfig::addPreEmitPass() {
  // Bug #387: resolve static-stack frame accesses to real globals. Runs after
  // PEI/ExpandPostRAPseudos have emitted the TI_STATIC_STACK target indices,
  // and before FinalLowering/BranchRelaxation see the final operands. A no-op
  // unless the "static-stack" feature marked some function's frame static.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809StaticStackAllocPass());

  // MC6809FinalLowering (bug #149) MUST run BEFORE BranchRelaxation.
  // Several of its classes (Class 1 offset-relax, Class 4 dup-store,
  // Class 5 store-reload, Class 6 branch-over-branch, Class 7 LEA-
  // pointer-spill) DELETE or SHRINK MachineInstrs. If BranchRelaxation
  // runs first, its block-size accounting (via getInstSizeInBytes) is
  // computed against the pre-shrink layout; downstream shrinks then
  // make the actual emit positions differ from BranchRelaxation's
  // bookkeeping by the size of every elided MI.
  addPass(createMC6809FinalLoweringPass());

  // MC6809LateOptimization (tailJMP, simplifyAndZero, redundant compare-zero
  // elision) runs HERE — after FinalLowering, before BranchRelaxation — not
  // in addPreSched2. The compare-zero elision (Bug #360) drops a redundant
  // CMP/TST #0 by letting the conditional branch read the producer's flags
  // directly; that is only sound on the FINAL instruction stream. Run earlier
  // (before FinalLowering), the elision removes the CMP and FinalLowering's
  // store-reload / LEA-pointer-spill classes then delete the very load whose
  // flags the branch now depends on (e.g. strrchr's `ldy slot; cmpy #0; bne`
  // null-check reload), leaving the branch reading an adjacent LEAX's
  // hardware Z — a miscompile (Bug #360 follow-up). (LEAX's Z-clobber is now
  // modelled in the MIR — Bug #365 — but this ordering is still required:
  // FinalLowering deletes the reload the branch depends on regardless.)
  // Post-FinalLowering the producer immediately before the CMP is final, so
  // the guards reflect reality. Must precede BranchRelaxation so block-size
  // accounting sees the elided/tail-jumped instructions.
  addPass(createMC6809LateOptimizationPass());

  // Bug #357: TFRp is now an isCopyInstr copy (MC6809InstrInfo::isCopyInstrImpl).
  // Run MachineCopyPropagation HERE — after FinalLowering has materialised every
  // TFRp (the two earlier machine-cp runs precede COPY->TFR expansion and never
  // see a TFR), after LateOptimization's #360 compare-zero elision (which needs
  // producer->branch adjacency a surviving TFR would already have blocked), and
  // before BranchRelaxation so block-size accounting sees the removed copies.
  // UseCopyInstr=true makes it consult the target hook. TFR sets no flags, so
  // this never touches the phantom-carry chain.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMachineCopyPropagationPass(/*UseCopyInstr=*/true));

  addPass(&BranchRelaxationPassID);
  // Encoding-overflow safety nets that historically lived in a dedicated
  // post-pass (MC6809NoShortBranches, retired in commit 2bf4696c62d7) have
  // moved into MC6809AsmBackend::applyFixup — PCRel8 (was bug #58), Rel8
  // (was bug #122 _o8 truncation) and Rel5 (was bug #122 _o5 truncation)
  // all get range-checked there at MC fixup time.

  // Bug #302 follow-up: PHANTOM_CARRY invariant guard.  Runs last so it
  // sees the MIR every preceding pass produces.  Aborts with a clear
  // diagnostic if any PHANTOM_CARRY physreg leaks to an explicit operand,
  // or if any pass inserts a $c-clobbering MI between a carry-chain
  // producer and its consumer.  See MC6809PhantomCarryGuard.cpp.
  addPass(createMC6809PhantomCarryGuardPass());
}

ScheduleDAGInstrs *MC6809TargetMachine::createMachineScheduler(MachineSchedContext *C) const {
  return new ScheduleDAGMILive(C, std::make_unique<MC6809SchedStrategy>(C));
}


namespace {

class MC6809CSEConfigFull : public CSEConfigFull {
public:
  virtual ~MC6809CSEConfigFull() = default;
  virtual bool shouldCSEOpc(unsigned Opc) override;
};

bool MC6809CSEConfigFull::shouldCSEOpc(unsigned Opc) {
  switch (Opc) {
  default:
    return CSEConfigFull::shouldCSEOpc(Opc);
  case MC6809::G_LSHRE:
  case MC6809::G_SHLE:
    return true;
  }
}

} // namespace

std::unique_ptr<CSEConfigBase> MC6809PassConfig::getCSEConfig() const {
  if (TM->getOptLevel() == CodeGenOptLevel::None)
    return std::make_unique<CSEConfigConstantOnly>();
  return std::make_unique<MC6809CSEConfigFull>();
}
