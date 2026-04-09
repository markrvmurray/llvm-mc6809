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
#include "llvm/Transforms/InstCombine/InstCombine.h"
#include "llvm/Transforms/Scalar/IndVarSimplify.h"
#include "llvm/Transforms/Utils.h"

#include "GISel/MC6809Combiner.h"
#include "MC6809.h"
#include "MC6809CopyOpt.h"
#include "MC6809DirectPageAlloc.h"
#include "MC6809IncDecPhi.h"
#include "MC6809IndexIV.h"
#include "MC6809InsertCopies.h"
#include "MC6809Internalize.h"
#include "MC6809BundleCC.h"
#include "MC6809LateOptimization.h"
#include "MC6809PostRASpillOpt.h"
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
  initializeMC6809IncDecPhiPass(PR);
  initializeMC6809InsertCopiesPass(PR);
  initializeMC6809InternalizePass(PR);
  initializeMC6809LateOptimizationPass(PR);
  initializeMC6809LowerSelectPass(PR);
  initializeMC6809NonReentrantPass(PR);
  initializeMC6809PostRAScavengingPass(PR);
  initializeMC6809PostRASpillOptPass(PR);
  initializeMC6809MaterializeSpillsPass(PR);
  initializeMC6809ShiftRotateChainPass(PR);
  initializeMC6809DirectPageAllocPass(PR);
}

// MC6809 is big-endian (Motorola byte order).
static const char *MC6809DataLayout = "E-p:16:8-S8-m:e-i1:8-i8:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:0-n8:16";

/// Processes a CPU name.
static StringRef getCPU(StringRef CPU) { return (CPU.empty() || CPU == "generic") ? "mc6809" : CPU; }

static Reloc::Model getEffectiveRelocModel(std::optional<Reloc::Model> RM) { return RM ? *RM : Reloc::Static; }

MC6809TargetMachine::MC6809TargetMachine(const Target &T, const Triple &TT, StringRef CPU, StringRef FS, const TargetOptions &Options, std::optional<Reloc::Model> RM, std::optional<CodeModel::Model> CM, CodeGenOptLevel OL, bool JIT)
    : CodeGenTargetMachineImpl(T, MC6809DataLayout, TT, getCPU(CPU), FS, Options, getEffectiveRelocModel(RM), getEffectiveCodeModel(CM, CodeModel::Small), OL),
      SubTarget(TT, getCPU(CPU).str(), FS.str(), *this) {
  this->TLOF = std::make_unique<MC6809TargetObjectFile>();

  initAsmInfo();

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
  TargetPassConfig::addIRPasses();
  // Clean up after LSR in particular.
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createInstructionCombiningPass());
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
  return false;
}

void MC6809PassConfig::addMachineSSAOptimization() {
  TargetPassConfig::addMachineSSAOptimization();
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
  if (getOptLevel() != CodeGenOptLevel::None)
    addPass(createMC6809PostRASpillOptPass());

  // This is currently mandatory, since it lowers CMPTermZ.
  addPass(createMC6809LateOptimizationPass());
}

void MC6809PassConfig::addPreEmitPass() { addPass(&BranchRelaxationPassID); }

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
