add_llvm_component_group(MC6809)

set(LLVM_TARGET_DEFINITIONS MC6809.td)

tablegen(LLVM MC6809GenAsmMatcher.inc -gen-asm-matcher)
tablegen(LLVM MC6809GenAsmWriter.inc -gen-asm-writer)
tablegen(LLVM MC6809GenCallingConv.inc -gen-callingconv)
tablegen(LLVM MC6809GenDAGISel.inc -gen-dag-isel)
tablegen(LLVM MC6809GenDisassemblerTables.inc -gen-disassembler)
tablegen(LLVM MC6809GenGICombiner.inc -gen-global-isel-combiner -combiners="MC6809CombinerHelper")
tablegen(LLVM MC6809GenGlobalISel.inc -gen-global-isel)
tablegen(LLVM MC6809GenInstrInfo.inc -gen-instr-info)
tablegen(LLVM MC6809GenMCCodeEmitter.inc -gen-emitter)
tablegen(LLVM MC6809GenMCPseudoLowering.inc -gen-pseudo-lowering)
tablegen(LLVM MC6809GenRegisterBank.inc -gen-register-bank)
tablegen(LLVM MC6809GenRegisterInfo.inc -gen-register-info)
tablegen(LLVM MC6809GenSearchableTables.inc -gen-searchable-tables)
tablegen(LLVM MC6809GenSubtargetInfo.inc -gen-subtarget)

add_public_tablegen_target(MC6809CommonTableGen)

add_llvm_target(MC6809CodeGen
  GISel/MC6809CallLowering.cpp
  GISel/MC6809InstructionSelector.cpp
  GISel/MC6809LegalizerInfo.cpp
  GISel/MC6809RegisterBankInfo.cpp
  GISel/MC6809Combiner.cpp
  MC6809AsmPrinter.cpp
  MC6809CallingConv.cpp
  MC6809FrameLowering.cpp
  MC6809ISelLowering.cpp
  MC6809IndexIV.cpp
  MC6809InstrInfo.cpp
  MC6809LowerSelect.cpp
  MC6809MCInstLower.cpp
  MC6809MachineFunctionInfo.cpp
  MC6809MachineScheduler.cpp
  MC6809NoRecurse.cpp
  MC6809PostRAScavenging.cpp
  MC6809RegisterInfo.cpp
  MC6809StaticStackAlloc.cpp
  MC6809Subtarget.cpp
  MC6809TargetMachine.cpp
  MC6809TargetObjectFile.cpp

  DEPENDS
  intrinsics_gen

  LINK_COMPONENTS
  AsmPrinter
  CodeGen
  Core
  GlobalISel
  MC
  IPO
  LTO
  Passes
  SelectionDAG
  Support
  Target

  ADD_TO_COMPONENT
  MC6809
)

add_subdirectory(AsmParser)
add_subdirectory(Disassembler)
add_subdirectory(MCTargetDesc)
add_subdirectory(TargetInfo)
