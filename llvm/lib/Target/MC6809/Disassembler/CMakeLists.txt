add_llvm_component_library(LLVMMC6809Disassembler
  MC6809Disassembler.cpp

  LINK_COMPONENTS
  MCDisassembler
  MC6809Info
  Support

  ADD_TO_COMPONENT
  MC6809
)


