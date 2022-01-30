add_llvm_component_library(LLVMMC6809AsmParser
  MC6809AsmParser.cpp

  LINK_COMPONENTS
  MC
  MCParser
  MC6809Desc
  MC6809Info
  Support

  ADD_TO_COMPONENT
  MC6809
)
