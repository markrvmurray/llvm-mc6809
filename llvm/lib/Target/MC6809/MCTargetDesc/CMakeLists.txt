add_llvm_component_library(LLVMMC6809Desc
  MC6809AsmBackend.cpp
  MC6809ELFObjectWriter.cpp
  MC6809FixupKinds.cpp
  MC6809InstPrinter.cpp
  MC6809MCAsmInfo.cpp
  MC6809MCCodeEmitter.cpp
  MC6809MCELFStreamer.cpp
  MC6809MCExpr.cpp
  MC6809MCTargetDesc.cpp
  MC6809TargetStreamer.cpp

  LINK_COMPONENTS
  MC
  MC6809Info
  Support

  ADD_TO_COMPONENT
  MC6809
)

add_dependencies(LLVMMC6809Desc MC6809CommonTableGen)
