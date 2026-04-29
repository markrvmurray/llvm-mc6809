add_lldb_library(lldbPluginABIMC6809 PLUGIN
  ABIMC6809.cpp

  LINK_COMPONENTS
    Support
    TargetParser
  LINK_LIBS
    lldbCore
    lldbSymbol
    lldbTarget
    lldbValueObject
  )
