set(LLVM_LINK_COMPONENTS
  Support
  )

add_llvm_tool(basic09c
  Basic09AST.cpp
  Basic09Lexer.cpp
  Basic09Parser.cpp
  Basic09Semantic.cpp
  Basic09Symbols.cpp
  Basic09Token.cpp
  basic09c.cpp
  )
