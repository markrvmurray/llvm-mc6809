// RUN: %clang_cc1 -triple mc6809-unknown-unknown -fsyntax-only -verify %s
//
// Bug #199: MC6809 target builtins — Sema constraint check for cwai.
// cwai requires a compile-time constant argument (hardware only accepts immediate).

void test_cwai_nonconstant(unsigned char m) {
  __builtin_mc6809_cwai(m); // expected-error{{argument to '__builtin_mc6809_cwai' must be a constant integer}}
}
