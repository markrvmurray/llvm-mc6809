// RUN: %clang_cc1 %s -triple mc6809 -fsyntax-only -verify

// Bug #178: __attribute__((directpage)) — accept on static-storage
// variables only; reject on extern, automatic, and function decls.

// File-scope static — OK.
static int g_static __attribute__((directpage));
static __directpage int g_macro_static;
static int g_macro_static2 __dp;

// File-scope extern (default linkage) — REJECT.
int g_extern __attribute__((directpage)); // expected-error {{'directpage' attribute requires static storage duration}}
__directpage int g_macro_extern; // expected-error {{'directpage' attribute requires static storage duration}}

// Function decl — REJECT (Subjects=[Var]).
__attribute__((directpage)) void f1(void); // expected-warning {{'directpage' attribute only applies to variables}}

void f(void) {
  // Function-local static — OK.
  static int local_static __attribute__((directpage));
  static __directpage int local_static2;

  // Function-local automatic — REJECT.
  int local_auto __attribute__((directpage)); // expected-error {{'directpage' attribute requires static storage duration}}
  __directpage int local_auto2; // expected-error {{'directpage' attribute requires static storage duration}}
}
