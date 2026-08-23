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

// A pointer to a direct-page object converts to a generic pointer
// implicitly (the direct page is a window inside generic memory); the
// other direction narrows and needs a cast.
static __directpage int dp_obj;
int *g_generic = &dp_obj;
void take_generic(int *p);
void conv(int *p) {
  take_generic(&dp_obj);
  int *q = &dp_obj;
  __attribute__((address_space(1))) int *r = p; // expected-error {{initializing '__attribute__((address_space(1))) int *' with an expression of type 'int *' changes address space of pointer}}
  __attribute__((address_space(1))) int *s = (__attribute__((address_space(1))) int *)p;
  (void)q; (void)r; (void)s;
}
