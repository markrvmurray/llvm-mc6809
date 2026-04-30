// RUN: %clang_cc1 -triple mc6809-unknown-unknown -fsyntax-only -verify %s
//
// Bug #198: MC6809 inline asm — type/size mismatch must still be rejected.
// 'A' and 'B' are 8-bit only; using with a 16-bit type must error.

void test_A_too_big(short x) { asm("" : : "A"(x)); } // expected-error{{invalid input size for constraint 'A'}}
void test_B_too_big(short x) { asm("" : : "B"(x)); } // expected-error{{invalid input size for constraint 'B'}}
void test_q_too_big(short x) { asm("" : : "q"(x)); } // expected-error{{invalid input size for constraint 'q'}}
