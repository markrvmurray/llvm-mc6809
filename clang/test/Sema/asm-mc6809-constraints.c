// RUN: %clang_cc1 -triple mc6809-unknown-unknown -fsyntax-only -verify %s
// expected-no-diagnostics
//
// Bug #198: MC6809 inline asm register constraints.
// These constraint letters must all be accepted (no diagnostics).

// 'd' = D register (16-bit accumulator) with short (16-bit on mc6809)
void test_d_add(short x)    { asm("addd #1" : "+d"(x)); }
void test_d_out(void)       { short v; asm("ldd #42" : "=d"(v)); }

// 'A' = A register (8-bit, high byte of D)
void test_A_out(void)       { char v; asm("lda #1" : "=A"(v)); }
void test_A_in(char x)      { asm("sta 0x1000" : : "A"(x)); }

// 'B' = B register (8-bit, low byte of D)
void test_B_out(void)       { char v; asm("ldb #2" : "=B"(v)); }
void test_B_in(char x)      { asm("stb 0x1001" : : "B"(x)); }

// 'q' = A or B (8-bit byte register)
void test_q_out(void)       { char v; asm("" : "=q"(v)); }
void test_q_in(char x)      { asm("" : : "q"(x)); }

// 'r' = general register (size-dependent)
void test_r8(char x)        { asm("" : : "r"(x)); }
void test_r16(short x)      { asm("" : : "r"(x)); }

// 'a' = 16-bit address registers (X, Y, U, S)
void test_a_in(short *p)    { asm("" : : "a"(p)); }

// 'x' = X index register (16-bit)
void test_x_in(short *p)    { asm("ldd ,x" : : "x"(p)); }

// 'y' = Y index register (16-bit)
void test_y_in(short *p)    { asm("ldd ,y" : : "y"(p)); }

// 'd' and 'x' also work with int (= 16-bit on mc6809, same as short)
void test_d_int(int x)      { asm("addd #1" : "+d"(x)); }
void test_x_int(int *p)     { asm("ldd ,x" : : "x"(p)); }

// Clobbers using gcc register names
void test_clobber_d(void)   { asm("ldd #0" : : : "d"); }
void test_clobber_x(void)   { asm("ldx #0" : : : "x"); }
