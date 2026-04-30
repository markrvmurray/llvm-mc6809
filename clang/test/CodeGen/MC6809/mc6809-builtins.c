// RUN: %clang_cc1 -triple mc6809-unknown-unknown -emit-llvm -o - %s | FileCheck %s
//
// Bug #199: MC6809 target built-in functions — IR emission checks.
// Note: 8-bit unsigned values ≥ 128 appear as negative i8 in LLVM IR
// (e.g. 0xBF = 191 unsigned = -65 as i8, 0xEF = 239 = -17 as i8).

// --- sei / cli ---

void test_sei(void) { __builtin_mc6809_sei(); }
// CHECK-LABEL: @test_sei
// CHECK: call void asm sideeffect "orcc #16", ""

void test_cli(void) { __builtin_mc6809_cli(); }
// CHECK-LABEL: @test_cli
// CHECK: call void asm sideeffect "andcc #239", ""

// --- orcc / andcc (immediate constraint) ---

void test_orcc(void)  { __builtin_mc6809_orcc(0x50); }
// CHECK-LABEL: @test_orcc
// CHECK: call void asm sideeffect "orcc $0", "i"(i8 80)

void test_andcc(void) { __builtin_mc6809_andcc(0xBF); }
// CHECK-LABEL: @test_andcc
// CHECK: call void asm sideeffect "andcc $0", "i"(i8 -65)

// --- nop / sync ---

void test_nop(void)  { __builtin_mc6809_nop(); }
// CHECK-LABEL: @test_nop
// CHECK: call void asm sideeffect "nop", ""

void test_sync(void) { __builtin_mc6809_sync(); }
// CHECK-LABEL: @test_sync
// CHECK: call void asm sideeffect "sync", ""

// --- cwai (constant arg, verified by Sema) ---

void test_cwai(void) { __builtin_mc6809_cwai(0xEF); }
// CHECK-LABEL: @test_cwai
// CHECK: call void asm sideeffect "cwai $0", "i"(i8 -17)

// --- software interrupts ---

void test_swi(void)  { __builtin_mc6809_swi(); }
// CHECK-LABEL: @test_swi
// CHECK: call void asm sideeffect "swi", ""

void test_swi2(void) { __builtin_mc6809_swi2(); }
// CHECK-LABEL: @test_swi2
// CHECK: call void asm sideeffect "swi2", ""

void test_swi3(void) { __builtin_mc6809_swi3(); }
// CHECK-LABEL: @test_swi3
// CHECK: call void asm sideeffect "swi3", ""

// --- peek / poke ---

unsigned char test_peek(void) { return __builtin_mc6809_peek(0xD000); }
// CHECK-LABEL: @test_peek
// CHECK: load volatile i8, ptr inttoptr

void test_poke(void) { __builtin_mc6809_poke(0xD000, 42); }
// CHECK-LABEL: @test_poke
// CHECK: store volatile i8 42, ptr inttoptr

unsigned short test_peekw(void) { return __builtin_mc6809_peekw(0xD000); }
// CHECK-LABEL: @test_peekw
// CHECK: load volatile i16, ptr inttoptr

void test_pokew(void) { __builtin_mc6809_pokew(0xD000, 0x1234); }
// CHECK-LABEL: @test_pokew
// CHECK: store volatile i16 {{[0-9]+}}, ptr inttoptr

// --- set_dp / get_dp ---

void test_set_dp(void) { __builtin_mc6809_set_dp(0x20); }
// CHECK-LABEL: @test_set_dp
// CHECK: call void asm sideeffect "tfr a,dp", "{a}"(i8 32)

unsigned char test_get_dp(void) { return __builtin_mc6809_get_dp(); }
// CHECK-LABEL: @test_get_dp
// CHECK: call i8 asm "tfr dp,a", "={a}"
