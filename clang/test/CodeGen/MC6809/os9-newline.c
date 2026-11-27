// REQUIRES: mc6809-registered-target
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -fos9-newline -DOS9 -emit-llvm -o - %s \
// RUN:   | FileCheck %s --check-prefixes=CHECK,OS9
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -emit-llvm -o - %s \
// RUN:   | FileCheck %s --check-prefixes=CHECK,LF

// With -fos9-newline, '\n' is a carriage return in every literal that
// becomes program data, '\r' stays one, and '\l' names a line feed.  An asm
// string is assembler source, not program data, so it keeps its line feeds.

// CHECK:      module asm
// CHECK-NEXT:   "; first"
// CHECK-NEXT:   "; second"
__asm__("; first\n; second");

// OS9: @nl = {{.*}}global i8 13
// LF:  @nl = {{.*}}global i8 10
char nl = '\n';

// CHECK: @cr = {{.*}}global i8 13
char cr = '\r';

#ifdef OS9
// OS9: @lf = {{.*}}global i8 10
char lf = '\l';
// OS9: @lfstr = {{.*}}global [3 x i8] c"\0Aa\00"
char lfstr[] = "\la";
#endif

// OS9: @str = {{.*}}global [6 x i8] c"a\0Db\0Dc\00"
// LF:  @str = {{.*}}global [6 x i8] c"a\0Ab\0Dc\00"
char str[] = "a\nb\rc";

// Every width follows the same rule.
// OS9: @u16 = {{.*}}global [2 x i16] [i16 13, i16 0]
// LF:  @u16 = {{.*}}global [2 x i16] [i16 10, i16 0]
__CHAR16_TYPE__ u16[] = u"\n";

// A numeric escape is the value it names.
// CHECK: @hex = {{.*}}global [3 x i8] c"\0A\0A\00"
char hex[] = "\x0a\012";

// The preprocessor evaluates '\n' as the program would.
#if '\n' == 13
int pp_newline = 13;
#else
int pp_newline = 10;
#endif
// OS9: @pp_newline = {{.*}}global i16 13
// LF:  @pp_newline = {{.*}}global i16 10


// CHECK-LABEL: define {{.*}}void @f(
// CHECK: call void asm sideeffect "nop\0A\09nop", ""()
void f(void) { __asm__ volatile("nop\n\tnop"); }
