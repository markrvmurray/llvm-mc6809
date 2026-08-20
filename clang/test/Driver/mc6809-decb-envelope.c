// A DECB program builds in one shot and comes out wrapped in the LOADM/EXEC
// envelope: a 5-byte preamble and a 5-byte postamble around the body.
//
// REQUIRES: mc6809-registered-target
// RUN: %clang --target=mc6809-unknown-decb -Os %s -o %t.bin
// RUN: od -An -v -tx1 %t.bin | FileCheck %s

// The preamble is type $00, the body length, then the load address, which
// the script puts at $3f00.
// CHECK: 00 {{[0-9a-f]+}} {{[0-9a-f]+}} 3f 00

// The postamble is $FF, $0000, then the exec address -- the entry symbol,
// which is _start in .text, *after* the direct page the script reserves at
// the load address.  If this ever reads 3f 00 again, the exec address has
// gone back to being the load address and EXEC would jump into data.
// CHECK: ff 00 00 40 00

int counter;
static const char msg[] = "hi";
int main(void) {
  counter = msg[0];
  return counter;
}
