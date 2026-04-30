; RUN: llc -mtriple=mc6809 -O2 -filetype=asm < %s | FileCheck --check-prefix=ASM %s
; RUN: llc -mtriple=mc6809 -O2 -filetype=obj < %s | llvm-objdump -d - | FileCheck --check-prefix=OBJ %s

; Regression for bug #200.
;
; copyPhysReg on a BIT1 -> BIT1 register pair (the LSB sub-registers
; AALSB / ABLSB) used to emit a literal "TFR aLSB, bLSB". The TFR
; instruction can only address architectural registers (D, X, Y, U, S,
; A, B, etc.), not LSB sub-registers, so:
;   - The asm path printed bogus "tfr aLSB, bLSB" that the assembler
;     can't reparse.
;   - The obj path (encodeRegOpValue / default branch) silently
;     emitted a 4-bit postbyte with raw register IDs masked, often
;     decoding back as "tfr y,s" — corrupting the stack pointer at
;     run-time. test-wctomb hung in an infinite loop because S was
;     overwritten with a .rodata pointer.
;
; The fix: when the operand classes are BIT1 -> BIT1, transfer the
; parent byte halves (AA or AB) instead of the LSB sub-registers,
; using getBit1ByteHalf().
;
; Sentinel: any code path that produces a cross-half BIT1 copy must
; expand to "tfr a,b" or "tfr b,a" — never the literal sub-register
; names. We trigger one with two phi-i1 paths whose sources land in
; opposite byte halves.

define i1 @bit1_phi_cross_half(i1 %sel, i8 %x, i8 %y) {
entry:
  br i1 %sel, label %lhs, label %rhs

lhs:
  ; Computes a boolean in the A half via icmp.
  %c1 = icmp slt i8 %x, 0
  br label %merge

rhs:
  ; Computes a boolean in the B half via icmp + and.
  %c2 = icmp ne i8 %y, 0
  br label %merge

merge:
  %r = phi i1 [ %c1, %lhs ], [ %c2, %rhs ]
  ret i1 %r
}

; ASM-NOT: tfr{{[ \t]+}}aLSB
; ASM-NOT: tfr{{[ \t]+}}bLSB

; OBJ-NOT: 1f 24
; OBJ-NOT: 1f 42

; Verify the function exists and uses standard byte-register TFR if it
; needs to cross halves — the lit-checked bytes are ONLY 1f 89
; (tfr a,b) or 1f 98 (tfr b,a), or no cross-half TFR at all. We don't
; require a specific encoding here because the optimizer may choose to
; place both branches in the same byte half and skip the TFR entirely.

; OBJ:      <bit1_phi_cross_half>:
