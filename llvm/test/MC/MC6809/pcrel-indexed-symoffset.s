; RUN: llvm-mc -triple mc6809 -filetype=obj %s -o %t.o
; RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=REL
; RUN: llvm-mc -triple mc6809 -show-encoding %s | FileCheck %s --check-prefix=ENC

; An indexed PC-relative operand with a symbolic value must always encode as the
; 16-bit form (postbyte 0x8d, PCRel16), never the 8-bit form (0x8c, PCRel8): the
; assembler has no o8PC->o16PC relaxation, and the target can be arbitrarily far
; (a jump table after the function body, a static-stack frame in a distant .bss).
; This must hold for a plain symbol AND a symbol+offset expression. A sym+offset
; is an MCBinaryExpr, not a plain MCSymbolRefExpr; it used to slip past the
; symbol guard to the 8-bit form and overflow against a far target (a real
; `leay frame+4,pc` static-stack frame address-of that failed to link).

	.text

; Plain symbol — already forced to 16-bit.
; ENC: leax extern_sym,pc {{.*}}[0x30,0x8d,A,A]
; REL: {{0*}}2 R_MC6809_PCREL_16 extern_sym{{$}}
	leax	extern_sym, pc

; Symbol + positive offset (MCBinaryExpr) — the regressed case.
; ENC: leay extern_sym+4,pc {{.*}}[0x31,0x8d,A,A]
; REL: {{0*}}6 R_MC6809_PCREL_16 extern_sym+0x4
	leay	extern_sym+4, pc

; Symbol - offset (MCBinaryExpr) — same path, negative addend.
; ENC: lda extern_sym-3,pc {{.*}}[0xa6,0x8d,A,A]
; REL: {{0*}}a R_MC6809_PCREL_16 extern_sym-0x3
	lda	extern_sym-3, pc

; A bare numeric offset that fits still uses the compact 8-bit form.
; ENC: leax 5,pc {{.*}}[0x30,0x8c,0x05]
	leax	5, pc
