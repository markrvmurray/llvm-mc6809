; RUN: llvm-mc -triple mc6809 -mattr=+hd6309 -filetype=obj %s -o %t.o
; RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=REL
; RUN: llvm-mc -triple mc6809 -mattr=+hd6309 -show-encoding %s | FileCheck %s --check-prefix=ENC

; Audit every (FixupKind, Offset) pair the operand encoder declares.
; The fixup offset recorded on each relocation must match the byte
; position of the operand field in the encoded instruction. Wrong
; offset = silent miscompile (linker writes the resolved value over
; another instruction field — postbyte, immediate, etc).
;
; Pairs that are NATURALLY REACHABLE from hand-written assembly via
; an external symbol (and therefore exercised here):
;   (PCRel8,  1) — short branch / bsr to external
;   (PCRel16, 1) — long branch / lbsr to external
;   (PCRel16, 2) — pc-relative indexed (direct + indirect)
;   (Addr8,   1) — direct-page load
;   (Addr8,   2) — hd6309 immediate-direct
;   (Addr16,  1) — extended load
;   (Addr16,  2) — indexed-extended-indirect (#105 regression)
;   (Addr16,  3) — hd6309 immediate-extended
;   (Rel16,   2) — IndexedOffset16W / IndexedOffset16WInd via the
;                  W-register indexed mode (hd6309). Reaches with
;                  externals because there's no smaller W-indexed
;                  form for the assembler to fall back to.
;
; Pairs that ARE NOT reachable with external symbols, but where the
; *byte layout* can be verified with literal values (encoding test only):
;   (Rel5,    1) — IndexedOffset5  (lda 5, x)
;   (Rel8,    1) — assembler always picks Rel5 for externals
;   (Rel8,    2) — IndexedOffset8 (lda 100, x)
;   (Rel16,   1) — codegen-only path
;   (PCRel8,  2) — IndexedOffset8PC (leax 5, pc)
;
; Pairs reachable only from CODEGEN (no user-asm syntax):
;   (PCRel8,  3), (PCRel16, 3) — pcrel*_imm_idx
;   (Rel5,    2), (Rel8,    3), (Rel16,   3) — ImmediateIndexed* family
; These have defensive fixes but no user-asm regression test.
;
; Regression for bug #105 (silent miscompile, addr16 fixup-offset)
; and the audit-driven follow-ups that closed bug #97.

	.text

;----------------------------------------------------------------------
; Section 1: relocation tests (external symbol forces real relocation)
;----------------------------------------------------------------------

; (PCRel8, 1)
; ENC: bra extern_sym {{.*}}[0x20,A]
; ENC: offset: 1, value: extern_sym, kind: PCRel8
	bra	extern_sym

; ENC: bsr extern_sym {{.*}}[0x8d,A]
; ENC: offset: 1, value: extern_sym, kind: PCRel8
	bsr	extern_sym

; (PCRel16, 1)
; ENC: lbra extern_sym {{.*}}[0x16,A,A]
; ENC: offset: 1, value: extern_sym, kind: PCRel16
	lbra	extern_sym

; ENC: lbsr extern_sym {{.*}}[0x17,A,A]
; ENC: offset: 1, value: extern_sym, kind: PCRel16
	lbsr	extern_sym

; (PCRel16, 2) — pc-relative indexed: [opc][postbyte=8d/9d][off_hi][off_lo]
; ENC: lda extern_sym,pc {{.*}}[0xa6,0x8d,A,A]
; ENC: offset: 2, value: extern_sym, kind: PCRel16
	lda	extern_sym, pc

; ENC: lda [extern_sym,pc] {{.*}}[0xa6,0x9d,A,A]
; ENC: offset: 2, value: extern_sym, kind: PCRel16
	lda	[extern_sym, pc]

; (Addr8, 1)
; ENC: lda <extern_sym {{.*}}[0x96,A]
; ENC: offset: 1, value: extern_sym, kind: Addr8
	lda	<extern_sym

; (Addr16, 1) — extended: [opc][addr_hi][addr_lo]
; ENC: lda extern_sym {{.*}}[0xb6,A,A]
; ENC: offset: 1, value: extern_sym, kind: Addr16
	lda	>extern_sym

; (Addr16, 2) — indexed-extended-indirect (bug #105 regression)
; ENC: jmp [extern_sym] {{.*}}[0x6e,0x9f,A,A]
; ENC: offset: 2, value: extern_sym, kind: Addr16
	jmp	[extern_sym]

; ENC: jsr [extern_sym] {{.*}}[0xad,0x9f,A,A]
; ENC: offset: 2, value: extern_sym, kind: Addr16
	jsr	[extern_sym]

; (Addr8, 2) — hd6309 immediate-direct: [opc][val][addr]
; ENC: aim #66,<direct_sym {{.*}}[0x02,0x42,A]
; ENC: offset: 2, value: direct_sym, kind: Addr8
	aim	#0x42, <direct_sym

; (Addr16, 3) — hd6309 immediate-extended: [opc][val][addr_hi][addr_lo]
; ENC: aim #66,extern_sym {{.*}}[0x72,0x42,A,A]
; ENC: offset: 2, value: extern_sym, kind: Addr16
	aim	#0x42, extern_sym

; (Rel16, 2) — IndexedOffset16W (hd6309): [opc][postbyte=af][off_hi][off_lo]
; The W-register indexed mode has no smaller variant, so an external
; symbol forces a real Rel16 relocation here.
; ENC: leax extern_sym,w {{.*}}[0x30,0xaf,A,A]
; ENC: offset: 2, value: extern_sym, kind: Rel16
	leax	extern_sym, w

; ENC: leax [extern_sym,w] {{.*}}[0x30,0xb0,A,A]
; ENC: offset: 2, value: extern_sym, kind: Rel16
	leax	[extern_sym, w]

;----------------------------------------------------------------------
; Section 2: encoding-only tests (literal values, no relocation).
; Verifies the byte LAYOUT for paths whose relocation form isn't
; user-asm-reachable. If a fixup-offset bug ever resurfaces in one
; of these formats, codegen output for the literal would be wrong
; in the same spot.
;----------------------------------------------------------------------

; (Rel5, 1) — IndexedOffset5: 5-bit offset packed into the postbyte.
; ENC: lda 5,x {{.*}}[0xa6,0x05]
	lda	5, x

; (Rel8, 2) — IndexedOffset8: [opc][postbyte=88/98][offset]
; ENC: lda 100,x {{.*}}[0xa6,0x88,0x64]
	lda	100, x

; ENC: lda [100,x] {{.*}}[0xa6,0x98,0x64]
	lda	[100, x]

; (Rel16, 2) — IndexedOffset16W literal smoke test
; ENC: leax 1000,w {{.*}}[0x30,0xaf,0x03,0xe8]
	leax	1000, w

; ENC: leax [1000,w] {{.*}}[0x30,0xb0,0x03,0xe8]
	leax	[1000, w]

; (PCRel8, 2) — IndexedOffset8PC: [opc][postbyte=8c/9c][offset]
; ENC: leax 5,pc {{.*}}[0x30,0x8c,0x05]
	leax	5, pc

; (PCRel16, 2) — IndexedOffset16PC literal smoke test
; ENC: leax 1000,pc {{.*}}[0x30,0x8d,0x03,0xe8]
	leax	1000, pc

; ENC: leax [1000,pc] {{.*}}[0x30,0x9d,0x03,0xe8]
	leax	[1000, pc]

;----------------------------------------------------------------------
; Section 3: relocation-record cross-check.
; Each external-symbol fixup recorded in section 1 must end up with
; an OFFSET column matching the documented byte position.
;----------------------------------------------------------------------

; REL: RELOCATION RECORDS FOR [.text]:
; REL: 00000001 R_MC6809_PCREL_8 extern_sym
; REL: 00000003 R_MC6809_PCREL_8 extern_sym
; REL: 00000005 R_MC6809_PCREL_16 extern_sym
; REL: 00000008 R_MC6809_PCREL_16 extern_sym
; REL: 0000000c R_MC6809_PCREL_16 extern_sym
; REL: 00000010 R_MC6809_PCREL_16 extern_sym
; REL: 00000013 R_MC6809_ADDR_8 extern_sym
; REL: 00000015 R_MC6809_ADDR_16 extern_sym
; REL: 00000019 R_MC6809_ADDR_16 extern_sym
; REL: 0000001d R_MC6809_ADDR_16 extern_sym
; REL: 00000021 R_MC6809_ADDR_8 direct_sym
; REL: 00000024 R_MC6809_ADDR_16 extern_sym
; REL: 00000028 R_MC6809_OFFSET_16 extern_sym
; REL: 0000002c R_MC6809_OFFSET_16 extern_sym
