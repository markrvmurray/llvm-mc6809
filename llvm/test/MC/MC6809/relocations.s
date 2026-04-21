; RUN: llvm-mc -triple mc6809 -mattr=+hd6309 -filetype=obj %s -o %t.o
; RUN: llvm-objdump -r %t.o | FileCheck %s --check-prefix=REL
; RUN: llvm-mc -triple mc6809 -mattr=+hd6309 -show-encoding %s | FileCheck %s --check-prefix=ENC

; Exercise addressing modes that carry a 16-bit or 8-bit address operand
; resolved via a relocation. The fixup offset recorded on the relocation
; must match the byte position of the address field in the encoded
; instruction. Getting the offset wrong causes the linker to write the
; resolved address over the wrong bytes (a postbyte, or an 8-bit
; immediate alongside the address) — a silent miscompile.
;
; Regression for bug #105 ("assembler silently miscompiles jmp [symbol]")
; and the addr8/addr16 fixup-offset audit.

	.text

; Extended: [opc][addr_hi][addr_lo]. Address at offset 1.
; ENC: lda {{.*}}extern_sym
; ENC: fixup A - offset: 1, value: extern_sym, kind: Addr16
	lda	>extern_sym

; Indexed-extended-indirect: [opc][postbyte=9f][addr_hi][addr_lo].
; Address at offset 2. Regression for bug #105.
; ENC: jmp [extern_sym]
; ENC: fixup A - offset: 2, value: extern_sym, kind: Addr16
	jmp	[extern_sym]

; ENC: jsr [extern_sym]
; ENC: fixup A - offset: 2, value: extern_sym, kind: Addr16
	jsr	[extern_sym]

; hd6309 Immediate-direct: [opc][val][addr]. Address at offset 2.
; ENC: aim {{.*}}direct_sym
; ENC: fixup A - offset: 2, value: direct_sym, kind: Addr8
	aim	#0x42, <direct_sym

; hd6309 Immediate-extended: [opc][val][addr_hi][addr_lo]. Address at offset 2.
; ENC: aim {{.*}}extern_sym
; ENC: fixup A - offset: 2, value: extern_sym, kind: Addr16
	aim	#0x42, extern_sym

; REL: RELOCATION RECORDS FOR [.text]:
; REL: 00000001 R_MC6809_ADDR_16 extern_sym
; REL: 00000005 R_MC6809_ADDR_16 extern_sym
; REL: 00000009 R_MC6809_ADDR_16 extern_sym
; REL: 0000000d R_MC6809_ADDR_8 direct_sym
; REL: 00000010 R_MC6809_ADDR_16 extern_sym
