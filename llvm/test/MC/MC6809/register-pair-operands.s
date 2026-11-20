; Register-pair operands (TFR, EXG, TFM and the HD6309 register-register
; arithmetic): the registers a postbyte can name, the HD6309 zero register
; written `0`, and an error -- never a silently dropped line -- for anything
; else.
;
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -show-encoding %s | FileCheck %s
; RUN: not llvm-mc -triple=mc6809 -mcpu=hd6309 --defsym=BAD=1 %s -o /dev/null 2>&1 | FileCheck %s --check-prefix=ERR6309
; RUN: not llvm-mc -triple=mc6809 -mcpu=mc6809 --defsym=BAD6809=1 %s -o /dev/null 2>&1 | FileCheck %s --check-prefix=ERR6809

.ifndef BAD
.ifndef BAD6809
	tfr	a,b
; CHECK: tfr a,b ; encoding: [0x1f,0x89]
	exg	x,y
; CHECK: exg x,y ; encoding: [0x1e,0x12]
	tfr	cc,dp
; CHECK: tfr cc,dp ; encoding: [0x1f,0xab]
	tfr	w,d
; CHECK: tfr w,d ; encoding: [0x1f,0x60]
	tfr	0,b
; CHECK: tfr 0,b ; encoding: [0x1f,0xc9]
	adcr	0,e
; CHECK: adcr 0,e ; encoding: [0x10,0x31,0xce]
	adcr	0,f
; CHECK: adcr 0,f ; encoding: [0x10,0x31,0xcf]
	adcr	0,w
; CHECK: adcr 0,w ; encoding: [0x10,0x31,0xc6]
	addr	e,a
; CHECK: addr e,a ; encoding: [0x10,0x30,0xe8]
.endif
.endif

.ifdef BAD
	tfr	C,b
; ERR6309: [[@LINE-1]]:2: error: operand must be one of d x y u s pc w v a b cc dp e f 0
	exg	b,c
; ERR6309: [[@LINE-1]]:2: error: operand must be one of
	tfr	q,b
; ERR6309: [[@LINE-1]]:2: error: operand must be one of
	tfr	b,zz
; ERR6309: [[@LINE-1]]:2: error: operand must be one of
	tfr	1,b
; ERR6309: [[@LINE-1]]:2: error: operand must be one of
.endif

.ifdef BAD6809
	tfr	w,d
; ERR6809: [[@LINE-1]]:2: error: operand must be one of d x y u s pc a b cc dp
	tfr	0,b
; ERR6809: [[@LINE-1]]:2: error: operand must be one of d x y u s pc a b cc dp
.endif
