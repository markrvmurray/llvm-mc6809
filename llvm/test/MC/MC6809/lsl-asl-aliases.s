; Bug #227: LSL is a synonym for ASL on MC6809/HD6309 (same encoding).
; Both spellings appear in hand-written sources and external assemblers
; (lwasm, asm6809) accept each. MnemonicAlias rewrites the input mnemonic
; before instruction matching.
;
; RUN: llvm-mc -triple=mc6809 -show-encoding %s 2>&1 | FileCheck %s
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -show-encoding %s 2>&1 | FileCheck %s

; --- Accumulator forms ---

; CHECK: encoding: [0x48]
        asla
; CHECK: encoding: [0x48]
        lsla
; CHECK: encoding: [0x58]
        aslb
; CHECK: encoding: [0x58]
        lslb

; --- Memory forms (direct, indexed, extended) ---

; CHECK: encoding: [0x08,0x00]
        asl     <0
; CHECK: encoding: [0x08,0x00]
        lsl     <0
; CHECK: encoding: [0x68,0x84]
        asl     ,x
; CHECK: encoding: [0x68,0x84]
        lsl     ,x
