; Bug #227: HD6309 LSLD synonym for ASLD (page-2 opcode 0x10 0x48).
;
; RUN: llvm-mc -triple=mc6809 -mcpu=hd6309 -show-encoding %s 2>&1 | FileCheck %s

; CHECK: encoding: [0x10,0x48]
        asld
; CHECK: encoding: [0x10,0x48]
        lsld
