# Bug #163 Phase 3: native lld OS-9 program-module output.
#
# Asserts that ld.lld with --oformat=os9-program-module emits a
# complete OS-9 / NitrOS-9 program module directly — no Python
# post-link wrapper needed.  The output should be byte-identical to
# what `ld.lld --oformat=binary` followed by `tools/os9-link` would
# have produced, validated end-to-end via the Phase 1 checker.
#
# REQUIRES: mc6809

# Minimal program: clrb; swi2; fcb F$Exit  (5 bytes of body).
# RUN: llvm-mc -triple=mc6809 -filetype=obj %s -o %t.o

## 1. --oformat=os9-program-module (CLI flag).
# RUN: ld.lld -o %t.os9 %t.o --oformat=os9-program-module \
# RUN:   --os9-name=hello --os9-mem=256

## Module size: 13 (header) + 6 (body) + 5 (fcs "hello") + 3 (CRC) = 27.
# RUN: wc -c < %t.os9 | FileCheck %s --check-prefix=SIZE
# SIZE: 27

## Byte-exact:
##   header[0..12] = 87 cd 00 1b 00 13 11 a0 0c 00 0d 01 00
##   body[13..18]  = 5f 10 3f 06 20 fe (clrb; swi2; fcb $06; bra .)
##   name[19..23]  = 68 65 6c 6c ef    (fcs "hello")
##   crc[24..26]   = 32 7c 43 (deterministic for this exact input)
# RUN: xxd -p -c 256 %t.os9 | FileCheck %s --check-prefix=BYTES
# BYTES: {{^87cd001b001311a00c000d01005f103f0620fe68656c6cef327c43$}}

## 2. Phase 1 validator must accept the output ($800FE3 CRC magic).
# RUN: %S/../../../tools/os9-module-check %t.os9 \
# RUN:   | FileCheck %s --check-prefix=VALID
# VALID: OK ({{[0-9]+}} bytes)
# VALID: type/lang: $11 (Prgrm|Objct)
# VALID: exec offset: $000D (13)
# VALID: name: "hello"
# VALID: CRC: valid (full CRC = $800FE3)

## 3. OUTPUT_FORMAT(os9-program-module) in a linker script does the same.
# RUN: echo 'OUTPUT_FORMAT(os9-program-module)' > %t.lds
# RUN: ld.lld -o %t-via-script.os9 %t.o -T %t.lds \
# RUN:   --os9-name=hello --os9-mem=256
# RUN: diff %t.os9 %t-via-script.os9

## 4. Missing --os9-name is an error.
# RUN: not ld.lld -o /dev/null %t.o --oformat=os9-program-module \
# RUN:   --os9-mem=256 2>&1 | FileCheck %s --check-prefix=NO_NAME
# NO_NAME: OS-9 program-module output requires --os9-name

## 5. Missing --os9-mem is an error.
# RUN: not ld.lld -o /dev/null %t.o --oformat=os9-program-module \
# RUN:   --os9-name=foo 2>&1 | FileCheck %s --check-prefix=NO_MEM
# NO_MEM: OS-9 program-module output requires --os9-mem

## 6. HD6309 native type byte ($17 = Prgrm|Obj6309).
# RUN: ld.lld -o %t-hd.os9 %t.o --oformat=os9-program-module \
# RUN:   --os9-name=hello --os9-type=0x17 --os9-mem=256
# RUN: %S/../../../tools/os9-module-check %t-hd.os9 \
# RUN:   | FileCheck %s --check-prefix=HD6309
# HD6309: type/lang: $17 (Prgrm|Obj6309)

        .text
        .globl _start
_start:
        clrb
        swi2
        .byte   0x06
1:      bra 1b
