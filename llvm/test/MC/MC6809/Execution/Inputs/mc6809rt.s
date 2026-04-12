;; mc6809rt.s — MC6809 compiler runtime library
;;
;; Contains all __*qi3, __*hi3, __*si3 builtins that the LLVM MC6809
;; backend emits calls to for operations the hardware can't do natively.
;;
;; Build:
;;   llvm-mc -triple=mc6809 -I <dir-containing-inc-files> \
;;     --filetype=obj -o mc6809rt.o mc6809rt.s
;;
;; Link with compiled code:
;;   ld.lld -T link.ld program.o mc6809rt.o -o program.elf

	.text

;; 8-bit shifts
.include "shiftqi3.inc"

;; 16-bit shifts
.include "shifthi3.inc"

;; 32-bit shifts
.include "shiftsi3.inc"

;; 16-bit multiply
.include "mulhi3.inc"

;; 32-bit multiply
.include "mulsi3.inc"

;; 16-bit division and modulo (signed + unsigned)
.include "divhi.inc"

;; 32-bit division and modulo (signed + unsigned)
.include "divsi.inc"

;; 64-bit shifts
.include "shiftdi3.inc"

;; 64-bit division and modulo (signed + unsigned)
.include "divdi.inc"

;; memcpy / memmove / memset — required by clang for struct copies
.include "memops.inc"
