; MC6839 Floating-Point ROM — embedded into the program image.
;
; Float09.bin is Motorola's ROM image, 8192 bytes, unaltered: written for
; Motorola by Joel Boney in 1980, and documented by Rich Kottke in 1989 as
; having been released into the public domain by Motorola in 1988.  The
; provenance, the chain of custody and where the same bytes appear in an
; installed toolchain are in llvm/docs/MC6809-Third-Party.md.
;
; There is no physical ROM chip. The 8KB MC6839 binary is linked as a
; normal section alongside .text; the linker places it wherever it fits.
; The MC6839 code is fully position-independent (OS-9 module, all internal
; addressing is PC-relative), so no fixed address is required.
;
; Entry-point offsets verified from Float09.bin / romsrc (frnbak.s):
;   FPREG  (register ABI): ROM_BASE + $3D
;   FPSTAK (stack ABI):    ROM_BASE + $3F
;
; Callers use:   lbsr __fpreg ; fcb FPOP_xxx
;
; Bug #289 (2026-05-24): `__fpreg` and `__fpstak` were previously
; link-time-constant labels here (`__fpreg = __mc6839_rom_start + 0x3D`
; / `+ 0x3F`).  They are now trampolines defined in fp_trampoline.S
; that jump through runtime-settable pointer globals.  This decouples
; the FP entry points from `__mc6839_rom_start` so OS-9 crt0 can
; relocate them to a dynamically-loaded ROM via `__mc6839_set_rom_base`.
;
; Bare-metal users see no behavioural change: the pointer globals
; initialise (at .data load time) to `__mc6839_rom_start + 0x3D / 0x3F`,
; exactly the old constants.  Trampoline overhead is ~7 cycles per
; FP libcall.
;
        .section .mc6839_rom, "ax"
        .global __mc6839_rom_start
__mc6839_rom_start:
        .incbin "Float09.bin"           ; 8192 bytes verbatim
