; MC6839 Floating-Point ROM — embedded into the program image.
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
        .section .mc6839_rom, "ax"
        .global __mc6839_rom_start
__mc6839_rom_start:
        .incbin "Float09.bin"           ; 8192 bytes verbatim

        .global __fpreg
        __fpreg  = __mc6839_rom_start + 0x3D

        .global __fpstak
        __fpstak = __mc6839_rom_start + 0x3F
