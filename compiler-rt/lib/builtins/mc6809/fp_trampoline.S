; MC6839 FP entry-point trampolines + runtime pointer indirection.
;
; Bug #289 (2026-05-24): replace the link-time-constant `__fpreg` /
; `__fpstak` definitions in mc6839_rom.S with trampolines that jump
; through runtime-settable pointer globals.  Why:
;
; - **Bare-metal**: the linker resolves `__mc6839_rom_start` to the
;   address of the embedded Float09.bin section.  The pointers
;   `__mc6839_fpreg_ptr` and `__mc6839_fpstak_ptr` are initialised at
;   `.data` load time to that address + 0x3D / 0x3F respectively.  The
;   trampoline's `jmp [ptr,pc]` reads the pointer and jumps to the
;   ROM entry — behaviour is identical to the old link-time-constant
;   approach plus ~7 cycles for the indirect jump.
;
; - **OS-9 (future)**: crt0 calls `F$Link("Float09")` which returns the
;   dynamic load address of the OS-9 Float09 module.  crt0 then calls
;   `__mc6839_set_rom_base(addr)` (defined in fenv.c) to update the
;   pointers.  Subsequent FP libcalls go through the now-updated
;   pointers to the dynamically-loaded ROM.  The bare-metal
;   `.incbin "Float09.bin"` in mc6839_rom.S becomes dead code (the
;   OS-9 linker script can drop the `.mc6839_rom` section to save
;   image space).
;
; Per-FP-call overhead: ~7 cycles for the trampoline's indirect jmp.
; Pointer storage cost: 6 bytes in .data (three 16-bit pointers).
;
; The trampoline uses `jmp [__mc6839_*_ptr,pc]` rather than
; `pul X; jmp ,X` because (a) it leaves no scratch register clobbered
; and (b) the indirect indexed addressing mode is one instruction
; vs. two.

        .include "mc6839_abi.inc"

;; -------------------------------------------------------------------
;; .data: runtime-settable pointer globals
;; -------------------------------------------------------------------
;;
;; Initial values link to the embedded Float09 ROM (`__mc6839_rom_start`
;; from mc6839_rom.S).  OS-9 crt0 overrides via __mc6839_set_rom_base.
;;
;; On OS-9 the ROM is not part of the program at all: it is the FPO9
;; module, which the start-up code links and whose address it passes to
;; __mc6839_set_rom_base, so these start empty and every wrapper reads
;; the entry out of the data area as it goes.

        .data

        .global __mc6839_rom_base
__mc6839_rom_base:
#ifdef __OS9__
        .word   0
#else
        .word   __mc6839_rom_start
#endif

        .global __mc6839_fpreg_ptr
__mc6839_fpreg_ptr:
#ifdef __OS9__
        .word   0
#else
        .word   __mc6839_rom_start + 0x3D
#endif

        .global __mc6839_fpstak_ptr
__mc6839_fpstak_ptr:
#ifdef __OS9__
        .word   0
#else
        .word   __mc6839_rom_start + 0x3F
#endif

;; -------------------------------------------------------------------
;; .text: trampolines.  Each is a single `jmp [ptr,pc]`.
;; -------------------------------------------------------------------
;;
;; These are for wrappers that reach the pointer PC-relatively, which an
;; OS-9 program cannot do for anything it writes: there the wrappers pick
;; the entry up from the data area before they fill the argument
;; registers, and call through the stack.

#ifndef __OS9__

        .text

        .global __fpreg
__fpreg:
        jmp     [__mc6839_fpreg_ptr,pc]

        .global __fpstak
__fpstak:
        jmp     [__mc6839_fpstak_ptr,pc]

#endif

;; -------------------------------------------------------------------
;; __mc6839_set_rom_base(void *base): point the three globals at a ROM
;; -------------------------------------------------------------------
;;
;; X = the base address of the ROM image.  REG is 0x3D into it and STAK
;; 0x3F, from the module's own entry table.  This is what OS-9 start-up
;; calls once it has linked FPO9; it sits here rather than in fenv.c so
;; that it comes along with the pointers, which every program doing
;; floating point pulls in -- start-up asks for it weakly and would
;; otherwise not find it.

        .text

        .global __mc6839_set_rom_base
__mc6839_set_rom_base:
#ifdef __OS9__
        stx     mc6809_os9_data(__mc6839_rom_base),u
        leax    0x3D,x
        stx     mc6809_os9_data(__mc6839_fpreg_ptr),u
        leax    2,x
        stx     mc6809_os9_data(__mc6839_fpstak_ptr),u
#else
        stx     __mc6839_rom_base
        leax    0x3D,x
        stx     __mc6839_fpreg_ptr
        leax    2,x
        stx     __mc6839_fpstak_ptr
#endif
        rts
