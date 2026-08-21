; MC6839 FP ROM: reaching the ROM and its control blocks.
;
; A wrapper hands the ROM D = &fpcb, U = &arg1, Y = &arg2, X = &result:
; every register is spoken for at the call, which is what makes OS-9
; different.  There the control blocks live in the process data area,
; addressed from U, and the ROM is a module the program links at run
; time, so its entry point is a word in that data area too.  Both have to
; be picked up while U still holds the data base -- before the wrapper
; repoints U at arg1 -- so FP_ENTRY reads them there and leaves the entry
; address on the stack, where the call reaches it without a register:
; `jsr [,s++]` takes the address and drops the slot in one go, so the ROM
; sees exactly the stack a `lbsr` would have left and still finds the
; FPOP byte at its return address.  Anything a wrapper addresses from S
; between FP_ENTRY and FP_CALL moves by FP_SLOT.
;
; Bare metal keeps the ROM in the program image and reaches both it and
; the control blocks PC-relatively, so FP_SLOT is zero and the sequences
; are the ones that were always there.
;
; The ROM has a second entry, FPSTAK at +$3F, which takes its arguments on
; the stack instead of in registers, and it is tempting on OS-9: nothing
; would need U, so the staging above would collapse to `jsr [n,u]` with the
; data base left where it is.  It is not worth it, for three reasons that
; all point the same way.
;
; The arguments are in the wrong place and the wrong order.  FPSTAK wants
; the operand *values* laid out contiguously -- arg2 below arg1, with the
; pointer to the fpcb pushed last, below both -- and it returns the result
; on the stack in their place.  What we have is LLVM's soft-float layout:
; arg1 below arg2 on the caller's stack, our return address between them
; and where the fpcb pointer would go, and a result buffer the caller
; allocated and passed in X.  Nothing can be used where it lies, so every
; call would copy eight bytes (f32) or sixteen (f64) into the ROM's order
; and copy the result back out again -- well over a hundred cycles to save
; the fifteen the staging costs.  The register call copies nothing: the
; values stay put and two `lea`s hand over their addresses.
;
; The frame is bigger where the room is smallest.  The module header
; declares each entry's stack appetite: 155 bytes for REG, 185 for STAK.
; On OS-9 the program, its data area, its stack and the 8K module share one
; 64K space, and floating-point programs are the ones that run out of it.
;
; And there is nothing else to gain.  Both entries push CC,D,X,Y,U and join
; at the ROM's mutual entry, so traps, rounding and exceptions behave
; identically; U would still have to be read to reach the fpcb and the
; entry pointer, since both live in the data area.  FPSTAK only changes how
; the fpcb pointer is handed over, from a register to a pushed word.
;
; This file is pulled in with #include, not .include: the assembler does
; not preprocess what it includes itself.

#ifdef __OS9__

FP_SLOT = 2

        ; U = the process data base, which the wrapper saved at \off,s.
        .macro  FP_BASE off
        ldu     \off,s
        .endm

        ; D = &fpcb, and the ROM entry point pushed.  U must hold the
        ; data base on entry; it is dead afterwards.
        .macro  FP_ENTRY fpcb
        leay    mc6809_os9_data(\fpcb),u
        ldu     mc6809_os9_data(__mc6839_fpreg_ptr),u
        pshs    u
        tfr     y,d
        .endm

        .macro  FP_CALL op
        jsr     [,s++]
        .byte   \op
        .endm

#else

FP_SLOT = 0

        .macro  FP_BASE off
        .endm

        .macro  FP_ENTRY fpcb
        leay    \fpcb,pc
        tfr     y,d
        .endm

        .macro  FP_CALL op
        lbsr    __fpreg
        .byte   \op
        .endm

#endif
