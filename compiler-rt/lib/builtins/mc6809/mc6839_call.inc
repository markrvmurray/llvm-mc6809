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
