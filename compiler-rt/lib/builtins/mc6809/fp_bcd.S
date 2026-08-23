; MC6839 FP — BCD-string conversion wrappers (Bug #230)
;
; Two ROM operations exposed for picolibc's float-string engine
; replacements (libc/stdio/{atof,atod,ftoa,dtoa}_engine_mc6809.c):
;
;   BINDEC ($1C): binary float -> 26-byte BCD string
;   DECBIN ($22): 26-byte BCD string -> binary float
;
; ROM register-call ABI (per /Users/markmurray/Documents/MC6839_ROM/abi/nfp09-abi.s
; and the manual section "Decimal String to Binary Floating Point" /
; "Binary Floating to Decimal String"):
;
;   BINDEC entry:
;     U = k (digit count, 1..17 for double, 1..9 for single)
;     Y -> binary float (the value to convert)
;     X -> 26-byte BCD result buffer (caller-allocated, written in place)
;     D = address of FPCB
;
;   DECBIN entry:
;     U -> 26-byte BCD string (input)
;     X -> binary float result buffer (caller-allocated, written in place)
;     D = address of FPCB
;
; The C entry signatures we expose here are libgcc-style. Argument
; layout follows the gcc6809 calling convention (CC_MC6809 in
; MC6809CallingConv.td):
;   - First pointer arg in IX
;   - sret pointer (for >2-byte returns) takes IX, displacing other args
;     to the stack
;   - Other args on the stack
;
; The standard pshs/lbsr/FCB/puls idiom from fp_arith.S is preserved.
; See fp_arith.S for the dispatch-mechanism narrative.

        .text

; Bug #287: opcodes from shared header (single source of truth).
.include "mc6839_fpops.inc"
#include "mc6839_call.inc"

; ---- BINDEC --------------------------------------------------------------
;
; void __mc6839_bindec_f64(uint8_t *bcd, int k, double val);
; void __mc6839_bindec_f32(uint8_t *bcd, int k, float  val);
;
; X holds bcd (caller's first pointer arg).  At LBSR:
;   S+0..1 = return PC
;   S+2..3 = k (int16)
;   S+4..11 = val (double)   or  S+4..7 = val (float)
;
; After pshs u,y (4 bytes pushed), offsets shift by 4:
;   S+4..5 = return PC
;   S+6..7 = k
;   S+8..  = val

        .global __mc6839_bindec_f64
__mc6839_bindec_f64:
        pshs    u,y                 ; save callee-saved U, Y
        FP_ENTRY __mc6839_fpcb_f64
        ldu     6+FP_SLOT,s                 ; U = k (digit count)
        leay    8+FP_SLOT,s                 ; Y -> double val (8 bytes at S+8..15)
        FP_CALL FPOP_BINDEC
        puls    u,y                 ; runs at FPOP+1, restore caller's U,Y
        rts

        .global __mc6839_bindec_f32
__mc6839_bindec_f32:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        ldu     6+FP_SLOT,s                 ; U = k
        leay    8+FP_SLOT,s                 ; Y -> float val (4 bytes at S+8..11)
        FP_CALL FPOP_BINDEC
        puls    u,y
        rts

; ---- DECBIN --------------------------------------------------------------
;
; double __mc6839_decbin_f64(const uint8_t *bcd);
; float  __mc6839_decbin_f32(const uint8_t *bcd);
;
; Both return a value larger than 2 bytes, so the sret pointer takes IX.
; The bcd arg falls through to the stack.  At LBSR:
;   X      = sret pointer (caller-allocated 8B for f64, 4B for f32)
;   S+0..1 = return PC
;   S+2..3 = bcd pointer
;
; After pshs u,y:
;   S+4..5 = return PC
;   S+6..7 = bcd pointer

        .global __mc6839_decbin_f64
__mc6839_decbin_f64:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f64
        ldu     6+FP_SLOT,s                 ; U -> BCD string
        FP_CALL FPOP_DECBIN
        puls    u,y
        rts

        .global __mc6839_decbin_f32
__mc6839_decbin_f32:
        pshs    u,y
        FP_ENTRY __mc6839_fpcb_f32
        ldu     6+FP_SLOT,s                 ; U -> BCD string
        FP_CALL FPOP_DECBIN
        puls    u,y
        rts
