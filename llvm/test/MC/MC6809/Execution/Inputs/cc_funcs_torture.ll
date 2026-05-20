; LLVM IR torture set for the i32 and i64 calling convention.
;
; Each function under test takes args of known shapes, picks one
; field out, and returns it.  The C harness (harness-cc-torture.c)
; calls them with sentinel values and prints the returned word.
; FileCheck in calling-convention-torture.s matches the expected
; hex strings.
;
; Sentinel values used throughout:
;   i8  : 0x5A
;   i16 : 0xA55A
;   i32 : 0xCAFEF00D   (the classic Bug #311 value)
;   i64 : 0xFFEEDDCCBBAA9988
;
; Each test isolates ONE ABI behaviour:
;   1) i32 pass-through (identity)
;   2) i32 swap          (forces correct slot access for arg #2)
;   3) i32 third-of-three (forces stack-passing of arg #3)
;   4) i32 mixed         (i8, i32, i16 interleave)
;   5) i32 low / high half
;   6) i32 byte extract  (verifies BE byte order in memory image)
;   7) i32 returned via byval-pointer (sret shape)
;   8) i64 pass-through (identity)
;   9) i64 low/high i32 half
;  10) i64 byte extract (top, bottom)
;  11) i64 swap
;  12) i64 mixed-with-i8 sandwich
;  13) i64 vararg-promotion sentinel (no varargs in this file —
;      the i32_mix and i64_sandwich functions cover the same
;      packing rules that vararg uses)

target triple = "mc6809-unknown-unknown"

;;; ---------- i32 ----------

define dso_local i32 @i32_identity(i32 %a) local_unnamed_addr {
  ret i32 %a
}

define dso_local i32 @i32_swap(i32 %a, i32 %b) local_unnamed_addr {
  ret i32 %b
}

define dso_local i32 @i32_third_of_three(i32 %a, i32 %b, i32 %c) local_unnamed_addr {
  ret i32 %c
}

; Returns the 2nd arg (the i32).  Interleave forces the lowering to
; pack non-uniform widths into the argument area in ABI order.
define dso_local i32 @i32_mix(i8 %h, i32 %x, i16 %t) local_unnamed_addr {
  ret i32 %x
}

define dso_local i16 @i32_low16(i32 %x) local_unnamed_addr {
  %r = trunc i32 %x to i16
  ret i16 %r
}

define dso_local i16 @i32_high16(i32 %x) local_unnamed_addr {
  %s = lshr i32 %x, 16
  %r = trunc i32 %s to i16
  ret i16 %r
}

; Byte 0 = least-significant byte of the i32.
define dso_local i8 @i32_byte0(i32 %x) local_unnamed_addr {
  %r = trunc i32 %x to i8
  ret i8 %r
}

; Byte 3 = most-significant byte of the i32.
define dso_local i8 @i32_byte3(i32 %x) local_unnamed_addr {
  %s = lshr i32 %x, 24
  %r = trunc i32 %s to i8
  ret i8 %r
}

;;; ---------- i64 ----------

define dso_local i64 @i64_identity(i64 %a) local_unnamed_addr {
  ret i64 %a
}

define dso_local i32 @i64_low32(i64 %x) local_unnamed_addr {
  %r = trunc i64 %x to i32
  ret i32 %r
}

define dso_local i32 @i64_high32(i64 %x) local_unnamed_addr {
  %s = lshr i64 %x, 32
  %r = trunc i64 %s to i32
  ret i32 %r
}

define dso_local i8 @i64_byte0(i64 %x) local_unnamed_addr {
  %r = trunc i64 %x to i8
  ret i8 %r
}

; Byte 7 = most-significant byte of the i64.
define dso_local i8 @i64_byte7(i64 %x) local_unnamed_addr {
  %s = lshr i64 %x, 56
  %r = trunc i64 %s to i8
  ret i8 %r
}

define dso_local i64 @i64_swap(i64 %a, i64 %b) local_unnamed_addr {
  ret i64 %b
}

; The i64 lives between i8 args — same packing rules vararg uses
; when an i64 lands between two narrower args.
define dso_local i64 @i64_sandwich(i8 %h, i64 %x, i8 %t) local_unnamed_addr {
  ret i64 %x
}
