; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -mcpu=hd6309 -O2 %s -o - | FileCheck %s
;
; Bug #301b (2026-05-16): `icmp slt/sge i32/i64 X, 0` and the
; optimizer-normalised `icmp sgt X, -1` / `icmp sle X, -1` forms must
; lower to a single TST on the high byte of the operand — NOT to a
; __cmpsi2 / __cmpdi2 libcall.
;
; The libcall path emits `lbsr __cmpsi2 + trunc i32→i8 + icmp eq 0`,
; and the post-RA spill placer reuses the slot holding cmpsi2's
; return value, overwriting it before the trunc consumes it.  The
; resulting `stb` stores a stale byte (typically the comparand's low
; byte), and inputs where that byte is 0 trigger a spurious BEQ.
;
; The manifest was gmtime_r's `if (rem < 0)` post-mod adjust at
; LTO+HD6309: ~1/256 of positive t values had `t%86400 & 0xff == 0`
; and got hour off by exactly +24.
;
; The fast-path lowering emits a single CMPD #0 / TSTD on the high i16
; word + a {bge,blt,lbge,lblt} branch.  The exact opcode (CMPD #0 vs
; TSTD) is a peephole concern; what matters is no __cmpsi2/__cmpdi2
; libcall is emitted.

; CHECK-LABEL: i32_slt_zero:
define i1 @i32_slt_zero(i32 %x) {
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lblt|blt}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp slt i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_sge_zero:
define i1 @i32_sge_zero(i32 %x) {
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lbge|bge}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sge i32 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_sgt_minus_one:
define i1 @i32_sgt_minus_one(i32 %x) {
  ; Optimizer-normalised "X >= 0" form — must also use fast path.
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lbge|bge}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sgt i32 %x, -1
  ret i1 %cmp
}

; CHECK-LABEL: i32_sle_minus_one:
define i1 @i32_sle_minus_one(i32 %x) {
  ; Optimizer-normalised "X < 0" form — must also use fast path.
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lblt|blt}}
  ; CHECK-NOT: __cmpsi2
  ; CHECK-NOT: __ucmpsi2
  %cmp = icmp sle i32 %x, -1
  ret i1 %cmp
}

; CHECK-LABEL: i64_slt_zero:
define i1 @i64_slt_zero(i64 %x) {
  ; i64 recursively splits to i32, which my custom rule reduces to
  ; i16 — single TST/CMPD on the i64's MSByte word.
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lblt|blt}}
  ; CHECK-NOT: __cmpdi2
  ; CHECK-NOT: __ucmpdi2
  %cmp = icmp slt i64 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i64_sge_zero:
define i1 @i64_sge_zero(i64 %x) {
  ; CHECK: {{cmpd|tstd}}
  ; CHECK: {{lbge|bge}}
  ; CHECK-NOT: __cmpdi2
  ; CHECK-NOT: __ucmpdi2
  %cmp = icmp sge i64 %x, 0
  ret i1 %cmp
}

; CHECK-LABEL: i32_slt_one:
define i1 @i32_slt_one(i32 %x) {
  ; NOT a sign test (constant 1, not 0 or -1) — must fall through to
  ; the libcall path.  The presence of __cmpsi2 here proves the
  ; fast-path predicate doesn't over-trigger.
  ; CHECK: lbsr __cmpsi2
  %cmp = icmp slt i32 %x, 1
  ret i1 %cmp
}
