; LLVM IR for struct-by-pointer execution tests.
;
; Note (2026-04-08): this file used to use the LLVM `byval(%struct.Point)`
; attribute, but LLVM-MC6809's CallLowering does not currently honour
; byval semantics — the byval pointer was lowered as a plain pointer in
; IX, with no caller-side copy. That happened to work for the read-only
; tests below but disagrees with gcc6809 (which passes structs by value
; on the stack as a contiguous block, not via a pointer). To make this
; test cleanly cross-validate against gcc6809 we now use a plain `ptr`
; parameter.
;
; Proper byval support (caller copies the struct to a local stack slot
; and passes its address; callee reads/may-modify its private copy) is
; a separate backend TODO. When it lands, this file should be updated
; back to use byval and the C harness updated to match.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

%struct.Point = type { i16, i16 }

define dso_local i16 @get_x(ptr %p) {
  %x = load i16, ptr %p, align 1
  ret i16 %x
}

define dso_local i16 @get_y(ptr %p) {
  %yptr = getelementptr inbounds %struct.Point, ptr %p, i16 0, i32 1
  %y = load i16, ptr %yptr, align 1
  ret i16 %y
}

define dso_local i16 @sum_point(ptr %p) {
  %x = load i16, ptr %p, align 1
  %yptr = getelementptr inbounds %struct.Point, ptr %p, i16 0, i32 1
  %y = load i16, ptr %yptr, align 1
  %r = add i16 %x, %y
  ret i16 %r
}
