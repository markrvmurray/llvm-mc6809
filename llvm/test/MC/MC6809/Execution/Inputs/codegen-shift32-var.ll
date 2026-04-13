; LLVM IR for i32 variable shift execution tests (via codegen → libcall).

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

define dso_local i32 @shl32(i32 noundef %a, i8 noundef %n) {
  %n32 = zext i8 %n to i32
  %r = shl i32 %a, %n32
  ret i32 %r
}

; ===================================================================
; gcc6809 ↔ LLVM-MC6809 long-return bridge
;
; Both compilers handle pointers identically (first ptr → IX), so
; the wrapper signature works even though the inner i32 conventions
; differ. The wrapper body is compiled by LLVM and uses LLVM's
; native i32 calling convention internally (and lowers the shift to
; the __ashlsi3 libcall).
; ===================================================================

define dso_local void @shl32_w(ptr noundef %out, i32 noundef %a, i8 noundef %n) local_unnamed_addr {
  %r = call i32 @shl32(i32 %a, i8 %n)
  store i32 %r, ptr %out, align 1
  ret void
}
