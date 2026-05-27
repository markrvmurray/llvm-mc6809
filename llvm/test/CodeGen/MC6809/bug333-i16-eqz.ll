; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 %s -o - | FileCheck %s
;
; Bug #333: a 16-bit equality/inequality test against zero must use the
; native 16-bit compare (the Z flag already covers all 16 bits after a
; 16-bit load/op), NOT a split into two byte compares + AND/OR.
;
; The old `narrow_i16_eq_ne_icmp` combine rewrote `icmp eq/ne i16 %x, 0`
; into `unmerge %x -> lo,hi; icmp lo==0; icmp hi==0; and/or`. That was
; both larger than the native path and miscompiled the inlined scanf
; integer-accumulation loop under LTO (sscanf("18","%d") yielded 8).
; Guard against re-introducing it: the byte-split ops must not appear.

define void @brz_i16(i16 %x, ptr %p) {
; CHECK-LABEL: brz_i16:
; CHECK:       cmp{{[dx]}} #0
; CHECK-NOT:   orb
; CHECK-NOT:   anda
entry:
  %c = icmp eq i16 %x, 0
  br i1 %c, label %z, label %nz
z:
  store i8 1, ptr %p
  br label %done
nz:
  store i8 2, ptr %p
  br label %done
done:
  ret void
}

define void @brnz_i16(i16 %x, ptr %p) {
; CHECK-LABEL: brnz_i16:
; CHECK:       cmp{{[dx]}} #0
; CHECK-NOT:   orb
; CHECK-NOT:   anda
entry:
  %c = icmp ne i16 %x, 0
  br i1 %c, label %nz, label %z
z:
  store i8 1, ptr %p
  br label %done
nz:
  store i8 2, ptr %p
  br label %done
done:
  ret void
}
