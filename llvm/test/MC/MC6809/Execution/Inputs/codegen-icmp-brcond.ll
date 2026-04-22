; LLVM IR functions exercising icmp+brcond patterns. Bug #114: the
; old codegen materialised the icmp result via a `lda #0/#1` diamond
; and then tested it via `tsta`, but any intervening i16 def into AD
; would clobber the bool LSB sitting inside AALSB or ABLSB. Every
; `icmp ult i16` consumed by an i16 select returned the false-side
; value regardless of the comparison. Fix routes brcond directly to
; a CompareBranch (bypassing the diamond materialisation entirely).

declare i16 @putchar(i16)

; Returns 1 if a < b unsigned, else 0. Exercises icmp ult i16 + br/select.
define dso_local i16 @ult_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp ult i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}

define dso_local i16 @ugt_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp ugt i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}

define dso_local i16 @ule_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp ule i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}

define dso_local i16 @uge_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp uge i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}

define dso_local i16 @eq_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp eq i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}

define dso_local i16 @ne_i16(i16 noundef %a, i16 noundef %b) local_unnamed_addr {
  %cmp = icmp ne i16 %a, %b
  %r = select i1 %cmp, i16 1, i16 0
  ret i16 %r
}
