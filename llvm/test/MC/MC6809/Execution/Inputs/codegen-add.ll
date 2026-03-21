; The LLVM IR functions that llc will compile.
; Just the 8-bit add functions (16/32-bit regalloc is broken).

define dso_local signext i8 @add_s_i8(i8 noundef signext %a, i8 noundef signext %b, i8 noundef signext %c, i8 noundef signext %d, i8 noundef signext %e) local_unnamed_addr {
entry:
  %add = add i8 %a, %b
  %add3 = add i8 %add, %c
  %add5 = add i8 %add3, %d
  %add7 = add i8 %add5, %e
  ret i8 %add7
}

define dso_local signext i8 @add_s_i8_consts(i8 noundef signext %a, i8 noundef signext %b, i8 noundef signext %c, i8 noundef signext %d) local_unnamed_addr {
entry:
  %add = add i8 %a, %b
  %add3 = add i8 %add, %c
  %add5 = add i8 %add3, %d
  %add7 = add i8 %add5, 5
  ret i8 %add7
}
