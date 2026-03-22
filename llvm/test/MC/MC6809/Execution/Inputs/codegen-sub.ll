; Subtract functions for codegen execution testing.

; 5 - a - b - c - d - e
define dso_local signext i8 @sub_s_i8_consts(i8 noundef signext %a, i8 noundef signext %b, i8 noundef signext %c, i8 noundef signext %d, i8 noundef signext %e) local_unnamed_addr {
entry:
  %sub4 = sub i8 5, %a
  %sub5 = sub i8 %sub4, %b
  %sub6 = sub i8 %sub5, %c
  %sub7 = sub i8 %sub6, %d
  %sub8 = sub i8 %sub7, %e
  ret i8 %sub8
}

; a - b (simple 2-arg)
define dso_local signext i8 @sub_simple(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
entry:
  %sub = sub i8 %a, %b
  ret i8 %sub
}
