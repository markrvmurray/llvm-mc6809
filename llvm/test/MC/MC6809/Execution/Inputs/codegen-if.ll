; if_s8: returns a if a > b (signed), else b
define dso_local signext i8 @if_s8(i8 %a, i8 %b) {
entry:
  %cmp = icmp sgt i8 %a, %b
  br i1 %cmp, label %then, label %else
then:
  ret i8 %a
else:
  ret i8 %b
}
