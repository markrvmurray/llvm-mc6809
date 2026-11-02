; _Bool arguments that do not all fit in registers. Weighting each one shows
; which value reached which parameter.

target triple = "mc6809-unknown-unknown"

define i8 @weigh3(i1 zeroext %a, i1 zeroext %b, i1 zeroext %c) noinline {
  %x = zext i1 %a to i8
  %y = zext i1 %b to i8
  %z = zext i1 %c to i8
  %y2 = shl i8 %y, 1
  %z2 = shl i8 %z, 2
  %s = or i8 %x, %y2
  %r = or i8 %s, %z2
  ret i8 %r
}

define i8 @call3(i8 %a, i8 %b, i8 %c) {
  %x = icmp ne i8 %a, 0
  %y = icmp ne i8 %b, 0
  %z = icmp ne i8 %c, 0
  %r = call i8 @weigh3(i1 zeroext %x, i1 zeroext %y, i1 zeroext %z)
  ret i8 %r
}
