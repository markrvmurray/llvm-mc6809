; LLVM IR functions for 8-bit multiply execution tests.

define dso_local signext i8 @mul_i8(i8 noundef signext %a, i8 noundef signext %b) local_unnamed_addr {
entry:
  %mul = mul i8 %a, %b
  ret i8 %mul
}

define dso_local signext i8 @mul_i8_const(i8 noundef signext %a) local_unnamed_addr {
entry:
  %mul = mul i8 %a, 7
  ret i8 %mul
}

define dso_local signext i8 @mul_i8_chain(i8 noundef signext %a, i8 noundef signext %b, i8 noundef signext %c) local_unnamed_addr {
entry:
  %mul1 = mul i8 %a, %b
  %mul2 = mul i8 %mul1, %c
  ret i8 %mul2
}
