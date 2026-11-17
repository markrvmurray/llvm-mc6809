; A 16-bit product of two unsigned bytes is one MUL (A x B -> D) rather than a
; call to __mulhi3. clang -O2 output for:
;
;   unsigned umul8(unsigned char a, unsigned char b) { return (unsigned)a * b; }
;   unsigned umul8_200(unsigned char a) { return a * 200u; }
;   unsigned umul8_load(const unsigned char *p, const unsigned char *q) {
;     return (unsigned)*p * *q;
;   }

target triple = "mc6809-unknown-unknown"


define i16 @umul8(i8 noundef zeroext %a, i8 noundef zeroext %b) {
entry:
  %conv = zext i8 %a to i16
  %conv1 = zext i8 %b to i16
  %mul = mul nuw i16 %conv1, %conv
  ret i16 %mul
}

define i16 @umul8_200(i8 noundef zeroext %a) {
entry:
  %conv = zext i8 %a to i16
  %mul = mul nuw i16 %conv, 200
  ret i16 %mul
}

define i16 @umul8_load(ptr nofree noundef readonly captures(none) %p, ptr nofree noundef readonly captures(none) %q) {
entry:
  %0 = load i8, ptr %p, align 1
  %conv = zext i8 %0 to i16
  %1 = load i8, ptr %q, align 1
  %conv1 = zext i8 %1 to i16
  %mul = mul nuw i16 %conv1, %conv
  ret i16 %mul
}

