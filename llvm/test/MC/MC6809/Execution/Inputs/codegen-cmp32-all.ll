; Every 32-bit integer comparison, which the 6809 does inline: equality as
; an OR of the XORed halves tested against zero, order as the borrow out of
; a 32-bit subtract, signed order with both sign bits flipped. clang -O2
; output for:
;
;   unsigned char cmp_all(int32_t a, int32_t b) {
;     uint32_t ua = a, ub = b;
;     return (a == b) | (a != b) << 1 | (a < b) << 2 | (a <= b) << 3 |
;            (a > b) << 4 | (a >= b) << 5 | (ua < ub) << 6 | (ua <= ub) << 7;
;   }
;   unsigned char cmp_ug(uint32_t a, uint32_t b) {
;     return (a > b) | (a >= b) << 1;
;   }

target triple = "mc6809-unknown-unknown"


define zeroext i8 @cmp_all(i32 noundef %a, i32 noundef %b) {
entry:
  %cmp = icmp eq i32 %a, %b
  %or = select i1 %cmp, i8 1, i8 2
  %cmp3 = icmp slt i32 %a, %b
  %shl5 = select i1 %cmp3, i8 4, i8 0
  %or6 = or disjoint i8 %or, %shl5
  %cmp7.not = icmp sgt i32 %a, %b
  %shl9 = select i1 %cmp7.not, i8 0, i8 8
  %or10 = or disjoint i8 %or6, %shl9
  %shl13 = select i1 %cmp7.not, i8 16, i8 0
  %or14 = or disjoint i8 %or10, %shl13
  %shl17 = select i1 %cmp3, i8 0, i8 32
  %or18 = or disjoint i8 %or14, %shl17
  %cmp19 = icmp ult i32 %a, %b
  %shl21 = select i1 %cmp19, i8 64, i8 0
  %or22 = or disjoint i8 %or18, %shl21
  %cmp23.not = icmp ugt i32 %a, %b
  %shl25 = select i1 %cmp23.not, i8 0, i8 -128
  %or26 = or i8 %or22, %shl25
  ret i8 %or26
}

define zeroext i8 @cmp_ug(i32 noundef %a, i32 noundef %b) {
entry:
  %cmp = icmp ugt i32 %a, %b
  %conv = zext i1 %cmp to i8
  %cmp1.not = icmp ult i32 %a, %b
  %shl = select i1 %cmp1.not, i8 0, i8 2
  %or = or disjoint i8 %shl, %conv
  ret i8 %or
}

