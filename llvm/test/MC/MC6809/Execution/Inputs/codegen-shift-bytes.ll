; Constant 16-bit shifts by a whole byte or more, which the legalizer splits
; into a move of the surviving byte and a shift of that byte alone, and the
; signed divisions by a power of two built from them. clang -O2 output for:
;
;   uint16_t shl8(uint16_t x)  { return x << 8; }
;   uint16_t shl11(uint16_t x) { return x << 11; }
;   uint16_t lshr8(uint16_t x) { return x >> 8; }
;   uint16_t lshr13(uint16_t x){ return x >> 13; }
;   int16_t  ashr8(int16_t x)  { return x >> 8; }
;   int16_t  ashr12(int16_t x) { return x >> 12; }
;   int16_t  ashr15(int16_t x) { return x >> 15; }
;   int16_t  sdiv4(int16_t x)  { return x / 4; }
;   int16_t  sdiv256(int16_t x){ return x / 256; }
;   uint16_t bext(uint8_t b)   { return (uint16_t)b << 9; }

target triple = "mc6809-unknown-unknown"


define noundef i16 @shl8(i16 noundef %x) {
entry:
  %shl = shl i16 %x, 8
  ret i16 %shl
}

define noundef i16 @shl11(i16 noundef %x) {
entry:
  %shl = shl i16 %x, 11
  ret i16 %shl
}

define noundef i16 @lshr8(i16 noundef %x) {
entry:
  %shr = lshr i16 %x, 8
  ret i16 %shr
}

define noundef i16 @lshr13(i16 noundef %x) {
entry:
  %shr = lshr i16 %x, 13
  ret i16 %shr
}

define noundef i16 @ashr8(i16 noundef %x) {
entry:
  %shr = ashr i16 %x, 8
  ret i16 %shr
}

define noundef i16 @ashr12(i16 noundef %x) {
entry:
  %shr = ashr i16 %x, 12
  ret i16 %shr
}

define noundef i16 @ashr15(i16 noundef %x) {
entry:
  %shr = ashr i16 %x, 15
  ret i16 %shr
}

define noundef i16 @sdiv4(i16 noundef %x) {
entry:
  %div = sdiv i16 %x, 4
  ret i16 %div
}

define noundef i16 @sdiv256(i16 noundef %x) {
entry:
  %div = sdiv i16 %x, 256
  ret i16 %div
}

define noundef i16 @bext(i8 noundef zeroext %b) {
entry:
  %conv = zext i8 %b to i16
  %shl = shl i16 %conv, 9
  ret i16 %shl
}

