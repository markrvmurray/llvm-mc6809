; LLVM IR for switch statement execution tests.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

define dso_local i16 @switch5(i16 noundef %x) {
entry:
  switch i16 %x, label %default [
    i16 0, label %case0
    i16 1, label %case1
    i16 2, label %case2
    i16 3, label %case3
    i16 4, label %case4
  ]
case0: ret i16 100
case1: ret i16 200
case2: ret i16 300
case3: ret i16 400
case4: ret i16 500
default: ret i16 999
}
