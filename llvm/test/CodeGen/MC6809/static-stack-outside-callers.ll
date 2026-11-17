; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 < %s | FileCheck %s

; A leaf that is never entered while already running may keep its locals in
; static memory. A function that can be called from outside the module is not
; such a leaf: an interrupt handler in another file, or in assembly, may call
; it at any moment, and nothing here can see that. So static storage goes only
; to functions that no outside caller and no handler can reach. `main` is
; entered once and does not count as an outside caller. clang -O2 output for:
;
;   #define BODY { int v[4]; ...; volatile int *p = v; return p[0]+p[1]+p[2]+p[3]; }
;   int callable_from_outside(int a, int b, int c, int d) BODY
;   static int reached_from_main_only(...) BODY                /* all noinline */
;   static int reached_from_handler(...) BODY
;   static int reached_through_outside(...) BODY
;   int result;
;   __attribute__((interrupt)) void handler(void) { result = reached_from_handler(1,2,3,4); }
;   int outside_entry(int a) { return reached_through_outside(a,2,3,4) + 1; }
;   int main(void) { return reached_from_main_only(1,2,3,4); }

target triple = "mc6809-unknown-unknown"


@result = global i16 0, align 1

define i16 @callable_from_outside(i16 noundef %a, i16 noundef %b, i16 noundef %c, i16 noundef %d) {
entry:
  %v.sroa.0 = alloca i16, align 1
  %v.sroa.4 = alloca i16, align 1
  %v.sroa.6 = alloca i16, align 1
  %v.sroa.8 = alloca i16, align 1
  store i16 %a, ptr %v.sroa.0, align 1
  store i16 %b, ptr %v.sroa.4, align 1
  store i16 %c, ptr %v.sroa.6, align 1
  store i16 %d, ptr %v.sroa.8, align 1
  %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0. = load volatile i16, ptr %v.sroa.0, align 1
  %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2. = load volatile i16, ptr %v.sroa.4, align 1
  %add = add nsw i16 %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2., %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.
  %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4. = load volatile i16, ptr %v.sroa.6, align 1
  %add7 = add nsw i16 %add, %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4.
  %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6. = load volatile i16, ptr %v.sroa.8, align 1
  %add9 = add nsw i16 %add7, %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6.
  ret i16 %add9
}

define void @handler() "interrupt" {
entry:
  %call = tail call fastcc i16 @reached_from_handler()
  store i16 %call, ptr @result, align 1
  ret void
}

define internal fastcc i16 @reached_from_handler() noinline {
entry:
  %v.sroa.0 = alloca i16, align 1
  %v.sroa.4 = alloca i16, align 1
  %v.sroa.6 = alloca i16, align 1
  %v.sroa.8 = alloca i16, align 1
  store i16 1, ptr %v.sroa.0, align 1
  store i16 2, ptr %v.sroa.4, align 1
  store i16 3, ptr %v.sroa.6, align 1
  store i16 4, ptr %v.sroa.8, align 1
  %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0. = load volatile i16, ptr %v.sroa.0, align 1
  %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2. = load volatile i16, ptr %v.sroa.4, align 1
  %add = add nsw i16 %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2., %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.
  %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4. = load volatile i16, ptr %v.sroa.6, align 1
  %add7 = add nsw i16 %add, %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4.
  %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6. = load volatile i16, ptr %v.sroa.8, align 1
  %add9 = add nsw i16 %add7, %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6.
  ret i16 %add9
}

define range(i16 -32767, -32768) i16 @outside_entry(i16 noundef %a) {
entry:
  %call = tail call fastcc i16 @reached_through_outside(i16 noundef %a)
  %add = add nsw i16 %call, 1
  ret i16 %add
}

define internal fastcc i16 @reached_through_outside(i16 noundef %a) noinline {
entry:
  %v.sroa.0 = alloca i16, align 1
  %v.sroa.4 = alloca i16, align 1
  %v.sroa.6 = alloca i16, align 1
  %v.sroa.8 = alloca i16, align 1
  store i16 %a, ptr %v.sroa.0, align 1
  store i16 2, ptr %v.sroa.4, align 1
  store i16 3, ptr %v.sroa.6, align 1
  store i16 4, ptr %v.sroa.8, align 1
  %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0. = load volatile i16, ptr %v.sroa.0, align 1
  %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2. = load volatile i16, ptr %v.sroa.4, align 1
  %add = add nsw i16 %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2., %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.
  %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4. = load volatile i16, ptr %v.sroa.6, align 1
  %add7 = add nsw i16 %add, %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4.
  %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6. = load volatile i16, ptr %v.sroa.8, align 1
  %add9 = add nsw i16 %add7, %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6.
  ret i16 %add9
}

define i16 @main() {
entry:
  %call = tail call fastcc i16 @reached_from_main_only()
  ret i16 %call
}

define internal fastcc i16 @reached_from_main_only() noinline {
entry:
  %v.sroa.0 = alloca i16, align 1
  %v.sroa.4 = alloca i16, align 1
  %v.sroa.6 = alloca i16, align 1
  %v.sroa.8 = alloca i16, align 1
  store i16 1, ptr %v.sroa.0, align 1
  store i16 2, ptr %v.sroa.4, align 1
  store i16 3, ptr %v.sroa.6, align 1
  store i16 4, ptr %v.sroa.8, align 1
  %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0. = load volatile i16, ptr %v.sroa.0, align 1
  %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2. = load volatile i16, ptr %v.sroa.4, align 1
  %add = add nsw i16 %v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.0.v.sroa.4.2., %v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.v.sroa.0.0.
  %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4. = load volatile i16, ptr %v.sroa.6, align 1
  %add7 = add nsw i16 %add, %v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.0.v.sroa.6.4.
  %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6. = load volatile i16, ptr %v.sroa.8, align 1
  %add9 = add nsw i16 %add7, %v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.0.v.sroa.8.6.
  ret i16 %add9
}


; CHECK-NOT:  callable_from_outside_sstk
; CHECK-NOT:  reached_from_handler_sstk
; CHECK-NOT:  reached_through_outside_sstk
; CHECK:      reached_from_main_only_sstk
; CHECK-NOT:  callable_from_outside_sstk
; CHECK-NOT:  reached_from_handler_sstk
; CHECK-NOT:  reached_through_outside_sstk
