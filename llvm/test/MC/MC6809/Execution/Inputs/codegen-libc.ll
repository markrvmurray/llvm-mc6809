; LLVM IR for libc smoke test functions.

target datalayout = "E-m:e-p:16:8-i16:8-i32:8-i64:8-f32:8-f64:8-a:8-Fi8-n8"
target triple = "mc6809"

; NOTE: memcpy/memset require multiple live pointers in loops, triggering
; frame pointer clobber or CC flag corruption (bug #38). Skipped for now.

; strlen using counter (avoids 2 live pointers which exhaust INDEX regs)
define dso_local i16 @my_strlen(ptr %s) {
entry:
  br label %loop

loop:
  %i = phi i16 [ 0, %entry ], [ %i.next, %cont ]
  %p = getelementptr i8, ptr %s, i16 %i
  %v = load i8, ptr %p, align 1
  %done = icmp eq i8 %v, 0
  br i1 %done, label %exit, label %cont

cont:
  %i.next = add i16 %i, 1
  br label %loop

exit:
  ret i16 %i
}

define dso_local i16 @my_strcmp(ptr %a, ptr %b) {
entry:
  br label %loop

loop:
  %pa = phi ptr [ %a, %entry ], [ %pa.next, %loop.inc ]
  %pb = phi ptr [ %b, %entry ], [ %pb.next, %loop.inc ]
  %va = load i8, ptr %pa, align 1
  %vb = load i8, ptr %pb, align 1
  %diff = sub i8 %va, %vb
  %neq = icmp ne i8 %diff, 0
  br i1 %neq, label %ret.diff, label %check.null

check.null:
  %isnull = icmp eq i8 %va, 0
  br i1 %isnull, label %ret.zero, label %loop.inc

loop.inc:
  %pa.next = getelementptr i8, ptr %pa, i16 1
  %pb.next = getelementptr i8, ptr %pb, i16 1
  br label %loop

ret.diff:
  %va16 = zext i8 %va to i16
  %vb16 = zext i8 %vb to i16
  %result = sub i16 %va16, %vb16
  ret i16 %result

ret.zero:
  ret i16 0
}

; Single-block loop structure (matches Clang output). The old multi-block
; version split digit-check and digit-processing across blocks, which
; caused register exhaustion at -O0 due to PHI pressure.
define dso_local i16 @my_atoi(ptr %s) {
entry:
  %c0 = load i8, ptr %s, align 1
  %t0 = add i8 %c0, -48
  %isdigit0 = icmp ult i8 %t0, 10
  br i1 %isdigit0, label %loop, label %done

loop:
  %ch = phi i8 [ %c0, %entry ], [ %cnext, %loop ]
  %acc = phi i16 [ 0, %entry ], [ %acc.next, %loop ]
  %p = phi ptr [ %s, %entry ], [ %p.next, %loop ]
  %mul = mul i16 %acc, 10
  %d = add i8 %ch, -48
  %d16 = zext i8 %d to i16
  %acc.next = add i16 %mul, %d16
  %p.next = getelementptr i8, ptr %p, i16 1
  %cnext = load i8, ptr %p.next, align 1
  %tnext = add i8 %cnext, -48
  %isdigit = icmp ult i8 %tnext, 10
  br i1 %isdigit, label %loop, label %done

done:
  %result = phi i16 [ 0, %entry ], [ %acc.next, %loop ]
  ret i16 %result
}
