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

define dso_local i16 @my_atoi(ptr %s) {
entry:
  br label %loop

loop:
  %p = phi ptr [ %s, %entry ], [ %p.next, %digit ]
  %acc = phi i16 [ 0, %entry ], [ %acc.next, %digit ]
  %ch = load i8, ptr %p, align 1
  %is.ge.0 = icmp uge i8 %ch, 48  ; '0'
  %is.le.9 = icmp ule i8 %ch, 57  ; '9'
  %is.digit = and i1 %is.ge.0, %is.le.9
  br i1 %is.digit, label %digit, label %done

digit:
  %d = sub i8 %ch, 48
  %d16 = zext i8 %d to i16
  %acc10 = mul i16 %acc, 10
  %acc.next = add i16 %acc10, %d16
  %p.next = getelementptr i8, ptr %p, i16 1
  br label %loop

done:
  ret i16 %acc
}
