; LLVM IR for loop execution tests.

; Countdown: loop n times and return n. Tests PHI, back-edge, exit.
define dso_local i16 @countdown(i16 noundef %n) {
entry:
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %loop, label %exit

loop:
  %i = phi i16 [ %n, %entry ], [ %i.next, %loop ]
  %i.next = add i16 %i, -1
  %done = icmp eq i16 %i.next, 0
  br i1 %done, label %exit, label %loop

exit:
  ret i16 %n
}

; Sum 1..n: return n*(n+1)/2. Two live i16 values (sum in D, counter in X).
define dso_local i16 @sum_to_n(i16 noundef %n) {
entry:
  %cmp = icmp sgt i16 %n, 0
  br i1 %cmp, label %loop, label %exit

loop:
  %i = phi i16 [ %n, %entry ], [ %i.next, %loop ]
  %sum = phi i16 [ 0, %entry ], [ %sum.next, %loop ]
  %sum.next = add i16 %sum, %i
  %i.next = add i16 %i, -1
  %done = icmp eq i16 %i.next, 0
  br i1 %done, label %exit, label %loop

exit:
  %result = phi i16 [ 0, %entry ], [ %sum.next, %loop ]
  ret i16 %result
}
