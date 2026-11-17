; A function where the allocator, with U free, puts a pointer into the
; function's own frame in U and walks it, while other code reads the same
; stack memory by its slot. A pass that took a U-based store for a store to a
; known frame slot would keep a stale copy of the element just written.
; clang -O2 output for:
;
;   extern void sink(int *, int *);
;   int walk(int n, int *out) {
;     int lo[6], hi[6];
;     int *p = lo, *q = hi + 5, *r = out;
;     for (int i = 0; i < n; i++) {
;       *p++ = i + 1;
;       *q-- = i * 3 + 2;
;       *r++ = lo[i] + hi[5 - i] + lo[0];
;     }
;     sink(lo, hi);
;     return lo[0] + lo[n - 1] + hi[5] + hi[6 - n];
;   }

target triple = "mc6809-unknown-unknown"


define i16 @walk(i16 noundef %n, ptr nofree noundef writeonly captures(none) %out) {
entry:
  %lo = alloca [6 x i16], align 1
  %hi = alloca [6 x i16], align 1
  %add.ptr = getelementptr inbounds nuw i8, ptr %hi, i16 10
  %cmp27 = icmp sgt i16 %n, 0
  br i1 %cmp27, label %for.body, label %for.cond.cleanup

for.cond.cleanup:                                 ; preds = %for.body, %entry
  call void @sink(ptr noundef nonnull %lo, ptr noundef nonnull %hi)
  %0 = load i16, ptr %lo, align 1
  %1 = getelementptr [2 x i8], ptr %lo, i16 %n
  %arrayidx13 = getelementptr i8, ptr %1, i16 -2
  %2 = load i16, ptr %arrayidx13, align 1
  %add14 = add nsw i16 %2, %0
  %3 = load i16, ptr %add.ptr, align 1
  %add16 = add nsw i16 %add14, %3
  %sub17 = sub nsw i16 6, %n
  %arrayidx18 = getelementptr inbounds [2 x i8], ptr %hi, i16 %sub17
  %4 = load i16, ptr %arrayidx18, align 1
  %add19 = add nsw i16 %add16, %4
  ret i16 %add19

for.body:                                         ; preds = %entry, %for.body
  %i.031 = phi i16 [ %add, %for.body ], [ 0, %entry ]
  %r.030 = phi ptr [ %incdec.ptr8, %for.body ], [ %out, %entry ]
  %q.029 = phi ptr [ %incdec.ptr3, %for.body ], [ %add.ptr, %entry ]
  %p.028 = phi ptr [ %incdec.ptr, %for.body ], [ %lo, %entry ]
  %add = add nuw nsw i16 %i.031, 1
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %p.028, i16 2
  store i16 %add, ptr %p.028, align 1
  %mul = mul nuw nsw i16 %i.031, 3
  %add2 = add nuw nsw i16 %mul, 2
  %incdec.ptr3 = getelementptr inbounds i8, ptr %q.029, i16 -2
  store i16 %add2, ptr %q.029, align 1
  %arrayidx = getelementptr inbounds nuw [2 x i8], ptr %lo, i16 %i.031
  %5 = load i16, ptr %arrayidx, align 1
  %sub = sub nuw nsw i16 5, %i.031
  %arrayidx4 = getelementptr inbounds nuw [2 x i8], ptr %hi, i16 %sub
  %6 = load i16, ptr %arrayidx4, align 1
  %add5 = add nsw i16 %6, %5
  %7 = load i16, ptr %lo, align 1
  %add7 = add nsw i16 %add5, %7
  %incdec.ptr8 = getelementptr inbounds nuw i8, ptr %r.030, i16 2
  store i16 %add7, ptr %r.030, align 1
  %exitcond.not = icmp eq i16 %add, %n
  br i1 %exitcond.not, label %for.cond.cleanup, label %for.body
}

declare void @sink(ptr noundef, ptr noundef)

