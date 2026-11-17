; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O2 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -global-isel -global-isel-abort=1 -mtriple=mc6809 -O1 -verify-machineinstrs < %s | FileCheck %s

; picolibc's four-byte strstr loop, `h2 = (h2 << 8) | c` against a 32-bit
; compare, ran the 6809 out of registers. `c` is zero-extended into D,
; pinned there until a COPY reads it out; the scheduler, which counted
; pressure on A and B but not on a physical D, hoisted the extension and
; left the COPY behind, so D stayed live across the byte moves of the
; shift, which need A and B. clang -O2 output for:
;
;   char *fourbyte(const unsigned char *hs, const unsigned char *ne) {
;     uint32_t h1 = ((uint32_t)ne[0] << 24) | ((uint32_t)ne[1] << 16) |
;                   ((uint32_t)ne[2] << 8) | ne[3];
;     uint32_t h2 = 0;
;     int c;
;     for (c = hs[0]; c != 0 && h1 != h2; c = *++hs)
;       h2 = (h2 << 8) | c;
;     return h1 == h2 ? (char *)hs - 4 : 0;
;   }

target triple = "mc6809-unknown-unknown"

; CHECK-LABEL: fourbyte:
; CHECK:       {{rts|,pc}}

define ptr @fourbyte(ptr nofree noundef readonly captures(ret: address, provenance) %hs, ptr nofree noundef readonly captures(none) %ne) {
entry:
  %0 = load i8, ptr %ne, align 1
  %conv = zext i8 %0 to i32
  %shl = shl nuw i32 %conv, 24
  %arrayidx1 = getelementptr inbounds nuw i8, ptr %ne, i16 1
  %1 = load i8, ptr %arrayidx1, align 1
  %conv2 = zext i8 %1 to i32
  %shl3 = shl nuw nsw i32 %conv2, 16
  %or = or disjoint i32 %shl3, %shl
  %arrayidx4 = getelementptr inbounds nuw i8, ptr %ne, i16 2
  %2 = load i8, ptr %arrayidx4, align 1
  %conv5 = zext i8 %2 to i32
  %shl6 = shl nuw nsw i32 %conv5, 8
  %or7 = or disjoint i32 %or, %shl6
  %arrayidx8 = getelementptr inbounds nuw i8, ptr %ne, i16 3
  %3 = load i8, ptr %arrayidx8, align 1
  %conv9 = zext i8 %3 to i32
  %or10 = or disjoint i32 %or7, %conv9
  %c.0.in31 = load i8, ptr %hs, align 1
  %cmp.not32 = icmp eq i8 %c.0.in31, 0
  %cmp14.not33 = icmp eq i32 %or10, 0
  %or.cond34 = select i1 %cmp.not32, i1 true, i1 %cmp14.not33
  br i1 %or.cond34, label %for.end, label %for.body

for.body:                                         ; preds = %entry, %for.body
  %c.0.in37 = phi i8 [ %c.0.in, %for.body ], [ %c.0.in31, %entry ]
  %h2.036 = phi i32 [ %or18, %for.body ], [ 0, %entry ]
  %hs.addr.035 = phi ptr [ %incdec.ptr, %for.body ], [ %hs, %entry ]
  %shl16 = shl i32 %h2.036, 8
  %conv17 = zext i8 %c.0.in37 to i32
  %or18 = or disjoint i32 %shl16, %conv17
  %incdec.ptr = getelementptr inbounds nuw i8, ptr %hs.addr.035, i16 1
  %c.0.in = load i8, ptr %incdec.ptr, align 1
  %cmp.not = icmp eq i8 %c.0.in, 0
  %cmp14.not = icmp eq i32 %or10, %or18
  %or.cond = select i1 %cmp.not, i1 true, i1 %cmp14.not
  br i1 %or.cond, label %for.end, label %for.body

for.end:                                          ; preds = %for.body, %entry
  %hs.addr.0.lcssa = phi ptr [ %hs, %entry ], [ %incdec.ptr, %for.body ]
  %cmp14.not.lcssa = phi i1 [ %cmp14.not33, %entry ], [ %cmp14.not, %for.body ]
  %add.ptr = getelementptr inbounds i8, ptr %hs.addr.0.lcssa, i16 -4
  %cond = select i1 %cmp14.not.lcssa, ptr %add.ptr, ptr null
  ret ptr %cond
}

