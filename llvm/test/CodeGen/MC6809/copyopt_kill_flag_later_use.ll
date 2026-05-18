; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 -verify-machineinstrs %s -o - | FileCheck %s
;
; CopyOpt sentinel — chain-shortening must NOT transfer a `kill` flag
; onto $OrigSrc if $OrigSrc has later uses in the same MBB.
;
; The COPY-of-COPY pattern is:
;   $Y = COPY $X
;   ...
;   $Z = COPY killed $Y   ; <-- chain-shortened to: $Z = COPY killed $X
;
; If $X (the original source) is read again later in the same MBB,
; transferring the `kill` flag from $Y to $X prematurely deallocates $X
; and the later read is "Using an undefined physical register".
;
; This was discovered during Bug #307 round 2: the
; InstructionSelector's `MaterializeCC_C_to_byte` bridge emits an
; implicit-use of $phantom_carry_1 a few MIs after a COPY chain.
;
; Fix: forward-scan from the second COPY to MBB end; abort the chain
; shortening if any later MI reads $OrigSrc before a re-definition.
;
; Compilation must succeed with -verify-machineinstrs.

define i64 @copyopt_imaxabs(i64 %x) {
; CHECK-LABEL: copyopt_imaxabs:
entry:
  %lt = icmp slt i64 %x, 0
  %neg = sub nsw i64 0, %x
  %r = select i1 %lt, i64 %neg, i64 %x
  ret i64 %r
}

; CHECK: rts
