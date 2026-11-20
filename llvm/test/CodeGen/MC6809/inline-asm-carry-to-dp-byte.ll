; An inline-asm carry output ("=c") whose value the allocator places in a
; direct-page byte register: on the HD6309, with A and B both outputs of
; the same asm, the carry goes to __rc0.  The flag must be read with ADC
; into an accumulator and stored, never transferred: TFR has no code for a
; flag, and on the HD6309 register code C in a TFR is the constant zero.
;
; RUN: llc -mtriple=mc6809-unknown-os9 -mcpu=hd6309 -verify-machineinstrs < %s | FileCheck %s --check-prefixes=CHECK,HD6309
; RUN: llc -mtriple=mc6809-unknown-os9 -mcpu=mc6809 -verify-machineinstrs < %s | FileCheck %s
; RUN: llc -mtriple=mc6809-unknown-os9 -mcpu=hd6309 -filetype=obj < %s -o /dev/null

; CHECK-LABEL: f:
; CHECK:       os9 132
; CHECK-NOT:   tfr C
; CHECK-NOT:   tfr V
; HD6309:      ldb #0
; HD6309-NEXT: adcb #0
; HD6309-NEXT: stb <__rc0
; CHECK:       rts

@g = dso_local local_unnamed_addr global i16 0, align 1

define dso_local i16 @f(ptr noundef %x, i16 noundef %mode, ptr noundef %pathp) #0 {
entry:
  %conv = trunc i16 %mode to i8
  %0 = tail call { i8, i8, i8, ptr } asm sideeffect "os9 132", "=c,=A,=B,=x,1,3,~{memory}"(i8 %conv, ptr %x)
  %asmresult = extractvalue { i8, i8, i8, ptr } %0, 0
  %tobool.not = icmp eq i8 %asmresult, 0
  br i1 %tobool.not, label %if.end, label %if.then

if.then:
  %asmresult2 = extractvalue { i8, i8, i8, ptr } %0, 2
  %conv4 = zext i8 %asmresult2 to i16
  store i16 %conv4, ptr @g, align 1
  br label %cleanup

if.end:
  %asmresult1 = extractvalue { i8, i8, i8, ptr } %0, 1
  %conv6 = zext i8 %asmresult1 to i16
  store i16 %conv6, ptr %pathp, align 1
  br label %cleanup

cleanup:
  %retval.0 = phi i16 [ %conv4, %if.then ], [ 0, %if.end ]
  ret i16 %retval.0
}

attributes #0 = { minsize nounwind optsize }
