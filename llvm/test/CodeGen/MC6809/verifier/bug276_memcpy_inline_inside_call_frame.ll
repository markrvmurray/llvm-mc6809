; RUN: llc -mtriple=mc6809 -global-isel -global-isel-abort=1 \
; RUN:   -verify-machineinstrs -O1 -o - %s 2>&1 | FileCheck %s

; Bug #276 (closed 2026-05-11):
; A G_MEMCPY/G_MEMSET sitting between an ADJCALLSTACKDOWN and its
; matching ADJCALLSTACKUP could legalise to a `memcpy` libcall, which
; emits its OWN nested ADJCALLSTACK pair. LLVM's MachineVerifier
; (verifyStackFrame) rejects nested call-frame setups:
;
;   *** Bad machine code: FrameSetup is after another FrameSetup ***
;   *** Bad machine code: FrameDestroy <n> is after FrameSetup <m> ***
;   *** Bad machine code: FrameDestroy is not after a FrameSetup ***
;
; Surface at picolibc Og-fp libm complex/long-double TUs: byval struct
; args bigger than the inline SizeLimit emit G_MEMCPY inside the outer
; call's ADJCALLSTACK frame; the legalizer's libcall fallback then
; produces the nested pair.
;
; Fix: detect "inside an active ADJCALLSTACKDOWN" in `legalizeMemOp`
; (walk MBB backward for the nearest FrameSetup) and emit a
; hand-rolled byte-by-byte inline loop, bypassing
; LegalizerHelper::findGISelOptimalMemOpLowering (which defers to
; MaxStoresPerMemcpy + allowsMisalignedMemoryAccesses and rejects
; common shapes at -Og + align=1).
;
; Closes 473 Og-fp verifier hits across 49 libm complex/long-double
; TUs (cacos, cacosh, cargl, casin/asinl/casinh, cpow, csqrt, cexp,
; cabsl, etc.) plus one test/test-stdlib/test-strtod regression.

; CHECK-LABEL: caller_with_large_byval:
; CHECK-NOT: Bad machine code

%complex_t = type { double, double }

declare void @callee_byval(ptr sret(%complex_t), ptr byval(%complex_t))

define void @caller_with_large_byval(ptr %out, ptr %z) {
entry:
  ; The frontend lowers byval args via a G_MEMCPY inside the call's
  ; ADJCALLSTACK frame. With sizeof(complex_t) = 16 > the -Og memcpy
  ; SizeLimit, the legalizer previously fell back to a libcall (=
  ; nested ADJCALLSTACK pair = verifier failure). Now it inlines.
  call void @callee_byval(ptr sret(%complex_t) %out,
                          ptr byval(%complex_t) %z)
  ret void
}
