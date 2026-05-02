; RUN: llc -global-isel -global-isel-abort=1 -O2 -mtriple=mc6809 -mcpu=hd6309 \
; RUN:     -stop-after=finalize-isel %s -o - | FileCheck %s

; Bug a5bb42f regression sentinel: short-circuit && over two icmps with
; an i1 PHI flowing through a diamond materialization (ConditionalImm)
; used to lower to a separated `Compare_*_* + ConditionalLongBranchRelative`
; pair in `emitConditionalImm`. Because the CCond vreg the branch consumed
; was not aliased to the underlying $n/$z/$v/$c flag bits at the MIR level,
; intervening loads with `implicit-def $nz/$v` were free to wedge between
; cmp and branch, clobbering the flags the long branch read. After post-RA
; expansion the result was `cmpd Imm / ldd Mem / lbXX target` — silently
; taking the wrong branch on the iteration where the loaded value is zero.
;
; Real-world impact: fgets() returned success without ever invoking the
; FILE's get callback under HD6309 -Oz; sibling failures across
; test-fdevopen / test-fmemopen / test-getdelim / test-funopen.
;
; Fix: route long-branch selections through the existing fused
; CompareBranch_*_* / TestBranch_*_* pseudos at the emission site
; (MC6809ISelLowering::emitConditionalImm). The fused pseudo is a single
; MachineInstr until expandFusedCompareBranch runs at postrapseudos —
; well after register allocation and scheduling — so no instruction can
; be wedged inside it.
;
; This regression checks at the MIR level (after finalize-isel) that
; the diamond-CFG materialisation emits a fused CompareBranch pseudo
; — NOT the separated Compare + ConditionalLongBranchRelative pair
; that exposed the original bug.

target triple = "mc6809"

declare i16 @produce()

define i16 @short_circuit_and_with_phi(i16 %n_init, i16 %idx_init) {
entry:
  br label %loop.header

loop.header:
  %n = phi i16 [ %n_init, %entry ], [ %n_next, %loop.body ]
  %idx = phi i16 [ %idx_init, %entry ], [ %idx_next, %loop.body ]
  %acc = phi i16 [ 0, %entry ], [ %acc_next, %loop.body ]
  %cmp_n = icmp ne i16 %n, 0
  %cmp_idx = icmp sgt i16 %idx, 0
  %and = and i1 %cmp_n, %cmp_idx
  br i1 %and, label %loop.body, label %loop.exit

loop.body:
  %v = call i16 @produce()
  %acc_next = add i16 %acc, %v
  %n_next = sub i16 %n, 1
  %idx_next = sub i16 %idx, 1
  br label %loop.header

loop.exit:
  ret i16 %acc
}

; CHECK-LABEL: name: short_circuit_and_with_phi

; The fix: the i1-mediated brcond materialisation through ConditionalImm
; emits a fused CompareBranch_*_* / TestBranch_*_*, not the separated
; Compare_*_* + ConditionalLongBranchRelative. So we must NOT see a
; bare ConditionalLongBranchRelative anywhere in this function.
; CHECK-NOT: ConditionalLongBranchRelative
