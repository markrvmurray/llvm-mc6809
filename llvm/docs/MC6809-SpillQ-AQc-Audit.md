# MC6809 / HD6309 — AQc / ACC32 implicit-def audit

*Bug #272 Phase B preparation, 2026-05-14.*

## Goal

Catalog every pseudo in the MC6809 backend that produces or consumes
an `AQc` or `ACC32` operand.  For each, document:

  1. Current `Defs = [...]` list.
  2. The pseudo's post-RA expansion behaviour (from
     `MC6809InstrInfo.cpp::expandLoadIdx` / `expandStoreIdx` /
     `expandPostRAPseudo`).
  3. What gets clobbered for **each** possible destination
     (AQ direct vs SPILL_Q*N via two-LDD path).
  4. Whether the current Defs list stays correct when `AQc` widens
     to include SPILL_Q*N — the Phase B core change.

This audit is the implementation-session prep: no code changes,
just the catalog so the next session has a concrete checklist.

## Why AQc widening is the core Phase B change

Bug #208 round 4 (2026-04) deliberately narrowed `Load_i32` and
`Store_i32`'s register-class constraint to `AQc = (add AQ)`,
specifically so that:

  - Regalloc knows the result lives in `$aq`.
  - Sub-register aliasing on the explicit `$dst` operand covers
    `$ad` / `$aw` / `$aa` / `$ab` / `$ae` / `$af` clobbers
    automatically.
  - No need to enumerate them in `Defs` — the OutOperand declaration
    is enough.

That design held because **AQc had exactly one register** and
sub-register aliasing did all the work.  Bug #265 / #296 wanted to
re-enable native `LDQ` / `STQ` emission for `s32` G_LOAD / G_STORE
(the original Bug #272 Phase B goal), which requires regalloc to
have spill recourse for `AQc` — which means widening `AQc` to
`(add AQ, SPILL_Q0..31)`.

When `AQc` widens, the sub-register-aliasing trick stops working
for the SPILL_Q*N case.  SPILL_Q*N have no sub-register
relationship with `$ad` (they're stack-frame placeholders); the
post-RA expansion uses the Bug #221 two-LDD path which clobbers
`$ad`.  That clobber is invisible to regalloc unless the pseudo's
`Defs` list says so explicitly.

The fix is symmetric with what `SpillLoad_i32_Mem` /
`SpillStore_i32_Mem` already do: list `AD` in `Defs` so regalloc
knows it's clobbered regardless of which destination is picked.

## Catalog

### AQc-using pseudos (these change when AQc widens)

| Pseudo | Defined in | dst/src class | Current `Defs` | Sub-reg covers AD/AW when? | Action after AQc widen |
|---|---|---|---|---|---|
| `Load_i32_Imm` | `InstrFamilies.td:40` (via `MC6809LoadBase`) | dst:AQc | `[NZ, V]` | only when dst lands on `$aq` | **Add `AD` to Defs**.  When dst is SPILL_Q*N, expansion uses two-LDD path (Bug #221) which clobbers `$ad`. |
| `Load_i32_Mem` | `InstrFamilies.td:58` | dst:AQc | `[NZ, V]` | same | same |
| `Store_i32_Imm` | `InstrFamilies.td:197` (via `MC6809StoreBase`) | src:AQc | `[NZ, V]` | only when src is `$aq` | **Add `AD` to Defs**.  When src is SPILL_Q*N, expansion uses two-LDD path which clobbers `$ad`. |
| `Store_i32_Mem` | `InstrFamilies.td:197` | src:AQc | `[NZ, V]` | same | same |

### ACC32-using pseudos (already include SPILL_Q*N; unaffected by AQc widen)

| Pseudo | Defined in | dst/src class | Current `Defs` | Notes |
|---|---|---|---|---|
| `SpillLoad_i32_Mem` | `InstrFamilies.td:83` | dst:ACC32 | `[NZ, V, AD]` | Bug #271 cat-3 + Bug #290 trim.  Already correct. |
| `SpillStore_i32_Mem` | `InstrFamilies.td:207` | src:ACC32 | `[NZ, V, AD]` | Same. |
| `SEX32Implicit` | `InstrLogical.td:411` | src:ACC16, dst:ACC32 | `[NZ, V, AD, AW]` | Bug #247 + Bug #283 + Bug #284.  Already covers `$ad` and `$aw` for the SPILL_Q* dst case. |
| `ZEX32Implicit` | `InstrLogical.td:480` | src:ACC16, dst:ACC32 | `[NZ, V, C, AD, AW]` | Bug #161 + Bug #208 + Bug #284.  Already correct. |
| `EXTRACT_LO_word_i32` | `InstrPseudos.td:159` | dst:ADc, src:ACC32 | (none explicit) | Inherits from `MC6809Pseudo`.  Post-RA: AQ source → `TFR W,D` (no clobber beyond `$ad`); SPILL_Q source → `LDD slot+2,U` (sets NZ/V, defines $ad).  Verify `[NZ, V]` is inherited from the base class. |
| `EXTRACT_HI_word_i32` | `InstrPseudos.td:167` | dst:ADc, src:ACC32 | (none explicit) | Symmetric.  Same audit. |
| `PSHSWxImplicit` | `InstrPseudos.td` | src:ACC32 | TBD | Outside the AQc-widening blast radius since src is already `ACC32`. |
| `PULSWxImplicit` | `InstrPseudos.td` | dst:ACC32 | TBD | Same. |

## Why only AD, not AW

Verified against `MC6809InstrInfo.cpp::expandLoadIdx` lines 4456–4489
(the SPILL_Q* dst branch under `isQSpillReg`):

> Instead, copy the i32 source via TWO LDD (16-bit) loads — one
> for each half of the dest spill slot. This touches only AD
> (which is the SpillDSaveRestore pass's responsibility — it
> saves/restores AD around any expansion that needs it). AW
> and the other AQ sub-registers stay untouched, preserving any
> live value regalloc allocated there.

The expansion code uses only `MC6809::AD` as the staging register
(both LDD operations write to `AD`; both STD operations read from
`AD`).  `AW` is never touched.

This matches **Bug #290's deliberate trim** of `SpillLoad_i32_Mem` /
`SpillStore_i32_Mem` from `[NZ, V, AA, AB, AE, AF, AD, AW]` down to
`[NZ, V, AD]`.  The wider list was over-claim that hurt regalloc's
clobber-driven re-evaluation (Bug #290 commit body explicit on this
point).

So the audit's `Defs = [NZ, V, AD]` recommendation is minimal AND
consistent with the existing sibling pseudos' precedent.

## Implementation plan

### Phase 2a — Defs extensions (low-risk)

Two `Defs` extensions in `MC6809InstrFamilies.td`.  Naive form:

```diff
 class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
-  let Defs = [NZ, V];
+  let Defs = [NZ, V, AD];        // AD for SPILL_Q*N two-LDD path
   ...
 }

 class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
-  let Defs = [NZ, V];
+  let Defs = [NZ, V, AD];        // same rationale
   ...
 }
```

**Caveat**: this widens `Defs` for `Load_i8 / Load_i16 / Load_iPtr /
Store_i8 / Store_i16 / Store_iPtr` too (they share the same base
class).  That's over-claim for those — they don't clobber `$ad`.
The user-visible cost is regalloc may spill more conservatively
around them.

**Recommended cleanup**: split the base into two TableGen base
classes — `MC6809LoadBaseAQ` / `MC6809StoreBaseAQ` for the AQc /
ACC32 variants (with the AD-extended `Defs`), and the existing
class for everything else (kept at `[NZ, V]`).

### Phase 2b — verify Bug #274 sentinels turn green

After Phase 2a, re-apply the two pieces from the failed retry:

  - The legalizer change (G_LOAD / G_STORE s32 legal on HD6309 —
    `MC6809LegalizerInfo.cpp` clampScalar bump from `s16` to `sMax`).
  - The AQc widening (`def AQc : MC6809Reg32Class<(add AQ, SPILL_Q0..31)>;`
    in `MC6809RegisterInfo.td`).

Then run the previously-failing tests:

  - `bug274_loadi32_qspill_dst_implicit_def.ll`
  - `bug274_spillstore_qspill_src_two_ldd.ll`

If they pass, the audit's hypothesis was right.  If they don't,
there's another layer beneath — likely in the post-RA expansion
code itself (`expandLoadIdx` / `expandStoreIdx` Bug #221 two-LDD
branch may need its own MIR implicit-def updates).

### Phase 2c — handle the two sentinel-update tests

  - `bug221_ldq_clobbers_aw.ll`: tighten `CHECK-NOT` to forbid
    only the spill-materialization LDQ pattern (the new LDQ for
    i32 load-from-stack-slot AFTER a function call is the legitimate
    Phase B win, not the Bug #221 hazard).
  - `test_return_i32_constant.ll`: update `CHECK` lines for the new
    LDQ-based return codegen.

### Phase 2d — verify EXTRACT_LO/HI / PSHSWx / PULSWx

Read those pseudos' expansions and confirm their `Defs` lists are
sound under the wider class.  If any need updates, add them.

### Phase 2e — bench validation

HD6309-only bench subset (`O2-hd6309-mame` + `Os-hd6309-mame` +
`Os-lto-hd6309-mame` and their FP variants, ~10 min wall) — verify
cycle counts drop on i32-heavy paths, no FAIL regressions.  Then
full 28-level bench (~35 min) for confirmation.

### Phase 2f — commit

Single commit landing all of: Phase 2a + the legalizer change +
the AQc widening + sentinel updates + any 2d additions.

## Estimated effort

Pre-audit estimate (probe 3 conclusion): 2–3 sessions.

Post-audit estimate: **1 focused session** if Phase 2a's hypothesis
holds.  If not, another investigation session for the deeper
expansion-code issue.

## What this audit did NOT cover

  - The actual TableGen IDs assigned to new SPILL_Q*N — already
    landed in Bug #296 Phase 2 (commit `9810fddb7ca9`).
  - Bench-cycle deltas — those come in Phase 2e.
  - Stress-testing on `strtol` / `asctime` / `snprintf` specifically
    — the historical risk loci (Bug #208 round 4 / Bug #210 round 5).
    These must be verified pass under the new model before closing
    Bug #272 Phase B.

## Cross-references

  - Bug #272 (open, this audit's subject) — comment thread on the
    git-bug carries the chronology of three probes plus this audit.
  - Bug #296 (closed) — Phase 2 SPILL_Q bump that this audit
    builds on.
  - Bug #271 cat-3 / Bug #274 / Bug #285 implicit-def lineage —
    precedent for this kind of audit work.
  - Bug #221 two-LDD path documented in `MC6809InstrInfo.cpp`
    `expandLoadIdx` SPILL_Q* dst branch.
  - Bug #290 — the deliberate trim of `SpillLoad_i32_Mem` /
    `SpillStore_i32_Mem` `Defs` to `[NZ, V, AD]`.

## Addendum 2026-05-14 (late): Phase 2a attempted, hypothesis falsified

The Phase 2a hypothesis above ("add `AD` to `Load_i32_Imm` /
`Load_i32_Mem` / `Store_i32_Imm` / `Store_i32_Mem` `Defs`")
**does not hold** as a standalone change.  Empirical result:

  - With AQc still AQ-only (pre-widening), adding `implicit-def
    dead $ad` to `Load_i32_Imm`'s post-isel MIR breaks the
    downstream `EXTRACT_LO_word_i32 %1:aqc` consumer.  At -O0
    FastRegAlloc reports "ran out of registers" for the simplest
    possible function (`define i32 @foo() { ret i32 12345678 }`).

The root cause is more subtle than the audit captured: the
OutOperand `$dst:AQc = $aq` makes `$ad` live across the
instruction (via sub-register aliasing of `$aq`), but adding
`implicit-def dead $ad` creates a competing "dead at this point"
live-range for `$ad`.  The two declarations contradict each other.
For dst=AQ specifically, **`$ad` is live (because it's part of
`$aq`)**, not dead.  Listing it in `Defs` with the "dead" flag
(which LLVM infers from "no live use") is wrong for this case.

When AQc widens to include SPILL_Q*N (the Phase B core change),
dst=SPILL_Q*N would have no sub-register aliasing with `$ad`, so
the dead-`$ad` annotation would NOT conflict for that case.  But
the AQ-direct case remains broken.

**Implication**: Phase 2a cannot stand alone.  Phase B's three
changes (legalizer + AQc widening + Defs extension) might still
land together — but if even the post-widening AQ case suffers
the same dead-vs-live contradiction, the fix isn't a pseudo-level
`Defs` extension at all.

### Better hypothesis (untested as of this addendum)

Drop the pseudo-level Defs change entirely.  Move the `$ad`
clobber annotation to the **post-RA expansion code** in
`MC6809InstrInfo.cpp::expandLoadIdx` /
`expandStoreIdx`'s SPILL_Q*N branch.  The expansion emits the
two-LDD path; those individual LDD/STD MIs already write `$ad`
naturally.  The verifier error in
`bug274_loadi32_qspill_dst_implicit_def.ll` might be fixable by
adjusting how the expansion chains implicit-defs across the
emitted MIs, not by changing the pseudo's `Defs`.

This is now the next-session starting point.  The diagnosis lands
in this doc; the implementation is a separate, more careful
session focused on `expandLoadIdx` / `expandStoreIdx` internals.

### What was committed in this attempt

- Audit doc itself (`e4bb9960278d`).
- Phase 2a TableGen change (Defs extension) was reverted before
  commit; no working-tree residue.
