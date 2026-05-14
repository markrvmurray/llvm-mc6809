# MC6809 / HD6309 — accumulator-hierarchy `Defs` over-claim audit

*Bug #272 Phase B sub-task — preparation for the un-over-claim pass.*

*Started 2026-05-14. Sister to `MC6809-SpillQ-AQc-Audit.md`.*

## Background

On Phase B's last reproduction
(see `MC6809-SpillQ-AQc-Audit.md` Addendum 3) the regalloc-time
verifier tripped because `LDD`'s `Defs = [..., AQ]` claims a **full**
write of the 32-bit `AQ` register, even though the physical instruction
writes only `AA:AB` (= `AD`, the low half of `AQ`); `AW` (the high
half) is preserved.

That mismatch is harmless when `AQc = (add AQ)` (only one allocatable
member, so nothing else can live in `$aq` to be killed).  It becomes
load-bearing the moment Phase B widens `AQc` to a real allocation
target: regalloc can't keep an i32 in `$aq` across any LDD-using path,
because every LDD claims to fully redefine `AQ`.

The same shape repeats throughout the accumulator hierarchy:

  * `AQ` = (`AD`:`AW`)  =  `(AA:AB):(AE:AF)`.
  * Any 8-bit op writing `AA` partially modifies `AD`, partially modifies `AQ`.
  * Any 16-bit op writing `AD` partially modifies `AQ`.
  * Same for `AB`, `AE`, `AF`, `AW`.

LLVM expresses partial-write semantics naturally via sub-register
aliasing on the OutOperand class (e.g., `AD` listed as the result class
implicitly partial-modifies `AQ`).  Adding the super-reg to `Defs`
**explicitly** declares a full write, which is the over-claim.

## Audit scope

Every `.td` line in
`llvm/lib/Target/MC6809/MC6809{InstrInfo,InstrFamilies,InstrLogical,InstrPseudos}.td`
that lists `AQ`, `AD`, or `AW` in its `Defs` list.

For each entry, classify:

  1. **Physical write set** (what does the hardware actually write?
     from MC6809 + HD6309 manuals).
  2. **Listed `Defs`** (verbatim from the `.td`).
  3. **Verdict**: `correct` if every listed reg is fully written, or
     `over-claim: X, Y, ...` listing the registers that are listed
     but only partially written (or not written at all).

The verdict drives the un-over-claim pass: only over-claims are
dropped; correct entries are left untouched.

## Catalogue

Counts in parentheses are the number of concrete addressing-mode
forms emitted by the `defm` (typically 4: `i8/i16` immediate, `d`
direct, `i` indexed, `e` extended).  All forms of a `defm` share the
same `Defs` list, so the verdict applies to all of them.

### Loads — `MC6809InstrFamilies.td`

| Op    | Physical write | Listed `Defs`                    | Verdict |
|-------|----------------|----------------------------------|---------|
| `LDA` (×4)  | `AA`            | `[NZ,N,Z,V,AA,AD,AQ]`            | over-claim: `AD`, `AQ` |
| `LDB` (×4)  | `AB`            | `[NZ,N,Z,V,AB,AD,AQ]`            | over-claim: `AD`, `AQ` |
| `LDD` (×4)  | `AA`, `AB` (= `AD`) | `[NZ,N,Z,V,AA,AB,AD,AQ]`     | over-claim: `AQ` |
| `LDX` (×4)  | `IX`            | `[NZ,N,Z,V,IX]`                  | correct |
| `LDU` (×4)  | `SU`            | `[NZ,N,Z,V,SU]`                  | correct |
| `LDE` (×4)  | `AE`            | `[NZ,N,Z,V,AE,AW,AQ]`            | over-claim: `AW`, `AQ` |
| `LDF` (×4)  | `AF`            | `[NZ,N,Z,V,AF,AW,AQ]`            | over-claim: `AW`, `AQ` |
| `LDY` (×4)  | `IY`            | `[NZ,N,Z,V,IY]`                  | correct |
| `LDS` (×4)  | `SS`            | `[NZ,N,Z,V,SS]`                  | correct |
| `LDW` (×4)  | `AE`, `AF` (= `AW`) | `[NZ,N,Z,V,AE,AF,AW,AQ]`     | over-claim: `AQ` |
| `LDQi32` `LDQd` `LDQi` `LDQe` | full `AQ` | `[NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ]` | correct |

### Stores — `MC6809InstrFamilies.td`

All stores (`STA`/`STB`/`STD`/`STX`/`STU`/`STE`/`STF`/`STY`/`STS`/`STW`)
use `Defs = [NZ,N,Z,V]` only (no accumulator listed).
**No over-claim.**

### 8-bit ALU A — `MC6809InstrInfo.td:296-314`

| Op    | Physical write | Listed `Defs`                      | Verdict |
|-------|----------------|------------------------------------|---------|
| `SUBA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,C,AA,AD,AQ]`       | over-claim: `AD`, `AQ` |
| `CMPA` (×4) | CC only           | `[NZ,N,Z,V,C]`                   | correct |
| `SBCA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,C,AA,AD,AQ]`       | over-claim: `AD`, `AQ` |
| `ANDA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,AA,AD,AQ]`         | over-claim: `AD`, `AQ` |
| `BITA` (×4) | CC only           | `[NZ,N,Z,V]`                     | correct |
| `EORA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,AA,AD,AQ]`         | over-claim: `AD`, `AQ` |
| `ADCA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,C,AA,AD,AQ]`       | over-claim: `AD`, `AQ` |
| `ORA`  (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,AA,AD,AQ]`         | over-claim: `AD`, `AQ` |
| `ADDA` (×4) | `AA`, CC          | `[AA,NZ,N,Z,V,C,AA,AD,AQ]`       | over-claim: `AD`, `AQ` |

### 8-bit ALU B — `MC6809InstrInfo.td:317-334`

Same shape as ALU A but with `AB` substituted for `AA`.

| Op    | Physical write | Verdict |
|-------|----------------|---------|
| `SUBB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `CMPB` (×4) | CC       | correct |
| `SBCB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `ANDB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `BITB` (×4) | CC       | correct |
| `EORB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `ADCB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `ORB`  (×4) | `AB`, CC | over-claim: `AD`, `AQ` |
| `ADDB` (×4) | `AB`, CC | over-claim: `AD`, `AQ` |

### 16-bit ALU D — `MC6809InstrInfo.td:337-340,839-850`

`AD` is fully written by these ops, so listing `AD` (and `AA`, `AB` as
sub-regs) in `Defs` is correct.  Only `AQ` is over-claimed.

| Op    | Physical write | Listed `Defs`                                  | Verdict |
|-------|----------------|------------------------------------------------|---------|
| `SUBD` (×4) | `AD` (= `AA:AB`), CC | `[AD,NZ,N,Z,V,C,AA,AB,AD,AQ]`             | over-claim: `AQ` |
| `ADDD` (×4) | same                 | same                                       | over-claim: `AQ` |
| `CMPX` (×4) | CC                   | `[NZ,N,Z,V,C]`                             | correct |
| `CMPD` (×4) | CC                   | `[NZ,N,Z,V,C]`                             | correct |
| `CMPY` (×4) | CC                   | `[NZ,N,Z,V,C]`                             | correct |
| `CMPU` (×4) | CC                   | `[NZ,N,Z,V,C]`                             | correct |
| `CMPS` (×4) | CC                   | `[NZ,N,Z,V,C]`                             | correct |
| `SBCD` (×4) | `AD`, CC             | `[AD,NZ,N,Z,V,C,AA,AB,AD,AQ]`              | over-claim: `AQ` |
| `ANDD` (×4) | `AD`, CC             | `[AD,NZ,N,Z,V,AA,AB,AD,AQ]`                | over-claim: `AQ` |
| `BITD` (×4) | CC                   | `[NZ,N,Z,V]`                               | correct |
| `EORD` (×4) | `AD`, CC             | `[AD,NZ,N,Z,V,AA,AB,AD,AQ]`                | over-claim: `AQ` |
| `ADCD` (×4) | `AD`, CC             | `[AD,NZ,N,Z,V,C,AA,AB,AD,AQ]`              | over-claim: `AQ` |
| `ORD`  (×4) | `AD`, CC             | `[AD,NZ,N,Z,V,AA,AB,AD,AQ]`                | over-claim: `AQ` |

### 16-bit ALU W — `MC6809InstrInfo.td:855-860`

`AW` is fully written by these ops; only `AQ` is over-claimed.

| Op    | Physical write | Verdict |
|-------|----------------|---------|
| `SUBW` (×4) | `AW` (= `AE:AF`), CC | over-claim: `AQ` |
| `CMPW` (×4) | CC                   | correct |
| `ADDW` (×4) | `AW`, CC             | over-claim: `AQ` |

### 8-bit ALU E (HD6309) — `MC6809InstrInfo.td:864-869`

| Op    | Physical write | Verdict |
|-------|----------------|---------|
| `SUBE` (×4) | `AE`, CC | over-claim: `AW`, `AQ` |
| `CMPE` (×4) | CC       | correct |
| `ADDE` (×4) | `AE`, CC | over-claim: `AW`, `AQ` |

### 8-bit ALU F (HD6309) — `MC6809InstrInfo.td:872-877`

| Op    | Physical write | Verdict |
|-------|----------------|---------|
| `SUBF` (×4) | `AF`, CC | over-claim: `AW`, `AQ` |
| `CMPF` (×4) | CC       | correct |
| `ADDF` (×4) | `AF`, CC | over-claim: `AW`, `AQ` |

### Unary accumulator A/B (Page 1) — `MC6809InstrInfo.td:380-410`

`NEG`/`COM`/`LSR`/`ROR`/`ASR`/`ASL`/`ROL`/`DEC`/`INC`/`CLR` each have
both `aA`-form (writes `AA`) and `bB`-form (writes `AB`).  All
over-claim `AD` and `AQ` in `Defs`.

| Op | Physical write (per form) | Verdict (per form) |
|----|---------------------------|--------------------|
| `NEG{A,B}` `COM{A,B}` `LSR{A,B}` `ROR{A,B}` `ASR{A,B}` `ASL{A,B}` `ROL{A,B}` `DEC{A,B}` `INC{A,B}` `CLR{A,B}` | `AA` (or `AB`), CC | over-claim: `AD`, `AQ` |

### Unary D accumulator (Page 2, HD6309) — `MC6809InstrInfo.td:881-891`

`NEGDa` `COMDa` `LSRDa` `RORDa` `ASRDa` `ASLDa` `ROLDa` `DECDa` `INCDa`
`CLRDa` (×1 each, inherent form only).  All write `AD` (= `AA:AB`) and
CC; only `AQ` is over-claimed.

| Op | Verdict |
|----|---------|
| each `*Da` | over-claim: `AQ` |

### Unary W accumulator (Page 2, HD6309) — `MC6809InstrInfo.td:892-899`

| Op | Verdict |
|----|---------|
| `COMWa` `LSRWa` `RORWa` `ROLWa` `DECWa` `INCWa` `CLRWa` | over-claim: `AQ` |

### Unary E accumulator (Page 3, HD6309) — `MC6809InstrInfo.td:904-908`

| Op | Verdict |
|----|---------|
| `COMEa` `DECEa` `INCEa` `CLREa` | over-claim: `AW`, `AQ` |

### Unary F accumulator (Page 3, HD6309) — `MC6809InstrInfo.td:909-913`

| Op | Verdict |
|----|---------|
| `COMFa` `DECFa` `INCFa` `CLRFa` | over-claim: `AW`, `AQ` |

### Inherent ops — `MC6809InstrInfo.td:938-1006`

| Op       | Physical write | Listed `Defs`                                | Verdict |
|----------|----------------|----------------------------------------------|---------|
| `DAAx`   | `AA`, CC       | `[AA,NZ,N,Z,C,AA,AD,AQ]`                     | over-claim: `AD`, `AQ` |
| `SEXx`   | `AA` (sign of `AB` extended; `AB` is unchanged) | `[NZ,N,Z,AA,AB,AD,AQ]` | over-claim: `AB`, `AD`, `AQ` |
| `MULx`   | `AA`, `AB` (= `AD` full), Z, C | `[Z,C,AA,AB,AD,AQ]`           | over-claim: `AQ` |
| `DIVDi8` `DIVDd` | `AA`, `AB` (= `AD` full), CC | `[NZ,N,Z,V,C,AA,AB,AD,AQ]`   | over-claim: `AQ` |
| `DIVQi16` `DIVQd` | full `AQ`, CC | `[NZ,N,Z,V,C,AA,AB,AD,AE,AF,AW,AQ]`         | correct |
| `MULDi16` | full `AQ`, NZ flags | `[NZ,N,Z,AA,AB,AD,AE,AF,AW,AQ]`           | correct |

### Pseudo — `MC6809InstrLogical.td:258-263`

| Op | Listed `Defs` | Verdict |
|----|---------------|---------|
| `BranchJumpTable` | `[AA, AB, AD, AQ, IX, NZ, N, Z, V, C]` | over-claim: `AQ` |

`BranchJumpTable` expansion is `ASLB+ROLA+LEAX+LDD+JMP`.  The LDD in
the chain has the same `AQ` over-claim as a standalone LDD; the
pseudo inherits it.

## Summary of over-claims

* **`AQ` over-claim** is present on every accumulator-hierarchy
  instruction that physically writes a strict subset of `AQ` (i.e.,
  one byte: AA/AB/AE/AF; or one word: AD/AW).  Total: ~110 concrete
  TableGen entries (counting each addressing-mode form separately).

* **`AD` over-claim** is present on 8-bit ops that physically write
  just `AA` or just `AB` (the byte halves of AD).  Total: ~70 entries
  (LDA/LDB + 7 ALU A + 7 ALU B + 10 unary AB pair + DAA + SEX).

* **`AW` over-claim** is present on 8-bit ops that physically write
  just `AE` or just `AF` (the byte halves of AW).  Total: ~30 entries
  (LDE/LDF + 2 ALU E + 2 ALU F + 4 unary E + 4 unary F).

* **`AB` over-claim** on SEX (1 entry) — SEX physically writes only
  `AA`; `AB` is unchanged.

* **Index-register ops** (`LDX`/`LDU`/`LDY`/`LDS`/`CMPX`/etc.) are
  **correct** — those registers have no super-reg, so listing them in
  `Defs` is exactly right.

* **Stores** are **correct** — they don't list accumulators in `Defs`.

* **`LDQ`, `DIVQ`, `MULD`** are **correct** — they physically write
  the full `AQ`.

## Proposed un-over-claim scope

The next session must pick from three concentric scopes:

### Scope A — `AQ`-only un-over-claim (minimum for Phase B)

Remove `AQ` from `Defs` everywhere it's listed as an over-claim.

* **What this fixes:** Phase B's regalloc-time verifier failure on
  `bug274_loadi32_qspill_dst_implicit_def.ll`: LDDs no longer claim
  to fully redefine `$aq`, so an i32 kept in `$aq` survives an
  intervening LDD.

* **What this doesn't fix:** the tighter over-claims on `AD` from
  LDA/LDB/SEX/etc., and on `AW` from LDE/LDF.  Those are correct
  for narrower future allocator targets (e.g., allowing an i16 to
  stay in `$ad` across an LDA-using path) but not load-bearing for
  Phase B.

* **Risk:** Smallest.  Touches only the AQ entries in the Defs list
  — sub-reg aliasing on the OutOperand handles partial-modification
  of AQ.  Each `defm` is one mechanical edit.

* **Validation:** Lit 105/105 must hold (incl. `bug271/`, `bug274/`,
  `bug275/`, `bug285/` sentinels — all live on Defs widening, so they
  also live on Defs narrowing).  Full 28-level bench, watching for
  HD6309-mame regressions where the wider accumulator hierarchy is
  hottest.

### Scope B — `AQ` + `AD`/`AW` un-over-claim (architectural cleanup)

Scope A plus: remove `AD` from LDA/LDB/SEX/ALU-A/ALU-B/unary-A-or-B/DAA;
remove `AW` from LDE/LDF/ALU-E/ALU-F/unary-E/unary-F.

* **What this enables:** future-proofing for an i16-in-AD path that
  Phase C might want (analogous to the i32-in-AQ Phase B wants).

* **What this doesn't fix:** SEX's over-claim of `AB`.  That's a
  separate (single-entry) fix.

* **Risk:** Larger — the `AD` over-claim has been load-bearing on
  some MaterializeSpills paths (Bug #271 series, Bug #275); the
  HD6309-mame matrix has been validated against the over-claim
  behaviour and may regress.

### Scope C — Full physical correctness

Scope B plus: remove `AB` from SEX.

* **Risk:** Trivial additional risk on top of Scope B — SEX is
  uncommon and the bug ("SEX doesn't actually write `AB`") is
  uncontroversial.

## Recommendation

**Scope A**.  Single concern, single mechanical pass, single
validation cycle.  Scopes B and C are good follow-ups but should
land separately so each piece has its own bisect anchor.

This mirrors Bug #290 → Bug #291: a small structural change first,
then the bigger architectural sweep, each landing clean.

## Next-session action items

1. Confirm the catalogue above against the actual `.td` lines (a
   second-pair-of-eyes pass to make sure no over-claim was missed
   and no `correct` entry was misclassified).
2. Land Scope A as a single commit.  Lit + 28-level bench.
3. File follow-up bug for Scope B (and Scope C as a piggyback) so
   the architectural cleanup doesn't get forgotten.

Working tree clean at end of audit; no `.td` edits applied here.
