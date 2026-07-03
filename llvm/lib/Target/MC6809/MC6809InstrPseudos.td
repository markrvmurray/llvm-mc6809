//===-- MC6809InstrPseudos.td - Describe MC6809 Pseudo-Instrs ---*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MC6809 pseudo-instructions.
//
// Pseudo-instructions allow deferring generation of specific instruction
// sequences from one pass until a later pass. This is useful whenever an early
// pass is required to make a decision that it doesn't have the information
// necessary to do well.
//
//===---------------------------------------------------------------------===//

class MC6809Pseudo : Instruction {
  let Namespace = "MC6809";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
}

//===---------------------------------------------------------------------===//
// Post RA Pseudos
//===---------------------------------------------------------------------===//
// These pseudoinstructions defer decisionmaking until after register
// allocation. They're used whenever the best instruction sequence to emit
// depends only on the placement of their arguments and the set of live
// registers before and after the pseudo.
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Soft stack
//===---------------------------------------------------------------------===//

// The call-frame pseudoinstructions form a signalling mechanism to pass the
// size of outgoing call frames from calling-convention lowering (in
// IRTranslator) through to the stack frame prolog-epilog insertion. The latter
// examines these pseudoinstructions to find the largest size that an outgoing
// call frame can be, then preallocates that much space to the stack frame if
// possible. This allows these pseudoinstructions to be summarily removed in
// most cases, since we can instead guarantee that a reserved region at the
// bottom of the stack will always have enough free space to set up the
// arguments of any outgoing call.
class MC6809CallFramePseudo : MC6809Pseudo {
  dag InOperandList = (ins i16imm:$bytes, i16imm:$prior);

  let Defs = [SS];
  let Uses = [SS];
}

// Call frame setup pseudo.
def ADJCALLSTACKDOWN : MC6809CallFramePseudo;

// Call frame destroy pseudo.
def ADJCALLSTACKUP : MC6809CallFramePseudo;

// Use the diamond control flow pattern to load one of two immediates
// based on the value of a byte flag.  $tst is ACC8 holding 0 or 1;
// the custom inserter tests bit 0.
def SelectImm : MC6809Pseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins ACC8:$tst, i1imm:$true, i1imm:$false);

  let usesCustomInserter = true;
}

// Diamond control flow: load one of two immediates based on a
// conditional (same conditional encoding as Bbc).  $dst is ACC8 with
// the byte's LSB carrying the boolean result.  Bare G_ICMP s1
// (which arrives at -O0 without an enclosing anyext) is intercepted
// in the selector and lowered to this pseudo manually.
def ConditionalImm : MC6809Pseudo {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins condcode:$cc, CCond:$tst, i1imm:$true, i1imm:$false);

  let usesCustomInserter = true;
}

//===---------------------------------------------------------------------===//
// Byte/word composition pseudos (bug #118 Layer 1, approach b).
//
// These replace virtual-register sub-register COPY edges between ACC16 and
// ACC8 that would otherwise be emitted as INSERT_SUBREG/EXTRACT_SUBREG or
// REG_SEQUENCE. Using opaque pseudos means the coalescer never sees a
// cross-class sub-register edge — it can only be rewritten post-RA, which
// is safe because regalloc has already chosen physical registers.
//
// Operand classes carry no sub-reg indices. Expansion in
// MC6809InstrInfo.cpp::expandPostRAPseudo resolves the 16-bit operand's
// byte halves via TRI->getSubReg and emits LDA/LDB/TFR as needed.
//===---------------------------------------------------------------------===//

// Defs cover CC-flag side effects of the LDA/LDB used during materialisation.
// AA/AB are NOT listed here even though the post-RA expansion may stage
// through them, because they are members of the out-operand classes
// (ACC8 / ADc). Listing them as implicit defs while also declaring the
// out in the same register set double-counts the class's pressure delta,
// which trips the pre-RA MachineScheduler's RegisterPressure invariant
// (RegisterPressure.cpp:1172 "PSet overflow/underflow"). Post-RA, the
// expanded sequence carries its own explicit Defs on LDA/LDB/TFR/etc.,
// so correctness is maintained after expansion.
let Defs = [NZ, V] in {
  // $dst is ACC8_AB ({AB, AA}). The natural assignment when $src sits in AD
  // is the matching half (AB for LO, AA for HI — a no-op extract); a
  // cross-half dst costs one TFR, routed through copyPhysReg by the
  // expansion. The historical single-half dst classes (ABc for LO / AAc for
  // HI, bug #118 Layer 1 approach b) became unallocatable once ABc/AAc
  // shrank to genuine one-register classes: a byte value live in the half
  // across the extract left the dst with zero candidates ("ran out of
  // registers"; bug267/bug271_cat6 sentinels). The bug #118 hazard — a
  // cross-half TFR clobbering a sibling extract's byte — is regalloc-visible
  // through the declared dst def and the source's interference with a
  // still-live AD, so the wider class does not reintroduce it (guarded by
  // the bug118/bug267 lit tests and test-strchr on usim).
  def EXTRACT_LO_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC8_AB:$dst);
    dag InOperandList  = (ins ACC16:$src);
    let isAsCheapAsAMove = true;
  }

  // Symmetric. See EXTRACT_LO_i16 comment above.
  def EXTRACT_HI_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC8_AB:$dst);
    dag InOperandList  = (ins ACC16:$src);
    let isAsCheapAsAMove = true;
  }

  // Full ACC8 inputs (not ACC8_AB, and not the singleton ABc/AAc): the
  // expansion is fully assignment-driven -- loadByteInto stages any
  // source (real half, E/F, RC imaginary, RS byte half, byte spill) into
  // the right half of AD, including the full (lo=AA, hi=AB) swap via EXG
  // -- and two simultaneously-live distinct values can never share one
  // physical register at the same instruction, so the historical
  // both-in-AB hazard cannot arise. The old singleton constraint
  // propagated through the tied byte carry chains feeding the merge
  // (machine-cse folds any isolating COPY straight back), pinning whole
  // overlapping chains to one register; and even the two-register
  // ACC8_AB deadlocked once the byte spill pseudo-registers were
  // retired: a spilled source's INF-weight reload could find both AA
  // and AB blocked by physical interference, with no imaginary home
  // admitted to take it.
  // ACC16 dst (not ADc): the expansion routes the AD-staged result to
  // ANY real destination (TFR for AW) or dematerialises to an imaginary,
  // and an ADc dst pinned zext/merge-fed PHIs to {AD, RS0-3} -- with AD's
  // halves busy in a hot loop the value landed in an RS direct-page slot
  // and its per-iteration rematerialisation cost strcmp +52% cycles on
  // HD6309 (where AW sat free the whole loop).
  def MERGE_LOHI_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC16:$dst);
    dag InOperandList  = (ins ACC8:$lo, ACC8:$hi);
    let isAsCheapAsAMove = true;
  }

  // Memory-operand siblings of MERGE_LOHI_i16: one byte half comes from a
  // frame slot. foldMemoryOperandImpl rewrites a merge whose lo/hi operand
  // was spilled into these, so the spiller never needs a register for the
  // reload -- the INF-weight single-byte reload interval was the last
  // remaining greedy deadlock shape once the byte spill pseudo-registers
  // were retired (an fp-heavy merge point can have both AA and AB blocked
  // by physical interference). The expansion loads the register half into
  // its AD half via loadByteInto, then reads the spilled half straight
  // from the slot with LDA/LDB.
  def MERGE_LOHI_i16_MemLo : MC6809Pseudo {
    dag OutOperandList = (outs ACC16:$dst);
    dag InOperandList  = (ins ACC8:$hi, INDEX16:$idx, unknown:$offset);
    let mayLoad = true;
  }
  def MERGE_LOHI_i16_MemHi : MC6809Pseudo {
    dag OutOperandList = (outs ACC16:$dst);
    dag InOperandList  = (ins ACC8:$lo, INDEX16:$idx, unknown:$offset);
    let mayLoad = true;
  }
  // Result of folding a spilled FAKE_USE operand (see
  // foldMemoryOperandImpl): identical debug-lifetime semantics for the
  // remaining register operands, but mayLoad so the generic
  // foldMemoryOperand wrapper can attach the stack-slot memoperand it
  // insists on. Erased at post-RA expansion.
  def FakeUse_Mem : MC6809Pseudo {
    dag OutOperandList = (outs);
    dag InOperandList  = (ins variable_ops);
    let mayLoad = true;
  }

  // Both halves spilled: lo slot first, hi slot second.
  def MERGE_LOHI_i16_Mem2 : MC6809Pseudo {
    dag OutOperandList = (outs ACC16:$dst);
    dag InOperandList  = (ins INDEX16:$lidx, unknown:$loff,
                              INDEX16:$hidx, unknown:$hoff);
    let mayLoad = true;
  }

  // An i32 store whose VALUE lives in a stack slot: memory-to-memory copy
  // through the D accumulator (two LDD/STD pairs), never touching $aq.
  // foldMemoryOperandImpl folds a spilled Store_i32_Mem value operand into
  // this form; without it, the value round-trips slot -> $aq -> memory and
  // the reload's {AQ}-class interval collides with whatever neighbouring
  // i32 already holds $aq (the -Og llabs/imaxdiv "ran out of registers"
  // shape once the SPILL_Q escape registers were retired). The AD transit
  // is declared honestly — there is no ACC32 destination operand to cover
  // it by aliasing, and no self-interference since nothing here needs $aq.
  let Predicates = [IsHD6309], mayLoad = true, mayStore = true,
      Defs = [NZ, V, AD] in
  def Store_i32_SlotToMem : MC6809Pseudo {
    dag OutOperandList = (outs);
    dag InOperandList  = (ins INDEX16:$sidx, unknown:$soff,
                              INDEX16:$didx, unknown:$doff);
  }

  // The source byte becomes the low byte of the i16 result; high byte is
  // left undefined. ACC8 for the same reasons as MERGE_LOHI_i16 above;
  // the expansion stages any source into AB.
  def ANYEXT_i8_to_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC16:$dst);
    dag InOperandList  = (ins ACC8:$src);
    let isAsCheapAsAMove = true;
  }

  // Bug #161 round 14: extract the low word (sub_lo_word) of an ACC32
  // value into ADc. Replaces the SDAG `(EXTRACT_SUBREG ACC32, sub_lo_word)`
  // pattern. Post-RA expansion: AQ source → TFR W,D. A spilled source
  // folds to a one-word frame load in foldMemoryOperandImpl instead.
  def EXTRACT_LO_word_i32 : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let isAsCheapAsAMove = true;
  }

  // Symmetric: extract the high word (sub_hi_word = D for AQ).
  def EXTRACT_HI_word_i32 : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let isAsCheapAsAMove = true;
  }

  // Bug #302 redesign Phase 1 (2026-05-17): new cross-width i32↔i16
  // pseudos as replacements for EXTRACT_LO/HI_word_i32 and the
  // REG_SEQUENCE-with-sub_lo_word/sub_hi_word destination-assembly
  // pattern.  Same operand shape as the EXTRACT pseudos above —
  // opaque ACC32:$src input, ADc:$dst output — but new opcodes so
  // every emission site can be switched explicitly in Phase 2 (then
  // the EXTRACT_LO/HI_word_i32 pseudos can be deleted in Phase 5).
  //
  // The structural goal of the redesign: stop modelling AQ as a
  // sub-register-decomposable register in TableGen.  Cross-width
  // data flow goes through these pseudos, which carry NO sub-
  // register index in their operands.  After Phase 3 drops AQ's
  // `sub_lo_word` / `sub_hi_word` SubRegIndices, the TableGen
  // intersection-class synthesis stops producing the
  // `ACC32_with_sub_*` classes that collapse to `{AQ}` only
  // (Bug #302 mechanism).
  //
  // Phase 1 (this commit): pseudos defined, inert (no emission
  // sites yet).  Post-RA expansion (in MC6809InstrInfo.cpp)
  // initially mirrors EXTRACT_LO/HI_word_i32: AQ source → TFR
  // W,D / no-op.  Phase 3 rewrites the expansion to use literal
  // AD/AW physregs instead of `getSubReg(AQ, sub_lo_word)`.
  def Extract16_i32_lo : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let isAsCheapAsAMove = true;
  }
  def Extract16_i32_hi : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let isAsCheapAsAMove = true;
  }

  // Build a 32-bit value from two 16-bit halves.  Replaces the
  // REG_SEQUENCE-with-sub_lo_word/sub_hi_word destination-assembly
  // pattern currently emitted in selectMerge for s32 ← MERGE(s16
  // lo, s16 hi) and the 4-byte MERGE variant (which goes through
  // MERGE_LOHI_i16 then this pseudo).
  //
  // Output is opaque ACC32 — the result vreg has no sub-word-
  // touching constraints in its def chain.  This is the structural
  // fix for the Bug #302 root cause: REG_SEQUENCE-style sub-word
  // writes leak intersection-class demand onto the destination
  // vreg; an opaque Build32_i16i16 pseudo doesn't.
  //
  // Post-RA expansion: marshal lo into AW, hi into AD (via TFR or
  // copyPhysReg).  The inputs may already be in AD or other
  // ADc-class members; expansion handles the staging.
  def Build32_i16i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC32:$dst);
    // ACC16 (not ADc): the two sources are simultaneously live on ONE
    // instruction, so with AD as the only real ADc member the pair would be
    // structurally unallocatable. ACC16 lets the second value sit in AW or
    // an RS imaginary; the post-RA expansion stages any member through
    // AD/AW via copyPhysReg.
    dag InOperandList  = (ins ACC16:$lo, ACC16:$hi);
    // NO explicit AD/AW Defs: the destination is always the real $aq now
    // (the SPILL_Q slots are retired), and an $aq def covers both halves
    // by sub-register aliasing. The old Defs = [AD, AW] existed for the
    // SPILL_Q-destination staging clobber; kept on a real-$aq destination
    // they are a fixed physical dead-def at the pseudo's own def slot,
    // which vetoes assigning the dst vreg to $aq — with {AQ} the whole
    // class, that is a guaranteed "ran out of registers".
    let isAsCheapAsAMove = true;
  }

  // Bug #302 redesign Phase 2 (2026-05-17): anyext i16 → i32.  i16
  // input becomes the low word of the i32 result; the high word is
  // undefined.  Replaces the `INSERT_SUBREG (i32 (IMPLICIT_DEF)),
  // ACC16:$word, sub_lo_word` TableGen pattern that references AQ's
  // sub_lo_word index — that reference goes away when Phase 3 drops
  // AQ's sub-register hierarchy.
  //
  // Input class is ACC16 (wider than ADc — accepts AW too) because
  // the upstream G_ANYEXT may land its source in any ACC16-class
  // register.  Output is opaque ACC32 — same no-sub-word-constraint
  // property as Build32_i16i16.
  //
  // Post-RA expansion: copyPhysReg($aw, $src) for the low half;
  // high word ($ad) stays undefined.
  def AnyExt32_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ACC32:$dst);
    dag InOperandList  = (ins ACC16:$src);
    let isAsCheapAsAMove = true;
  }

  // Bug #301b Phase B (2026-05-16): native HD6309 i32 sign test.
  // Produces an i1 whose value is the sign bit of the i32 input
  // (1 if negative, 0 if non-negative).
  //
  // Replaces the __cmpsi2 + trunc i32→i8 + icmp eq 0 lowering for the
  // four sign-test predicates:
  //   icmp slt X, 0    →  SignTest_i32 X
  //   icmp sge X, 0    →  XOR (SignTest_i32 X) 1
  //   icmp sgt X, -1   →  XOR (SignTest_i32 X) 1     (sge-0 normalised)
  //   icmp sle X, -1   →  SignTest_i32 X             (slt-0 normalised)
  //
  // Why a target-specific pseudo instead of generic IR (G_UNMERGE_VALUES
  // + G_ICMP_i16 on the high half, attempted in the reverted Bug #301
  // fix attempt 2026-05-16): the generic-IR path selects to
  // EXTRACT_HI_word_i32 + CompareBranch_i16, which reads $aq's sub_hi_word
  // through the TableGen-synthesised ACC32_with_sub_hi_byte intersection
  // class — historically that collapsed when the source landed in a
  // SPILL_Q escape register (see feedback-bug301-option-a-dead-end).
  //
  // This pseudo bypasses the entire EXTRACT_*_word_i32 path: post-RA
  // expansion (in MC6809InstrInfo.cpp) works on the MSByte in AA, then
  // captures the sign bit into the ACC8 dst via the ASLA + LDA#0 +
  // ADCA#0 pattern.
  //
  // Output is ACC8 with the byte's LSB carrying the boolean result;
  // the post-RA expansion produces a 0/1 byte (high 7 bits cleared by
  // LDB #0, bit 0 set by ADCB).
  //
  // Defs cover CC flag clobber (ASLB sets C; LDB/ADCB set NZ) AND
  // AQ — the AQ-source expansion path uses `TFR A,B` to bring the
  // MSByte into AB for shifting, which CLOBBERS the AB sub-byte of
  // AQ.  Without declaring AQ in Defs, regalloc believed AQ was
  // preserved across SignTest_i32 and left live-range uses of the
  // i32 source vreg reading the corrupted AQ — manifesting as Bug
  // #304 (`fcvtf_r(-1.0)` returning "5039" instead of "10000",
  // because __ftoa_engine's subsequent STQ of "val" to __negsf2's
  // arg location stored corrupted bytes derived from the sign-bit
  // extraction).  Declaring AQ as a clobber forces regalloc to
  // materialise a copy of src when it's needed past SignTest_i32.
  def SignTest_i32 : MC6809Pseudo {
    dag OutOperandList = (outs ACC8:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let Defs = [NZ, V, C, AA, AW];
  }

  // Bug #301 (2026-05-16): native HD6309 i32 equal-to-zero test.
  // Produces an i1 whose value is 1 if the i32 is zero, else 0.
  //
  // Replaces __cmpsi2 + trunc i32→i8 + icmp eq 0 for the two zero-test
  // predicates:
  //   icmp eq X, 0    →  EqZero_i32 X
  //   icmp ne X, 0    →  XOR (EqZero_i32 X) 1
  //
  // Defs note (stock-spilling migration): the AQ scratch clobber is
  // declared as AA + AW — the parts of AQ DISJOINT from the $ab byte
  // the expansion leaves the result in (MC6809InsertCopies pins the
  // dst there and copies out). Declaring AQ itself put the result
  // register inside its own clobber set: the covering dead-AQ def at
  // the same slot as the $ab result def confused liveness downstream
  // (manifest: strtol test-TU miscompile at O2 HD6309). AA + AW +
  // the explicit $ab def cover every regunit of AQ, so the clobber
  // strength is unchanged.
  //
  // Post-RA expansion (in MC6809InstrInfo.cpp): OR the 4 source bytes
  // through AB then extract CC.Z into bit 0 of AB (TFR CC,B; ANDB #4;
  // LSRB; LSRB). AQ source uses the byte chain AA:AB:AE:AF via
  // ORR chain.  No EXTRACT_*_word_i32 emission — bypasses Bug #301a's
  // intersection-class collapse like SignTest_i32.
  //
  // Output is ACC8 with the byte's LSB carrying the boolean result;
  // the post-RA expansion produces a 0/1 byte (high 7 bits cleared).
  //
  // Defs cover CC clobber AND AQ — same reason as SignTest_i32
  // (Bug #304): the AQ-source expansion path OR-chains AA:AB:AE:AF
  // through AB (ORR A,B / ORR E,B / ORR F,B), clobbering AB and
  // thus part of AQ.  Without declaring AQ in Defs, regalloc
  // believed AQ was preserved and left live-range uses of the i32
  // source vreg reading corrupted bytes after the test.  Even
  // though no observed manifest existed when SignTest_i32 was
  // fixed, the same shape applies here — declaring AQ as clobber
  // forces regalloc to materialise a copy of src when needed past
  // EqZero_i32.
  def EqZero_i32 : MC6809Pseudo {
    dag OutOperandList = (outs ACC8:$dst);
    dag InOperandList  = (ins ACC32:$src);
    let Defs = [NZ, V, C, AA, AW];
  }

  // Bug #301 (2026-05-16): native HD6309 i32 equal-to-
  // const test for non-zero constant.  Produces an i1 (encoded as 0/1
  // byte) whose value is 1 if the i32 is equal to the const, else 0.
  //
  // Replaces __cmpsi2 + trunc i32→i8 + icmp eq 0 for:
  //   icmp eq X, K   →  EqConst_i32 X, K
  //   icmp ne X, K   →  XOR (EqConst_i32 X, K) 1
  // where K is a non-zero i32 constant.  (K=0 uses EqZero_i32 above.)
  //
  // Post-RA expansion: SUBW #low16(K); SBCD #high16(K).  This computes
  // X - K and sets CC.Z if X == K.  Then CC.Z is extracted to a 0/1
  // byte via TFR CC,A; ANDA #4; LSRA; LSRA; TFR A,B (same as EqZero_i32).
  // CLOBBERS $aq's AA byte — same dead-LHS caveat as EqZero_i32.
  //
  // RHS is a 32-bit immediate, split into two 16-bit halves at
  // expansion time.  Defs cover CC clobber (SUBW/SBCD both set NZVC).
  def EqConst_i32 : MC6809Pseudo {
    dag OutOperandList = (outs ACC8:$dst);
    dag InOperandList  = (ins ACC32:$src, i32imm:$rhs);
    // Bug #321 (2026-05-22): preventive AQ clobber declaration —
    // same shape as EqZero_i32 (Bug #304) and SignTest_i32.  The
    // expansion's SUBW + SBCD writes both AW and AD (subregs of AQ),
    // Without AQ in
    // Defs, regalloc could leave a live unrelated value in $aq
    // across this pseudo and the expansion would silently destroy
    // it — the Bug #311 pattern.  No observed manifest yet; this is
    // a preventive fix from the Bug #321 audit.
    let Defs = [NZ, V, C, AA, AW];
  }
}

// MaterializeCC family — read a CC flag bit directly and deposit
// it as a 0/1 byte in $ab.  Used when the carry-out / overflow-out
// of an AddSetCarry*/SubSetCarry*/AddSetOverflow*/SubSetOverflow*
// pseudo must be consumed AS A BYTE (anyext / zext / sext / test)
// rather than as another carry-in.
//
// Why no input operand: the AddSetCarry/Overflow chain communicates
// its s1 result via the IR-side phantom vreg the producer's
// `implicit-def %X:bit1` declares; that vreg's allocated byte-LSB
// does NOT hold the bit at runtime — CC.C / CC.V does.  See the
// long comment block on MC6809ArithmeticBaseCarry
// (MC6809InstrFamilies.td:380-445; Bug #186 v5, 2026-04-27).
// The IR carry chain is tracked on the selector side
// (CCFlagChainTracker in MC6809InstructionSelector.cpp); these
// pseudos read $cc directly via their post-RA expansion.
//
// Post-RA expansion (in MC6809InstrInfo.cpp):
//   C: LDB #0; ADCB #0           (deposit CC.C in AB bit 0)
//   V: TFR CC,B; LSRB; ANDB #1   (CC.V = bit 1, shift to bit 0)
//   Z: TFR CC,B; LSRB×2; ANDB #1 (CC.Z = bit 2)
//   N: TFR CC,B; LSRB×3; ANDB #1 (CC.N = bit 3)
// CC layout (msb→lsb): E F H I N Z V C.
//
// $dst is ACC8_AB (either page-1 half; 6309's AE/AF stay excluded --
// no ADCE/ADCF / LSRE / LSRF). The expansion picks the A- or B-family
// opcodes from the assigned register.
//
// hasSideEffects=1 is the scheduling barrier: the producer's CC
// bit must not be clobbered between its write and this materialise.
// Together with same-BB insertion right after the producer (in
// MC6809InstructionSelector.cpp) it preserves the producer↔
// freeze adjacency the IR carry vreg used to enforce.
//
// Asymmetric Defs-but-no-Uses (mirrors Bug #186 v5's
// MC6809ArithmeticBaseCarry choice — see the long comment block
// in MC6809InstrFamilies.td:380-445).  Adding `Uses=[C]`/`[V]`/
// etc. tells MachineVerifier that the named physreg must be
// live-in to every BB containing this MI; verifier fails when an
// intervening MI declares the flag dead (e.g.
// `Store_i16_Mem ..., implicit-def dead $v` interposed between
// AddSetOverflowUse and the consumer).  Post-RA expansion still
// reads CC honestly via `TFR CC,B` (or `ADCB #0` reading CC.C),
// which is correct because $cc is always live in MC6809 — we
// just suppress the verifier's flag-tracking check.
// hasSideEffects + same-BB insertion right after the producer
// preserves the ordering invariant the dropped Uses was a proxy
// for.
def MaterializeCC_C_to_byte : MC6809Pseudo {
  dag OutOperandList = (outs ACC8_AB:$dst);
  dag InOperandList  = (ins);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}
def MaterializeCC_V_to_byte : MC6809Pseudo {
  dag OutOperandList = (outs ACC8_AB:$dst);
  dag InOperandList  = (ins);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}
def MaterializeCC_Z_to_byte : MC6809Pseudo {
  dag OutOperandList = (outs ACC8_AB:$dst);
  dag InOperandList  = (ins);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}
def MaterializeCC_N_to_byte : MC6809Pseudo {
  dag OutOperandList = (outs ACC8_AB:$dst);
  dag InOperandList  = (ins);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}

// Bug #184 inverse of MaterializeCC_C_to_byte: restore CC.C from a 0/1
// byte that was previously frozen via MaterializeCC_C_to_byte. Used to
// preserve a SubSetCarry chain's borrow across a CC-clobbering region
// (e.g. a CompareBranch or any other MI that clobbers C between a
// SubSetCarry producer and its SubSetCarryUse consumer).
//
// $src is ACC8_AB; the expansion emits LSRA or LSRB by the assigned
// register (LSR shifts bit 0 into CC.C; the byte is consumed here).
//
// Defs=[NZ,V,C] + hasSideEffects=1: same scheduling barrier as the
// to-byte sibling — pin the restoration to its MIR position so no
// further C-clobber slips between this and the consuming SubSetCarryUse.
def MaterializeByteToCarry_i8 : MC6809Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList  = (ins ACC8_AB:$src);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}

// Bug #186 follow-up — V-flag mirror of
// `MaterializeByteToCarry_i8`. Restore CC.V from a 0/1 byte that was
// previously frozen via `MaterializeCC_V_to_byte`. Used to
// preserve a SubSetOverflowUse chain's V-flag across a CC-clobbering
// region in same-BB or cross-BB chains.
//
// Expansion (`ANDB #1; ADDB #0x7F`):
//   ANDB #1     ; ensure B ∈ {0, 1}
//   ADDB #0x7F  ; sets CC.V = 1 iff B was 1 (0x7F + 1 = 0x80 overflows
//                 signed). Other CC bits are clobbered, but the
//                 immediately-following SubSetOverflowUse will
//                 overwrite them anyway.
//
// $src is ACC8_AB; the expansion picks the opcode family by the
// assigned register.
//
// Defs=[NZ,V,C] + hasSideEffects=1: same scheduling barrier as the
// C-flag sibling — pin restoration to its MIR position so no further
// V-clobber slips between this and the consuming SubSetOverflowUse.
def MaterializeByteToOverflow_i8 : MC6809Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList  = (ins ACC8_AB:$src);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}

// Bug #186 v5 PLACEHOLDER (2026-04-27): catch-all COPY for register
// pairs that copyPhysReg does not yet have a proper sequence for —
// chiefly $c↔byte / $v↔byte / cross-flag pairs that emerge from
// regalloc once SetCarry/SetCarryUse drop their explicit operand and
// the carry vreg gets allocated to whatever copyPhysReg-able physreg
// regalloc chooses.
//
// Polymorphic operand types (`unknown`) so the same pseudo serves
// any class pair. AsmPrinter expands to a comment + SWI3 trap so
// runtime hits surface noisily and asm-grep finds the offending sites.
//
// Once we know which class pairs are actually exercised by real
// codegen, replace the placeholder with proper lowerings in
// copyPhysReg (and probably retire this pseudo entirely).
def COPY_CC_PLACEHOLDER : MC6809Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList  = (ins unknown:$dst, unknown:$src);
  let hasSideEffects = 1;
  let isCodeGenOnly = 1;
  // Bug #186 follow-up Phase 4 (2026-04-28): AsmPrinter expands to a
  // single SWI3 trap (1 byte). Without `let Size`, getInstSizeInBytes
  // falls back to MaxInstLength = 5, over-counting BR's accounting by
  // 4 bytes per emission. Currently no codegen path emits this pseudo
  // — the over-count is harmless today (extra widening only) — but
  // declaring the honest size prevents a future bug class if a new
  // copyPhysReg gap surfaces and the pseudo starts firing.
  let Size = 1;
}

