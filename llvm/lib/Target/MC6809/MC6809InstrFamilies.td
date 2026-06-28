//===-- MC6809InstrFamilies.td - Instruction Family Groupings --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Groups pseudo-instructions, concrete instructions, and patterns for each
// instruction family together for maintainability. Pseudo-instructions are
// register-agnostic (used by GlobalISel), concrete instructions are
// register-specific (used by assembler/disassembler), and patterns connect
// LLVM IR operations to pseudo-instructions.
//
//===----------------------------------------------------------------------===//

//===----------------------------------------------------------------------===//
// Load Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayLoad = true;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = operand;
}

multiclass MC6809Load<RegisterClass dst, ValueType vtype> {
  let isAsCheapAsAMove = true, isMoveImm = true in {
    def _Imm : MC6809LoadBase<dst, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    // Indirect-indexed load: dst = *(*(idx + offset)). The pointer itself lives
    // in memory at [idx+offset]; the 6809 indexed-indirect mode reads it and
    // derefs in one instruction, so the explicit pointer load AND the index
    // register it would occupy both disappear. Selected by the am_ls_indexed_
    // indirect pattern (a OneUse pointer-load folded into its deref); expanded
    // post-PEI to the concrete [n,r] indirect opcode (same operands as _Mem,
    // just the indirect postbyte). Two memory reads, so mayLoad only.
    def _MemIndirect : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    // load of a global's address with the register choice deferred to
    // regalloc (dst is the class vreg). The post-RA expander (expandLoadSym)
    // picks the concrete reg-specific opcode and the addressing mode — direct
    // page / extended / PC-relative — from the global and the relocation model.
    def _Sym : MC6809LoadBase<dst, (ins unknown:$sym)>;
  }
}

defm Load_i8 : MC6809Load<ACC8, i8>;
defm Load_i16 : MC6809Load<ACC16, i16>;
defm Load_iPtr : MC6809Load<INDEX16, i16>;
let Predicates = [IsHD6309] in {
  // Bug #208 round 4: dst constrained to AQc (= just AQ) instead of
  // the wider ACC32 (= AQ + SPILL_Q0..3). expandLoadImm's SPILL_Q*
  // path stages through AQ via LDQ, which clobbers AA/AB/AE/AF/AD/AW
  // (sub-regs of AQ). Constraining to AQc forces regalloc to know the
  // result lives in AQ; sub-reg aliasing then prevents it from
  // keeping AA/etc. live across the pseudo. If a downstream consumer
  // wants the value in a SPILL_Q* slot, regalloc inserts a normal
  // ACC32 → SPILL_Q* COPY whose own clobber set (just an STQ, no
  // LDQ) is bounded.
  // Bug #210 round 5 (2026-05-04): kept AQc despite hash.c starvation
  // pressure — the libcall-based G_AND/G_OR/G_XOR i32 (in
  // legalizeBitwise → __andsi3 / __orsi3 / __xorsi3) eliminates the
  // in-line UNMERGE/MERGE artifacts that previously starved
  // __call_hash. The AQc constraint stays load-bearing for
  // strtol/snprintf/asctime correctness (bug #208).
  //
  // Bug #299 (2026-05-15): split _Imm and _Mem variants because
  // they have different sub-reg clobber footprints.
  //
  // Bug #308 (2026-05-19): the original Bug #221 two-LDD-via-$ad
  // expansion in expandLoadIdx was replaced with LDQ-via-$aq
  // bracketed by `emitAQPreservedOverHardStackScratch`.  The new
  // expansion preserves the full AQ family at runtime, so
  // cross-load AD-vregs survive correctly.  Therefore the pseudo's
  // old `Defs = [NZ, V, AD]` is now actively harmful: it tells
  // regalloc that AD is clobbered, which the dead-marking
  // heuristic then turns into a zero-width interval that fails
  // the verifier (Bug #308 manifest in `__split_page`, `wcrtomb`,
  // `ffsll`, `flsl`, `flsll`).  Drop AD from Defs.
  //
  // For the AQ-dst case (no SPILL_Q* fallback): regalloc still
  // gets AD-aliasing eviction protection via the explicit AQc dst
  // operand (AD is a sub-reg of AQ).  After expandLoadIdx
  // converts to the concrete LDQi_oN, that instruction's own
  // Defs=[AA,AB,AD,AE,AF,AW,AQ] cover the full clobber for
  // post-RA passes.
  let isAsCheapAsAMove = true, isMoveImm = true, mayLoad = true in
    def Load_i32_Imm : MC6809LoadBase<AQc, (ins i32imm:$imm)>;
  let mayLoad = true, Defs = [NZ, V] in
    def Load_i32_Mem : MC6809LoadBase<AQc, (ins INDEX16:$idx, unknown:$offset)>;
  // i32 indirect-indexed load: AQ = *(*(idx+offset)). Single LDQ [n,r] (the AQc
  // dest never takes expandLoadIdx's SPILL_Q two-LDD path, which has no indirect
  // composition). HD6309-only, like all i32.
  let mayLoad = true, Defs = [NZ, V] in
    def Load_i32_MemIndirect : MC6809LoadBase<AQc, (ins INDEX16:$idx, unknown:$offset)>;
  // i32 global load, register deferred (AQc), expanded by expandLoadSym.
  let mayLoad = true, Defs = [NZ, V] in
    def Load_i32_Sym : MC6809LoadBase<AQc, (ins unknown:$sym)>;

  // Bug #271 cat-3: spill-framework-only i32 load with ACC32 operand
  // class. The narrow AQc constraint on Load_i32 above is load-bearing
  // for user codegen (Bug #208 round 4 sub-reg aliasing), but the spill
  // framework's `loadStoreRegStackSlot` emits with the original spilled
  // vreg whose class is ACC32 (= AQ + SPILL_Q0..3) — a non-subclass of
  // AQc, which -verify-machineinstrs flags. This variant accepts ACC32
  // directly; its post-RA expansion is identical to Load_i32_Mem (both
  // route to `expandLoadIdx`, which already handles SPILL_Q* dests via
  // the Bug #221 two-LDD path with no LDQ sub-reg clobber).
  //
  // Bug #290: trimmed Defs from the old worst-case AQ sub-reg list
  // `[NZ, V, AA, AB, AE, AF, AD, AW]` to `[NZ, V, AD]`.  Sub-reg
  // aliasing of $ad → $aa/$ab covers the byte-level clobber when
  // dst is SPILL_Q* (Bug #221 two-LDD path); the rest of the AQ
  // sub-reg tree ($aw/$ae/$af) is NOT clobbered by the two-LDD path
  // and was over-claimed historically.  The over-claim hurts
  // Bug #290's clobber-driven re-evaluation: it sweeps too wide,
  // creating LTO+FP regalloc pressure with no real backing.
  // The reg-offset fallback in `expandLoadIdx`'s SPILL_Q* dst path
  // (which falls through to a full LDQ and would clobber the AQ
  // sub-reg tree) is unreachable for spill MIs because the spiller
  // always emits immediate frame-index offsets.
  // Bug #308 (2026-05-19): dropped AD from Defs (was [NZ, V, AD]) —
  // same rationale as Load_i32_Mem above.  The two-LDD-via-$ad
  // expansion was replaced with LDQ-via-$aq + save/restore, so AD
  // is preserved across the pseudo at runtime.
  let mayLoad = true, Defs = [NZ, V] in
    def SpillLoad_i32_Mem : MC6809LogicalInstr {
      dag OutOperandList = (outs ACC32:$dst);
      dag InOperandList = (ins INDEX16:$idx, unknown:$offset);
    }
}

// Loads a 0/1 boolean into the LSB of an ACC8 byte (or directly into
// a CC bit when the dst is a named NZVC physreg).  The expansion
// dispatches on the chosen dst register.
def Load_i1_Imm : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins i1imm:$val);
  let isAsCheapAsAMove = true;
  let isMoveImm = true;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---

// 8-bit loads: LDA/LDB (6809 Page 1)
defm LDA : Load8Concrete<"lda", Opcode<0x86,1>, Opcode<0x96,1>,
                          Opcode<0xA6,1>, Opcode<0xB6,1>,
                          [NZ,N,Z,V,AA]>;
defm LDB : Load8Concrete<"ldb", Opcode<0xC6,1>, Opcode<0xD6,1>,
                          Opcode<0xE6,1>, Opcode<0xF6,1>,
                          [NZ,N,Z,V,AB]>;

// 16-bit loads: LDD/LDX/LDU (6809 Page 1)
//
// Bug #299 (2026-05-15): LDD's Defs lists AQ because AD is the
// sub_hi_word of AQ.  Without AQ in Defs the regalloc may keep
// an i32 live in AQ (Phase B LDQ-loaded) across an unrelated LDD,
// silently corrupting AQ's high half (the i32's high 16 bits).
// Bug #272 Phase B Scope A previously dropped AQ from this list as
// an over-claim — correct pre-Phase-B (no i32-in-AQ live ranges),
// undercoverage post-Phase-B.  Re-add the conservative AQ claim
// to force regalloc to evict any vreg currently in AQ before LDD.
defm LDD : Load16Concrete<"ldd", Opcode<0xCC,1>, Opcode<0xDC,1>,
                           Opcode<0xEC,1>, Opcode<0xFC,1>,
                           [NZ,N,Z,V,AA,AB,AD,AQ]>;
defm LDX : Load16Concrete<"ldx", Opcode<0x8E,1>, Opcode<0x9E,1>,
                           Opcode<0xAE,1>, Opcode<0xBE,1>,
                           [NZ,N,Z,V,IX]>;
defm LDU : Load16Concrete<"ldu", Opcode<0xCE,1>, Opcode<0xDE,1>,
                           Opcode<0xEE,1>, Opcode<0xFE,1>,
                           [NZ,N,Z,V,SU]>;

// 8-bit loads: LDE/LDF (6309 Page 3)
defm LDE : Load8Concrete_P3<"lde", Opcode<0x86,3>, Opcode<0x96,3>,
                             Opcode<0xA6,3>, Opcode<0xB6,3>,
                             [NZ,N,Z,V,AE]>;
defm LDF : Load8Concrete_P3<"ldf", Opcode<0xC6,3>, Opcode<0xD6,3>,
                             Opcode<0xE6,3>, Opcode<0xF6,3>,
                             [NZ,N,Z,V,AF]>;

// 16-bit loads: LDY/LDS (6809 Page 2), LDW (6309 Page 2)
defm LDY : Load16Concrete_P2<"ldy", Opcode<0x8E,2>, Opcode<0x9E,2>,
                              Opcode<0xAE,2>, Opcode<0xBE,2>,
                              [NZ,N,Z,V,IY]>;
defm LDS : Load16Concrete_P2<"lds", Opcode<0xCE,2>, Opcode<0xDE,2>,
                              Opcode<0xEE,2>, Opcode<0xFE,2>,
                              [NZ,N,Z,V,SS]>;
let Predicates = [IsHD6309] in {
  defm LDW : Load16Concrete_P2<"ldw", Opcode<0x86,2>, Opcode<0x96,2>,
                                Opcode<0xA6,2>, Opcode<0xB6,2>,
                                [NZ,N,Z,V,AE,AF,AW]>;
}

// LDQ (6309, 32-bit): immediate on Page 1, dir/idx/ext on Page 2.
let Predicates = [IsHD6309], mayLoad = true in {
  def LDQi32 : MC6809Immediate32Bit_P1<(outs), "ldq", Opcode<0xCD,1>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  def LDQd   : MC6809Direct_P2<(outs), "ldq", Opcode<0xDC,2>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  defm LDQi  : MC6809Indexed_P2<"ldq", Opcode<0xEC,2>, (outs),
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
  def LDQe   : MC6809Extended_P2<(outs), "ldq", Opcode<0xFC,2>,
               [NZ,N,Z,V,AA,AB,AD,AE,AF,AW,AQ], []>;
}

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

// Bug #311 Phase 1 step 1.1 (2026-05-20): retired.  Load_i1_Imm's dst
// moved to ACC8; legalizer extends G_CONSTANT i1 → G_CONSTANT i8 before
// isel, so this pattern's `(i1 imm)` shape no longer appears.

multiclass MC6809LoadPat<ValueType vtype> {
  def _#vtype#_Imm : Pat<(vtype imm:$val), (!cast<Instruction>("Load_"#vtype#"_Imm") imm:$val)>;
  def _#vtype#_Idx_Zero : Pat<(vtype (load INDEX16:$idx)), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, (i16 0))>;
  def _#vtype#_Idx_Imm : Pat<(vtype (load (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Unmerge_Idx_Imm : Pat<(vtype (load (am_ls_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Idx_Reg8 : Pat<(vtype (load (am_ls_indexed_reg_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, ACC8:$offset)>;
  def _#vtype#_Idx_Reg16 : Pat<(vtype (load (am_ls_indexed_reg_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, ACC16:$offset)>;
  def _#vtype#_Frame_Index : Pat<(vtype (load (am_ls_frame_index INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_Mem") INDEX16:$idx, i16imm:$offset)>;
}

defm LoadPat_i8 : MC6809LoadPat<i8>;
defm LoadPat_i16 : MC6809LoadPat<i16>;
let Predicates = [IsHD6309] in {
  defm LoadPat_i32 : MC6809LoadPat<i32>;
}

// Indirect-indexed loads (i8/i16 only -- i32 has no _MemIndirect pseudo). Deref
// a pointer that itself sits at [idx+offset] -> ld [offset,idx]. Higher
// complexity than the plain deref patterns so it is tried first (it folds an
// extra load away).
multiclass MC6809LoadIndirectPat<ValueType vtype> {
  let AddedComplexity = 20 in
  def _#vtype#_Idx_Indirect : Pat<(vtype (load (am_ls_indexed_indirect INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>("Load_"#vtype#"_MemIndirect") INDEX16:$idx, i16imm:$offset)>;
}
defm LoadIndirectPat_i8  : MC6809LoadIndirectPat<i8>;
defm LoadIndirectPat_i16 : MC6809LoadIndirectPat<i16>;
let Predicates = [IsHD6309] in {
  defm LoadIndirectPat_i32 : MC6809LoadIndirectPat<i32>;
}

//===----------------------------------------------------------------------===//
// Store Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  let mayLoad = true in
  // Indirect-indexed store: *(*(idx + offset)) = src. mayLoad too (it reads the
  // in-memory pointer at [idx+offset] before storing through it). Same shape as
  // _Mem; expanded to the [n,r] indirect store opcode.
  def _MemIndirect : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
  def _Mem : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
  // store to a global's address, register choice deferred to regalloc
  // (src is the class vreg). expandStoreSym picks the concrete reg-specific
  // opcode + addressing mode (direct page / extended / PC-relative).
  def _Sym : MC6809StoreBase<src, (ins unknown:$sym)>;
}

defm Store_i8 : MC6809Store<ACC8>;
defm Store_i16 : MC6809Store<ACC16>;
defm Store_iPtr : MC6809Store<INDEX16>;

// Indirect-through-memory call, fused pre-RA from a Load_iPtr_Mem whose loaded
// function pointer is used only by indirect JSRi_o0 calls (e.g. qsort's
// comparator argument). The pointer is read from [idx + offset] as part of the
// call (MC6809 `jsr [n,r]`), so the separate register load is removed and no
// index register is tied up across the call. The addressing operands mirror
// Load_iPtr_Mem (idx, offset) so eliminateFrameIndex resolves a frame-index
// base; expandJsrIdx then selects the JSRi_*I indirect opcode by the resolved
// offset width. The call's register mask and implicit argument/clobber operands
// are appended at fusion time.
let isCall = true, mayLoad = true in
def JSR_iPtr_Mem : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins INDEX16:$idx, unknown:$offset);
}

//===----------------------------------------------------------------------===//
// Post-modify (auto-increment / auto-decrement) Load/Store Family
//===----------------------------------------------------------------------===//
//
// Fused at instruction selection from a Load/Store_*_Mem(p, 0) plus a pointer
// advance of +/- the access size: *p++ -> PostInc (access *p, then p += N),
// *--p -> PreDec (p -= N, then access *p). $ptr_out is tied to $ptr_in so the
// allocator keeps the pointer in one index register across the advance. The
// post-RA expander emits the ,R+ / ,R++ / ,-R / ,--R indexed encodings.

class MC6809LoadPostMod<RegisterClass dst> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayLoad = true;
  dag OutOperandList = (outs dst:$dst, INDEX16:$ptr_out);
  dag InOperandList = (ins INDEX16:$ptr_in);
  let Constraints = "$ptr_in = $ptr_out";
}

class MC6809StorePostMod<RegisterClass src> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs INDEX16:$ptr_out);
  dag InOperandList = (ins src:$src, INDEX16:$ptr_in);
  let Constraints = "$ptr_in = $ptr_out";
}

def Load_i8_PostInc   : MC6809LoadPostMod<ACC8>;
def Load_i8_PreDec    : MC6809LoadPostMod<ACC8>;
def Load_i16_PostInc  : MC6809LoadPostMod<ACC16>;
def Load_i16_PreDec   : MC6809LoadPostMod<ACC16>;
def Load_iPtr_PostInc : MC6809LoadPostMod<INDEX16>;
def Load_iPtr_PreDec  : MC6809LoadPostMod<INDEX16>;

def Store_i8_PostInc   : MC6809StorePostMod<ACC8>;
def Store_i8_PreDec    : MC6809StorePostMod<ACC8>;
def Store_i16_PostInc  : MC6809StorePostMod<ACC16>;
def Store_i16_PreDec   : MC6809StorePostMod<ACC16>;
def Store_iPtr_PostInc : MC6809StorePostMod<INDEX16>;
def Store_iPtr_PreDec  : MC6809StorePostMod<INDEX16>;

// Address of a global with the index register deferred to regalloc, for the
// PIC/PIE G_GLOBAL_VALUE materialisation (the non-PIC case uses Load_iPtr_Imm's
// absolute LDX #addr). expandLeaSym emits LEA{X,Y,U} sym,pc for the allocated
// register, so the address is not pinned to IX + a COPY.
// isReMaterializable / isAsCheapAsAMove: the address is a relocatable constant
// recomputable by a single LEA, so regalloc rematerialises it under pressure
// rather than spilling+reloading (a PC-relative LEA re-resolves at any location,
// so cloning is correct). Mirrors Load_iPtr_Imm's flags for the non-PIC case.
let Defs = [Z], isReMaterializable = 1, isAsCheapAsAMove = 1 in
def Lea_iPtr_Sym : MC6809LogicalInstr {
  dag OutOperandList = (outs INDEX16:$dst);
  dag InOperandList = (ins unknown:$sym);
}

let Predicates = [IsHD6309] in {
  // Bug #208 follow-up: src constrained to AQc — same rationale as
  // Load_i32. Bug #210 round 5 kept this constraint because the
  // libcall-based bitwise-i32 (legalizeBitwise → __andsi3 / __orsi3
  // / __xorsi3) eliminates hash.c's aqc starvation without needing
  // the wider class.
  //
  // Bug #300 (2026-05-15): symmetric to the Bug #299 Load_i32_Mem
  // Bug #308 (2026-05-19): symmetric to Load_i32_Mem — the original
  // Bug #221 two-LDD-slot-to-slot expansion in expandStoreIdx was
  // replaced with LDQ-from-slot + STQ-to-dst via $aq bracketed by
  // `emitAQPreservedOverHardStackScratch`.  Cross-pseudo AD-vregs
  // survive correctly at runtime, so drop AD from Defs (it was
  // dead-marked by regalloc anyway, producing the verifier hits
  // Bug #308 catalogues).
  let mayStore = true, Defs = [NZ, V] in
    def Store_i32_Mem : MC6809StoreBase<AQc, (ins INDEX16:$idx, unknown:$offset)>;
  // i32 indirect-indexed store: *(*(idx+offset)) = AQ -> STQ [n,r]. mayLoad too
  // (reads the in-memory pointer first).
  let mayStore = true, mayLoad = true, Defs = [NZ, V] in
    def Store_i32_MemIndirect : MC6809StoreBase<AQc, (ins INDEX16:$idx, unknown:$offset)>;
  // i32 global store, register deferred (AQc), expanded by expandStoreSym.
  let mayStore = true, Defs = [NZ, V] in
    def Store_i32_Sym : MC6809StoreBase<AQc, (ins unknown:$sym)>;

  // Bug #271 cat-3: spill-framework-only i32 store with ACC32 operand
  // class.  Mirrors SpillLoad_i32_Mem above.  Bug #308 dropped AD
  // from Defs for the same reason as Store_i32_Mem above.
  let mayStore = true, Defs = [NZ, V] in
    def SpillStore_i32_Mem : MC6809LogicalInstr {
      dag OutOperandList = (outs);
      dag InOperandList = (ins ACC32:$src, INDEX16:$idx, unknown:$offset);
    }
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---

// 8-bit stores: STA/STB (6809 Page 1)
defm STA : Store8Concrete<"sta", Opcode<0x97,1>, Opcode<0xA7,1>, Opcode<0xB7,1>>;
defm STB : Store8Concrete<"stb", Opcode<0xD7,1>, Opcode<0xE7,1>, Opcode<0xF7,1>>;

// 16-bit stores: STD/STX/STU (6809 Page 1)
defm STD : Store16Concrete<"std", Opcode<0xDD,1>, Opcode<0xED,1>, Opcode<0xFD,1>>;
defm STX : Store16Concrete<"stx", Opcode<0x9F,1>, Opcode<0xAF,1>, Opcode<0xBF,1>>;
defm STU : Store16Concrete<"stu", Opcode<0xDF,1>, Opcode<0xEF,1>, Opcode<0xFF,1>>;

// 8-bit stores: STE/STF (6309 Page 3)
defm STE : Store8Concrete_P3<"ste", Opcode<0x97,3>, Opcode<0xA7,3>, Opcode<0xB7,3>>;
defm STF : Store8Concrete_P3<"stf", Opcode<0xD7,3>, Opcode<0xE7,3>, Opcode<0xF7,3>>;

// 16-bit stores: STY/STS (6809 Page 2), STW (6309 Page 2)
defm STY : Store16Concrete_P2<"sty", Opcode<0x9F,2>, Opcode<0xAF,2>, Opcode<0xBF,2>>;
defm STS : Store16Concrete_P2<"sts", Opcode<0xDF,2>, Opcode<0xEF,2>, Opcode<0xFF,2>>;
let Predicates = [IsHD6309] in {
  defm STW : Store16Concrete_P2<"stw", Opcode<0x97,2>, Opcode<0xA7,2>, Opcode<0xB7,2>>;
}

// STQ (6309, 32-bit): dir/idx/ext on Page 2.
let Predicates = [IsHD6309], mayStore = true in {
  def STQd  : MC6809Direct_P2<(outs), "stq", Opcode<0xDD,2>, [NZ,N,Z,V], []>;
  defm STQi : MC6809Indexed_P2<"stq", Opcode<0xED,2>, (outs), [NZ,N,Z,V], []>;
  def STQe  : MC6809Extended_P2<(outs), "stq", Opcode<0xFD,2>, [NZ,N,Z,V], []>;
}

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809StorePat<ValueType vtype, RegisterClass reg> {
  def _#vtype#_Idx_Zero : Pat<(store reg:$src, INDEX16:$idx), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, (i16 0))>;
  def _#vtype#_Idx_Imm : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
  def _#vtype#_Unmerge_Idx_Imm : Pat<(store reg:$src, (am_ls_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
  // NOTE: register-offset STORES are deliberately NOT wired to
  // am_ls_indexed_reg_offset yet. On this 2-accumulator target the store value
  // and the offset both have to be READ, and a byte value is always a sub-reg
  // of the only 16-bit offset (D = A:B), so `st<byte> d,x` aliases value and
  // offset -> miscompile. An 8-bit (A/B) offset is only safe when offset != the
  // value reg. Until that alias guard exists these stay on the (dead, constant-
  // only) imm matcher so stores fall back to the correct LEAX + zero-offset form.
  def _#vtype#_Idx_Reg8 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC8:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _#vtype#_Idx_Reg16 : Pat<(store reg:$src, (am_ls_indexed_imm_offset INDEX16:$idx, ACC16:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset)>;
  def _#vtype#_Frame_Index : Pat<(store reg:$src, (am_ls_frame_index INDEX16:$idx, ACC16:$offset)), (!cast<Instruction>("Store_"#vtype#"_Mem") reg:$src, INDEX16:$idx, i16imm:$offset)>;
}

defm StorePat_i8 : MC6809StorePat<i8, ACC8>;
defm StorePat_i16 : MC6809StorePat<i16, ACC16>;
defm StorePat_iPtr : MC6809StorePat<i16, INDEX16>;
let Predicates = [IsHD6309] in {
  // Match the Store_i32 pseudo's AQc src class (bug #208).
  defm StorePat_i32 : MC6809StorePat<i32, AQc>;
}

// Indirect-indexed stores (i8/i16 only -- i32 has no _MemIndirect pseudo).
// Store through a pointer that itself sits at [idx+offset] -> st [offset,idx].
multiclass MC6809StoreIndirectPat<ValueType vtype, RegisterClass reg> {
  let AddedComplexity = 20 in
  def _#vtype#_Idx_Indirect : Pat<(store reg:$src, (am_ls_indexed_indirect INDEX16:$idx, i16imm:$offset)), (!cast<Instruction>("Store_"#vtype#"_MemIndirect") reg:$src, INDEX16:$idx, i16imm:$offset)>;
}
defm StoreIndirectPat_i8  : MC6809StoreIndirectPat<i8, ACC8>;
defm StoreIndirectPat_i16 : MC6809StoreIndirectPat<i16, ACC16>;
let Predicates = [IsHD6309] in {
  defm StoreIndirectPat_i32 : MC6809StoreIndirectPat<i32, AQc>;
}

//===----------------------------------------------------------------------===//
// Arithmetic (Add/Sub) Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809ArithmeticBase<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #214 / Bug #217 — per-half byte _Reg variant.
// path-(c) of emit6809RegByteFromMem clobbers the hardware accumulator
// half used by LHS and to host the result. To declare this honestly to
// regalloc WITHOUT the union-claim that broke #217 at non-LTO opt
// levels: dst is constrained to ABc (AB + SPILL_B*) so regalloc
// commits to B-half pre-RA, and Defs declares only AB clobbered.
// AA-half is untouched; values in AA are preserved without
// regalloc-inserted save/restore.
class MC6809ArithmeticBaseRegByteB<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AB];
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList = !con((ins ABc:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #221 Phase A — A-half mirror of MC6809ArithmeticBaseRegByteB.
// Constrains dst to AAc (AA + SPILL_A*) and declares Defs += AA
// only. Used by the parallel `_RegA` pseudos defined alongside the
// `_RegB` ones; Phase B wires the selector to choose between them
// based on the upstream value's natural half. Phase A introduces the
// pseudo without selector wiring — `_RegA` exists but no emission
// path uses it, so the binary is byte-identical to today.
class MC6809ArithmeticBaseRegByteA<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AA];
  dag OutOperandList = (outs AAc:$dst);
  dag InOperandList = !con((ins AAc:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Arithmetic<ValueType vtype, RegisterClass dst, RegisterClass src, bit IsByte = 0> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBase<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    // P3b: indirect-indexed (`op [n,r]`) -- same operands as _Mem, expanded to
    // the indirect-sibling opcode via indirectSiblingOf in expandIdxImm.
    def _MemIndirect : MC6809ArithmeticBase<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809ArithmeticBase<dst, src, (ins STACK16:$idx)>;
    let Predicates = [IsHD6309] in {
      if !eq(IsByte, 1) then {
        // Bug #214/#217 default: B-half routing. `_Reg` keeps the
        // current emission path so existing C++ refs to
        // MC6809::<X>_i8_Reg remain valid and unchanged.
        def _Reg  : MC6809ArithmeticBaseRegByteB<(ins ABc:$src2)>;
        // Bug #221 Phase A: parallel A-half pseudo. Phase B's selector
        // heuristic will choose between _RegA and _Reg (= _RegB) based
        // on the upstream value's natural half. Currently unused.
        def _RegA : MC6809ArithmeticBaseRegByteA<(ins AAc:$src2)>;
      } else {
        def _Reg : MC6809ArithmeticBase<dst, src, (ins src:$src2)>;
      }
    }
  }
}

let isAdd = true in {
  defm Add_i8 : MC6809Arithmetic<i8, ACC8, ACC8, /*IsByte=*/1>;
  defm Add_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;
  // Bug #297 (2026-05-15): native HD6309 i32 ADD via page-2 ADDW (low
  // word in AW) + ADCD (high word in AD, carry-in). Pseudos are dead
  // code at this commit — no selector or expander references them yet;
  // selector wiring and legalizer flip land in later #297 commits.
  let Predicates = [IsHD6309] in
    defm Add_i32 : MC6809Arithmetic<i32, ACC32, ACC32>;
}
defm Sub_i8 : MC6809Arithmetic<i8, ACC8, ACC8, /*IsByte=*/1>;
defm Sub_i16 : MC6809Arithmetic<i16, ACC16, ACC16>;
// Bug #297: native HD6309 i32 SUB via page-2 SUBW + SBCD.
let Predicates = [IsHD6309] in
  defm Sub_i32 : MC6809Arithmetic<i32, ACC32, ACC32>;

// --- Carry-setting variants ---
//
// SetCarry / SetCarryUse pairs implement the lo/hi halves of an
// i16-narrowed-from-i32 add-with-carry: SetCarry produces a carry-out
// (a scheduling-phantom s1 vreg in PHANTOM_CARRY) and SetCarryUse
// consumes it.  The vreg threads the SSA dependency producer→consumer;
// the actual carry value flows through CC.C in the emitted hardware.
//
// Two invariants protect CC.C across the pair:
//
//   1. `Defs = [NZ, V, C]` — declare the CC.C clobber honestly.
//   2. `hasSideEffects = 1` — make the pseudo a scheduling barrier so
//      the pre-RA scheduler can't slip a CC-clobbering instruction
//      between SetCarry and SetCarryUse.
//
// The IR-level carry vreg is tracked selector-side via
// CCFlagChainTracker in MC6809InstructionSelector.cpp.  Same base class
// is shared with SetOverflow because the hardware expansion is identical
// — only the IR vreg's semantic flag (C vs V) differs, recorded by the
// tracker.
class MC6809ArithmeticBaseCarry<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  // bug #57: scheduling barrier — see the long comment above.
  let hasSideEffects = 1;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #214 / Bug #217 — per-half byte SetCarry _Reg variant.
// See MC6809ArithmeticBaseRegByteB for the rationale. Carries the
// hasSideEffects scheduling barrier (bug #57).
class MC6809ArithmeticBaseCarryRegByteB<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AB];
  let hasSideEffects = 1;
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList = !con((ins ABc:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #221 Phase A — A-half mirror of the SetCarry byte _Reg variant.
class MC6809ArithmeticBaseCarryRegByteA<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AA];
  let hasSideEffects = 1;
  dag OutOperandList = (outs AAc:$dst);
  dag InOperandList = !con((ins AAc:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809ArithmeticCarry<ValueType vtype, RegisterClass dst, RegisterClass src, bit IsByte = 0> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseCarry<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809ArithmeticBaseCarry<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809ArithmeticBaseCarry<dst, src, (ins STACK16:$idx)>;
    let Predicates = [IsHD6309] in {
      if !eq(IsByte, 1) then {
        def _Reg  : MC6809ArithmeticBaseCarryRegByteB<(ins ABc:$src2)>;
        // Bug #221 Phase A: parallel A-half pseudo (currently unused).
        def _RegA : MC6809ArithmeticBaseCarryRegByteA<(ins AAc:$src2)>;
      } else {
        def _Reg : MC6809ArithmeticBaseCarry<dst, src, (ins src:$src2)>;
      }
    }
  }
}

let isAdd = true in {
  defm AddSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, ACC8, /*IsByte=*/1>;
  defm AddSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, ACC16>;
  // Bug #297: i32 carry-out variant. Used by G_UADDO i32 selector
  // (commit 4); currently dormant.
  let Predicates = [IsHD6309] in
    defm AddSetCarry_i32 : MC6809ArithmeticCarry<i32, ACC32, ACC32>;
}
defm SubSetCarry_i8 : MC6809ArithmeticCarry<i8, ACC8, ACC8, /*IsByte=*/1>;
defm SubSetCarry_i16 : MC6809ArithmeticCarry<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in
  defm SubSetCarry_i32 : MC6809ArithmeticCarry<i32, ACC32, ACC32>;

// Bug #147: parallel pseudos for *signed* G_SADDO / G_SSUBO. Same machine
// expansion as SetCarry (same ADD/SUB/ADC/SBC instructions); the IR
// vreg's value semantically represents the V flag rather than C. The
// selector's CCFlagChainTracker records `Flag = MC6809::V` for these.
let isAdd = true in {
  defm AddSetOverflow_i8  : MC6809ArithmeticCarry<i8,  ACC8,  ACC8, /*IsByte=*/1>;
  defm AddSetOverflow_i16 : MC6809ArithmeticCarry<i16, ACC16, ACC16>;
  // Bug #297: i32 signed-overflow variant.
  let Predicates = [IsHD6309] in
    defm AddSetOverflow_i32 : MC6809ArithmeticCarry<i32, ACC32, ACC32>;
}
defm SubSetOverflow_i8  : MC6809ArithmeticCarry<i8,  ACC8,  ACC8, /*IsByte=*/1>;
defm SubSetOverflow_i16 : MC6809ArithmeticCarry<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in
  defm SubSetOverflow_i32 : MC6809ArithmeticCarry<i32, ACC32, ACC32>;

// --- Carry-using variants ---
//
// SetCarryUse / SetOverflowUse pseudos consume the producer's
// scheduling-phantom carry/overflow vreg AND read CC.C / CC.V
// implicitly via Uses=[C] / Uses=[V].  Selector places the consumer
// adjacent to its producer when possible; for cross-BB transitions or
// CC-clobbering intermediates it inserts a byte materialise via
// MaterializeCC_C_to_byte / MaterializeCC_V_to_byte so the value
// survives the gap.  Base class is split (SetCarryUse / SetOverflowUse)
// to keep the Uses list flag-specific.
class MC6809ArithmeticBaseSetCarryUse<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  let Uses = [C];
  let hasSideEffects = 1;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

class MC6809ArithmeticBaseSetOverflowUse<RegisterClass dst, RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  let Uses = [V];
  let hasSideEffects = 1;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins src:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #214 / Bug #217 — per-half byte SetCarryUse / SetOverflowUse _Reg
// variants. See MC6809ArithmeticBaseRegByteB for the rationale.
class MC6809ArithmeticBaseSetCarryUseRegByteB<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AB];
  let Uses = [C];
  let hasSideEffects = 1;
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList = !con((ins ABc:$src), operand);
  let Constraints = "$dst = $src";
}

class MC6809ArithmeticBaseSetOverflowUseRegByteB<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AB];
  let Uses = [V];
  let hasSideEffects = 1;
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList = !con((ins ABc:$src), operand);
  let Constraints = "$dst = $src";
}

// Bug #221 Phase A — A-half mirrors of the SetCarryUse / SetOverflowUse
// byte _Reg variants. Carry-in / overflow-in flag flow declared via
// Uses=[C] / Uses=[V] (same as the B-half variants); the only delta is
// dst class + Defs += AA.
class MC6809ArithmeticBaseSetCarryUseRegByteA<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AA];
  let Uses = [C];
  let hasSideEffects = 1;
  dag OutOperandList = (outs AAc:$dst);
  dag InOperandList = !con((ins AAc:$src), operand);
  let Constraints = "$dst = $src";
}

class MC6809ArithmeticBaseSetOverflowUseRegByteA<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V, C, AA];
  let Uses = [V];
  let hasSideEffects = 1;
  dag OutOperandList = (outs AAc:$dst);
  dag InOperandList = !con((ins AAc:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809ArithmeticCarryUse<ValueType vtype, RegisterClass dst, RegisterClass src, bit IsByte = 0> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseSetCarryUse<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809ArithmeticBaseSetCarryUse<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    let Predicates = [IsHD6309] in {
      if !eq(IsByte, 1) then {
        def _Reg  : MC6809ArithmeticBaseSetCarryUseRegByteB<(ins ABc:$src2)>;
        // Bug #221 Phase A: parallel A-half pseudo (currently unused).
        def _RegA : MC6809ArithmeticBaseSetCarryUseRegByteA<(ins AAc:$src2)>;
      } else {
        def _Reg : MC6809ArithmeticBaseSetCarryUse<dst, src, (ins src:$src2)>;
      }
    }
  }
}

multiclass MC6809ArithmeticOverflowUse<ValueType vtype, RegisterClass dst, RegisterClass src, bit IsByte = 0> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809ArithmeticBaseSetOverflowUse<dst, src, (ins !cast<Operand>(vtype#"imm"):$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809ArithmeticBaseSetOverflowUse<dst, src, (ins INDEX16:$idx, unknown:$offset)>;
    let Predicates = [IsHD6309] in {
      if !eq(IsByte, 1) then {
        def _Reg  : MC6809ArithmeticBaseSetOverflowUseRegByteB<(ins ABc:$src2)>;
        // Bug #221 Phase A: parallel A-half pseudo (currently unused).
        def _RegA : MC6809ArithmeticBaseSetOverflowUseRegByteA<(ins AAc:$src2)>;
      } else {
        def _Reg : MC6809ArithmeticBaseSetOverflowUse<dst, src, (ins src:$src2)>;
      }
    }
  }
}

let isAdd = true in {
  defm AddSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8, /*IsByte=*/1>;
  defm AddSetCarryUse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;
  // Bug #297: i32 carry-in chain participant. Used by G_UADDE i32
  // selector (commit 4) to model the second word's add-with-carry.
  let Predicates = [IsHD6309] in
    defm AddSetCarryUse_i32 : MC6809ArithmeticCarryUse<i32, ACC32, ACC32>;
}

defm SubSetCarryUse_i8 : MC6809ArithmeticCarryUse<i8, ACC8, ACC8, /*IsByte=*/1>;
defm SubSetCarryUse_i16 : MC6809ArithmeticCarryUse<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in
  defm SubSetCarryUse_i32 : MC6809ArithmeticCarryUse<i32, ACC32, ACC32>;

// Bug #147: chain-extend variants for *signed* G_SADDE / G_SSUBE.
// Uses=[V] on the *Use base class.
let isAdd = true in {
  defm AddSetOverflowUse_i8  : MC6809ArithmeticOverflowUse<i8,  ACC8,  ACC8, /*IsByte=*/1>;
  defm AddSetOverflowUse_i16 : MC6809ArithmeticOverflowUse<i16, ACC16, ACC16>;
  // Bug #297: i32 signed-overflow chain participant.
  let Predicates = [IsHD6309] in
    defm AddSetOverflowUse_i32 : MC6809ArithmeticOverflowUse<i32, ACC32, ACC32>;
}
defm SubSetOverflowUse_i8  : MC6809ArithmeticOverflowUse<i8,  ACC8,  ACC8, /*IsByte=*/1>;
defm SubSetOverflowUse_i16 : MC6809ArithmeticOverflowUse<i16, ACC16, ACC16>;
let Predicates = [IsHD6309] in
  defm SubSetOverflowUse_i32 : MC6809ArithmeticOverflowUse<i32, ACC32, ACC32>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete arithmetic instructions (ADDA, ADDB, ADDD, SUBA, SUBB, SUBD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809ArithPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node reg:$src, imm:$val)), (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val)>;
  def _Idx_Imm : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Indirect : Pat<(vtype (node reg:$src, (am_indexed_indirect INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_MemIndirect") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Unmerge_Idx_Imm : Pat<(vtype (node reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Reg8 : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(vtype (node reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset)>;
  let Predicates = [IsHD6309] in {
    def _Reg : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)>;
  }
  let Predicates = [NotHD6309] in {
    def _Push_Pull : Pat<(vtype (node reg:$src, reg:$src2)), (!cast<Instruction>(inst#"_"#vtype#"_Pull") reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2))>;
  }
}

multiclass MC6809ArithRevPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Imm : Pat<(vtype (node imm:$val, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Imm") reg:$src, imm:$val))>;
  def _Idx_Imm : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Idx_Indirect : Pat<(vtype (node (am_indexed_indirect INDEX16:$idx, i16imm:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_MemIndirect") reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Unmerge_Idx_Imm : Pat<(vtype (node (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, imm:$offset))>;
  def _Idx_Reg8 : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC8:$offset))>;
  def _Idx_Reg16 : Pat<(vtype (node (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Mem") reg:$src, INDEX16:$idx, ACC16:$offset))>;
  let Predicates = [IsHD6309] in {
    def _Reg : Pat<(vtype (node reg:$src2, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, reg:$src2)))>;
  }
  let Predicates = [NotHD6309] in {
    def _Push_Pull : Pat<(vtype (node reg:$src2, reg:$src)), (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>("Negate_"#vtype) (!cast<Instruction>(inst#"_"#vtype#"_Pull") reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2))))>;
  }
}

defm AddPat_i8 : MC6809ArithPat<add, i8, ACC8, "Add">;
defm SubPat_i8 : MC6809ArithPat<sub, i8, ACC8, "Sub">;
defm SubRevPat_i8 : MC6809ArithRevPat<sub, i8, ACC8, "Sub">;

defm AddPat_ii6 : MC6809ArithPat<add, i16, ACC16, "Add">;
defm SubPat_ii6 : MC6809ArithPat<sub, i16, ACC16, "Sub">;
defm SubRevPat_ii6 : MC6809ArithRevPat<sub, i16, ACC16, "Sub">;

// Bug #297 commit 4/6 (2026-05-15): native HD6309 i32 ADD/SUB pattern
// instantiation, inline (i.e. NOT via MC6809ArithPat).  The shared
// multiclass would reference `Push_i32` for the _Push_Pull NotHD6309
// arm, but native i32 arith is HD6309-only — defining Push_i32 just to
// satisfy TableGen's eager `!cast` would create a never-selected
// 32-bit-to-stack pseudo with no meaningful semantics on plain MC6809.
// Inline patterns sidestep the issue and keep the surface minimal:
// only the forms a legalizer-flipped G_ADD/G_SUB s32 will produce.
// Carry-out / carry-in chain patterns (AddSetCarry_i32 etc.) are
// emitted by the C++ selector in selectAddO / selectAddE (commit 4),
// not by these patterns.
let Predicates = [IsHD6309] in {
  // G_ADD i32 → Add_i32_*
  def AddPat_i32_Imm : Pat<(i32 (add ACC32:$src, imm:$val)),
                            (Add_i32_Imm ACC32:$src, imm:$val)>;
  def AddPat_i32_Idx_Imm : Pat<(i32 (add ACC32:$src,
                                      (am_indexed_imm_offset INDEX16:$idx,
                                                             i16imm:$offset))),
                                (Add_i32_Mem ACC32:$src, INDEX16:$idx, imm:$offset)>;
  def AddPat_i32_Reg : Pat<(i32 (add ACC32:$src, ACC32:$src2)),
                            (Add_i32_Reg ACC32:$src, ACC32:$src2)>;

  // G_SUB i32 → Sub_i32_*
  def SubPat_i32_Imm : Pat<(i32 (sub ACC32:$src, imm:$val)),
                            (Sub_i32_Imm ACC32:$src, imm:$val)>;
  def SubPat_i32_Idx_Imm : Pat<(i32 (sub ACC32:$src,
                                      (am_indexed_imm_offset INDEX16:$idx,
                                                             i16imm:$offset))),
                                (Sub_i32_Mem ACC32:$src, INDEX16:$idx, imm:$offset)>;
  def SubPat_i32_Reg : Pat<(i32 (sub ACC32:$src, ACC32:$src2)),
                            (Sub_i32_Reg ACC32:$src, ACC32:$src2)>;
}

// LEA-based i16 add/sub for INDEX registers is handled in
// MC6809InstructionSelector.cpp (hand-lowering required for GlobalISel
// because constants are in registers, not bare immediates).

//===----------------------------------------------------------------------===//
// Bitwise (AND/OR/XOR) Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809BitwiseBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = !con((ins reg:$src), operand);
  let Constraints = "$dst = $src";
}

multiclass MC6809Bitwise<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809BitwiseBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  // Bug #196: AND/OR/XOR are mathematically commutative; the post-RA
  // expander (expandANDReg/expandORReg/expandXORReg in
  // MC6809InstrInfo.cpp) always picks op2 as the memory-resident
  // source, so swapping op1<->op2 cleanly flips which source becomes
  // accumulator-bound vs. memory-resident. This lets
  // TwoAddressInstructionPass commute when one operand is easier to
  // keep in the accumulator across uses than the other.
  let isCommutable = 1 in {
    def _Reg : MC6809BitwiseBase<reg, (ins reg:$src2)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809BitwiseBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    // P3b: indirect-indexed (`op [n,r]`) -- see MC6809Arithmetic _MemIndirect.
    def _MemIndirect : MC6809BitwiseBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809BitwiseBase<reg, (ins STACK16:$idx)>;
  }
}

// G_AND / G_OR / G_XOR on i1 lowers to the i8 byte pseudos below.
// i1 values live in ACC8 with bit 0 carrying the boolean; the high
// 7 bits are already 0 on a well-formed i1, so AND[A|B] / OR[A|B] /
// EOR[A|B] act correctly on bit 0.
defm AND_i8 : MC6809Bitwise<i8, ACC8>;
defm OR_i8 : MC6809Bitwise<i8, ACC8>;
defm XOR_i8 : MC6809Bitwise<i8, ACC8>;
defm AND_i16 : MC6809Bitwise<i16, ACC16>;
defm OR_i16 : MC6809Bitwise<i16, ACC16>;
defm XOR_i16 : MC6809Bitwise<i16, ACC16>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete bitwise instructions (ANDA, ANDB, ORA, ORB, EORA, EORB, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---
// Bitwise patterns reuse MC6809ArithPat from the Arithmetic family above.

defm ANDPat_i8 : MC6809ArithPat<and, i8, ACC8, "AND">;
defm ORPat_i8 : MC6809ArithPat<or, i8, ACC8, "OR">;
defm XORPat_i8 : MC6809ArithPat<xor, i8, ACC8, "XOR">;
defm ANDPat_i16 : MC6809ArithPat<and, i16, ACC16, "AND">;
defm ORPat_i16 : MC6809ArithPat<or, i16, ACC16, "OR">;
defm XORPat_i16 : MC6809ArithPat<xor, i16, ACC16, "XOR">;

//===----------------------------------------------------------------------===//
// Shift/Rotate Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809ShiftBase<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src, i8imm:$val);
  let Constraints = "$dst = $src";
}

multiclass MC6809Shift<RegisterClass reg> {
  def _Reg : MC6809ShiftBase<reg>;
}

defm LSL_i8 : MC6809Shift<ACC8_AB>;
defm LSL_i16 : MC6809Shift<ADc>;

defm ASR_i8 : MC6809Shift<ACC8_AB>;
defm ASR_i16 : MC6809Shift<ADc>;
defm LSR_i8 : MC6809Shift<ACC8_AB>;
defm LSR_i16 : MC6809Shift<ADc>;

// Rotate pseudos for carry-chain shifts (multi-byte shift-by-1).
// ROL: carry → bit 0, bit 7 → carry. Used for hi byte of left shift chain.
// ROR: carry → bit 7, bit 0 → carry. Used for lo byte of right shift chain.
defm ROL_i8 : MC6809Shift<ACC8_AB>;
defm ROR_i8 : MC6809Shift<ACC8_AB>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete shift instructions (ASLA, ASLB, ASRA, ASRB, LSLA, LSLB, LSRA, LSRB, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

multiclass MC6809ShiftPat<SDNode node, ValueType vtype, RegisterClass reg, string inst> {
  def _Reg : Pat<(vtype (node reg:$src, (i8 imm:$val))), (!cast<Instruction>(inst#"_"#vtype#"_Reg") reg:$src, imm:$val)>;
}

defm LSL_Pat_i8 : MC6809ShiftPat<shl, i8, ACC8_AB, "LSL">;
defm LSL_Pat_i16 : MC6809ShiftPat<shl, i16, ADc, "LSL">;

defm ASR_Pat_i8 : MC6809ShiftPat<sra, i8, ACC8_AB, "ASR">;
defm ASR_Pat_i16 : MC6809ShiftPat<sra, i16, ADc, "ASR">;
defm LSR_Pat_i8 : MC6809ShiftPat<srl, i8, ACC8_AB, "LSR">;
defm LSR_Pat_i16 : MC6809ShiftPat<srl, i16, ADc, "LSR">;

//===----------------------------------------------------------------------===//
// Compare Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809CompareBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  // Bug #186 follow-up Phase 2b NOTE (2026-04-28): the CC.C clobber on
  // CMP/CMPB/CMPD/etc. is INTENTIONALLY NOT declared via `let Defs`
  // here, because the explicit `outs CCond:$dst` already represents
  // the new value of CC. Declaring Defs=[NZ,V,C] on top double-counts:
  // regalloc tries to allocate the CCond vreg to $cc and trips on
  // the now-imaginary implicit-def of the same physregs at the same
  // MI ("Some interferences are not with virtual registers" → "ran
  // out of registers" at -O1+). The CCond output IS the honest
  // declaration of CC clobber for these forms.
  //
  // The fused compare-branch / test-branch forms (CompareBranchBase,
  // TestBranchBase) DO have explicit Defs=[NZ,V,C] — they have no
  // CCond output, so the Defs are the only way to declare the CMP
  // clobber. See those classes.
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc, reg:$src), operand);
  let isCompare = true;
}

multiclass MC6809Compare<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809CompareBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  def _Reg : MC6809CompareBase<reg, (ins reg:$src2)>;
  let mayLoad = true in {
    def _Mem : MC6809CompareBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    // P3b: indirect-indexed compare (`cmp [n,r]`) -- expanded to the indirect-
    // sibling opcode via indirectSiblingOf in expandCompareIdx.
    def _MemIndirect : MC6809CompareBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809CompareBase<reg, (ins STACK16:$idx)>;
  }
}

defm Compare_i8 : MC6809Compare<i8, ACC8>;
defm Compare_i16 : MC6809Compare<i16, ACC16>;
defm Compare_ptr: MC6809Compare<i16, INDEX16>;

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete compare instructions (CMPA, CMPB, CMPD, CMPX, CMPY, CMPU, CMPS, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

// Bug #311 Phase 1 step 1.3 WIP (2026-05-20): pattern result reshaped
// to (i8 (anyext (i1 (setcc ...)))) so ConditionalImm's new ACC8 dst
// types cleanly.  Drops the standalone `(i1 (setcc ...))` shape — the
// selector C++ arm (step 1.3 follow-up, not yet landed) handles that
// shape for -O0 direct G_ICMP s1.  See plan
// ~/.claude/plans/jiggly-orbiting-grove.md Phase 1 step 1.3.
multiclass MC6809SetIfCondPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _i8_Imm : Pat<(i8 (anyext (i1 (setcc reg:$src, imm:$val, CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Imm") cc, reg:$src, imm:$val), 1, 0)>;
  def _i8_Idx_Imm : Pat<(i8 (anyext (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset), 1, 0)>;
  def _i8_Unmerge_Idx_Imm : Pat<(i8 (anyext (i1 (setcc reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset), 1, 0)>;
  def _i8_Idx_Reg8 : Pat<(i8 (anyext (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC8:$offset), 1, 0)>;
  def _i8_Idx_Reg16 : Pat<(i8 (anyext (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC16:$offset), 1, 0)>;
  let Predicates = [IsHD6309] in {
    def _i8_Reg : Pat<(i8 (anyext (i1 (setcc reg:$src, reg:$src2, CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Reg") cc, reg:$src, reg:$src2), 1, 0)>;
  }
  let Predicates = [NotHD6309] in {
    def _i8_Push_Pull : Pat<(i8 (anyext (i1 (setcc reg:$src, reg:$src2, CC)))), (ConditionalImm cc, (!cast<Instruction>("Compare_"#vtype#"_Pull") cc, reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2)), 1, 0)>;
  }
}

multiclass MC6809SetIfCompCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809SetIfCondPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809SetIfCondPat<CC, ccode, i16, ACC16>;
  defm _ptr : MC6809SetIfCondPat<CC, ccode, i16, INDEX16>;
}

defm SetIfCompPat_HI : MC6809SetIfCompCondPat<SETUGT, 2>;
defm SetIfCompPat_HS : MC6809SetIfCompCondPat<SETUGE, 4>;
defm SetIfCompPat_LS : MC6809SetIfCompCondPat<SETULE, 3>;
defm SetIfCompPat_LO : MC6809SetIfCompCondPat<SETULT, 5>;
defm SetIfCompPat_GT : MC6809SetIfCompCondPat<SETGT, 14>;
defm SetIfCompPat_GE : MC6809SetIfCompCondPat<SETGE, 12>;
defm SetIfCompPat_LE : MC6809SetIfCompCondPat<SETLE, 15>;
defm SetIfCompPat_LT : MC6809SetIfCompCondPat<SETLT, 13>;
defm SetIfCompPat_EQ : MC6809SetIfCompCondPat<SETEQ, 7>;
defm SetIfCompPat_NE : MC6809SetIfCompCondPat<SETNE, 6>;

//===----------------------------------------------------------------------===//
// Test Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809TestBase<dag operand> : MC6809LogicalInstr {
  // CCond output represents CC clobber — see MC6809CompareBase note.
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc), operand);
  let isCompare = true;
}

multiclass MC6809Test<RegisterClass reg, list<Register> MemDefs = []> {
  def _Reg : MC6809TestBase<(ins reg:$src)>;
  let mayLoad = true, Defs = MemDefs in {
    def _Mem : MC6809TestBase<(ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm Test_i8 : MC6809Test<ACC8>;
let Predicates = [IsHD6309] in {
  // Test_i16_Mem expansion (see MC6809InstrInfo.cpp:expandTestMem) uses LDD,
  // since no TST-memory-16 exists on MC6809 or HD6309. AD is clobbered.
  defm Test_i16 : MC6809Test<ACC16, [AD]>;
}

// --- Fused compare-and-branch pseudos (bug #42 fix) ---
// These combine a compare/test with a conditional branch into a single
// instruction so the register allocator cannot insert spill reloads between
// them (which would clobber the CC flags on the 6809, where all loads set NZV).
// CC is internal — not exposed as an output register.
// Expanded post-RA into separate Compare/Test + ConditionalBranchRelative.

class MC6809TestBranchBase<dag operand> : MC6809LogicalInstr {
  // Bug #186 follow-up Phase 2b (2026-04-28): honest CC.C clobber.
  // The TST at the front of the fused test-branch writes
  // CC.{N,Z,V,C}. See MC6809CompareBase for the rationale.
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins condcode:$cc), operand, (ins label:$tgt));
  let isBranch = true;
  let isTerminator = true;
}

multiclass MC6809TestBranch<RegisterClass reg, list<Register> MemDefs = []> {
  def _Reg : MC6809TestBranchBase<(ins reg:$src)>;
  let mayLoad = true, Defs = MemDefs in {
    def _Mem : MC6809TestBranchBase<(ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm TestBranch_i8 : MC6809TestBranch<ACC8>;
// TestBranch_i16_Mem splits (post-RA) into Test_i16_Mem + LBxx, and
// Test_i16_Mem expands to LDD on both MC6809 and HD6309 — hence AD clobber.
defm TestBranch_i16 : MC6809TestBranch<ACC16, [AD]>;

class MC6809CompareBranchBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  // Bug #186 follow-up Phase 2b (2026-04-28): honest CC.C clobber.
  // The CMP at the front of the fused compare-branch writes
  // CC.{N,Z,V,C}. See MC6809CompareBase for the rationale.
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins condcode:$cc, reg:$src), operand, (ins label:$tgt));
  let isBranch = true;
  let isTerminator = true;
}

multiclass MC6809CompareBranch<ValueType vtype, RegisterClass reg> {
  def _Imm : MC6809CompareBranchBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  def _Reg : MC6809CompareBranchBase<reg, (ins reg:$src2)>;
  let mayLoad = true in {
    def _Mem : MC6809CompareBranchBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    // P3b: indirect-indexed fused compare-branch (`cmp [n,r]; b<cc>`).
    def _MemIndirect : MC6809CompareBranchBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
    def _Pull : MC6809CompareBranchBase<reg, (ins STACK16:$idx)>;
  }
}

defm CompareBranch_i8 : MC6809CompareBranch<i8, ACC8>;
defm CompareBranch_i16 : MC6809CompareBranch<i16, ACC16>;
// Bug #359: index-domain fused compare-branch against an immediate. Selected
// (via selectImpl) for a pointer compared against a constant once
// RegisterBankInfo banks the operand INDEX, so it realises CMPX/CMPY/CMPU
// #imm (including #0 for a null test) instead of moving into D. Only the
// immediate form: register/memory pointer compares are rare and the _Reg /
// _Pull forms have no Push_ptr / index reg-reg compare on base MC6809.
def CompareBranch_ptr_Imm : MC6809CompareBranchBase<INDEX16, (ins i16imm:$imm)>;
// Index-domain fused compare-branch of two pointers. Keeps both pointers in
// index registers so the compare is HD6309 CMPR X,Y (copy-free) or, on base
// 6809, PSHS one + CMPX ,S++ — either way avoiding the bank-cross transfer to
// D that an accumulator-bank reg-reg compare of two pointers would force.
def CompareBranch_ptr_Reg : MC6809CompareBranchBase<INDEX16, (ins INDEX16:$src2)>;
// Index-domain fused compare-branch of a pointer against a memory operand.
// foldMemoryOperandImpl folds a spilled CompareBranch_ptr_Reg operand into this
// form (CMPX/CMPY off,r vs the spill slot) so the compare needs only one index
// register — avoiding the two-index-register pressure of CMPR that otherwise
// reloads both spilled operands through the single IY staging register.
let mayLoad = true in
def CompareBranch_ptr_Mem : MC6809CompareBranchBase<INDEX16, (ins INDEX16:$idx, unknown:$offset)>;

// Bug #301 Phase D step 3.1 (2026-05-16 night): native HD6309 i32
// compare-and-branch fused pseudo.  Single MachineInstr covering
// SUBW #low16 + SBCD #high16 + LB<cc> $tgt.  Mirrors the
// MC6809CompareBranchBase shape for i16, just at i32 width.
//
// Why a SINGLE fused MI?  The reverted Phase D step 3 attempt
// (Compare_i32_Imm + ConditionalImm + ConditionalLongBranchRelative)
// triggered Bug A: the CCond vreg has no use after post-RA expansion
// (LBlbc reads physregs, not vregs), so DCE/regalloc deleted
// Compare_i32_Imm as dead — and the resulting LBlbc reads undef
// $n/$z/$v/$c flags.  A single fused pseudo with isTerminator=true
// cannot be DCE'd as dead, and its post-RA expansion emits the SUBW,
// SBCD, and LBlbc adjacent (no scheduler can wedge a flag-clobber
// between them).
//
// Only the _Imm form lands in step 3.1.  _Reg / _Mem / _Pull follow
// in steps 5+6.
let Predicates = [IsHD6309] in {
  // Bug #321 (2026-05-22): override base's Defs=[NZ,V,C] with the
  // wider [NZ,V,C,AQ] to capture the expansion's clobbers.  The
  // post-RA expansion of this terminator emits LDQ (for SPILL_Q*N
  // src) + SUBW + SBCD, writing AW and AD individually.  Without
  // AQ in Defs, regalloc could place an unrelated vreg in $aq
  // across this branch and the successor BBs would read the
  // clobbered value — the Bug #311 pattern.  AQ as super-reg
  // covers AD/AW/AA/AB/AE/AF.
  let Defs = [NZ, V, C, AQ] in
    def CompareBranch_i32_Imm
      : MC6809CompareBranchBase<ACC32, (ins i32imm:$imm)>;
}

// Bug #301 (2026-05-16): native HD6309 i32 compare-and-set fused
// pseudo.  Single MachineInstr covering SUBW #low16 + SBCD #high16
// + per-predicate branch-free CC bit extraction to a 0/1 byte in
// ACC8 $dst.  Mirrors the philosophy of X86's SETcc / AArch64's CSET
// / ARM's MOVCC — every use site re-executes the compare, no
// cross-BB CCond live range.
//
// Used when an i1 result of i32 G_ICMP (non-eq/ne, constant RHS) is
// materialized (not branched/selected).  The CompareBranch_i32_Imm
// fused-branch path above is preferred when available; this is the
// fallback for G_STORE i1, G_TRUNC i1→byte, G_AND/OR/XOR i1,
// G_ZEXT/SEXT i1→wider chains, or any other shape where the i1
// value itself is consumed.
//
// Single fused MI like CompareBranch_i32_Imm — same DCE-immunity
// properties (isCompare=true, can't be DCE'd as dead, SUBW+SBCD+
// extraction emitted contiguously).
let Predicates = [IsHD6309] in {
  def CompareSet_i8_i32_Imm : MC6809Pseudo {
    let isCompare = true;
    dag OutOperandList = (outs ACC8:$dst);
    dag InOperandList  = (ins condcode:$cc, ACC32:$src, i32imm:$imm);
    // Bug #321 (2026-05-22): same AQ clobber as CompareBranch_i32_Imm
    // / EqZero_i32 / SignTest_i32.  Expansion LDQs for spill src,
    // SUBW+SBCD writes AW+AD, then TFR/ANDA/LSRA writes AA — all of
    // these are sub-regs of AQ.  Without AQ in Defs, regalloc could
    // leave an unrelated value live in $aq across this pseudo.
    let Defs = [NZ, V, C, AQ];
  }
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete test instructions (TSTA, TSTB, TSTD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

// Bug #311 Phase 1 step 1.3 WIP (2026-05-20): see SetIfCondPat above.
multiclass MC6809TestPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(i8 (anyext (i1 (setcc (vtype 0), reg:$src, CC)))), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Reg") cc, reg:$src), 1, 0)>;
  def _Idx_Imm : Pat<(i8 (anyext (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset), 1, 0)>;
  def _Unmerge_Idx_Imm : Pat<(i8 (anyext (i1 (setcc (vtype 0), (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset), 1, 0)>;
  def _Idx_Reg8 : Pat<(i8 (anyext (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC8:$offset), 1, 0)>;
  def _Idx_Reg16 : Pat<(i8 (anyext (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)))), (ConditionalImm cc, (!cast<Instruction>("Test_"#vtype#"_Mem") cc, INDEX16:$idx, ACC16:$offset), 1, 0)>;
}

multiclass MC6809TestCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestPat<CC, ccode, i8, ACC8>;
  let Predicates = [IsHD6309] in {
    defm _i16 : MC6809TestPat<CC, ccode, i16, ACC16>;
  }
}

defm TestPat_HI : MC6809TestCondPat<SETUGT, 2>;
defm TestPat_HS : MC6809TestCondPat<SETUGE, 4>;
defm TestPat_LS : MC6809TestCondPat<SETULE, 3>;
defm TestPat_LO : MC6809TestCondPat<SETULT, 5>;
defm TestPat_GT : MC6809TestCondPat<SETGT, 14>;
defm TestPat_GE : MC6809TestCondPat<SETGE, 12>;
defm TestPat_LE : MC6809TestCondPat<SETLE, 15>;
defm TestPat_LT : MC6809TestCondPat<SETLT, 13>;
defm TestPat_EQ : MC6809TestCondPat<SETEQ, 7>;
defm TestPat_NE : MC6809TestCondPat<SETNE, 6>;

//===----------------------------------------------------------------------===//
// BitTest Family
//===----------------------------------------------------------------------===//

// --- Pseudo-instructions (register-agnostic, GlobalISel) ---

class MC6809BitTestBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  // CCond output represents CC clobber — see MC6809CompareBase note.
  dag OutOperandList = (outs CCond:$dst);
  dag InOperandList = !con((ins i8imm:$cc, reg:$src), operand);
  let isCompare = true;
}

multiclass MC6809BitTest<ValueType vtype, RegisterClass reg> {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809BitTestBase<reg, (ins !cast<Operand>(vtype#"imm"):$imm)>;
  }
  def _Reg : MC6809BitTestBase<reg, (ins reg:$src2)>;
  let mayLoad = true in {
    def _Mem : MC6809BitTestBase<reg, (ins INDEX16:$idx, unknown:$offset)>;
  }
}

defm BitTest_i8 : MC6809BitTest<i8, ACC8_AB>;
let Predicates = [IsHD6309] in {
  // Was ACC16_D (1-register = {AD}) before the Layer-2 regalloc experiment
  // (bug #118). See `project_regalloc_two_layer_bottleneck.md`: the real
  // D-specific constraint is handled by the post-RA expansion routing
  // through AD, not by pinning the operand class. Using ACC16 here lets
  // the regalloc pick any 16-bit imaginary register and relieves the
  // coalescer's tightening to a 1-register intersection.
  defm BitTest_i16 : MC6809BitTest<i16, ACC16>;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete bit-test instructions (BITA, BITB, BITD, etc.)
// are defined in MC6809InstrInfo.td.

// --- Patterns (connect LLVM IR to pseudo-instructions) ---

//===----------------------------------------------------------------------===//
// Multiply Family
//===----------------------------------------------------------------------===//

// --- 8-bit multiply pseudo-instruction ---
// The 6809 MUL instruction multiplies A × B → D (16-bit result).
// We model this as a single pseudo taking D as input/output to avoid
// the D/A/B register aliasing problem: if A and B are separate operands,
// the register allocator can't handle the fact that loading one (via LDD
// or TFR) clobbers the other through D.
//
// Patterns use REG_SEQUENCE to pack the two i8 operands into D, then
// EXTRACT_SUBREG to pull the result byte out.

class MC6809MultiplyD : MC6809LogicalInstr {
  // Always clobbers the real accumulator (AA, AB, AD) because the expansion
  // loads into D, does MUL, then stores back — even when the operand is a
  // spill register. Without these implicit defs, the allocator could leave
  // values in A/B that get destroyed during the MUL expansion.
  let Defs = [Z, C, AA, AB, AD];
  dag OutOperandList = (outs ADc:$dst);
  dag InOperandList = (ins ADc:$src);
  let Constraints = "$dst = $src";
}

def MUL_D : MC6809MultiplyD;

// --- 8-bit multiply patterns ---
// Pack two i8 operands into D via MERGE_LOHI_i16, multiply (MUL_D reads
// and writes AD — constrained to ADc), extract result via EXTRACT_LO/HI_i16.
// Going through the byte-composition pseudos avoids sub-register COPY edges
// between ACC8 and ACC16 in the virtual-register graph — bug #118 Layer 1,
// approach b.
// mul  → low byte of the product
// mulhu/mulhs → high byte of the product
def MulPat_i8 : Pat<(i8 (mul ACC8:$b, ACC8:$a)),
  (EXTRACT_LO_i16 (MUL_D (MERGE_LOHI_i16 ACC8:$b, ACC8:$a)))>;

def MulHUPat_i8 : Pat<(i8 (mulhu ACC8:$b, ACC8:$a)),
  (EXTRACT_HI_i16 (MUL_D (MERGE_LOHI_i16 ACC8:$b, ACC8:$a)))>;

def MulHSPat_i8 : Pat<(i8 (mulhs ACC8:$b, ACC8:$a)),
  (EXTRACT_HI_i16 (MUL_D (MERGE_LOHI_i16 ACC8:$b, ACC8:$a)))>;

// --- 16-bit multiply pseudo-instructions (6309 only) ---

// Bug #161 round 14: widen $dst from AWc to ACC16 so back-to-back
// multiplies in i64 multiply chains don't all collide on the single AW
// allocation slot. Post-RA expansion emits a TFR W,<dst> after the MULD
// when the regalloc-assigned dst isn't AW.
//
// Bug #251 (2026-05-09): MULD on HD6309 clobbers BOTH AD (high half) and
// AW (low half = the multiply result). Defs must list AW too, otherwise
// regalloc may keep an unrelated acc16 value in AW across the pseudo and
// silently lose it when MULD overwrites AW with the LOW product.
// Concrete case: picolibc's calloc has `elem` (the second arg of
// __builtin_mul_overflow) regalloc'd to AW; MULD destroys it; the
// subsequent `max(elem, 1)` overflow-check divisor reads the freshly-
// written LOW product instead of `elem`, calloc(SIZE_MAX, SIZE_MAX) no
// longer detects overflow.
class MC6809MultiplyBase<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, AD, AW];
  dag InOperandList = !con((ins ADc:$src), operand);
  dag OutOperandList = (outs ACC16:$dst);
}

multiclass MC6809Multiply {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809MultiplyBase<(ins i16imm:$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809MultiplyBase<(ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809MultiplyBase<(ins ACC16:$src2)>;
  }
}

let Predicates = [IsHD6309] in {
  defm Multiply_i16 : MC6809Multiply;
}

// Bug #251: same Defs widening as MC6809MultiplyBase — MULD clobbers
// both halves of the AQ register file.
class MC6809MultiplyHighBase<dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, AD, AW];
  dag OutOperandList = (outs ADc:$dst);
  dag InOperandList = !con((ins ADc:$src), operand);
}

multiclass MC6809MultiplyHigh {
  let isAsCheapAsAMove = true in {
    def _Imm : MC6809MultiplyHighBase<(ins i16imm:$val)>;
  }
  let mayLoad = true in {
    def _Mem : MC6809MultiplyHighBase<(ins INDEX16:$idx, unknown:$offset)>;
    def _Reg : MC6809MultiplyHighBase<(ins ACC16:$src2)>;
  }
}

let Predicates = [IsHD6309] in {
  defm MultiplyHigh_i16 : MC6809MultiplyHigh;
}

// --- Concrete instructions (register-specific, assembler/disassembler) ---
// Concrete multiply instructions (MUL, MULD, etc.)
// are defined in MC6809InstrInfo.td.

// --- 16-bit multiply patterns ---

multiclass MC6809MultiplyPat<SDNode node, string inst> {
  def _Imm : Pat<(i16 (node ADc:$src, imm:$val)), (!cast<Instruction>(inst#"_i16_Imm") ADc:$src, imm:$val)>;
  def _Idx_Imm : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, imm:$offset)>;
  def _Unmerge_Idx_Imm : Pat<(i16 (node ADc:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, imm:$offset)>;
  def _Idx_Reg8 : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, ACC8:$offset)>;
  def _Idx_Reg16 : Pat<(i16 (node ADc:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset))), (!cast<Instruction>(inst#"_i16_Mem") ADc:$src, INDEX16:$idx, ACC16:$offset)>;
  def _Reg : Pat<(i16 (node ADc:$src, ACC16:$src2)), (!cast<Instruction>(inst#"_i16_Reg") ADc:$src, ACC16:$src2)>;
}

let Predicates = [IsHD6309] in {
  defm MulPat_i16 : MC6809MultiplyPat<mul, "Multiply">;
  defm MulHSPat_i16 : MC6809MultiplyPat<mulhs, "MultiplyHigh">;
  defm MulHUPat_i16 : MC6809MultiplyPat<mulhu, "MultiplyHigh">;
}

//===----------------------------------------------------------------------===//
// Fused Compare/Branch Patterns
//===----------------------------------------------------------------------===//

multiclass MC6809CompBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg, bit IsIndex = 0> {
  if !eq(IsIndex, 0) then {
    def _Imm : Pat<(brcond (i1 (setcc reg:$src, imm:$val, CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Imm") cc, reg:$src, imm:$val, bb:$tgt)>;
    def _Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, i16imm:$offset, bb:$tgt)>;
    def _Unmerge_Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, imm:$offset, bb:$tgt)>;
    def _Idx_Reg8 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC8:$offset, bb:$tgt)>;
    def _Idx_Reg16 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Mem") cc, reg:$src, INDEX16:$idx, ACC16:$offset, bb:$tgt)>;
    let Predicates = [IsHD6309] in {
      def _Reg : Pat<(brcond (i1 (setcc reg:$src, reg:$src2, CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Reg") cc, reg:$src, reg:$src2, bb:$tgt)>;
    }
    let Predicates = [NotHD6309] in {
      def _Push_Pull : Pat<(brcond (i1 (setcc reg:$src, reg:$src2, CC)), bb:$tgt), (!cast<Instruction>("CompareBranch_"#vtype#"_Pull") cc, reg:$src, (!cast<Instruction>("Push_"#vtype) reg:$src2), bb:$tgt)>;
    }
  } else {
    // Index/pointer compare-branch. Prefer folding a memory operand into the
    // compare's own addressing mode (CMPX/CMPY off,r) — set up here at ISel so
    // that operand never needs its own index register, keeping later register
    // pressure low. Only when BOTH operands are live registers does it become
    // CompareBranch_ptr_Reg (CMPR X,Y on HD6309, PSHS one + CMPX ,s++ on base
    // 6809 via expandCompareReg) — one form for both CPUs, no _Pull.
    def _Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (CompareBranch_ptr_Mem cc, reg:$src, INDEX16:$idx, i16imm:$offset, bb:$tgt)>;
    def _Unmerge_Idx_Imm : Pat<(brcond (i1 (setcc reg:$src, (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (CompareBranch_ptr_Mem cc, reg:$src, INDEX16:$idx, imm:$offset, bb:$tgt)>;
    def _Idx_Reg8 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (CompareBranch_ptr_Mem cc, reg:$src, INDEX16:$idx, ACC8:$offset, bb:$tgt)>;
    def _Idx_Reg16 : Pat<(brcond (i1 (setcc reg:$src, (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (CompareBranch_ptr_Mem cc, reg:$src, INDEX16:$idx, ACC16:$offset, bb:$tgt)>;
    def _Reg : Pat<(brcond (i1 (setcc reg:$src, reg:$src2, CC)), bb:$tgt),
                   (CompareBranch_ptr_Reg cc, reg:$src, reg:$src2, bb:$tgt)>;
  }
}

multiclass MC6809CompBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809CompBraPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809CompBraPat<CC, ccode, i16, ACC16>;
  // Bug #359: index-domain pointer compare-branch, immediate RHS only.  Once
  // RegisterBankInfo banks the pointer operand INDEX, the _i16 (ACC16) pattern
  // can't match it, so this fires instead and realises CMPX/CMPY #imm.
  // Written directly rather than via MC6809CompBraPat because that multiclass
  // uses vtype for both the value type and the instruction-name suffix and so
  // can't express an i16-typed operand with a ptr-named instruction.
  def _ptr_Imm : Pat<(brcond (i1 (setcc INDEX16:$src, imm:$val, CC)), bb:$tgt),
                     (CompareBranch_ptr_Imm ccode, INDEX16:$src, imm:$val, bb:$tgt)>;
  // Index-domain pointer compare-branch, register RHS: keeps both pointers in
  // the index bank instead of bank-crossing into D. IsIndex=1 selects the
  // CompareBranch_ptr_Reg form (vtype is unused in that path).
  defm _ptr : MC6809CompBraPat<CC, ccode, i16, INDEX16, /*IsIndex=*/1>;
}

defm CompBraPat_HI : MC6809CompBraCondPat<SETUGT, 2>;
defm CompBraPat_HS : MC6809CompBraCondPat<SETUGE, 4>;
defm CompBraPat_LS : MC6809CompBraCondPat<SETULE, 3>;
defm CompBraPat_LO : MC6809CompBraCondPat<SETULT, 5>;
defm CompBraPat_GT : MC6809CompBraCondPat<SETGT, 14>;
defm CompBraPat_GE : MC6809CompBraCondPat<SETGE, 12>;
defm CompBraPat_LE : MC6809CompBraCondPat<SETLE, 15>;
defm CompBraPat_LT : MC6809CompBraCondPat<SETLT, 13>;
defm CompBraPat_EQ : MC6809CompBraCondPat<SETEQ, 7>;
defm CompBraPat_NE : MC6809CompBraCondPat<SETNE, 6>;

//===----------------------------------------------------------------------===//
// Fused Test/Branch Patterns
//===----------------------------------------------------------------------===//

multiclass MC6809TestBraPat<CondCode CC, int cc, ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(brcond (i1 (setcc reg:$src, (vtype 0), CC)), bb:$tgt), (!cast<Instruction>("TestBranch_"#vtype#"_Reg") cc, reg:$src, bb:$tgt)>;
  def _Idx_Imm : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (!cast<Instruction>("TestBranch_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset, bb:$tgt)>;
  def _Unmerge_Idx_Imm : Pat<(brcond (i1 (setcc (vtype 0), (am_unmerge_indexed_imm_offset INDEX16:$idx, i16imm:$offset), CC)), bb:$tgt), (!cast<Instruction>("TestBranch_"#vtype#"_Mem") cc, INDEX16:$idx, i16imm:$offset, bb:$tgt)>;
  def _Idx_Reg8 : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC8:$offset), CC)), bb:$tgt), (!cast<Instruction>("TestBranch_"#vtype#"_Mem") cc, INDEX16:$idx, ACC8:$offset, bb:$tgt)>;
  def _Idx_Reg16 : Pat<(brcond (i1 (setcc (vtype 0), (am_indexed_imm_offset INDEX16:$idx, ACC16:$offset), CC)), bb:$tgt), (!cast<Instruction>("TestBranch_"#vtype#"_Mem") cc, INDEX16:$idx, ACC16:$offset, bb:$tgt)>;
}

multiclass MC6809TestBraCondPat<CondCode CC, int ccode> {
  defm _i8 : MC6809TestBraPat<CC, ccode, i8, ACC8>;
  defm _i16 : MC6809TestBraPat<CC, ccode, i16, ACC16>;
}

defm TestBraPat_HI : MC6809TestBraCondPat<SETUGT, 2>;
defm TestBraPat_HS : MC6809TestBraCondPat<SETUGE, 4>;
defm TestBraPat_LS : MC6809TestBraCondPat<SETULE, 3>;
defm TestBraPat_LO : MC6809TestBraCondPat<SETULT, 5>;
defm TestBraPat_GT : MC6809TestBraCondPat<SETGT, 14>;
defm TestBraPat_GE : MC6809TestBraCondPat<SETGE, 12>;
defm TestBraPat_LE : MC6809TestBraCondPat<SETLE, 15>;
defm TestBraPat_LT : MC6809TestBraCondPat<SETLT, 13>;
defm TestBraPat_EQ : MC6809TestBraCondPat<SETEQ, 7>;
defm TestBraPat_NE : MC6809TestBraCondPat<SETNE, 6>;
