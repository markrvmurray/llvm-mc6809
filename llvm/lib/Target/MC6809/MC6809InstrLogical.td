//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
// By examining its register operands, every logical instruction can be mapped
// to exactly one real target instruction. Pseudoinstructions where this
// property does not hold are recorded in MC6809InstrPseudos.td instead. These are
// more traditional pseudo-instructions; they may expand out to short snippets
// of code required to perform higher-level operations that cannot be expaneded
// inline, for one reason or another.
//
// There are some exceptions; Compare is tied to CondtionalBranch where possible,
// and some wide instructions are treated as single instructions.
//
//===---------------------------------------------------------------------===//
class MC6809LogicalInstr : Instruction {
  let Namespace = "MC6809";

  dag OutOperandList = (outs);
  dag InOperandList = (ins);

  let hasSideEffects = false;
  let mayLoad = false;
  let mayStore = false;
  let isPseudo = true;
  let isCodeGenOnly = true;
  let isReMaterializable = true;
}

//===---------------------------------------------------------------------===//
// Inter-register copies and exchanges
//===---------------------------------------------------------------------===//
class MC6809Transfer<dag ins, dag outs, string constraints = ""> : MC6809LogicalInstr {
  dag OutOperandList = outs;
  dag InOperandList = ins;
  let Constraints = constraints;
}

def Copy8 : MC6809Transfer<(ins REGPAIR8:$src), (outs REGPAIR8:$dst), "">;
def Copy16 : MC6809Transfer<(ins REGPAIR16:$src), (outs REGPAIR16:$dst), "">;

def Exchange8 : MC6809Transfer<(ins REGPAIR8:$src1, REGPAIR8:$src2), (outs REGPAIR8:$dst1, REGPAIR8:$dst2), "$dst1 = $src2, $dst2 = $src1">;
def Exchange16 : MC6809Transfer<(ins REGPAIR16:$src1, REGPAIR16:$src2), (outs REGPAIR16:$dst1, REGPAIR16:$dst2), "$dst1 = $src2, $dst2 = $src1">;

//===---------------------------------------------------------------------===//
// Memory block operations
//===---------------------------------------------------------------------===//
class MC6809BlockTransfer<dag ins, dag outs, string constraints = ""> : MC6809LogicalInstr {
  dag OutOperandList = outs;
  dag InOperandList = ins;
  let Defs = [AW];
  let Uses = [AW];
  let Constraints = constraints;
  let Predicates = [IsHD6309];
  // The TFM block transfer copies memory; the only register def the
  // pseudo carries is the count register (AW), and that's not used
  // downstream. Without overriding MC6809LogicalInstr's
  // hasSideEffects/mayLoad/mayStore=false defaults,
  // DeadMachineInstructionElim concludes the BlockCopy is dead and
  // removes it — silently breaking every memcpy lowered via
  // tryTFMBlockCopy. The companion legalizer change
  // (MC6809LegalizerInfo::tryTFMBlockCopy) ensures src/dst arrive
  // as INDEX16 vregs so PEI's eliminateFrameIndex doesn't trip on
  // the operand layout.
  let hasSideEffects = true;
  let mayLoad = true;
  let mayStore = true;
  // Re-emitting a BlockCopy elsewhere would re-execute the memory
  // transfer — definitely not safe to rematerialize.
  let isReMaterializable = false;
}

// BlockCopy_* are emitted by the legalizer (tryTFMBlockCopy) with two REGTFM
// ({IX,IY}) register operands; the byte count lives in AW, which is implicit
// via MC6809BlockTransfer's Defs/Uses. The legalizer puts the count into AW
// right before the pseudo, so the count is modelled the way the real TFM
// models it (implicit W), not as an explicit operand. An explicit AWc:$count
// slot would have to be filled with either an immediate (illegal in a register
// operand) or a vreg that doubles the implicit AW use, both of which the
// machine verifier rejects.
//
// The pointer operands are tied def+use ($src = $src_wb, $dst = $dst_wb): the
// TFM post-increments / -decrements both pointers as it runs, so it *clobbers*
// the registers. Modelling that write is essential -- without it the coalescer
// would happily merge a pointer that is still live after the copy into the
// register the TFM destroys (e.g. tmpnam's strcpy(s,"TXXXXXX") followed by
// mktemp(s)), silently corrupting it. The write-back vregs are dead; the tie
// just forces the interference that keeps a live-after pointer in its own
// register.
def BlockCopy_Inc_Inc : MC6809BlockTransfer<(ins REGTFM:$src, REGTFM:$dst), (outs REGTFM:$src_wb, REGTFM:$dst_wb), "$src = $src_wb, $dst = $dst_wb">;
def BlockCopy_Dec_Dec : MC6809BlockTransfer<(ins REGTFM:$src, REGTFM:$dst), (outs REGTFM:$src_wb, REGTFM:$dst_wb), "$src = $src_wb, $dst = $dst_wb">;
def BlockCopy_Inc_Stay : MC6809BlockTransfer<(ins REGTFM:$src, REGTFM:$dst), (outs REGTFM:$src_wb, REGTFM:$dst_wb), "$src = $src_wb, $dst = $dst_wb">;
def BlockCopy_Stay_Inc : MC6809BlockTransfer<(ins REGTFM:$src, REGTFM:$dst), (outs REGTFM:$src_wb, REGTFM:$dst_wb), "$src = $src_wb, $dst = $dst_wb">;

//===---------------------------------------------------------------------===//
// Stack instructions. These will be squashed together later.
//===---------------------------------------------------------------------===//

// The Push_*/Pull_* pseudo pairs and their STACK16 plumbing are gone: the
// _Reg operand forms cover reg-reg arith and compares on both CPUs with the
// push/op/pop emitted atomically inside one post-RA expansion, so nothing
// creates a separated push-to-pull window any more (the shape whose window
// the register allocator filled with mis-offset S-relative spill code).

//===---------------------------------------------------------------------===//
// Operandless Accumulator Instructions
//===---------------------------------------------------------------------===//
class MC6809Accumulator<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ, C, V];
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
  let Constraints = "$dst = $src";
}

def Negate_i8 : MC6809Accumulator<ACC8>;
let Predicates = [IsHD6309] in {
  def Negate_i16 : MC6809Accumulator<ACC16>;
}

multiclass MC6809Negate1Pat<ValueType vtype, RegisterClass reg> {
  def _Reg : Pat<(vtype (sub (vtype 0), reg:$src)), (!cast<Instruction>("Negate_"#vtype) reg:$src)>;
}

defm NegPat_i8 : MC6809Negate1Pat<i8, ACC8>;
let Predicates = [IsHD6309] in {
  defm NegPat_i16 : MC6809Negate1Pat<i16, ACC16>;
}

let InOperandList = (ins), Constraints = "" in {
  def Clear_i8 : MC6809Accumulator<ACC8>;
  let Predicates = [IsHD6309] in {
    def Clear_i16 : MC6809Accumulator<ACC16>;
  }
}

//===---------------------------------------------------------------------===//
// Loads and Stores
//===---------------------------------------------------------------------===//
// Load pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// Store pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

//===---------------------------------------------------------------------===//
// Set/Clear CC bits
//===---------------------------------------------------------------------===//

// Arithmetic pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// Bitwise pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// Shift/Rotate pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

//===---------------------------------------------------------------------===//
// CC register
//===---------------------------------------------------------------------===//

class MC6809ConditionCodes<RegisterClass cc> : MC6809LogicalInstr {
  dag OutOperandList = (outs cc:$dst);
  dag InOperandList = (ins cc:$src, i8imm:$val);
  let Constraints = "$dst = $src";
}

def ORCC_Imm : MC6809ConditionCodes<CCond>;
def ANDCC_Imm : MC6809ConditionCodes<CCond>;

// Multiply pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

// Unconditional relative branches

class MC6809Branch : MC6809LogicalInstr {
  let isBranch = true;
  let isTerminator = true;
}

class MC6809UnconditionalBranch : MC6809Branch {
  let isBarrier = true;
  dag InOperandList = (ins label:$tgt);
}

def BranchRelative : MC6809UnconditionalBranch;
def LongBranchRelative : MC6809UnconditionalBranch {
  // Default to LONG branches everywhere. A future relaxation pass will
  // promote them back to short BRA when the offset fits — until then,
  // emitting short by default risks silent truncation. Was bug #58.
  let Pattern = [(br bb:$tgt)];
}

def JumpRelative : MC6809UnconditionalBranch;
def LongJumpRelative : MC6809UnconditionalBranch;

// Branches to $tgt if $flag = $val.  $flag is an ACC8 byte holding
// 0/1; the post-RA expansion compares bit 0.
def BranchFlagIs : MC6809Branch {
  dag InOperandList = (ins label:$tgt, ACC8:$flag, i1imm:$val);
}

// Conditional relative branches

class MC6809ConditionalBranch : MC6809Branch {
  //let Uses = [NZVC, NZ, N, Z, V, C];
  dag InOperandList = (ins condcode:$cc, label:$tgt, CCond:$bits);
}

// Covers all conditional branch instructions. Branches to $tgt if $cc is true.
def ConditionalBranchRelative : MC6809ConditionalBranch;
def ConditionalLongBranchRelative : MC6809ConditionalBranch;

// Subroutines

class MC6809Subroutine : MC6809LogicalInstr {
  let isCall = true;
  dag InOperandList = (ins label:$tgt);
}

// Branch via jump table: index must be materializable to D, table
// identified by JTI. Expanded post-RA to ASLB+ROLA+LEAX table,PCR
// +LDD D,X+JMP D,X — that sequence is hardcoded against D, so the
// $idx operand is constrained to ADc (AD + SPILL_D0..7 only).
// Constraining to plain ACC16 here was bug #68: regalloc was free
// to pick IX/IY, and the expansion blindly shifted whatever D held.
// Not marked isBranch to avoid branch relaxation analysis (which
// can't handle indirect branches with no single destination).
// Defs matches what the ASLB+ROLA+LEAX+LDD+JMP expansion in
// MC6809AsmPrinter actually clobbers. Without this, regalloc
// treats D as preserved across the dispatch and extends live
// ranges into the jump-table successors — the "preserved" value
// in landing pads is then the jump offset, not the source value.
// Bug #127c (2026-04-20): __i_vfprintf's `c` variable was being
// overwritten by 0xF111 (a jump offset) via a spill cascade.
//
// Bug #187 (2026-04-27): explicit `let Size = 10` so BranchRelaxation
// accounts for the AsmPrinter expansion's true byte length:
//   ASLB(1) + ROLA(1) + LEAXi_o16PC(4) + LDDi_oD(2) + JMPi_oD(2) = 10
// Without this, MCID.getSize() returned 0 and getInstSizeInBytes
// fell back to MaxInstLength (5), under-counting every BJT by 5
// bytes. BR's block-size accounting then concluded short branches
// across BJT-containing blocks were in range when they weren't,
// emitting Bbc that overflowed the PCRel8 fixup at MC encoding
// time. Surfaced first by vfprintf_s.c at -O2 where a Bbc landed
// at exactly +128 encoded displacement (one byte over).
let isBranch = false, isTerminator = true, isBarrier = true in
def BranchJumpTable : MC6809LogicalInstr {
  let isIndirectBranch = true;
  let Defs = [AA, AB, AD, IX, NZ, N, Z, V, C];
  let Size = 10; // ASLB+ROLA+LEAX+LDD+JMP — see MC6809AsmPrinter::emitInstruction
  dag InOperandList = (ins ADc:$idx, unknown:$jti);
}

// PseudoInstExpansion-based pseudos: their MCID size defaults to 0
// because TableGen doesn't infer a size from the expansion target.
// Declare it explicitly so getInstSizeInBytes returns the real
// post-expansion byte length, which BranchRelaxation needs to size
// blocks correctly. Was bug #174 / #183: undeclared sizes silently
// undercounted block sizes during BR.
def BranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(BSRb pcrel8:$tgt)> {
  let Size = 2; // → BSRb
}
def LongBranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(LBSRlb pcrel16:$tgt)> {
  let Size = 3; // → LBSRlb
}

def JumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o8PC pcrel8_idx:$tgt)> {
  let Size = 3; // → JSRi_o8PC
}
def LongJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o16PC pcrel16_idx:$tgt)> {
  let Size = 4; // → JSRi_o16PC
}
def IndirectJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_eI addr16_o2:$tgt)> {
  let isIndirectBranch = true;
  dag InOperandList = (ins addr16_o2:$tgt);
  let Size = 4; // → JSRi_eI
}

class MC6809Return<Instruction Opcode> : MC6809LogicalInstr, PseudoInstExpansion<(Opcode)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
  let Size = 1; // → RTSr ($39) / RTIr ($3B), both single-byte
}

def ReturnImplicit : MC6809Return<RTSr>;
def ReturnIRQImplicit : MC6809Return<RTIr>;

// Other unconditional jumps

def JumpAbsolute : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPe addr16:$tgt)> {
  let InOperandList = (ins label:$tgt);
  let Size = 3; // → JMPe ($7E + addr16)
}
def JumpIndir : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_eI addr16_o2:$tgt)> {
  let isIndirectBranch = true;
  let InOperandList = (ins label:$tgt);
  let Size = 4; // → JMPi_eI
}

def TailJump : MC6809LogicalInstr, PseudoInstExpansion<(LBRAlb pcrel16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isBarrier = true;
  let isTerminator = true;
  let isCall = true;
  let Size = 3; // → LBRAlb
  let isReturn = true;
}

// Compare pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// Test pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// BitTest pseudos, concrete instructions, and patterns are in MC6809InstrFamilies.td

// Fused Compare/Branch patterns are in MC6809InstrFamilies.td

// Fused Test/Branch patterns are in MC6809InstrFamilies.td

//===---------------------------------------------------------------------===//
// Pointer instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  dag InOperandList = !con((ins idx:$src), operand);
  let isAdd = true;
  let Constraints = "$dst = $src";
}

// LEAr n,r or LEAr a,r
def LEAPtrAdd_Imm : MC6809PtrAdd<INDEX16, (ins i16imm:$offset)>;
def LEAPtrAdd_Reg8 : MC6809PtrAdd<INDEX16, (ins ACC8:$offset)>;
def LEAPtrAdd_Reg16 : MC6809PtrAdd<INDEX16, (ins ACC16:$offset)>;

def LEAPtrAddPatImm : Pat<(i16 (ptradd INDEX16:$src, (i16 imm:$offset))), (LEAPtrAdd_Imm INDEX16:$src, imm:$offset)>;
def LEAPtrAddPatReg8 : Pat<(i16 (ptradd INDEX16:$src, ACC8:$offset)), (LEAPtrAdd_Reg8 INDEX16:$src, ACC8:$offset)>;
def LEAPtrAddPatReg16 : Pat<(i16 (ptradd INDEX16:$src, ACC16:$offset)), (LEAPtrAdd_Reg16 INDEX16:$src, ACC16:$offset)>;

class MC6809LoadEffectiveAddress<RegisterClass idx, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  dag InOperandList = !con((ins idx:$src), operand);
  let mayLoad = true;
}

// LEAr n,r or LEAr a,r
def LEA_Ptr_Imm : MC6809LoadEffectiveAddress<INDEX16, (ins i16imm:$offset)>;
def LEA_Ptr_Reg8 : MC6809LoadEffectiveAddress<INDEX16, (ins ACC8:$offset)>;
def LEA_Ptr_Reg16 : MC6809LoadEffectiveAddress<INDEX16, (ins ACC16:$offset)>;

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

// Bug #142: every ZEX*Implicit / SEX*Implicit pseudo expands post-RA to a
// real instruction that touches CC. Declaring those Defs (and hasSideEffects)
// is required by the rule "a pseudo's Defs must cover every reg/CC its
// expansion clobbers" — see MC6809InstrFamilies.td:234-279 (bug #57 design)
// for why the carry-chain pseudos rely on hasSideEffects barriers, and
// what happens when an extend sneaks between chain MIs without one.

def SEX8Implicit : MC6809LogicalInstr {
  // Expansion: AND[A|B] #1 + NEG[A|B] (or TFR + …).  NEG sets NZVC.
  //
  // dst is constrained to ACC8_AB ({AB, AA}) — the two registers the
  // AND/NEG expansion actually operates on. This makes the pseudo's
  // explicit dst operand the honest clobber declaration and removes
  // the old `AD` entry from Defs. That entry (the Bug #283/#284
  // WorkReg protection for an AE/AF dst) aliased BOTH byte halves, so
  // once the byte classes lost their SPILL_* members the dst had no
  // assignable register at all ("Some interferences are not with
  // virtual registers" → ran out of registers). With dst limited to
  // A/B the expansion's WorkReg path is unreachable: it works on the
  // dst itself. An AE/AF-bound consumer costs one TFR after the
  // pseudo instead.
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
  // $src is ACC8; the byte's LSB carries the i1 value.  No SDAG
  // Pattern — TableGen rejects `(i1 ACC8:$src)` when i1 and i8 share
  // a register class.  G_SEXT s1→s8 is selected via a C++ intercept
  // in MC6809InstructionSelector.
  dag InOperandList = (ins ACC8:$src);
  dag OutOperandList = (outs ACC8_AB:$dst);
}

def SEX16Implicit : MC6809LogicalInstr {
  // Expansion: SEX.  Sets NZ; V undefined on 6809, cleared on 6309 —
  // declare V for safety (no-wrong-code if hardware leaves V untouched).
  //
  // Bug #283-shape audit (Bug #284): the SEXx hardware instruction
  // sign-extends AB into AA, producing the i16 result in AD. The
  // physical AD register IS the destination computation path, even
  // when regalloc binds $dst to a SPILL_D* slot (in which case the
  // post-RA expansion does an STD from AD to the spill slot after
  // SEXx). If AD isn't listed in Defs, regalloc could leave AD live
  // with an unrelated value across this pseudo and SEXx would
  // silently destroy it. Mirrors the SEX32Implicit fix (Bug #247).
  //
  // 2026-06-28 (indexed-mode churn fix): produce the result in PHYSICAL $ad
  // and let selection append `%dst = COPY $ad` (see the G_SEXT i8->i16 C++
  // intercept). Previously a vreg dst (ADc) PLUS a separate implicit AD def:
  // that implicit def is a fixed physical dead-def at the def slot, so greedy's
  // interference check vetoes assigning the dst vreg to AD -> it parks the value
  // in $spill_d0 -> MaterializeSpills turns it into real stack traffic (the a[i]
  // churn). Defining $ad physically (a) makes the output ALWAYS cover AA, so the
  // Bug #283/#284 unrelated-AA-clobber protection holds for every dst (verified:
  // dropping the Defs without this breaks 6309 test-memmem), and (b) removes the
  // self-interference. The COPY coalesces to AD when free (no spill) or lowers
  // to a single STD when contended. Uses=[AB]: the src low byte is preserved as
  // AD's low half (MOVZX-style super-reg def reading its own sub-reg).
  let Defs = [NZ, V, AD];
  let Uses = [AB];
  let hasSideEffects = 1;
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
}

def SEX32Implicit : MC6809LogicalInstr {
  // Expansion: SEXW (HD6309).  Sets NZ; V cleared.  Hardware SEXW
  // sign-extends AW into AD, so the expansion physically clobbers
  // AD (and AA/AB via subreg hierarchy).
  // Bug #247 (same shape as #208 round 2 / #211 for ZEX16Implicit /
  // ZEX32Implicit): when regalloc binds dst to a SPILL_Q* slot the
  // OutOperand only marks the spill slot as written — physical AD
  // is not covered.  An i16 value live in AD across this pseudo
  // (e.g. test-timegm's `actual` operand of an i16 != i16 compare)
  // would survive the pseudo per regalloc's view, then get
  // overwritten by SEXW with sign(W), so the downstream cmpr saw 0
  // instead of the real value -> "got 0 want N" across all
  // tm_h/m/s checks.  Fix: list AD in Defs so regalloc spills any
  // live AD value across this pseudo regardless of dst binding.
  // (When dst lands in AQ the OutOperand already covers AD via
  // class hierarchy; the explicit Defs entry is a no-op then.)
  // Bug #274: widened $src to ACC16 (mirroring ZEX32Implicit per
  // Bug #161) so an i16 produced by unrelated codegen — typically
  // MERGE_LOHI_i16 landing in ADc — can feed this pseudo without
  // tripping -verify-machineinstrs. The expansion routes every
  // non-AW source through copyPhysReg before SEXWx.
  //
  // Bug #283-shape audit (Bug #284): the expansion ALSO clobbers AW
  // when the source isn't already in AW (via the pre-SEXWx
  // copyPhysReg call). Without AW in Defs the regalloc could leave
  // AW live with an unrelated value across this pseudo, and the
  // src→AW copy would destroy it.
  let Defs = [NZ, V, AD, AW];
  let hasSideEffects = 1;
  dag InOperandList = (ins ACC16:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (sext ACC16:$src))];
  let Predicates = [IsHD6309];
}

def ZEX8Implicit : MC6809LogicalInstr {
  // Expansion: AND[A|B] #1.  AND[A|B] sets NZ and clears V; C unchanged.
  // No SDAG Pattern — same TableGen i1/i8 type-binding rejection as
  // SEX8Implicit above.  G_ZEXT s1→s8 is selected via a C++ intercept.
  let Defs = [NZ, V];
  let hasSideEffects = 1;
  dag InOperandList = (ins ACC8:$src);
  dag OutOperandList = (outs ACC8:$dst);
}

def ZEX16Implicit : MC6809LogicalInstr {
  // Expansion: CLRA.  Sets N=0 Z=1 V=0 C=0 AND writes AA=0.
  //
  // Without these Defs (and hasSideEffects) the pre-RA scheduler
  // reorders ZEX16Implicit into the middle of a Sub*Carry chain,
  // post-RA CLRA clears CC.C between two SBCAs, and the high-byte
  // borrow vanishes — bug #142.
  //
  // Bug #283: AA must be in Defs too. The expansion emits CLRAa
  // which clobbers AA. Without AA listed, the register allocator
  // could leave AA live across this pseudo with an unrelated value
  // (e.g. the high byte of a function argument loaded into D, where
  // only the low byte AB is consumed by ZEX16Implicit's $src). The
  // post-RA CLRA then silently destroys that value, corrupting
  // downstream uses. Manifest at -O2 + -mcpu=hd6309 in memmem's
  // `if (ne_len == 1) return memchr(hs, ne[0], hs_len);` codegen —
  // hs_len's high byte was held in AA across the ZEX16Implicit of
  // ne[0] and got zeroed, passing memchr a truncated length.
  //
  // 2026-06-28 (indexed-mode churn fix): produce the result in PHYSICAL $ad and
  // let selection append `%dst = COPY $ad` (see the G_ZEXT i8->i16 C++ intercept).
  // The old vreg dst (ACC16) + implicit AA def self-interfered: AA is a sub-reg
  // of AD, so its fixed physical dead-def at the def slot made greedy veto
  // assigning the dst vreg to AD -> $spill_d0 -> MaterializeSpills stack churn on
  // a[i]. Defining $ad physically makes the output ALWAYS cover AA, so the Bug
  // #283 protection (an unrelated AA value must not be left live across the CLRA)
  // holds for EVERY dst -- not just dst==AD. (Verified necessary: merely dropping
  // AA from Defs makes 6309 test-memmem fail again.) Uses=[AB]: src low byte is
  // AD's low half. C kept for the Bug #142 scheduler hazard.
  let Defs = [NZ, V, C, AD];
  let Uses = [AB];
  let hasSideEffects = 1;
  dag InOperandList = (ins);
  dag OutOperandList = (outs);
}

def ZEX32Implicit : MC6809LogicalInstr {
  // Expansion: CLRD (HD6309), preceded by a copyPhysReg that lands the
  // i16 source in AW unless it is already there.  Sets N=0 Z=1 V=0 C=0.
  // Same scheduler hazard as ZEX16Implicit above, just for the 32-bit
  // case.
  // Bug #161: widened $src to ACC16 so an i16 produced by unrelated
  // codegen (e.g. MERGE_LOHI_i16 in ADc) can feed this pseudo.
  // Bug #208: the post-RA expansion routes every non-AW source (AD,
  // RS0..RS3, and any future ACC16 member that survives
  // MaterializeSpills) through copyPhysReg before CLRDa, so AQ always
  // ends up as (D=0 | W=src).  The earlier code only handled AD and
  // silently corrupted the low half whenever regalloc placed the
  // source in an RSn imaginary register.
  //
  // Bug #283-shape audit (Bug #284): expansion clobbers AD via CLRDa
  // AND AW via the src→AW copyPhysReg (when src != AW). The
  // OutOperand only covers AQ (and AD/AW via super-reg) when dst=AQ;
  // for SPILL_Q* dst the OutOperand covers only the spill slot. Add
  // AD and AW to Defs so regalloc spills any live values in those
  // physical registers across this pseudo.
  let Defs = [NZ, V, C, AD, AW];
  let hasSideEffects = 1;
  dag InOperandList = (ins ACC16:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (zext ACC16:$src))];
  let Predicates = [IsHD6309];
}

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

// i1 lives in ACC8 (the byte's LSB carries the value; see
// MC6809ISelLowering's MVT::i1 binding).  G_ANYEXT s1→s8 / G_ZEXT
// s1→s8 from phantom-carry producers (carry/overflow chains) is
// routed by the selector via the MaterializeCC_* pseudos; non-
// phantom s1→s8 from byte-LSB sources is a plain byte COPY.
//
// i16 anyext from i8 -- go through ANYEXT_i8_to_i16 pseudo so the
// coalescer never sees a sub_lo_byte COPY edge between ACC8 and ACC16.
// Bug #118 Layer 1, approach b.
def : Pat<(i16 (anyext ACC8:$byte)), (ANYEXT_i8_to_i16 ACC8:$byte)>;

// i8 trunc from i16 — go through EXTRACT_LO_i16 pseudo, same reason.
def : Pat<(i8 (trunc ADc:$word)), (EXTRACT_LO_i16 ADc:$word)>;
let Predicates = [IsHD6309] in {
  // Bug #302 redesign Phase 2 (2026-05-17): i16 trunc from i32 goes
  // through Extract16_i32_lo (Phase 1's replacement for
  // EXTRACT_LO_word_i32).  Same post-RA expansion: AQ via TFR W,D,
  // SPILL_Q*N via LDD slot+2,$su.
  def : Pat<(i16 (trunc ACC32:$dword)), (Extract16_i32_lo ACC32:$dword)>;
  // Bug #302 redesign Phase 2 (2026-05-17): i32 anyext from i16
  // goes through AnyExt32_i16 (a Phase 2 addition).  Replaces the
  // INSERT_SUBREG-via-sub_lo_word pattern that breaks when Phase 3
  // drops AQ's sub-register hierarchy.  The upper word stays
  // undefined per IR anyext semantics.
  def : Pat<(i32 (anyext ACC16:$word)), (AnyExt32_i16 ACC16:$word)>;
}

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Oddball instructions
//===---------------------------------------------------------------------===//

