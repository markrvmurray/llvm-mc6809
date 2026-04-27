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

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a BIT1/Flag.
def SelectImm : MC6809Pseudo {
  dag OutOperandList = (outs unknown:$dst);
  dag InOperandList = (ins BIT1:$tst, i1imm:$true, i1imm:$false);

  let usesCustomInserter = true;
}

// Use the diamond control flow pattern to load one of two immediates based on
// the value of a conditional - same as is used in the Bbc $cond instruction..
def ConditionalImm : MC6809Pseudo {
  dag OutOperandList = (outs BIT1:$dst);
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
  // $dst constrained to ABc so, when $src is allocated to AD, the extract
  // maps to AD's natural low-byte sub-register AB and expansion is a no-op.
  // Without this, regalloc may pick $dst in AAc (= AD's hi half); expansion
  // then emits `TFR A,B` which overwrites the low byte we still need for a
  // sibling EXTRACT_HI_i16 of the same AD. (bug #118 Layer 1, approach b;
  // failure seen at -Oz in picolibc test-strchr where `ptr - haystack`
  // compiled as `ptr - ptr` because haystack_lo was clobbered between
  // EXTRACT_HI and EXTRACT_LO of the same source AD.)
  def EXTRACT_LO_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ABc:$dst);
    dag InOperandList  = (ins ADc:$src);
    let isAsCheapAsAMove = true;
  }

  // Symmetric: $dst in AAc so, when $src = AD, extract maps to AD's hi half
  // AA and expansion is a no-op. See EXTRACT_LO_i16 comment above.
  def EXTRACT_HI_i16 : MC6809Pseudo {
    dag OutOperandList = (outs AAc:$dst);
    dag InOperandList  = (ins ADc:$src);
    let isAsCheapAsAMove = true;
  }

  // $lo constrained to ABc (AB-half + B-spills), $hi to AAc (AA-half +
  // A-spills) so regalloc cannot collapse both inputs into the same
  // physical byte register. Without this, it will happily allocate both
  // to $ab, then emit reloads that overwrite each other before MERGE
  // expansion runs — the resulting code loses the low byte entirely.
  // (bug #118 Layer 1, approach b; failure seen in picolibc's memcpy at
  // -Oz where the i16 counter increment split into byte halves merged
  // back through two AB reloads.)
  def MERGE_LOHI_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ABc:$lo, AAc:$hi);
    let isAsCheapAsAMove = true;
  }

  // The source byte becomes the low byte of the i16 result; high byte is
  // left undefined. Constrain to ABc so the byte naturally lives in the
  // low half of AD without a staging TFR in the common case.
  def ANYEXT_i8_to_i16 : MC6809Pseudo {
    dag OutOperandList = (outs ADc:$dst);
    dag InOperandList  = (ins ABc:$src);
    let isAsCheapAsAMove = true;
  }
}

// Materialise the CC.C flag into byte 0/1. Used when a phantom BIT1 vreg
// (the carry-out of an AddSetCarry*/SubSetCarry*/AddSetCarryUse*/
// SubSetCarryUse* pseudo) must be consumed AS A BYTE by a subsequent
// anyext / zext / sext / test, not as another carry-in.
//
// The BIT1 vreg from those pseudos is a scheduling phantom — see the long
// comment on MC6809ArithmeticBaseCarry in MC6809InstrFamilies.td. The
// physical byte-LSB allocated to the vreg does NOT hold the carry bit;
// the only authoritative source is CC.C. Consumers that read byte-LSB
// (copyPhysReg BIT1→ACC8, SEX8Implicit, TableGen anyext BIT1→i8 patterns)
// therefore emit wrong code for phantom BIT1; they work only by accident
// when regalloc happens to pick an LSB whose byte value matches the carry.
//
// This pseudo is the honest materialisation: post-RA it expands to
// `LDB #0; ADCB #0`, which reads CC.C (produced by the phantom's
// expansion) and deposits it in $dst as a proper 0/1 byte.
//
// Defs=[NZ,V,C] + hasSideEffects=1: the pseudo reads CC.C, so a
// scheduling barrier is needed to prevent any C-clobbering instruction
// from being moved between the phantom producer and this materialisation.
// The same hasSideEffects barrier on the SetCarry/SetCarryUse pseudos
// pins the other side of the gap (bug #57 mechanism).
//
// $dst constrained to ABc so the expansion is always LDB/ADCB and never
// needs a staging TFR (6309's AE/AF have no ADCE/ADCF instruction).
def MaterializeCarryToByte_i8 : MC6809Pseudo {
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList  = (ins BIT1:$src);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}

// Bug #147 sibling of MaterializeCarryToByte_i8: materialise the V flag
// (signed overflow) into a 0/1 byte. The BIT1 src is a scheduling phantom
// whose runtime value is in CC.V — set by AddSetOverflow*/SubSetOverflow*
// pseudos (same MC expansion as the SetCarry siblings; only the s1 semantic
// differs — V vs C). $dst constrained to ABc so the expansion always lands
// in $ab. Post-RA expansion: TFR CC,B; LSRB; ANDB #1 — branch-free.
def MaterializeOverflowToByte_i8 : MC6809Pseudo {
  dag OutOperandList = (outs ABc:$dst);
  dag InOperandList  = (ins BIT1:$src);
  let Defs = [NZ, V, C];
  let hasSideEffects = 1;
}

// Bug #184 inverse of MaterializeCarryToByte_i8: restore CC.C from a 0/1
// byte that was previously frozen via MaterializeCarryToByte_i8. Used to
// preserve a SubSetCarry chain's borrow across a CC-clobbering region
// (e.g. a CompareBranch or any other MI that clobbers C between a
// SubSetCarry producer and its SubSetCarryUse consumer).
//
// $src constrained to ABc so the expansion is always LSRB on $ab —
// LSRB shifts bit 0 of B into CC.C and zeroes B (we don't care, B's value
// is consumed here).
//
// Defs=[NZ,V,C] + hasSideEffects=1: same scheduling barrier as the
// to-byte sibling — pin the restoration to its MIR position so no
// further C-clobber slips between this and the consuming SubSetCarryUse.
def MaterializeByteToCarry_i8 : MC6809Pseudo {
  dag OutOperandList = (outs);
  dag InOperandList  = (ins ABc:$src);
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
}

