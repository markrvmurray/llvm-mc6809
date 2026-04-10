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
}

def BlockCopy_Inc_Inc : MC6809Transfer<(ins REGTFM:$src1, REGTFM:$dst1, AWc:$count), (outs REGTFM:$src2, REGTFM:$dst2, AWc:$count1), "$dst1 = $dst2, $src1 = $src2, $count = $count1">;
def BlockCopy_Dec_Dec : MC6809Transfer<(ins REGTFM:$src1, REGTFM:$dst1, AWc:$count), (outs REGTFM:$src2, REGTFM:$dst2, AWc:$count1), "$dst1 = $dst2, $src1 = $src2, $count = $count1">;
def BlockCopy_Inc_Stay : MC6809Transfer<(ins REGTFM:$src1, REGTFM:$dst, AWc:$count), (outs REGTFM:$src2, AWc:$count1), "$src1 = $src2, $count = $count1">;
def BlockCopy_Stay_Inc : MC6809Transfer<(ins REGTFM:$src, REGTFM:$dst1, AWc:$count), (outs REGTFM:$dst2, AWc:$count1), "$dst1 = $dst2, $count = $count1">;

//===---------------------------------------------------------------------===//
// Stack instructions. These will be squashed together later.
//===---------------------------------------------------------------------===//

class MC6809StackPush<RegisterClass index, RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [SS];
  let Uses = [SS];
  dag OutOperandList = (outs index:$stack);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

class MC6809StackPull<RegisterClass index, RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [SS];
  let Uses = [SS];
  dag OutOperandList = (outs index:$stack, reg:$reg);
  dag InOperandList = (ins);
  let mayLoad = true;
}

def Push_i8 : MC6809StackPush<STACK16, ACC8>;
def Pull_i8 : MC6809StackPull<STACK16, ACC8>;
def Push_i16 : MC6809StackPush<STACK16, ACC16>;
def Pull_i16 : MC6809StackPull<STACK16, ACC16>;
def Push_Ptr : MC6809StackPush<STACK16, INDEX16>;
def Pull_Ptr : MC6809StackPull<STACK16, INDEX16>;

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

// Branches to $tgt if $flag = $val.
def BranchFlagIs : MC6809Branch {
  dag InOperandList = (ins label:$tgt, BIT1:$flag, i1imm:$val);
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
let isBranch = false, isTerminator = true, isBarrier = true in
def BranchJumpTable : MC6809LogicalInstr {
  let isIndirectBranch = true;
  dag InOperandList = (ins ADc:$idx, unknown:$jti);
}

def BranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(BSRb pcrel8:$tgt)>;
def LongBranchSubroutine : MC6809Subroutine, PseudoInstExpansion<(LBSRlb pcrel16:$tgt)>;

def JumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o8PC pcrel8_idx:$tgt)>;
def LongJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_o16PC pcrel16_idx:$tgt)>;
def IndirectJumpSubroutine : MC6809Subroutine, PseudoInstExpansion<(JSRi_eI addr16:$tgt)> {
  let isIndirectBranch = true;
  dag InOperandList = (ins addr16:$tgt);
}

class MC6809Return<Instruction Opcode> : MC6809LogicalInstr, PseudoInstExpansion<(Opcode)> {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def ReturnImplicit : MC6809Return<RTSr>;
def ReturnIRQImplicit : MC6809Return<RTIr>;

// Other unconditional jumps

def JumpAbsolute : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPe addr16:$tgt)> {
  let InOperandList = (ins label:$tgt);
}
def JumpIndir : MC6809UnconditionalBranch, PseudoInstExpansion<(JMPi_eI addr16:$tgt)> {
  let isIndirectBranch = true;
  let InOperandList = (ins label:$tgt);
}

def TailJump : MC6809LogicalInstr, PseudoInstExpansion<(LBRAlb pcrel16:$tgt)> {
  dag InOperandList = (ins label:$tgt);
  let isBarrier = true;
  let isTerminator = true;
  let isCall = true;
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

def SEX8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins BIT1:$src);
  dag OutOperandList = (outs ACC8:$dst);
  let Pattern = [(set ACC8:$dst, (sext BIT1:$src))];
}

def SEX16Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ABc:$src);
  dag OutOperandList = (outs ADc:$dst);
  let Pattern = [(set ADc:$dst, (sext ABc:$src))];
}

def SEX32Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins AWc:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (sext AWc:$src))];
  let Predicates = [IsHD6309];
}

def ZEX8Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ABLSBc:$src);
  dag OutOperandList = (outs ABc:$dst);
  let Pattern = [(set ABc:$dst, (zext ABLSBc:$src))];
}

def ZEX16Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins ABc:$src);
  dag OutOperandList = (outs ACC16:$dst);
  let Pattern = [(set ACC16:$dst, (zext ABc:$src))];
}

def ZEX32Implicit : MC6809LogicalInstr {
  dag InOperandList = (ins AWc:$src);
  dag OutOperandList = (outs ACC32:$dst);
  let Pattern = [(set ACC32:$dst, (zext AWc:$src))];
  let Predicates = [IsHD6309];
}

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def : Pat<(i8 (anyext BIT1:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), BIT1:$bit, sub_lsb)>;
def : Pat<(i8 (zext CARRY:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), CARRY:$bit, sub_lsb)>;
def : Pat<(i16 (anyext ACC8:$byte)), (INSERT_SUBREG (IMPLICIT_DEF), ACC8:$byte, sub_lo_byte)>;
def : Pat<(i16 (anyext BIT1:$bit)), (INSERT_SUBREG (IMPLICIT_DEF), (INSERT_SUBREG (i8 (IMPLICIT_DEF)), BIT1:$bit, sub_lsb), sub_lo_byte)>;

def : Pat<(i8 (trunc ACC16:$word)), (EXTRACT_SUBREG ACC16:$word, sub_lo_byte)>;
let Predicates = [IsHD6309] in {
  def : Pat<(i16 (trunc ACC32:$dword)), (EXTRACT_SUBREG ACC32:$dword, sub_lo_word)>;
}

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

//===---------------------------------------------------------------------===//
// Oddball instructions
//===---------------------------------------------------------------------===//

