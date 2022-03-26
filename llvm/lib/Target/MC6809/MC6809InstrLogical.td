//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===---------------------------------------------------------------------===//
//
// This file describes the MC6809 logical instruction set.
//
// When viewed logically, the 6502 instruction set is more regular than it might
// appear. For example, consider the three instructions: LDA #imm, LDX #imm, and
// LDY #imm. Each of these instructions takes a single immediate operand, loads
// one of A, X, or Y, and sets NZ based on the immediate value. Even though
// there are three separate opcodes, the only difference in their behavior is
// which register is loaded.
//
// The logical instruction set merges these instructions together into a single
// logical LDImm8 #imm instruction. The logical instruction takes a "ACC"
// register class as an argument. When the logical instructions are lowered to
// real instructions in AsmPrinter, the register operand is examined to
// determine which real instruction is emitted. This scheme comes with some
// considerable advantages.
//
// First, selecting a LDImm8 instruction doesn't constrain where the destination
// can be placed other than that it needs to be one of A, B, E or F. This allows
// earlier passes that cannot effectively make such decisions to defer them to
// the register allocator. Similarly, LDImm16 loads any of D, W, X, Y, U or S.
//
// Second, the description of the side effects and characteristics of LDImm8 only
// need to be specified once. Any switch() statements need to have only one
// entry for LDImm8, not three for LDAImm, LDXImm, and LDYImm. This considerably
// reduces the amount of code needed to cover the 6502 instruction set.
//
// Third, exposing these regularities to LLVM passes make the 6502 more like a
// regular CISC target (X86-ish). LLVM already has very good code paths for
// dealing with this sort of architecture, and maintaining a logical instruction
// set allows us to stay within those "sane", well-tested code paths, for the
// most part.
//
// By examining its register operands, every logical instruction can be mapped
// to exactly one real target instruction. Pseudoinstructions where this
// property does not hold are recorded in MC6809InstrPseudos.td instead. These are
// more traditional pseudo-instructions; they may expand out to short snippets
// of code required to perform higher-level operations that cannot be expaneded
// inline, for one reason or another.
//
// All instructions take 1-register register classes for fixed operands: e.g.
// ACC8, INDEX16, INDEX16, etc. This allows the each instruction to natively use virtual
// registers that can be resolved by the register allocator or register
// scavenger. NZ is modeled implicitly for the most part; too many instructions
// clobber it for it to be useful outside very specific scenarios.
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
//let isReMaterializable = true;
}

//===---------------------------------------------------------------------===//
// Addressing modes
//===---------------------------------------------------------------------===//

class MC6809ArithmeticBaseCarry<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst, NZc:$nz, Vc:$vout, Cc:$carryout);
  dag InOperandList = !con((ins dst:$l, Cc:$carryin), operand);
  let Constraints = "$dst = $l, $carryout = $carryin";
}

multiclass MC6809ArithmeticCarry<string size> {
  def Imm : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
  def Reg : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<RegisterClass>(!strconcat("ACC", size)):$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins addr16:$addr)>;
    def IdxZero : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx)>;
    def IdxImm : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IndirIdxImm : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IdxReg8 : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IdxReg16 : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
    def IndirIdxReg8 : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IndirIdxReg16 : MC6809ArithmeticBaseCarry<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
  }
}

class MC6809ArithmeticBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst, NZc:$nz, Vc:$vout, Cc:$carryout);
  dag InOperandList = !con((ins dst:$l), operand);
  let Constraints = "$dst = $l";
}

multiclass MC6809Arithmetic<string size> {
  def Imm : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
  def Reg : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<RegisterClass>(!strconcat("ACC", size)):$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins addr16:$addr)>;
    def IdxZero : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx)>;
    def IdxImm : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IndirIdxImm : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IdxReg8 : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IdxReg16 : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
    def IndirIdxReg8 : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IndirIdxReg16 : MC6809ArithmeticBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
  }
}

class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst, NZc:$nz, Vc:$vout, Cc:$carryout);
  dag InOperandList = operand;
}

multiclass MC6809Load<string size> {
  let mayLoad = true in {
    def Imm : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
    def Reg : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins !cast<RegisterClass>(!strconcat("ACC", size)):$src)>;
    def Abs : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins addr16:$addr)>;
    def IdxZero : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx)>;
    def IdxImm : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IndirIdxImm : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, i16imm:$offset)>;
    def IdxReg8 : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IdxReg16 : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
    def IndirIdxReg8 : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC8:$offset)>;
    def IndirIdxReg16 : MC6809LoadBase<!cast<RegisterClass>(!strconcat("ACC", size)), (ins INDEX16:$idx, ACC16:$offset)>;
  }
}

class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  dag OutOperandList = !con(operand, (outs NZc:$nz, Vc:$vout, Cc:$carryout));
  dag InOperandList = (ins src:$src);
}

multiclass MC6809Store<string size> {
  def Reg : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs !cast<RegisterClass>(!strconcat("ACC", size)):$src)>;
  def Abs : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs addr16:$addr)>;
  def IdxZero : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx)>;
  def IdxImm : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, i16imm:$offset)>;
  def IndirIdxImm : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, i16imm:$offset)>;
  def IdxReg8 : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, ACC8:$offset)>;
  def IdxReg16 : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, ACC16:$offset)>;
  def IndirIdxReg8 : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, ACC8:$offset)>;
  def IndirIdxReg16 : MC6809StoreBase<!cast<RegisterClass>(!strconcat("ACC", size)), (outs INDEX16:$idx, ACC16:$offset)>;
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

let isAdd = true in {
  defm AddCarry8 : MC6809ArithmeticCarry<"8">;
  defm AddCarry16 : MC6809ArithmeticCarry<"16">;
  defm AddCarry32 : MC6809ArithmeticCarry<"32">;
}
defm SubBorrow8 : MC6809ArithmeticCarry<"8">;
defm SubBorrow16 : MC6809ArithmeticCarry<"16">;
defm SubBorrow32 : MC6809ArithmeticCarry<"32">;

let isAdd = true in {
  defm Add8 : MC6809Arithmetic<"8">;
  defm Add16 : MC6809Arithmetic<"16">;
  defm Add32 : MC6809Arithmetic<"32">;
}
defm Sub8 : MC6809Arithmetic<"8">;
defm Sub16 : MC6809Arithmetic<"16">;
defm Sub32 : MC6809Arithmetic<"32">;

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

defm AND8 : MC6809Arithmetic<"8">;
defm AND16 : MC6809Arithmetic<"16">;
defm OR8 : MC6809Arithmetic<"8">;
defm OR16 : MC6809Arithmetic<"16">;
defm XOR8 : MC6809Arithmetic<"8">;
defm XOR16 : MC6809Arithmetic<"16">;

//===---------------------------------------------------------------------===//
// Shift/Rotate Instructions
//===---------------------------------------------------------------------===//

class MC6809ShiftRotate : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst, Cc:$carryout);
}

class MC6809Shift : MC6809ShiftRotate {
  dag InOperandList = (ins ACC8:$src);
  let Constraints = "$dst = $src";
}
class MC6809Rotate : MC6809ShiftRotate {
  dag InOperandList = (ins ACC8:$src, Cc:$carryin);
  let Constraints = "$dst = $src, $carryout = $carryin";
}

class MC6809ShiftRotateRMW : MC6809LogicalInstr {
  dag OutOperandList = (outs Cc:$carryout);
  let mayLoad = true;
  let mayStore = true;
}

class MC6809ShiftRMW : MC6809ShiftRotateRMW {}
class MC6809RotateRMW : MC6809ShiftRotateRMW {
  let Constraints = "$carryout = $carryin";
}

def ASL : MC6809Shift;
def LSR : MC6809Shift;

def ASLAbs : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr);
}
def ASLAbsIdx : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}
def LSRAbs : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr);
}
def LSRAbsIdx : MC6809ShiftRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}

def ROL : MC6809Rotate;
def ROR : MC6809Rotate;

def ROLAbs : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, Cc:$carryin);
}
def ROLAbsIdx : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx, Cc:$carryin);
}
def RORAbs : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, Cc:$carryin);
}
def RORAbsIdx : MC6809RotateRMW {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx, Cc:$carryin);
}

//===---------------------------------------------------------------------===//
// BIT Instructions
//===---------------------------------------------------------------------===//
// Ever the odd duck.
//===---------------------------------------------------------------------===//

def BITAbs : MC6809LogicalInstr {
  dag OutOperandList = (outs Vc:$v);
  dag InOperandList = (ins ACC8:$l, i16imm:$r);
  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

class MC6809Branch : MC6809LogicalInstr {
  let isBranch = true;
  let isTerminator = false;
  let isBarrier = false;
}

// Covers all branch instructions. Branches to $tgt if $flag = $val.
class MC6809ConditionalBranch : MC6809Branch;

def IfJumpRelative : MC6809ConditionalBranch {
  dag InOperandList = (ins label:$tgt, CCFlag:$flag, i1imm:$val);
}

class MC6809UnconditionalBranch : MC6809Branch {
  let isBarrier = true;
}

def JumpRelative : MC6809UnconditionalBranch {
  let InOperandList = (ins label:$tgt);
}
def JumpIndir : MC6809UnconditionalBranch {
  let InOperandList = (ins INDEX16:$tgt);
  let isIndirectBranch = true;
}

class MC6809UnconditionalCall : MC6809Branch {
  let isCall = true;
}

def CallRelative : MC6809UnconditionalCall {
  dag InOperandList = (ins label:$tgt);
}
def CallIndir : MC6809UnconditionalCall {
  let InOperandList = (ins INDEX16:$tgt);
  let isIndirectBranch = true;
}

class MC6809Return : MC6809LogicalInstr {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
}

def ReturnImplicit : MC6809Return;
def ReturnIRQImplicit : MC6809Return;

//===---------------------------------------------------------------------===//
// Immediate Load Instructions
//===---------------------------------------------------------------------===//

//class MC6809ImmediateLoad<RegisterClass dst, Operand val> : MC6809LogicalInstr {
//  dag OutOperandList = (outs dst:$dst);
//  dag InOperandList = (ins val:$val);
//
//  let isAsCheapAsAMove = true;
//  let isMoveImm = true;
//}

// Set and clear CC bits
//def LDNImm : MC6809ImmediateLoad<Nc, i1imm>;
//def LDZImm : MC6809ImmediateLoad<Zc, i1imm>;
//def LDVImm : MC6809ImmediateLoad<Vc, i1imm>;
//def LDCImm : MC6809ImmediateLoad<Cc, i1imm>;

// All the loads
defm Load8 : MC6809Load<"8">;
defm Load16 : MC6809Load<"16">;
defm Load32 : MC6809Load<"32">;

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//

class MC6809CMP : MC6809LogicalInstr {
  dag OutOperandList = (outs Cc:$carry);
  let isCompare = true;
}

def CMPImm : MC6809CMP { dag InOperandList = (ins ACC8:$l, i8imm:$r); }

let mayLoad = true in {
  def CMPAbs : MC6809CMP { dag InOperandList = (ins ACC8:$l, i16imm:$addr); }
  def CMPAbsIdx : MC6809CMP { dag InOperandList = (ins ACC8:$l, i16imm:$addr, INDEX16:$idx); }
  def CMPIndirIdx : MC6809CMP {
    dag InOperandList = (ins ACC8:$l, ACC16:$addr, INDEX16:$idx);
  }
}

//===---------------------------------------------------------------------===//
// Increment/Decrement Instructions
//===---------------------------------------------------------------------===//

multiclass MC6809IncDec {
  def NAME : MC6809LogicalInstr {
    dag OutOperandList = (outs INDEX16:$dst);
    dag InOperandList = (ins INDEX16:$src);
    let Constraints = "$src = $dst";
  }

  let mayLoad = true, mayStore = true in {
    def CAbs : MC6809LogicalInstr {
      dag InOperandList = (ins i16imm:$addr);
    }
    def CAbsIdx : MC6809LogicalInstr {
      dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
    }
  }
}

// DEX, DEY, DEC
defm DE : MC6809IncDec;

// INX, INY, INC
let isAdd = true in {
  defm IN : MC6809IncDec;
}

//===---------------------------------------------------------------------===//
// Stack Instructions
//===---------------------------------------------------------------------===//

// PHA, PHP
def PH : MC6809LogicalInstr {
  dag InOperandList = (ins ACC8:$src);

  let mayStore = true;
}

// PLA, PLP
def PL : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst);

  let mayLoad = true;
}

//===---------------------------------------------------------------------===//
// Pointer add instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  let isAdd = true;
  let Constraints = "$dst = $src";
}

class MC6809PtrAddImm<RegisterClass idx, Operand val> : MC6809PtrAdd<idx> {
  dag InOperandList = (ins idx:$src, val:$val);
}

class MC6809PtrAddReg<RegisterClass idx, RegisterClass val> : MC6809PtrAdd<idx> {
  dag InOperandList = (ins idx:$src, val:$val);
}

// LEAr n,r or LEAr a,r
def LEAPtrAddImm : MC6809PtrAddImm<INDEX16, i16imm>;
def LEAPtrAddReg8 : MC6809PtrAddReg<INDEX16, ACC8>;
def LEAPtrAddReg16 : MC6809PtrAddReg<INDEX16, ACC16>;

//===---------------------------------------------------------------------===//
// Load/Store Instructions
//===---------------------------------------------------------------------===//

class MC6809Load<RegisterClass dst> : MC6809LogicalInstr {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins i16imm:$addr);
  let mayLoad = true;
}

// LDr abs
def  LD8Abs : MC6809Load<ACC8>;
def  LD16Abs : MC6809Load<ACC16>;

class MC6809LoadIndexedNoOffset<RegisterClass dst, RegisterClass idx> : MC6809Load<dst> {
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = (ins idx:$idx);
}

// LDr offset,x ; LDr 0,x LDr ,x
def LD8Idx : MC6809LoadIndexedNoOffset<ACC8, INDEX16>;
def LD16Idx : MC6809LoadIndexedNoOffset<REG16, INDEX16>;
def LD32Idx : MC6809LoadIndexedNoOffset<ACC32, INDEX16>;

////class MC6809LoadIndexed<RegisterClass dst, RegisterClass idx> : MC6809Load<dst> {
//  dag OutOperandList = (outs dst:$dst);
//  dag InOperandList = (ins i16imm:$addr, idx:$idx);
//}
//// LDA abs,x; LDA abs,y
//def LDAAbsIdx : MC6809LoadIndexed<ACC8, INDEX16>;
//// LDX abs,y
//def LDXAbsIdx : MC6809LoadIndexed<INDEX16, INDEX16>;
//// LDY abs,x
//def LDYAbsIdx : MC6809LoadIndexed<INDEX16, INDEX16>;

//// LDA (zp),y
//def LDIndirIdx : MC6809Load<dst> {
//  dag OutOperandList = (outs ACC8:$dst);
//  dag InOperandList = (ins ACC16:$addr, INDEX16:$offset);
//}

class MC6809Store : MC6809LogicalInstr {
  let mayStore = true;
}
// STA abs, STX abs
def STAbs : MC6809Store {
  dag InOperandList = (ins ACC8:$src, i16imm:$dst);
}
// STA abs,x; STA abs,y
def STAbsIdx : MC6809Store {
  dag InOperandList = (ins ACC8:$src, i16imm:$addr, INDEX16:$idx);
}
// STA (zp),y
def STIndirIdx : MC6809Store {
  dag InOperandList = (ins ACC8:$src, ACC16:$addr, INDEX16:$offset);
}

// STZ abs
def STZAbs : MC6809Store {
  dag InOperandList = (ins i16imm:$addr);
}
// STZ abs,x
def STZAbsIdx : MC6809Store {
  dag InOperandList = (ins i16imm:$addr, INDEX16:$idx);
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Patterns
//===---------------------------------------------------------------------===//

//def : Pat<(add ACC:$l, (i8 imm:$imm)), (ADCImm ACC:$l, imm:$imm, (LDCImm 0))>;
//def : Pat<(sub ACC:$l, (i8 imm:$imm)), (SBCImm ACC:$l, imm:$imm, (LDCImm -1))>;

//===---------------------------------------------------------------------===//
// Integer Extension/Truncation Patterns
//===---------------------------------------------------------------------===//

def SEX8Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins BIT1:$lsb);
}

def SEX16Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs ADc:$ad);
  dag InOperandList = (ins ABc:$ab);
}

def SEX32Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs AQc:$aq);
  dag InOperandList = (ins AWc:$aw);
}
def ZEX8Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs ACC8:$dst);
  dag InOperandList = (ins BIT1:$lsb);
}

def ZEX16Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs ADc:$ad);
  dag InOperandList = (ins ABc:$ab);
}

def ZEX32Implicit : MC6809LogicalInstr {
  dag OutOperandList = (outs AQc:$aq);
  dag InOperandList = (ins AWc:$aw);
}

// def : Pat<(i8 (anyext BIT1:$s)), (INSERT_SUBREG (IMPLICIT_DEF), BIT1:$s, sub_lsb)>;
// def : Pat<(i16 (anyext ACC8:$s)), (INSERT_SUBREG (IMPLICIT_DEF), ACC8:$s, sub_lo_byte)>;
// def : Pat<(i16 (anyext BIT1:$s)), (INSERT_SUBREG (IMPLICIT_DEF), (INSERT_SUBREG (i8 (IMPLICIT_DEF)), BIT1:$s, sub_lsb), sub_lo_byte)>;

// def : Pat<(i1 (trunc ACC8:$s)), (EXTRACT_SUBREG (ANDImm ACC8:$s, 1), sub_lsb)>;
// def : Pat<(i8 (trunc ACC16:$s)), (EXTRACT_SUBREG ACC16:$s, sub_lo_byte)>;
// def : Pat<(i1 (trunc ACC16:$s)), (EXTRACT_SUBREG (ANDImm (EXTRACT_SUBREG ACC16:$s, sub_lo_byte), 1), sub_lsb)>;

//===---------------------------------------------------------------------===//
// Constant Patterns
//===---------------------------------------------------------------------===//

// def : Pat<(i1 imm:$s), (LDImm1 imm:$s)>;
// def : Pat<(i8 imm:$s), (LDImm8 imm:$s)>;
// def : Pat<(i16 imm:$s), (LDImm16 imm:$s)>;
