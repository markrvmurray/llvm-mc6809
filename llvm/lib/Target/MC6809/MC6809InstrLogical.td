//===-- MC6809InstrLogical.td - Describe MC6809 Logical Instrs --*- tablegen -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
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
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst, Cc:$carryout);
  dag InOperandList = !con((ins dst:$l, Cc:$carryin), operand);
  let Constraints = "$dst = $l, $carryout = $carryin";
}

multiclass MC6809ArithmeticCarry<RegisterClass dst, string size> {
  def Imm : MC6809ArithmeticBaseCarry<dst, (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
  def Reg : MC6809ArithmeticBaseCarry<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBaseCarry<dst, (ins addr16:$addr)>;
    def IdxZero : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx)>;
    def IdxImm : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, i16imm:$offset)>;
    def IndirIdxImm : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, i16imm:$offset)>;
    def IdxReg8 : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, ACC8:$offset)>;
    def IdxReg16 : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, ACC16:$offset)>;
    def IndirIdxReg8 : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, ACC8:$offset)>;
    def IndirIdxReg16 : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, ACC16:$offset)>;
  }
}

class MC6809ArithmeticBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins dst:$l), operand);
  let Constraints = "$dst = $l";
}

multiclass MC6809Arithmetic<RegisterClass dst, string size> {
  def Imm : MC6809ArithmeticBase<dst, (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
  def Reg : MC6809ArithmeticBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBase<dst, (ins addr16:$addr)>;
    def IdxZero : MC6809ArithmeticBase<dst, (ins INDEX16:$idx)>;
    def IdxImm : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, i16imm:$offset)>;
    def IndirIdxImm : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, i16imm:$offset)>;
    def IdxReg8 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC8:$offset)>;
    def IdxReg16 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC16:$offset)>;
    def IndirIdxReg8 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC8:$offset)>;
    def IndirIdxReg16 : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, ACC16:$offset)>;
  }
}

//===---------------------------------------------------------------------===//
// Loads and Stores
//===---------------------------------------------------------------------===//
// Load base class
class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayLoad = true;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = operand;
}

multiclass MC6809Load<RegisterClass dst, string size> {
  def Imm : MC6809LoadBase<dst, (ins !cast<DAGOperand>(!strconcat("i", size, "imm")):$imm)>;
  def Abs : MC6809LoadBase<dst, (ins addr16:$addr)>;
  def IdxZero : MC6809LoadBase<dst, (ins INDEX16:$idx)>;
  def IdxImm : MC6809LoadBase<dst, (ins INDEX16:$idx, i16imm:$offset)>;
  def IndirIdxImm : MC6809LoadBase<dst, (ins INDEX16:$idx, i16imm:$offset)>;
  def IdxReg8 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC8:$offset)>;
  def IdxReg16 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC16:$offset)>;
  def IndirIdxReg8 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC8:$offset)>;
  def IndirIdxReg16 : MC6809LoadBase<dst, (ins INDEX16:$idx, ACC16:$offset)>;
}

// All the loads
defm Load8 : MC6809Load<ACC8, "8">;
defm Load16 : MC6809Load<ALL16, "16">;
defm Load32 : MC6809Load<ACC32, "32">;

// Store base class
class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  def Abs : MC6809StoreBase<src, (ins addr16:$addr)>;
  def IdxZero : MC6809StoreBase<src, (ins INDEX16:$idx)>;
  def IdxImm : MC6809StoreBase<src, (ins INDEX16:$idx, i16imm:$offset)>;
  def IndirIdxImm : MC6809StoreBase<src, (ins INDEX16:$idx, i16imm:$offset)>;
  def IdxReg8 : MC6809StoreBase<src, (ins INDEX16:$idx, ACC8:$offset)>;
  def IdxReg16 : MC6809StoreBase<src, (ins INDEX16:$idx, ACC16:$offset)>;
  def IndirIdxReg8 : MC6809StoreBase<src, (ins INDEX16:$idx, ACC8:$offset)>;
  def IndirIdxReg16 : MC6809StoreBase<src, (ins INDEX16:$idx, ACC16:$offset)>;
}

// All the stores
defm Store8 : MC6809Store<ACC8>;
defm Store16 : MC6809Store<ALL16>;
defm Store32 : MC6809Store<ACC32>;

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

let isAdd = true in {
  defm AddCarry8 : MC6809ArithmeticCarry<ACC8, "8">;
  defm AddCarry16 : MC6809ArithmeticCarry<ACC16, "16">;
}
defm SubBorrow8 : MC6809ArithmeticCarry<ACC8, "8">;
defm SubBorrow16 : MC6809ArithmeticCarry<ACC16, "16">;

let isAdd = true in {
  defm Add8 : MC6809ArithmeticCarry<ACC8, "8">;
  defm Add16 : MC6809ArithmeticCarry<ACC16, "16">;
}
defm Sub8 : MC6809ArithmeticCarry<ACC8, "8">;
defm Sub16 : MC6809ArithmeticCarry<ACC16, "16">;

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//

defm AND8 : MC6809Arithmetic<ACC8, "8">;
defm AND16 : MC6809Arithmetic<ACC16, "16">;
defm OR8 : MC6809Arithmetic<ACC8, "8">;
defm OR16 : MC6809Arithmetic<ACC16, "16">;
defm XOR8 : MC6809Arithmetic<ACC8, "8">;
defm XOR16 : MC6809Arithmetic<ACC16, "16">;

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
// Pointer instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  let isAdd = true;
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
// Stack instructions. These will be squashed together later.
//===---------------------------------------------------------------------===//

class MC6809StackPush<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

class MC6809StackPull<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$reg);
  dag InOperandList = (ins);
  let mayLoad = true;
}

let Defs = [SS], Uses = [SS] in {
  def Push8 : MC6809StackPush<ACC8>;
  def Push16 : MC6809StackPush<ALL16>;
  def Pull8 : MC6809StackPull<ACC8>;
  def Pull16 : MC6809StackPull<ALL16>;
}

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