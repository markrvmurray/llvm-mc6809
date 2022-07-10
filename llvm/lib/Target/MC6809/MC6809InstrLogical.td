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
// Loads and Stores
//===---------------------------------------------------------------------===//
// Load base class
class MC6809LoadBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayLoad = true;
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = operand;
}

multiclass MC6809Load<RegisterClass dst> {
  let isAsCheapAsAMove = true, isMoveImm = true, isReMaterializable = true in {
    def Imm : MC6809LoadBase<dst, (ins i16imm:$imm)>;
  }
  def Abs : MC6809LoadBase<dst, (ins addr16:$addr)>;
  def Idx : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
  def Indir : MC6809LoadBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
}

// All the loads
defm Load8 : MC6809Load<ACC8>;
defm Load16 : MC6809Load<ACC16>;
defm Load32 : MC6809Load<ACC32>;
defm LoadPtr : MC6809Load<INDEX16>;

// Store base class
class MC6809StoreBase<RegisterClass src, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  let mayStore = true;
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins src:$src), operand);
}

multiclass MC6809Store<RegisterClass src> {
  def Abs : MC6809StoreBase<src, (ins addr16:$addr)>;
  def Idx : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
  def Indir : MC6809StoreBase<src, (ins INDEX16:$idx, unknown:$offset)>;
}

// All the stores
defm Store8 : MC6809Store<ACC8>;
defm Store16 : MC6809Store<ACC16>;
defm Store32 : MC6809Store<ACC32>;
defm StorePtr : MC6809Store<INDEX16>;

//===---------------------------------------------------------------------===//
// Inter-register copies and exchanges
//===---------------------------------------------------------------------===//
class MC6809Transfer<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
}

def Copy8 : MC6809Transfer<REGPAIR8>;
def Copy16 : MC6809Transfer<REGPAIR16>;

class MC6809Exchange<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$dst1, reg:$dst2);
  dag InOperandList = (ins reg:$src1, reg:$src2);
  let Constraints = "$dst1 = $src2, $dst2 = $src1";
}

def Exchange8 : MC6809Exchange<REGPAIR8>;
def Exchange16 : MC6809Exchange<REGPAIR16>;

//===---------------------------------------------------------------------===//
// In-register operations without operands
//===---------------------------------------------------------------------===//
class MC6809RegisterOp<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs reg:$dst);
  dag InOperandList = (ins reg:$src);
}

def Complement8 : MC6809RegisterOp<ACC8>;
def Complement16 : MC6809RegisterOp<ACC16>;
def Complement32 : MC6809RegisterOp<ACC32>;

def Negate8 : MC6809RegisterOp<ACC8>;
def Negate16 : MC6809RegisterOp<ACC16>;
def Negate32 : MC6809RegisterOp<ACC32>;

//===---------------------------------------------------------------------===//
// Arithmetic base classes
//===---------------------------------------------------------------------===//

class MC6809ArithmeticBaseCarry<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ];
  dag OutOperandList = (outs dst:$dst, Cc:$carryout, Vc:$overflow);
  dag InOperandList = !con((ins dst:$l, Cc:$carryin), operand);
  let Constraints = "$dst = $l, $carryout = $carryin";
}

multiclass MC6809ArithmeticCarry<RegisterClass dst> {
  def Imm : MC6809ArithmeticBaseCarry<dst, (ins i16imm:$imm)>;
  def Reg : MC6809ArithmeticBaseCarry<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBaseCarry<dst, (ins addr16:$addr)>;
    def Idx : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809ArithmeticBaseCarry<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809ArithmeticBaseCarry<dst, (ins)> {
      let Defs = [SS, NZ];
      let Uses = [SS];
    }
  }
}

class MC6809ArithmeticBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ];
  dag OutOperandList = (outs dst:$dst, Cc:$carryout, Vc:$overflow);
  dag InOperandList = !con((ins dst:$l), operand);
  let Constraints = "$dst = $l";
}

multiclass MC6809Arithmetic<RegisterClass dst> {
  def Imm : MC6809ArithmeticBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809ArithmeticBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809ArithmeticBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809ArithmeticBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809ArithmeticBase<dst, (ins)> {
      let Defs = [SS, NZ];
      let Uses = [SS];
    }
  }
}

//===---------------------------------------------------------------------===//
// Addition/Subtraction Instructions
//===---------------------------------------------------------------------===//

let isAdd = true in {
  defm AddCarry8 : MC6809ArithmeticCarry<ACC8>;
  defm AddCarry16 : MC6809ArithmeticCarry<ACC16>;
  defm AddCarry32 : MC6809ArithmeticCarry<ACC32>;
}
defm SubBorrow8 : MC6809ArithmeticCarry<ACC8>;
defm SubBorrow16 : MC6809ArithmeticCarry<ACC16>;
defm SubBorrow32 : MC6809ArithmeticCarry<ACC32>;

let isAdd = true in {
  defm Add8 : MC6809Arithmetic<ACC8>;
  defm Add16 : MC6809Arithmetic<ACC16>;
  defm Add32 : MC6809Arithmetic<ACC32>;
}
defm Sub8 : MC6809Arithmetic<ACC8>;
defm Sub16 : MC6809Arithmetic<ACC16>;
defm Sub32 : MC6809Arithmetic<ACC32>;

defm Test8 : MC6809Arithmetic<ACC8>;
defm Test16 : MC6809Arithmetic<ACC16>;

//===---------------------------------------------------------------------===//
// Bitwise instructions
//===---------------------------------------------------------------------===//
class MC6809BitwiseBase<RegisterClass dst, dag operand> : MC6809LogicalInstr {
  let Defs = [NZ, V];
  dag OutOperandList = (outs dst:$dst);
  dag InOperandList = !con((ins dst:$l), operand);
  let Constraints = "$dst = $l";
}

multiclass MC6809Bitwise<RegisterClass dst> {
  def Imm : MC6809BitwiseBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809BitwiseBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809BitwiseBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809BitwiseBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809BitwiseBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809BitwiseBase<dst, (ins)> {
      let Defs = [SS, NZ, V];
      let Uses = [SS];
    }
  }
}

defm AND8 : MC6809Bitwise<ACC8>;
defm AND16 : MC6809Bitwise<ACC16>;
defm OR8 : MC6809Bitwise<ACC8>;
defm OR16 : MC6809Bitwise<ACC16>;
defm XOR8 : MC6809Bitwise<ACC8>;
defm XOR16 : MC6809Bitwise<ACC16>;
defm BIT8 : MC6809Bitwise<ACC8>;

//===---------------------------------------------------------------------===//
// Comparison Instructions
//===---------------------------------------------------------------------===//
class MC6809CompareBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let isCompare = true;
  let Defs = [NZ, V, C];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins reg:$reg), operand);
}

multiclass MC6809Compare<RegisterClass dst> {
  def Imm : MC6809CompareBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809CompareBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809CompareBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809CompareBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809CompareBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809CompareBase<dst, (ins)> {
      let Defs = [SS, NZ, V, C];
      let Uses = [SS];
    }
  }
}

defm Compare8 : MC6809Compare<ACC8>;
defm Compare16 : MC6809Compare<ACC16>;
defm ComparePtr : MC6809Compare<INDEX16>;

class MC6809BitBase<RegisterClass reg, dag operand> : MC6809LogicalInstr {
  let isCompare = true;
  let Defs = [NZ, V];
  dag OutOperandList = (outs);
  dag InOperandList = !con((ins reg:$reg), operand);
}

multiclass MC6809Bit<RegisterClass dst> {
  def Imm : MC6809BitBase<dst, (ins i16imm:$imm)>;
  def Reg : MC6809BitBase<dst, (ins dst:$src)>;
  let mayLoad = true in {
    def Abs : MC6809BitBase<dst, (ins addr16:$addr)>;
    def Idx : MC6809BitBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Indir : MC6809BitBase<dst, (ins INDEX16:$idx, unknown:$offset)>;
    def Pop : MC6809BitBase<dst, (ins)> {
      let Defs = [SS, NZ, V];
      let Uses = [SS];
    }
  }
}

defm Bit8 : MC6809Bit<ACC8>;

//===---------------------------------------------------------------------===//
// Operandless Accumulator Instructions
//===---------------------------------------------------------------------===//
class MC6809Accumulator<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [NZ];
  dag OutOperandList = (outs reg:$dst, Cc:$carryout, Vc:$overflow);
  dag InOperandList = (ins reg:$src);
}

def Neg8 : MC6809Accumulator<ACC8>;
def Neg16 : MC6809Accumulator<ACC16>;
def Neg32 : MC6809Accumulator<ACC32>;

//===---------------------------------------------------------------------===//
// Control Flow Instructions
//===---------------------------------------------------------------------===//

class MC6809Branch : MC6809LogicalInstr {
  let isBranch = true;
  let isTerminator = true;
  let isBarrier = false;
  let Uses = [];
  let InOperandList = (ins label:$tgt);
}

def JumpConditionalRelative : MC6809Branch {
  let Uses = [NZ, V, C];
  dag InOperandList = (ins unknown:$cond, label:$tgt);
}

def JumpRelative : MC6809Branch {
  let isBarrier = true;
  let Pattern = [(br bb:$tgt)];
}

def JumpIndir : MC6809Branch {
  let isBarrier = true;
  let InOperandList = (ins INDEX16:$tgt);
  let isIndirectBranch = true;
}

def CallRelative : MC6809Branch {
  let isCall = true;
  let isTerminator = false;
  dag InOperandList = (ins label:$tgt);
}

def CallIndir : MC6809Branch {
  let isCall = true;
  let isTerminator = false;
  let isIndirectBranch = true;
  let InOperandList = (ins INDEX16:$tgt);
}

class MC6809Return : MC6809Branch {
  let isBarrier = true;
  let isReturn = true;
  let isTerminator = true;
  let isBranch = false;
  let InOperandList = (ins);
}

def ReturnImplicit : MC6809Return;
def ReturnIRQImplicit : MC6809Return;

//===---------------------------------------------------------------------===//
// Pointer instructions
//===---------------------------------------------------------------------===//

class MC6809PtrAdd<RegisterClass idx> : MC6809LogicalInstr {
  dag OutOperandList = (outs idx:$dst);
  dag InOperandList = (ins idx:$src, unknown:$val);
  let isAdd = true;
}

// LEAr n,r or LEAr a,r
def LEAPtrAdd : MC6809PtrAdd<INDEX16>;
//def LEAPtrAddReg8 : MC6809PtrAddReg<INDEX16, ACC8>;
//def LEAPtrAddReg16 : MC6809PtrAddReg<INDEX16, ACC16>;

//===---------------------------------------------------------------------===//
// Stack instructions. These will be squashed together later.
//===---------------------------------------------------------------------===//

class MC6809StackPush<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

class MC6809StackPull<RegisterClass reg> : MC6809LogicalInstr {
  let Defs = [SS];
  let Uses = [SS];
  dag OutOperandList = (outs reg:$reg);
  dag InOperandList = (ins);
  let mayLoad = true;
}

def Push8 : MC6809StackPush<ACC8>;
def Push16 : MC6809StackPush<ACC16>;
def Push32 : MC6809StackPush<ACC32>;
def PushPtr : MC6809StackPush<INDEX16>;
def Pull8 : MC6809StackPull<ACC8>;
def Pull16 : MC6809StackPull<ACC16>;
def Pull32 : MC6809StackPull<ACC32>;
def PullPtr : MC6809StackPull<INDEX16>;

// The "Op" variants are intended to couple with <Instr>Pop instructions.
class MC6809StackPushOp<RegisterClass reg> : MC6809LogicalInstr {
  dag OutOperandList = (outs);
  dag InOperandList = (ins reg:$reg);
  let mayStore = true;
}

def PushOp8 : MC6809StackPushOp<ACC8>;
def PushOp16 : MC6809StackPushOp<ACC16>;
def PushOp32 : MC6809StackPushOp<ACC32>;
def PushOpPtr : MC6809StackPushOp<INDEX16>;

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
