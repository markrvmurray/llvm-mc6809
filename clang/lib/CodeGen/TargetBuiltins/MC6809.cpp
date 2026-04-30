//===------ MC6809.cpp - Emit LLVM Code for MC6809 builtins ---------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Bug #199: MC6809-specific __builtin_mc6809_* functions.
//
// All builtins lower to inline asm strings with hasSideEffects=true so
// the optimizer cannot remove or reorder them.  No MC6809-specific LLVM
// intrinsics are needed.
//
//===----------------------------------------------------------------------===//

#include "CodeGenFunction.h"
#include "clang/AST/CharUnits.h"
#include "clang/Basic/TargetBuiltins.h"
#include "llvm/IR/InlineAsm.h"

using namespace clang;
using namespace CodeGen;
using namespace llvm;

// Helper: emit a void no-argument inline asm with side effects.
static Value *emitVoidNoArgAsm(CodeGenFunction &CGF, StringRef AsmStr) {
  llvm::FunctionType *FTy = llvm::FunctionType::get(CGF.VoidTy, false);
  llvm::InlineAsm *IA =
      llvm::InlineAsm::get(FTy, AsmStr, "", /*hasSideEffects=*/true);
  return CGF.Builder.CreateCall(IA);
}

// Helper: emit void asm with one 8-bit immediate argument ("i" constraint).
static Value *emitVoidImm8Asm(CodeGenFunction &CGF, StringRef AsmStr,
                                const CallExpr *E) {
  Value *Arg = CGF.EmitScalarExpr(E->getArg(0));
  llvm::FunctionType *FTy =
      llvm::FunctionType::get(CGF.VoidTy, {CGF.Int8Ty}, false);
  llvm::InlineAsm *IA =
      llvm::InlineAsm::get(FTy, AsmStr, "i", /*hasSideEffects=*/true);
  return CGF.Builder.CreateCall(IA, {Arg});
}

Value *CodeGenFunction::EmitMC6809BuiltinExpr(unsigned BuiltinID,
                                               const CallExpr *E) {
  switch (BuiltinID) {
  default:
    llvm_unreachable("Unhandled MC6809 builtin");

  // --- CC register: set/clear individual bits ---
  case MC6809::BI__builtin_mc6809_sei:
    return emitVoidNoArgAsm(*this, "orcc #16");   // set interrupt mask I=1
  case MC6809::BI__builtin_mc6809_cli:
    return emitVoidNoArgAsm(*this, "andcc #239"); // clear interrupt mask I=0

  // --- CC register: arbitrary mask ---
  // MC6809 ORCC/ANDCC only accept an 8-bit immediate; the "i" constraint
  // causes LLVM to error if the argument is not a compile-time constant.
  case MC6809::BI__builtin_mc6809_orcc:
    return emitVoidImm8Asm(*this, "orcc $0", E);
  case MC6809::BI__builtin_mc6809_andcc:
    return emitVoidImm8Asm(*this, "andcc $0", E);

  // --- CPU control ---
  case MC6809::BI__builtin_mc6809_nop:
    return emitVoidNoArgAsm(*this, "nop");
  case MC6809::BI__builtin_mc6809_sync:
    return emitVoidNoArgAsm(*this, "sync");

  // cwai: AND CC with mask then wait for interrupt.  Mask is guaranteed
  // compile-time constant by CheckMC6809BuiltinFunctionCall in SemaMC6809.
  case MC6809::BI__builtin_mc6809_cwai:
    return emitVoidImm8Asm(*this, "cwai $0", E);

  // --- Software interrupts ---
  case MC6809::BI__builtin_mc6809_swi:
    return emitVoidNoArgAsm(*this, "swi");
  case MC6809::BI__builtin_mc6809_swi2:
    return emitVoidNoArgAsm(*this, "swi2");
  case MC6809::BI__builtin_mc6809_swi3:
    return emitVoidNoArgAsm(*this, "swi3");

  // --- peek: *(volatile uint8_t *)(addr) ---
  case MC6809::BI__builtin_mc6809_peek: {
    Value *Addr = EmitScalarExpr(E->getArg(0));
    Value *Ptr = Builder.CreateIntToPtr(Addr, llvm::PointerType::get(getLLVMContext(), 0));
    LoadInst *LI = Builder.CreateAlignedLoad(Int8Ty, Ptr, CharUnits::One());
    LI->setVolatile(true);
    return LI;
  }

  // --- poke: *(volatile uint8_t *)(addr) = val ---
  case MC6809::BI__builtin_mc6809_poke: {
    Value *Addr = EmitScalarExpr(E->getArg(0));
    Value *Val  = EmitScalarExpr(E->getArg(1));
    Value *Ptr = Builder.CreateIntToPtr(Addr, llvm::PointerType::get(getLLVMContext(), 0));
    StoreInst *SI = Builder.CreateAlignedStore(Val, Ptr, CharUnits::One());
    SI->setVolatile(true);
    return SI;
  }

  // --- peekw: *(volatile uint16_t *)(addr) ---
  case MC6809::BI__builtin_mc6809_peekw: {
    Value *Addr = EmitScalarExpr(E->getArg(0));
    Value *Ptr = Builder.CreateIntToPtr(Addr, llvm::PointerType::get(getLLVMContext(), 0));
    LoadInst *LI = Builder.CreateAlignedLoad(Int16Ty, Ptr, CharUnits::One());
    LI->setVolatile(true);
    return LI;
  }

  // --- pokew: *(volatile uint16_t *)(addr) = val ---
  case MC6809::BI__builtin_mc6809_pokew: {
    Value *Addr = EmitScalarExpr(E->getArg(0));
    Value *Val  = EmitScalarExpr(E->getArg(1));
    Value *Ptr = Builder.CreateIntToPtr(Addr, llvm::PointerType::get(getLLVMContext(), 0));
    StoreInst *SI = Builder.CreateAlignedStore(Val, Ptr, CharUnits::One());
    SI->setVolatile(true);
    return SI;
  }

  // --- set_dp: TFR A,DP  (pass page in A register) ---
  case MC6809::BI__builtin_mc6809_set_dp: {
    Value *Page = EmitScalarExpr(E->getArg(0));
    llvm::FunctionType *FTy =
        llvm::FunctionType::get(VoidTy, {Int8Ty}, false);
    // "{a}" = pass argument in the MC6809 A register (gcc name "a")
    llvm::InlineAsm *IA =
        llvm::InlineAsm::get(FTy, "tfr a,dp", "{a}", /*hasSideEffects=*/true);
    return Builder.CreateCall(IA, {Page});
  }

  // --- get_dp: TFR DP,A  (return value in A register) ---
  case MC6809::BI__builtin_mc6809_get_dp: {
    llvm::FunctionType *FTy = llvm::FunctionType::get(Int8Ty, false);
    // "={a}" = output in the MC6809 A register
    llvm::InlineAsm *IA =
        llvm::InlineAsm::get(FTy, "tfr dp,a", "={a}", /*hasSideEffects=*/false);
    return Builder.CreateCall(IA);
  }
  }
}
