//===-- MC6809CallLowering.cpp - MC6809 Call lowering -----------*- C++ -*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the lowering of LLVM calls to machine code calls for
// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "MC6809CallLowering.h"

#include "MC6809CallingConv.h"
#include "MC6809FrameLowering.h"
#include "MC6809MachineFunctionInfo.h"
#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/CallingConvLower.h"
#include "llvm/CodeGen/FunctionLoweringInfo.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/TargetCallingConv.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/CodeGen/TargetOpcodes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/CallingConv.h"
#include "llvm/Target/TargetMachine.h"
#include <memory>

using namespace llvm;

#define DEBUG_TYPE "mc6809-call-lowering"

namespace {

struct MC6809ValueAssigner : CallLowering::ValueAssigner {

  /// Cached copy of the reserved register set for the current fn. These
  /// registers must be avoided when selecting registers for arguments.
  BitVector Reserved;

  MC6809ValueAssigner(bool IsIncoming, MachineRegisterInfo &MRI, const MachineFunction &MF, bool IsReturn = false)
    : CallLowering::ValueAssigner(IsIncoming,
        IsReturn ? RetCC_MC6809 : CC_MC6809,
        CC_MC6809_VarArgs) {
    Reserved = MRI.getTargetRegisterInfo()->getReservedRegs(MF);
  }

  bool assignArg(unsigned ValNo, EVT OrigVT, MVT ValVT, MVT LocVT, CCValAssign::LocInfo LocInfo, const CallLowering::ArgInfo &Info, ISD::ArgFlagsTy Flags, CCState &State) override {
    // Ensure that reserved registers are not used in calling convention by
    // marking them as already allocated.
    for (Register R : Reserved.set_bits())
      State.AllocateReg(R);

    // gcc6809's __gcccall variadic ABI puts ALL args (including the
    // named ones like printf's fmt) on the stack. The variadic CC
    // table CC_MC6809_VarArgs is `CCAssignToStack<0,1>` for all
    // types — exactly the all-on-stack layout gcc6809 uses on both
    // the call site and the callee side. State.isVarArg() carries
    // the function-level variadic flag through from
    // determineAndHandleAssignments, so the call site / formal-args
    // path passes its `IsVarArg` flag and we just dispatch on it.
    if (getAssignFn(State.isVarArg())(ValNo, ValVT, LocVT, LocInfo, Flags, Info.Ty, State))
      return true;
    StackSize = State.getStackSize();
    return false;
  }
};

/// Handler to pass values outward to calls and return statements.
struct MC6809OutgoingValueHandler : CallLowering::OutgoingValueHandler {
  /// The instruction causing control flow to leave the current function. This
  /// instruction will be annotated with implicit use operands to record
  /// registers used to pass arguments.
  MachineInstrBuilder &MIB;

  MC6809OutgoingValueHandler(MachineIRBuilder &MIRBuilder, MachineInstrBuilder &MIB, MachineRegisterInfo &MRI) : OutgoingValueHandler(MIRBuilder, MRI), MIB(MIB) {}

  void assignValueToReg(Register ValVReg, Register PhysReg, const CCValAssign &VA, ISD::ArgFlagsTy Flags) override {
    // Ensure that the physical remains alive until control flow leaves the
    // current function.
    MIB.addUse(PhysReg, RegState::Implicit);
    Register ExtReg = extendRegister(ValVReg, VA);
    MIRBuilder.buildCopy(PhysReg, ExtReg);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy, const MachinePointerInfo &MPO, const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOStore, MemTy, inferAlignFromPtrInfo(MF, MPO));
    Register ExtReg = extendRegister(ValVReg, VA);
    MIRBuilder.buildStore(ExtReg, Addr, *MMO);
  }
};

struct MC6809OutgoingArgsHandler : MC6809OutgoingValueHandler {
  /// A VReg containing the value of the stack pointer right before control flow
  /// leaves the current function. The VReg is cached to avoid generating it
  /// more than once.
  Register SPReg = 0;

  MC6809OutgoingArgsHandler(MachineIRBuilder &MIRBuilder, MachineInstrBuilder &MIB, MachineRegisterInfo &MRI) : MC6809OutgoingValueHandler(MIRBuilder, MIB, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset, MachinePointerInfo &MPO, ISD::ArgFlagsTy Flags) override {
    // gcc6809 convention: i8 stack args occupy 1-byte slots (not
    // padded to a word). The CC table now uses CCAssignToStack<1,1>
    // for i8, so Offset already points at the value byte and no
    // adjustment is needed here.
    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);

    LLT P = LLT::pointer(0, 16);

    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(P, Register(MC6809::SS)).getReg(0);

    auto OffsetReg = MIRBuilder.buildConstant(LLT::scalar(16), Offset).getReg(0);
    return MIRBuilder.buildPtrAdd(P, SPReg, OffsetReg).getReg(0);
  }
};

struct MC6809OutgoingReturnHandler : MC6809OutgoingValueHandler {
  MC6809OutgoingReturnHandler(MachineIRBuilder &MIRBuilder, MachineInstrBuilder &MIB, MachineRegisterInfo &MRI) : MC6809OutgoingValueHandler(MIRBuilder, MIB, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset, MachinePointerInfo &MPO, ISD::ArgFlagsTy Flags) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, false);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    auto AddrReg = MIRBuilder.buildFrameIndex(LLT::pointer(0, 16), FI);
    return AddrReg.getReg(0);
  }
};

/// Handler to receive values from formal arguments and call returns.
struct MC6809IncomingValueHandler : CallLowering::IncomingValueHandler {
  MC6809IncomingValueHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI) : IncomingValueHandler(MIRBuilder, MRI) {}

  void assignValueToReg(Register ValVReg, Register PhysReg, const CCValAssign &VA, ISD::ArgFlagsTy Flags = {}) override {
    switch (VA.getLocVT().getSizeInBits()) {
    default:
      report_fatal_error("Not yet implemented.");
    case 8:
    case 16:
    case 32:
      break;
    }

    // Ensure that the physical register is considerd live at the point control
    // flow (re)enters to the current function.
    makeLive(PhysReg);

    MIRBuilder.buildCopy(ValVReg, PhysReg);
  }

  /// Mark the given register as live-in. These will be function-level live-ins
  /// or implicit defs for formal arguments or call statements, respectively.
  virtual void makeLive(Register PhysReg) = 0;
};

struct MC6809IncomingArgsHandler : public MC6809IncomingValueHandler {
  MC6809IncomingArgsHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI) : MC6809IncomingValueHandler(MIRBuilder, MRI) {}

  Register getStackAddress(uint64_t Size, int64_t Offset, MachinePointerInfo &MPO, ISD::ArgFlagsTy Flags) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    // gcc6809 convention: i8 stack args occupy 1-byte slots. The CC
    // table allocates 1 byte per i8 arg via CCAssignToStack<1,1>, so
    // Offset already points at the value byte. No padding to a word.
    int FI = MFI.CreateFixedObject(Size, Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    auto AddrReg = MIRBuilder.buildFrameIndex(LLT::pointer(0, 16), FI);
    return AddrReg.getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy, const MachinePointerInfo &MPO, const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    LLT ValTy = MRI.getType(ValVReg);

    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, ValTy, inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  void makeLive(Register PhysReg) override { MIRBuilder.getMBB().addLiveIn(PhysReg); }
};

struct MC6809IncomingReturnHandler : public MC6809IncomingValueHandler {
  MachineInstrBuilder &Call;

  /// A VReg containing the value of the stack pointer right after control flow
  /// returned to the current function. The VReg is cached to avoid generating
  /// it more than once.
  Register SPReg = 0;

  MC6809IncomingReturnHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI, MachineInstrBuilder &Call) : MC6809IncomingValueHandler(MIRBuilder, MRI), Call(Call) {}

  Register getStackAddress(uint64_t Size, int64_t Offset, MachinePointerInfo &MPO, ISD::ArgFlagsTy Flags) override {
    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Offset);

    LLT P = LLT::pointer(0, 16);

    // Cache the SP virtual register to avoid generating it more than once.
    if (!SPReg)
      SPReg = MIRBuilder.buildCopy(P, Register(MC6809::SS)).getReg(0);

    auto OffsetReg = MIRBuilder.buildConstant(LLT::scalar(16), Offset).getReg(0);
    return MIRBuilder.buildPtrAdd(P, SPReg, OffsetReg).getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, LLT MemTy, const MachinePointerInfo &MPO, const CCValAssign &VA) override {
    MachineFunction &MF = MIRBuilder.getMF();
    // Such loads are not invariant; the same stack region may be reused for
    // many different calls.
    auto *MMO = MF.getMachineMemOperand(MPO, MachineMemOperand::MOLoad, MemTy, inferAlignFromPtrInfo(MF, MPO));
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  void makeLive(Register PhysReg) override { Call.addDef(PhysReg, RegState::Implicit); }
};

// Add missing pointer information from the LLT to the argument flags for the
// corresponding MVT. The MVT doesn't contain pointer information, so this would
// otherwise be unavailable for use by the calling convention (i.e., CCIfPtr).
void adjustArgFlags(CallLowering::ArgInfo &Arg, LLT Ty) {
  if (!Ty.isPointer())
    return;

  auto &Flags = Arg.Flags[0];
  Flags.setPointer();
  Flags.setPointerAddrSpace(Ty.getAddressSpace());
}

/// Returns true if the given return type is too large to fit in a single
/// 6809 register and must therefore be returned via the gcc6809 sret
/// convention: caller allocates a buffer, passes its address in IX, callee
/// writes the result through that pointer. The threshold is 2 bytes — the
/// size of the largest 6809 hardware register (X/Y/U/D). Anything larger
/// (i32, i64, structs, doubles, …) is returned via sret.
///
/// Mirrors gcc6809's `(GET_MODE_SIZE (MODE) > 2)` test in `m6809.h`.
bool isLargeReturnType(const Type *Ty, const DataLayout &DL) {
  if (Ty->isVoidTy())
    return false;
  return DL.getTypeStoreSize(const_cast<Type *>(Ty)).getKnownMinValue() > 2;
}

/// Returns true if the given call's callee is a runtime libcall (compiler-rt
/// helper) whose hand-written assembly still uses the OLD i32 calling
/// convention (X = a_hi, stack = a_lo/b_hi/b_lo, X = result_hi, stack =
/// result_lo). These functions live in `lib/Target/MC6809/Runtime/*.inc` and
/// will be rewritten to the new sret convention in a follow-up commit. Until
/// then, lowerCall must NOT apply the sret transformation when calling them,
/// otherwise LLVM-compiled callers and the hand-written runtime asm would
/// disagree on the layout.
///
/// Conventionally, all compiler-rt helpers begin with `__` (e.g. `__mulsi3`,
/// `__udivsi3`, `__ashlsi3`). The MC6809 indirect-call thunk `__call_indir`
/// also matches but is harmless because it returns void.
///
/// TODO(ABI #4 step 2): delete this helper and its callers once
/// mulsi3.inc / divsi.inc / shiftsi3.inc are rewritten for the sret CC.
bool isOldCCRuntimeLibcall(const MachineOperand &Callee) {
  StringRef Name;
  if (Callee.isGlobal())
    Name = Callee.getGlobal()->getName();
  else if (Callee.isSymbol())
    Name = Callee.getSymbolName();
  return Name.starts_with("__");
}

} // namespace

bool MC6809CallLowering::lowerReturn(MachineIRBuilder &MIRBuilder, const Value *Val, ArrayRef<Register> VRegs, FunctionLoweringInfo &FLI) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const auto &TFI = static_cast<const MC6809FrameLowering &>(*MF.getSubtarget().getFrameLowering());
  const Function &F = MF.getFunction();

  auto Return = MIRBuilder.buildInstrNoInsert(TFI.isISR(MF) ? MC6809::RTIr : MC6809::RTSr);

  if (Val) {
    MachineRegisterInfo &MRI = MF.getRegInfo();
    const TargetLowering &TLI = *getTLI();
    const DataLayout &DL = MF.getDataLayout();
    LLVMContext &Ctx = Val->getContext();

    // gcc6809 sret convention: when the return type is larger than 2 bytes,
    // the value is written through the implicit pointer that lowerFormal-
    // Arguments stashed in MC6809FunctionInfo::SRetReturnReg. The pointer
    // arrived in IX as the synthetic first arg; the regular RetCC path is
    // skipped entirely (the function effectively returns void at the
    // hardware level — RTS with no result register).
    if (isLargeReturnType(Val->getType(), DL)) {
      auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>();
      Register SRetVReg = FuncInfo->SRetReturnReg;
      assert(SRetVReg.isValid() && "lowerFormalArguments must allocate SRetReturnReg "
                                   "for large-return functions");
      assert(VRegs.size() == 1 && "expected single vreg for large return");
      LLT ValLLT = MRI.getType(VRegs[0]);
      auto *MMO = MF.getMachineMemOperand(
          MachinePointerInfo(), MachineMemOperand::MOStore, ValLLT, Align(1));
      MIRBuilder.buildStore(VRegs[0], SRetVReg, *MMO);
      MIRBuilder.insertInstr(Return);
      return true;
    }

    SmallVector<EVT> ValueVTs;
    ComputeValueVTs(TLI, DL, Val->getType(), ValueVTs);

    // The LLTs here are mostly redundant, except they contain information
    // missing from the VTs about whether or not the argument is a pointer. This
    // information is added to the arg flags via adjustArgFlags below.
    SmallVector<LLT> ValueLLTs;
    computeValueLLTs(DL, *Val->getType(), ValueLLTs);
    assert(ValueVTs.size() == VRegs.size() && "Need one MVT for each VReg.");
    assert(ValueLLTs.size() == VRegs.size() && "Need one LLT for each VReg.");

    // Copy flags from the instruction definition over to the return value
    // description for TableGen compatibility layer.
    SmallVector<ArgInfo> Args;
    for (const auto &[VReg, ValueVT, ValueLLT] : zip(VRegs, ValueVTs, ValueLLTs)) {
      Args.emplace_back(VReg, ValueVT.getTypeForEVT(Ctx), 0);
      setArgFlags(Args.back(), AttributeList::ReturnIndex, DL, F);
      adjustArgFlags(Args.back(), ValueLLT);
    }

    // Invoke TableGen compatibility layer. This will generate copies and stores
    // from the return value virtual register to physical and stack locations.
    // Return values always use the regular RetCC_MC6809 — never the variadic
    // CC — even for variadic functions, so the caller can find the result in
    // a known location (X for i16/ptr, B for i8). The MC6809ValueAssigner
    // constructor sets RetCC when IsReturn=true, but we still need to pass
    // IsVarArg=false here so that getAssignFn() picks the (non-vararg) RetCC
    // branch rather than going through the all-on-stack VarArgs path.
    MC6809OutgoingReturnHandler Handler(MIRBuilder, Return, MRI);
    MC6809ValueAssigner Assigner(/*IsIncoming=*/false, MRI, MF, /*IsReturn=*/true);
    if (!determineAndHandleAssignments(Handler, Assigner, Args, MIRBuilder, F.getCallingConv(), /*IsVarArg=*/false))
      return false;
  }

  // Insert the final return once the return values are in place.
  MIRBuilder.insertInstr(Return);
  return true;
}

bool MC6809CallLowering::lowerFormalArguments(MachineIRBuilder &MIRBuilder, const Function &F, ArrayRef<ArrayRef<Register>> VRegs, FunctionLoweringInfo &FLI) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const DataLayout &DL = MF.getDataLayout();
  MachineRegisterInfo &MRI = MF.getRegInfo();


  SmallVector<ArgInfo> SplitArgs;
  unsigned Idx = 0;
  for (auto &Arg : F.args()) {
    if (DL.getTypeStoreSize(Arg.getType()).isZero())
      continue;

    // Copy flag information over from the function to the argument descriptors.
    ArgInfo OrigArg{VRegs[Idx], Arg.getType(), Idx};
    setArgFlags(OrigArg, Idx + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArg, SplitArgs, DL);
    ++Idx;
  }

  // gcc6809 sret convention: when this function returns a value larger than
  // 2 bytes (i32, i64, structs, …), the caller passes an implicit pointer to
  // the return-value buffer in IX. We model this by prepending a synthetic
  // ptr ArgInfo to the SplitArgs list. CC_MC6809's first-ptr-in-IX rule
  // routes it to IX, leaving the user args to fill the stack (the first
  // user i16/ptr arg, which would normally have gone in IX, falls through
  // to CCAssignToStack — exactly matching gcc6809).
  //
  // The vreg holding the incoming pointer is stashed in MC6809FunctionInfo
  // so lowerReturn can store the result through it.
  if (isLargeReturnType(F.getReturnType(), DL)) {
    LLT PtrLLT = LLT::pointer(0, 16);
    Register SRetVReg = MRI.createGenericVirtualRegister(PtrLLT);
    Type *PtrTy = PointerType::get(F.getContext(), 0);
    // Use OrigArgIndex = ~0u to mark this as a synthetic arg distinct from
    // any real function-level arg index.
    ArgInfo SRetArg(SRetVReg, PtrTy, /*OrigArgIndex=*/~0u);
    adjustArgFlags(SRetArg, PtrLLT);
    SRetArg.Flags[0].setSRet();
    SplitArgs.insert(SplitArgs.begin(), SRetArg);

    auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>();
    FuncInfo->SRetReturnReg = SRetVReg;
  }

  MC6809IncomingArgsHandler Handler(MIRBuilder, MRI);
  MC6809ValueAssigner Assigner(/*IsIncoming=*/true, MRI, MF);
  // gcc6809's __gcccall variadic ABI puts ALL args (including named ones
  // like printf's `fmt`) on the stack — even at the callee, where the
  // named args land on the stack alongside the variadic ones. So we
  // pass F.isVarArg() here and let the assigner dispatch to
  // CC_MC6809_VarArgs (all-on-stack) when this function is variadic.
  if (!determineAndHandleAssignments(Handler, Assigner, SplitArgs, MIRBuilder, F.getCallingConv(), F.isVarArg()))
    return false;

  // Record the beginning of the varargs region of the stack by creating a fake
  // stack argument at that location. The varargs instructions are lowered by
  // walking a pointer forward from that memory location. Assigner.StackSize
  // is the size of the named args' stack region, so the first vararg sits
  // immediately above it (or at offset 0 when all named args are in regs).
  if (F.isVarArg()) {
    auto *FuncInfo = MF.getInfo<MC6809FunctionInfo>();
    FuncInfo->VarArgsStackIndex = MF.getFrameInfo().CreateFixedObject(
        /*Size=*/1, Assigner.StackSize, /*IsImmutable=*/true);
  }

  return true;
}

bool MC6809CallLowering::lowerCall(MachineIRBuilder &MIRBuilder, CallLoweringInfo &Info) const {
  if (Info.IsMustTailCall)
    report_fatal_error("Musttail calls not supported.");

  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const DataLayout &DL = MF.getDataLayout();
  const MC6809Subtarget &STI = MF.getSubtarget<MC6809Subtarget>();
  const TargetRegisterInfo &TRI = *STI.getRegisterInfo();

  bool IsIndirect = Info.Callee.isReg();
  if (IsIndirect) {
    // Call __call_indir to execute the indirect call.
    Info.Callee.ChangeToES("__call_indir");
  }

  // Generate the setup call frame pseudo instruction. This will record the size
  // of the outgoing stack frame once it's known. Usually, all such pseudos can
  // be folded into the prolog/epilog of the function without emitting any
  // additional code.
  auto CallSeqStart = MIRBuilder.buildInstr(MC6809::ADJCALLSTACKDOWN);

  auto Call = MIRBuilder.buildInstrNoInsert(MC6809::LongBranchSubroutine).add(Info.Callee).addRegMask(TRI.getCallPreservedMask(MF, Info.CallConv));

  SmallVector<ArgInfo, 8> OutArgs;
  for (auto &OrigArg : Info.OrigArgs) {
    splitToValueTypes(OrigArg, OutArgs, DL);
  }

  // gcc6809 sret convention: when the callee returns a value larger than
  // 2 bytes (i32, i64, structs, …), the caller allocates a buffer on its
  // own stack frame, passes the buffer's address as an implicit first
  // pointer arg in IX, and the callee writes the result through that
  // pointer. After the call, we G_LOAD the result back from the buffer
  // into the original return-value vreg.
  //
  // Libcall exclusion: the runtime helpers in lib/Target/MC6809/Runtime/
  // (mulsi3.inc, divsi.inc, shiftsi3.inc) still hardcode the OLD i32
  // return convention (X = result_hi, stack = result_lo). Until they are
  // rewritten in a follow-up commit, leave libcalls on the old path.
  bool LargeRet = !Info.OrigRet.Ty->isVoidTy() &&
                  isLargeReturnType(Info.OrigRet.Ty, DL);
  bool UseSret = LargeRet && !isOldCCRuntimeLibcall(Info.Callee);
  int SRetFI = -1;
  Register SRetAddrVReg;
  if (UseSret) {
    LLT PtrLLT = LLT::pointer(0, 16);
    uint64_t RetSize = DL.getTypeStoreSize(Info.OrigRet.Ty).getKnownMinValue();
    SRetFI = MF.getFrameInfo().CreateStackObject(RetSize, Align(1),
                                                 /*isSpillSlot=*/false);
    auto FIReg = MIRBuilder.buildFrameIndex(PtrLLT, SRetFI);
    SRetAddrVReg = FIReg.getReg(0);
    Type *PtrTy = PointerType::get(Info.OrigRet.Ty->getContext(), 0);
    ArgInfo SRetArg(SRetAddrVReg, PtrTy, /*OrigArgIndex=*/~0u);
    adjustArgFlags(SRetArg, PtrLLT);
    SRetArg.Flags[0].setSRet();
    OutArgs.insert(OutArgs.begin(), SRetArg);
  }

  SmallVector<ArgInfo, 8> InArgs;
  if (!Info.OrigRet.Ty->isVoidTy() && !UseSret)
    splitToValueTypes(Info.OrigRet, InArgs, DL);

  // Copy arguments from virtual registers to their real physical locations.
  // For variadic calls, gcc6809 puts ALL args on the stack (named and
  // variadic alike), so we just propagate Info.IsVarArg through to the
  // assigner and let it dispatch to CC_MC6809_VarArgs (all-on-stack)
  // for the whole call.
  MC6809OutgoingArgsHandler ArgsHandler(MIRBuilder, Call, MRI);
  MC6809ValueAssigner ArgsAssigner(/*IsIncoming=*/false, MRI, MF);
  if (!determineAndHandleAssignments(ArgsHandler, ArgsAssigner, OutArgs, MIRBuilder, Info.CallConv, Info.IsVarArg))
    return false;

  // Insert the call once the outgoing arguments are in place.
  MIRBuilder.insertInstr(Call);

  uint64_t StackSize = ArgsAssigner.StackSize;

  if (UseSret) {
    // Load the result back from the sret buffer into the call's
    // original return-value vreg(s). For a single-vreg return (the
    // common case for i32/i64), this is one G_LOAD; the legalizer
    // will narrow it as needed.
    assert(Info.OrigRet.Regs.size() == 1 &&
           "multi-vreg sret returns not yet supported");
    LLT RetLLT = MRI.getType(Info.OrigRet.Regs[0]);
    auto *MMO = MF.getMachineMemOperand(
        MachinePointerInfo::getFixedStack(MF, SRetFI),
        MachineMemOperand::MOLoad, RetLLT, Align(1));
    MIRBuilder.buildLoad(Info.OrigRet.Regs[0], SRetAddrVReg, *MMO);
  } else if (!Info.OrigRet.Ty->isVoidTy()) {
    // Copy the return value from its physical location into a virtual register.
    MC6809IncomingReturnHandler RetHandler(MIRBuilder, MRI, Call);
    MC6809ValueAssigner RetAssigner(/*IsIncoming=*/true, MRI, MF, /*IsReturn=*/true);
    if (!determineAndHandleAssignments(RetHandler, RetAssigner, InArgs, MIRBuilder, Info.CallConv, Info.IsVarArg))
      return false;
    StackSize = std::max(StackSize, RetAssigner.StackSize);
  }

  // Now that the size of the argument stack region is known, the setup call
  // frame pseudo can be given its arguments.
  CallSeqStart.addImm(StackSize).addImm(0);

  // Generate the call frame destroy pseudo with the correct sizes.
  MIRBuilder.buildInstr(MC6809::ADJCALLSTACKUP).addImm(StackSize).addImm(0);

  // Mark the function as having calls so the register allocator knows to
  // preserve caller-saved registers and the frame pointer.
  MF.getFrameInfo().setHasCalls(true);
  MF.getFrameInfo().setAdjustsStack(true);
  return true;
}

void MC6809CallLowering::splitToValueTypes(const ArgInfo &OrigArg, SmallVectorImpl<ArgInfo> &SplitArgs, const DataLayout &DL) const {
  size_t OldSize = SplitArgs.size();
  CallLowering::splitToValueTypes(OrigArg, SplitArgs, DL, CallingConv::C);
  auto NewArgs = make_range(SplitArgs.begin() + OldSize, SplitArgs.end());

  // Transfer is-pointer information from LLTs to argument flags.
  SmallVector<LLT> SplitLLTs;
  computeValueLLTs(DL, *OrigArg.Ty, SplitLLTs);
  assert((size_t)size(NewArgs) == SplitLLTs.size());
  for (const auto &I : zip(NewArgs, SplitLLTs))
    std::apply(adjustArgFlags, I);
}
