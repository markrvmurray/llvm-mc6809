//===-- MC6809InstrInfo.h - MC6809 Instruction Information ------------*- C++
//-*-===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file contains the MC6809 implementation of the TargetInstrInfo class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809INSTRINFO_H
#define LLVM_LIB_TARGET_MC6809_MC6809INSTRINFO_H

#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/TargetInstrInfo.h"

#include "MC6809RegisterInfo.h"

using namespace llvm;

#define GET_INSTRINFO_HEADER
#include "MC6809.h"
#include "MC6809GenInstrInfo.inc"

namespace llvm {
struct RegPlusOffsetLen {
  Register Reg;
  int OffsetLen;
  bool operator==(const RegPlusOffsetLen &Other) const { return Reg == Other.Reg && OffsetLen == Other.OffsetLen; }
};

template <> struct DenseMapInfo<RegPlusOffsetLen> {
  static inline RegPlusOffsetLen getEmptyKey() { return {((unsigned)(-1)), -1}; }
  static inline RegPlusOffsetLen getTombstoneKey() { return {((unsigned)(-1)) - 1, -2}; }
  static unsigned getHashValue(const RegPlusOffsetLen &V) { return hash_combine(DenseMapInfo<int>::getHashValue(V.Reg), DenseMapInfo<int>::getHashValue(V.OffsetLen)); }
  static bool isEqual(const RegPlusOffsetLen &A, const RegPlusOffsetLen &B) { return A == B; }
};

struct RegPlusReg {
  Register DestReg;
  Register OffsetReg;
  bool operator==(const RegPlusReg &Other) const { return DestReg == Other.DestReg && OffsetReg == Other.OffsetReg; }
};

template <> struct DenseMapInfo<RegPlusReg> {
  static inline RegPlusReg getEmptyKey() { return {((unsigned)(-1)), ((unsigned)(-1))}; }
  static inline RegPlusReg getTombstoneKey() { return {((unsigned)(-1)) - 1, ((unsigned)(-1)) - 1}; }
  static unsigned getHashValue(const RegPlusReg &V) { return hash_combine(DenseMapInfo<int>::getHashValue(V.DestReg), DenseMapInfo<int>::getHashValue(V.OffsetReg)); }
  static bool isEqual(const RegPlusReg &A, const RegPlusReg &B) { return A == B; }
};

} // namespace llvm

namespace llvm {

class MC6809Subtarget;
class MC6809RegisterBankInfo;

class MC6809InstrInfo final : public MC6809GenInstrInfo {
  // const MC6809Subtarget &STI;
  const MC6809RegisterInfo RI;

public:
  MC6809InstrInfo(const MC6809Subtarget &STI);

  /// getRegisterInfo - TargetInstrInfo is a superset of MRegister info.  As
  /// such, whenever a client has an instance of instruction info, it should
  /// always be able to get register info as well (through this method).
  ///
  const MC6809RegisterInfo &getRegisterInfo() const { return RI; }

#if 0
  bool isReallyTriviallyReMaterializable(const MachineInstr &MI, AAResults *AA) const override;
#endif
  Register isLoadFromStackSlot(const MachineInstr &MI, int &FrameIndex) const override;

  Register isStoreToStackSlot(const MachineInstr &MI, int &FrameIndex) const override;

  void reMaterialize(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg, unsigned SubIdx, const MachineInstr &Orig) const override;

  MachineInstr *commuteInstructionImpl(MachineInstr &MI, bool NewMI, unsigned OpIdx1, unsigned OpIdx2) const override;

  unsigned getInstSizeInBytes(const MachineInstr &MI) const override;

  unsigned getInstBundleLength(const MachineInstr &MI) const;

  bool findCommutedOpIndices(const MachineInstr &MI, unsigned &SrcOpIdx1, unsigned &SrcOpIdx2) const override;

#if 0
  bool isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const override;
#endif

  MachineBasicBlock *getBranchDestBlock(const MachineInstr &MI) const override;

  bool analyzeBranch(MachineBasicBlock &MBB, MachineBasicBlock *&TBB, MachineBasicBlock *&FBB, SmallVectorImpl<MachineOperand> &Cond, bool AllowModify = false) const override;

  unsigned removeBranch(MachineBasicBlock &MBB, int *BytesRemoved = nullptr) const override;

  unsigned insertBranch(MachineBasicBlock &MBB, MachineBasicBlock *TBB, MachineBasicBlock *FBB, ArrayRef<MachineOperand> Cond, const DebugLoc &DL, int *BytesAdded = nullptr) const override;

  void insertIndirectBranch(MachineBasicBlock &MBB, MachineBasicBlock &NewDestBB, MachineBasicBlock &RestoreBB, const DebugLoc &DL, int64_t BrOffset = 0, RegScavenger *RS = nullptr) const override;

  bool isBranchOffsetInRange(unsigned BranchOpc, int64_t BrOffset) const override;

  void copyPhysReg(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, const DebugLoc &DL, Register DestReg, Register SrcReg, bool KillSrc, bool RenamableDest = false, bool RenamableSrc = false) const override;

  void storeRegToStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register SrcReg, bool isKill, int FrameIndex, const TargetRegisterClass *RC, Register VReg, MachineInstr::MIFlag Flags = MachineInstr::NoFlags) const override;

  const TargetRegisterClass *canFoldCopy(const MachineInstr &MI, const TargetInstrInfo &TII, unsigned FoldIdx) const override;

  void loadRegFromStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register DestReg, int FrameIndex, const TargetRegisterClass *RC, Register VReg, unsigned SubReg = 0, MachineInstr::MIFlag Flags = MachineInstr::NoFlags) const override;

  void loadStoreRegStackSlot(MachineBasicBlock &MBB, MachineBasicBlock::iterator MI, Register Reg, bool IsKill, int FrameIndex, const TargetRegisterClass *RC, bool IsLoad) const;

  bool expandPostRAPseudo(MachineInstr &MI) const override;

  bool reverseBranchCondition(SmallVectorImpl<MachineOperand> &Cond) const override;

  std::pair<unsigned, unsigned> decomposeMachineOperandsTargetFlags(unsigned TF) const override;

  ArrayRef<std::pair<unsigned, const char *>> getSerializableDirectMachineOperandTargetFlags() const override;

  int64_t getFramePoppedByCallee(const MachineInstr &MI) const {
    assert(isFrameInstr(MI) && "Not a frame instruction");
    assert(MI.getOperand(1).getImm() >= 0 && "Size must not be negative");
    return MI.getOperand(1).getImm();
  }

private:
  // const MC6809RegisterBankInfo &RBI;

  // void copyPhysRegImpl(MachineBasicBlock &MBB, MachineIRBuilder &Builder, Register DestReg, Register SrcReg) const;

  // Post RA pseudos
  void expandSetIf(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandCallRelative(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandLEAPtrAdd(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandLoadIdx(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandLoadImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandLoad1Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandLDImmRemat(MachineIRBuilder &Builder) const;
  // void expandLDZ(MachineIRBuilder &Builder) const;
  // void expandIncDec(MachineIRBuilder &Builder) const;
  void expandStoreIdx(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandNegate(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandShiftLeft(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandShiftRight(MachineIRBuilder &Builder, MachineInstr &MI, bool Arithmetic) const;
  void expandMulD(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMul16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMulH16Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMul16IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMulH16IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMul16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMulH16IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandMul16Pop(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMul16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandMulH16Reg(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandANDReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandANDPull(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandORReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandORPull(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandXORReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandXORPull(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandAddPull(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandAddReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandAddSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandAddSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandAdd32IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandAdd32IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandSubReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandSubImmRev(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSubSetCarryReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSubSetCarryUseReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSubPull(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSub32IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSub32IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandSub32Pop(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandCompareImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandCompareIdx(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandCompareReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandComparePop(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandCompare32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandCompare32IdxImm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  // void expandCompare32IdxReg(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandTestReg(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandTestReg32(MachineIRBuilder &Builder, MachineInstr &MI) const;

  void expandAdd32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSub32Imm(MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandSub32ImmRev(MachineIRBuilder &Builder, MachineInstr &MI) const;

  // NZ pseudos
  // void expandNZ(MachineIRBuilder &Builder) const;
  // void expandCMPTerm(MachineIRBuilder &Builder) const;

  // Control flow pseudos
  // void expandGBR(MachineIRBuilder &Builder) const;

  DenseMap<RegPlusOffsetLen, unsigned> LEAPtrAddImmOpcode;
  DenseMap<RegPlusReg, unsigned> LEAPtrAddRegOpcode;

  DenseMap<Register, unsigned> LoadImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> LoadIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> LoadIdxRegOpcode;

  DenseMap<RegPlusOffsetLen, unsigned> StoreIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> StoreIdxRegOpcode;

  DenseMap<Register, unsigned> AddImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> AddIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> AddIdxRegOpcode;
  DenseMap<Register, unsigned> AddPopOpcode;

  DenseMap<Register, unsigned> AddCarryImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> AddCarryIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> AddCarryIdxRegOpcode;

  DenseMap<Register, unsigned> AddPullOpcode;

  DenseMap<Register, unsigned> SubImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> SubIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> SubIdxRegOpcode;
  DenseMap<Register, unsigned> SubPullOpcode;

  DenseMap<Register, unsigned> SubBorrowImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> SubBorrowIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> SubBorrowIdxRegOpcode;
  // DenseMap<Register, unsigned> SubBorrowPopOpcode;

  DenseMap<Register, unsigned> CompareImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> CompareIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> CompareIdxRegOpcode;
  // DenseMap<Register, unsigned> ComparePopOpcode;

  DenseMap<Register, unsigned> TestRegOpcode;

  DenseMap<Register, unsigned> ANDImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> ANDIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> ANDIdxRegOpcode;
  DenseMap<Register, unsigned> ANDPullOpcode;
  DenseMap<Register, unsigned> ORImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> ORIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> ORIdxRegOpcode;
  DenseMap<Register, unsigned> ORPullOpcode;
  DenseMap<Register, unsigned> XORImmediateOpcode;
  DenseMap<RegPlusOffsetLen, unsigned> XORIdxImmOpcode;
  DenseMap<RegPlusReg, unsigned> XORIdxRegOpcode;
  DenseMap<Register, unsigned> XORPullOpcode;

  struct ContextImmediate {
    DenseMap<Register, unsigned> *Opcode;
    int IdentityValue;
  };

  ContextImmediate AddImm = {&AddImmediateOpcode, 0};
  ContextImmediate AddCarryImm = {&AddCarryImmediateOpcode, 0};
  ContextImmediate SubImm = {&SubImmediateOpcode, 0};
  ContextImmediate SubBorrowImm = {&SubBorrowImmediateOpcode, 0};
  ContextImmediate ANDImm = {&ANDImmediateOpcode, -1};
  ContextImmediate ORImm = {&ORImmediateOpcode, 0};
  ContextImmediate XORImm = {&XORImmediateOpcode, 0};

  void expandImm(ContextImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandCarryImm16(bool IsAdd, MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandCarryMem16(bool IsAdd, MachineIRBuilder &Builder, MachineInstr &MI) const;

  struct ContextIndexImmediate {
    DenseMap<RegPlusOffsetLen, unsigned> *Opcode;
  };

  ContextIndexImmediate AddIdxImm = {&AddIdxImmOpcode};
  ContextIndexImmediate AddCarryIdxImm = {&AddCarryIdxImmOpcode};
  ContextIndexImmediate SubIdxImm = {&SubIdxImmOpcode};
  ContextIndexImmediate SubBorrowIdxImm = {&SubBorrowIdxImmOpcode};
  ContextIndexImmediate ANDIdxImm = {&ANDIdxImmOpcode};
  ContextIndexImmediate ORIdxImm = {&ORIdxImmOpcode};
  ContextIndexImmediate XORIdxImm = {&XORIdxImmOpcode};

  void expandIdxImm(ContextIndexImmediate Context, MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandBitwiseImm16(ContextImmediate Context, unsigned OpcA, unsigned OpcB,
                          MachineIRBuilder &Builder, MachineInstr &MI) const;
  void expandBitwiseMem16(ContextIndexImmediate Context,
                          MachineIRBuilder &Builder, MachineInstr &MI) const;

  struct ContextIndexRegister {
    DenseMap<RegPlusReg, unsigned> *Opcode;
  };

  ContextIndexRegister AddIdxReg = {&AddIdxRegOpcode};
  ContextIndexRegister SubIdxReg = {&SubIdxRegOpcode};
  ContextIndexRegister ANDIdxReg = {&ANDIdxRegOpcode};
  ContextIndexRegister ORIdxReg = {&ORIdxRegOpcode};
  ContextIndexRegister XORIdxReg = {&XORIdxRegOpcode};

  void expandIdxReg(ContextIndexRegister Context, MachineIRBuilder &Builder, MachineInstr &MI) const;

  //  DenseMap<Register, unsigned> NegRegOpcode;

  static int offsetSizeInBits(MachineOperand &OffsetOp);

  bool isJumpTableBranch(const MachineBasicBlock::instr_iterator &I) const;
  bool isIndirBranch(const MachineBasicBlock::instr_iterator &I) const;
  bool isCondBranch(const MachineBasicBlock::instr_iterator &I) const;
  bool isUnCondBranch(const MachineBasicBlock::instr_iterator &I) const;
  MachineBasicBlock *getBB(const MachineBasicBlock::instr_iterator &I) const;
};

namespace MC6809 {

enum AddressSpace { AS_Memory, AS_DirectPage, NumAddrSpaces };

MC6809CC::CondCode GetBranchConditionForPredicate(CmpInst::Predicate Pred, bool &IsSigned);

enum TOF {
  MO_NO_FLAGS = 0,
  MO_LO,
  MO_HI,
  MO_HI_JT,
};

/// emitFrameOffset - Emit instructions as needed to set DestReg to SrcReg
/// plus Offset.  This is intended to be used from within the prolog/epilog
/// insertion (PEI) pass, where a virtual scratch register may be allocated
/// if necessary, to be replaced by the scavenger at the end of PEI.
void emitFrameOffset(MachineBasicBlock &MBB, MachineBasicBlock::iterator MBBI, const DebugLoc &DL, unsigned DestReg, unsigned SrcReg, StackOffset Offset, const TargetInstrInfo *TII, MachineInstr::MIFlag = MachineInstr::NoFlags);

} // namespace MC6809

} // namespace llvm

#endif // not LLVM_LIB_TARGET_MC6809_MC6809INSTRINFO_H
