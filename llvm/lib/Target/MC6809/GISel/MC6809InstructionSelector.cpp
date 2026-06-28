//===-- MC6809InstructionSelector.cpp - MC6809 Instruction Selector -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the MC6809 instruction selector. Instructions selected here
// are abstract pseudo-instructions which will allow register allocation to be
// applied later.
//
//===----------------------------------------------------------------------===//

#include "MC6809InstructionSelector.h"
#include "MC6809.h"
#include "MC6809RegisterBankInfo.h"

#include "MC6809RegisterInfo.h"
#include "MC6809Subtarget.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/APFloat.h"
#include "llvm/Analysis/AliasAnalysis.h"
#include "llvm/CodeGen/GlobalISel/GIMatchTableExecutorImpl.h"
#include "llvm/CodeGen/GlobalISel/GenericMachineInstrs.h"
#include "llvm/CodeGen/GlobalISel/InstructionSelector.h"
#include "llvm/CodeGen/GlobalISel/MIPatternMatch.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/CodeGen/GlobalISel/Utils.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineOperand.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterBankInfo.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/BinaryFormat/Dwarf.h"
#include "llvm/Support/ErrorHandling.h"

using namespace llvm;
using namespace MIPatternMatch;

#define DEBUG_TYPE "mc6809-isel"

namespace {

// Bug #319 (2026-05-21): narrow a vreg's class to a tighter target
// class when the producer (e.g. ConditionalImm / SEX8Implicit /
// ZEX8Implicit, post-BIT1-elimination) emits at the broader ACC8 but
// the consumer pseudo (AddSetCarryUse_i8_Reg, MERGE_LOHI_i16, etc.)
// requires a sub-class.
//
// At -O0/-O2/etc. the regalloc already narrows via
// constrainSelectedInstRegOperands.  At -Og with -fextend-lifetimes
// the FAKE_USE on the wider class blocks the narrow — verifier
// rejects "Expected ABc, got ACC8" on hash_buf.c et al.
//
// hasFakeUse: returns true iff the surrounding MachineFunction has any
// FAKE_USE — used as a heuristic to gate the narrowing logic so it
// fires only when -fextend-lifetimes is active.
static bool hasFakeUse(const MachineInstr &MI) {
  for (const auto &BB : *MI.getMF())
    for (const auto &Op : BB)
      if (Op.getOpcode() == TargetOpcode::FAKE_USE)
        return true;
  return false;
}

static unsigned getOS9WritableGlobalTargetFlags(const GlobalValue *GV) {
  const auto *GVar = dyn_cast<GlobalVariable>(GV->getAliaseeObject());
  if (!GVar || GVar->isConstant())
    return MC6809::MO_NO_FLAGS;
  if (GVar->isDeclaration())
    return MC6809::MO_OS9_DATA;

  StringRef Section = GVar->getSection();
  if (Section.starts_with(".bss"))
    return MC6809::MO_OS9_BSS;
  if (Section.starts_with(".data"))
    return MC6809::MO_OS9_DATA;

  const Constant *Init = GVar->getInitializer();
  return Init->isNullValue() ? MC6809::MO_OS9_BSS : MC6809::MO_OS9_DATA;
}

// narrowToClass: returns a vreg of class `RC` holding the same value
// as `R`.  If `R` is already in `RC`, returns `R` unchanged.
// Otherwise emits a COPY to a fresh `RC`-classed vreg and returns
// that.
static Register narrowToClass(MachineIRBuilder &Builder,
                              MachineRegisterInfo *MRI,
                              Register R,
                              const TargetRegisterClass *RC) {
  if (MRI->getRegClassOrNull(R) == RC)
    return R;
  Register Copy = MRI->createVirtualRegister(RC);
  Builder.buildCopy(Copy, R);
  return Copy;
}

#define GET_GLOBALISEL_PREDICATE_BITSET
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATE_BITSET

class MC6809InstructionSelector : public InstructionSelector {
public:
  MC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI);

  bool select(MachineInstr &MI) override;
  static const char *getName() { return DEBUG_TYPE; }

  void setupMF(MachineFunction &MF, GISelValueTracking *VT, CodeGenCoverage *CovInfo, ProfileSummaryInfo *PSI, BlockFrequencyInfo *BFI, AAResults *AA) override;

private:
  const MC6809Subtarget &STI;
  const MC6809InstrInfo &TII;
  const MC6809RegisterInfo &TRI;
  const MC6809RegisterBankInfo &RBI;

  MachineBasicBlock *MBB;
  MachineFunction *MF;
  MachineRegisterInfo *MRI;
  MachineIRBuilder MIB;

  // Post-tablegen selection functions. If these return false, it is an error.
  bool tryFusePostModify(MachineInstr &MI) const;
  bool selectFrameIndex(MachineInstr &MI);
  bool selectMergeValues(MachineInstr &MI);
  bool selectUnMergeValues(MachineInstr &MI);
  bool selectAddO(MachineInstr &MI);
  bool selectAddE(MachineInstr &MI);
  bool selectSubO(MachineInstr &MI);
  bool selectSubE(MachineInstr &MI);
  bool selectShiftExtend(MachineInstr &MI);
  bool selectShift16(MachineInstr &MI);

  // Select instructions that correspond 1:1 to a target instruction.
  bool selectGeneric(MachineInstr &MI);

  void constrainGenericOp(MachineInstr &MI);

  void constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass);

  /// tblgenerated 'select' implementation, used as the initial selector for
  /// the patterns that don't require complex C++.
  bool selectImpl(MachineInstr &MI, CodeGenCoverage &CoverageInfo) const;

  LLT S1 = LLT::scalar(1);
  LLT S2 = LLT::scalar(2);
  LLT S8 = LLT::scalar(8);
  LLT S16 = LLT::scalar(16);
  LLT S32 = LLT::scalar(32);
  LLT P = LLT::pointer(0, 16);

  // Bug #186 v5 (2026-04-27): tracker mapping IR-level carry/overflow
  // vregs to the flag they semantically represent. Populated by
  // selectAddO/SubO/AddE/SubE when they emit SetCarry/SetCarryUse-family
  // pseudos (which no longer have an explicit carry-out operand —
  // see MC6809InstrFamilies.td). Consulted by `getPhantomCarryFlag`
  // before its old producer-walking heuristic so consumers know
  // which flag (C or V) the IR vreg is meant to be.
  //
  // Cleared per-function in setupMF.
  DenseMap<Register, MCPhysReg> CarryFlagOf;

  // Bug #186 follow-up Phase 2a (2026-04-28): cache for the byte-bridging
  // workaround. Maps a SetCarry/SetCarryUse producer MI to the ABc byte
  // vreg holding its CC.C value (frozen via MaterializeCarryToByte_i8
  // immediately after the producer). Multiple consumers reading the same
  // producer's CC.C share this byte — `MaterializeCarryToByte_i8` is
  // emitted exactly ONCE per producer (it reads CC.C, which is only
  // valid right after the producer); each consumer emits its own
  // `MaterializeByteToCarry_i8` (LSRB) right before its use.
  // Cleared per-function in setupMF.
  DenseMap<MachineInstr *, Register> BridgedByteFor;
  ComplexRendererFns selectLSIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectLSIndexedRegOffset(MachineOperand &Root) const;
  ComplexRendererFns selectLSIndexedIndirect(MachineOperand &Root) const;
  ComplexRendererFns selectLSUnmergeIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectLSFrameIndex(MachineOperand &Root) const;
  ComplexRendererFns selectAMImmediate(MachineOperand &Root) const;
  ComplexRendererFns selectAMIndexedImmOffset(MachineOperand &Root) const;
  ComplexRendererFns selectAMUnmergeIndexedImmOffset(MachineOperand &Root) const;

#define GET_GLOBALISEL_PREDICATES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_DECL

#define GET_GLOBALISEL_TEMPORARIES_DECL
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_DECL
};

void MC6809InstructionSelector::setupMF(MachineFunction &MF,
                                     GISelValueTracking *VT,
                                     CodeGenCoverage *CovInfo,
                                     ProfileSummaryInfo *PSI,
                                     BlockFrequencyInfo *BFI, AAResults *AA) {
  InstructionSelector::setupMF(MF, VT, CovInfo, PSI, BFI, AA);

  // Bug #186 v5: per-function carry/overflow flag tracking.
  CarryFlagOf.clear();
  // Bug #186 follow-up Phase 2a: per-function byte-bridge cache.
  BridgedByteFor.clear();

  // The machine verifier doesn't allow COPY instructions to have differing
  // types, but the various GlobalISel utilities used in the instruction
  // selector really need to be able to look through G_PTRTOINT and G_INTTOPTR
  // as if they were copies. To avoid maintaining separate versions of these, we
  // temporarily lower these to technically-illegal COPY instructions, but only
  // for the duration of this one pass.
  for (MachineBasicBlock &MBB : MF) {
    for (MachineInstr &MI : MBB) {
      switch (MI.getOpcode()) {
      case MC6809::G_PTRTOINT:
      case MC6809::G_INTTOPTR:
        MI.setDesc(TII.get(MC6809::COPY));
        break;
      }
    }
  }
}

// Unified phantom-carry query.
//
// Follow a chain of G_FREEZE / COPY from \p SrcReg back to its ultimate
// producer.  If that producer is one whose s1 result is a scheduling
// phantom — i.e. the value really lives in a specific CC bit, not in
// the byte the regalloc allocated — return the CC physreg holding the
// real value:
//   - MC6809::C  for unsigned carry-out producers (G_U*ADD/SUB*O/E,
//                 AddSetCarry*/SubSetCarry* pseudos + *Use variants).
//   - MC6809::V  for signed overflow-out producers (G_S*ADD/SUB*O/E,
//                 AddSetOverflow*/SubSetOverflow* + *Use).
// Returns std::nullopt for genuine s1 byte values (from G_ICMP,
// ConditionalImm 0/1, G_CONSTANT, G_LOAD bool, etc.) — the byte's
// LSB carries the boolean directly.
//
// Consumer sites use this to decide whether to emit a flag-reading
// sequence (true phantom) or a byte-LSB-reading sequence (real byte).
static std::optional<MCPhysReg>
getPhantomCarryFlag(Register SrcReg, const MachineRegisterInfo &MRI,
                   const DenseMap<Register, MCPhysReg> *CarryFlagOf = nullptr) {
  Register Cur = SrcReg;
  while (Cur.isVirtual()) {
    // Bug #186 v5: tracker lookup. If the IR-level carry/overflow vreg
    // was registered when its SetCarry/SetCarryUse producer was
    // selected (because v5 dropped the explicit carry-out operand from
    // those pseudos), the tracker tells us which CC bit the vreg
    // semantically represents — without needing to find an MI def for it.
    if (CarryFlagOf) {
      auto It = CarryFlagOf->find(Cur);
      if (It != CarryFlagOf->end())
        return It->second;
    }
    MachineInstr *Def = MRI.getVRegDef(Cur);
    if (!Def) return std::nullopt;
    unsigned Op = Def->getOpcode();
    // Bug #186 follow-up Phase 1a (2026-04-28): the v5 IMPLICIT_DEF
    // placeholder is gone — selectAddO/SubO/AddE/SubE now attach an
    // implicit-def of the IR carry vreg directly onto the SetCarry/
    // SetOverflow pseudo, so MRI->getVRegDef returns the producing
    // pseudo and the switch below recognises it. This branch only
    // fires for genuinely undef carry vregs (legalizer artefacts);
    // safe to terminate the walk.
    if (Op == TargetOpcode::IMPLICIT_DEF) return std::nullopt;
    // Walk through transparent pass-throughs.
    if (Op == TargetOpcode::G_FREEZE || Op == TargetOpcode::COPY) {
      if (!Def->getOperand(1).isReg()) return std::nullopt;
      Cur = Def->getOperand(1).getReg();
      continue;
    }
    // Phantom-carry producers.  Keep in sync with MC6809InstrInfo.cpp
    // expansions for each SetCarry / SetOverflow family.
    switch (Op) {
    // Carry producers.
    case TargetOpcode::G_USUBO:
    case TargetOpcode::G_UADDO:
    case TargetOpcode::G_USUBE:
    case TargetOpcode::G_UADDE:
    case MC6809::SubSetCarry_i8_Imm:    case MC6809::SubSetCarry_i16_Imm:    case MC6809::SubSetCarry_i32_Imm:
    case MC6809::SubSetCarry_i8_Mem:    case MC6809::SubSetCarry_i16_Mem:    case MC6809::SubSetCarry_i32_Mem:
    case MC6809::SubSetCarry_i8_Pull:   case MC6809::SubSetCarry_i16_Pull:
    case MC6809::SubSetCarry_i8_Reg:    case MC6809::SubSetCarry_i16_Reg:    case MC6809::SubSetCarry_i32_Reg:
    case MC6809::AddSetCarry_i8_Imm:    case MC6809::AddSetCarry_i16_Imm:    case MC6809::AddSetCarry_i32_Imm:
    case MC6809::AddSetCarry_i8_Mem:    case MC6809::AddSetCarry_i16_Mem:    case MC6809::AddSetCarry_i32_Mem:
    case MC6809::AddSetCarry_i8_Pull:   case MC6809::AddSetCarry_i16_Pull:
    case MC6809::AddSetCarry_i8_Reg:    case MC6809::AddSetCarry_i16_Reg:    case MC6809::AddSetCarry_i32_Reg:
    case MC6809::SubSetCarryUse_i8_Imm: case MC6809::SubSetCarryUse_i16_Imm: case MC6809::SubSetCarryUse_i32_Imm:
    case MC6809::SubSetCarryUse_i8_Mem: case MC6809::SubSetCarryUse_i16_Mem: case MC6809::SubSetCarryUse_i32_Mem:
    case MC6809::SubSetCarryUse_i8_Reg: case MC6809::SubSetCarryUse_i16_Reg: case MC6809::SubSetCarryUse_i32_Reg:
    case MC6809::AddSetCarryUse_i8_Imm: case MC6809::AddSetCarryUse_i16_Imm: case MC6809::AddSetCarryUse_i32_Imm:
    case MC6809::AddSetCarryUse_i8_Mem: case MC6809::AddSetCarryUse_i16_Mem: case MC6809::AddSetCarryUse_i32_Mem:
    case MC6809::AddSetCarryUse_i8_Reg: case MC6809::AddSetCarryUse_i16_Reg: case MC6809::AddSetCarryUse_i32_Reg:
      return MC6809::C;
    // Overflow producers.
    case TargetOpcode::G_SSUBO:
    case TargetOpcode::G_SADDO:
    case TargetOpcode::G_SSUBE:
    case TargetOpcode::G_SADDE:
    case MC6809::SubSetOverflow_i8_Imm:     case MC6809::SubSetOverflow_i16_Imm:     case MC6809::SubSetOverflow_i32_Imm:
    case MC6809::SubSetOverflow_i8_Mem:     case MC6809::SubSetOverflow_i16_Mem:     case MC6809::SubSetOverflow_i32_Mem:
    case MC6809::SubSetOverflow_i8_Pull:    case MC6809::SubSetOverflow_i16_Pull:
    case MC6809::SubSetOverflow_i8_Reg:     case MC6809::SubSetOverflow_i16_Reg:     case MC6809::SubSetOverflow_i32_Reg:
    case MC6809::AddSetOverflow_i8_Imm:     case MC6809::AddSetOverflow_i16_Imm:     case MC6809::AddSetOverflow_i32_Imm:
    case MC6809::AddSetOverflow_i8_Mem:     case MC6809::AddSetOverflow_i16_Mem:     case MC6809::AddSetOverflow_i32_Mem:
    case MC6809::AddSetOverflow_i8_Pull:    case MC6809::AddSetOverflow_i16_Pull:
    case MC6809::AddSetOverflow_i8_Reg:     case MC6809::AddSetOverflow_i16_Reg:     case MC6809::AddSetOverflow_i32_Reg:
    case MC6809::SubSetOverflowUse_i8_Imm:  case MC6809::SubSetOverflowUse_i16_Imm:  case MC6809::SubSetOverflowUse_i32_Imm:
    case MC6809::SubSetOverflowUse_i8_Mem:  case MC6809::SubSetOverflowUse_i16_Mem:  case MC6809::SubSetOverflowUse_i32_Mem:
    case MC6809::SubSetOverflowUse_i8_Reg:  case MC6809::SubSetOverflowUse_i16_Reg:  case MC6809::SubSetOverflowUse_i32_Reg:
    case MC6809::AddSetOverflowUse_i8_Imm:  case MC6809::AddSetOverflowUse_i16_Imm:  case MC6809::AddSetOverflowUse_i32_Imm:
    case MC6809::AddSetOverflowUse_i8_Mem:  case MC6809::AddSetOverflowUse_i16_Mem:  case MC6809::AddSetOverflowUse_i32_Mem:
    case MC6809::AddSetOverflowUse_i8_Reg:  case MC6809::AddSetOverflowUse_i16_Reg:  case MC6809::AddSetOverflowUse_i32_Reg:
      return MC6809::V;
    default:
      return std::nullopt;
    }
  }
  return std::nullopt;
}

/// Bug #186 follow-up Phase 2a / Phase 5 (2026-04-28): does this MI
/// clobber the given CC bit (MC6809::C or MC6809::V)?
///
/// Generic check via MachineInstr::modifiesRegister — covers explicit Defs,
/// implicit Defs, and tied operands. Self-updating when new CC-clobbering
/// pseudos appear in TableGen; no hardcoded opcode list to maintain.
///
/// Fast paths skip MIs that obviously can't touch a physreg they don't
/// declare (COPY/PHI/IMPLICIT_DEF/DBG_VALUE).
static bool clobbersCCFlag(const MachineInstr &MI,
                           const TargetRegisterInfo &TRI,
                           MCPhysReg Flag) {
  if (MI.isCopyLike() || MI.isPHI() || MI.isImplicitDef() ||
      MI.isDebugInstr())
    return false;
  return MI.modifiesRegister(Flag, &TRI);
}

/// Bug #186 follow-up Phase 2a (2026-04-28): formerly
/// `ensureCarryAcrossBlocks` (cross-BB only); now handles BOTH cross-BB
/// AND same-BB cases where a CC.C-clobbering MI sits between a SetCarry/
/// SetCarryUse producer and a SubSetCarryUse / AddSetCarryUse consumer.
///
/// Background: regalloc tracks the carry as an IR vreg, but its physical
/// realisation is CC.C. Without honest `Defs = [NZ, V, C]` on every CC.C-
/// clobbering MI (CMP/TST/BIT etc.), regalloc thinks CC.C survives those
/// instructions — and silently breaks the borrow chain when an intervening
/// CMP overwrites it.
///
/// The fix (v5+1a-compatible): freeze CC.C into a byte vreg via
/// `MaterializeCarryToByte_i8` (LDB #0; ADCB #0) immediately after the
/// producer, then restore CC.C from the byte's LSB via
/// `MaterializeByteToCarry_i8` (LSRB) immediately before each consumer.
/// The byte vreg is a regular ABc — regalloc tracks it like any other byte
/// value, no $c spilling needed. Mirrors how X86 / AArch64 handle EFLAGS
/// / NZCV: they don't spill the flag physreg either; either the chain is
/// adjacent or it's bridged through normal regs.
///
/// Multi-consumer cache: i32+ chains have several SubSetCarryUse pseudos
/// hanging off one SetCarry. The producer-side `MaterializeCarryToByte_i8`
/// is emitted EXACTLY ONCE per producer (it reads CC.C, which is only
/// valid right after the producer). Each consumer emits its own
/// `MaterializeByteToCarry_i8` and reuses the cached byte vreg.
///
/// Cross-BB chains: always bridge (any cross-BB path crosses unknown
/// CC-clobbering code; safer to assume).
/// Same-BB chains: scan MIs strictly between producer and consumer for
/// any clobbersCarryC. Only bridge if at least one is found —
/// preserves Phase 1a's wins on chains where the SetCarry/SetCarryUse
/// pair is adjacent (the common case, pinned by hasSideEffects).
/// Returns true if the carry was bridged through a byte vreg.  When this
/// returns true, the *caller* should NOT add CarryIn as an implicit-use
/// on the new pseudo it builds — the bridge has decoupled the
/// phantom_carry data flow from the Consumer (the actual carry value
/// flows via CC.C, set by the `ToCCPseudo` emitted right before the
/// Consumer's position).  Keeping the implicit-use would force regalloc
/// to keep the phantom_carry vreg alive across the bridge, which it
/// can't do across calls or cross-BB paths (phantom physregs are
/// caller-clobbered and have no real spill storage).
///
/// Returns false if no bridge was needed (adjacent producer-consumer
/// with no intervening CC clobber).  In that case the caller's
/// `.addUse(CarryIn, RegState::Implicit)` is still required for DCE
/// protection (Bug #161 round 12 — keeps the upstream alive when its
/// non-carry byte result is dead).
static bool
ensureCarryChainIntegrity(MachineInstr &Consumer, Register CarryIn,
                          MachineRegisterInfo &MRI,
                          const TargetInstrInfo &TII,
                          const TargetRegisterInfo &TRI,
                          const RegisterBankInfo &RBI,
                          const DenseMap<Register, MCPhysReg> *CarryFlagOf,
                          DenseMap<MachineInstr *, Register> &BridgedByteFor) {
  auto Flag = getPhantomCarryFlag(CarryIn, MRI, CarryFlagOf);
  if (!Flag || (*Flag != MC6809::C && *Flag != MC6809::V))
    return false;

  // Pick the right materialise pair for the flag we're bridging.
  //   C: LDB #0 ; ADCB #0      (or LSRB to invert sense)
  //   V: TFR CC,B ; LSRB ; ANDB #1   (or ANDB #1 ; ADDB #0x7F)
  // See the pseudo definitions in MC6809InstrPseudos.td.
  //
  // CC.C / CC.V is read directly via the pseudo's `Uses = [C]` /
  // `Uses = [V]`.  The inserted freeze goes immediately after the
  // producer in the producer's MBB, so the flag is live at the read.
  // `hasSideEffects = 1` on both producer and freeze keeps them
  // ordered through scheduling.
  unsigned ToBytePseudo = (*Flag == MC6809::C)
      ? MC6809::MaterializeCC_C_to_byte
      : MC6809::MaterializeCC_V_to_byte;
  unsigned ToCCPseudo = (*Flag == MC6809::C)
      ? MC6809::MaterializeByteToCarry_i8
      : MC6809::MaterializeByteToOverflow_i8;

  // Walk back through COPY / G_FREEZE to find the actual producer MI.
  // Bug #186 v5/1a: the chain may go through IMPLICIT_DEF placeholders
  // (legacy v5 path) OR end at a SetCarry pseudo with implicit-def of
  // %CarryOut directly (Phase 1a). Both cases handled below.
  Register Cur = CarryIn;
  MachineInstr *Producer = nullptr;
  while (Cur.isVirtual()) {
    MachineInstr *Def = MRI.getVRegDef(Cur);
    if (!Def)
      return false;
    unsigned Op = Def->getOpcode();
    if (Op == TargetOpcode::G_FREEZE || Op == TargetOpcode::COPY) {
      if (!Def->getOperand(1).isReg())
        return false;
      Cur = Def->getOperand(1).getReg();
      continue;
    }
    if (Op == TargetOpcode::IMPLICIT_DEF) {
      // Legacy v5 path (pre-Phase-1a): the real producer is the
      // SetCarry/SetCarryUse pseudo emitted immediately before this
      // IMPLICIT_DEF. Step back one MI.
      auto It = MachineBasicBlock::iterator(*Def);
      if (It == Def->getParent()->begin())
        return false;
      --It;
      Producer = &*It;
      break;
    }
    Producer = Def;
    break;
  }
  if (!Producer)
    return false;

  // Decide whether to bridge:
  //  - Cross-BB: always bridge (any cross-BB path crosses unknown CC clobbers).
  //  - Same-BB: scan for an intervening CC-clobbering MI; bridge only if found.
  bool CrossBB = (Producer->getParent() != Consumer.getParent());
  bool NeedsBridge = CrossBB;
  if (!NeedsBridge) {
    // Producer must precede Consumer in the same BB. Defensive guard:
    auto PIt = MachineBasicBlock::iterator(*Producer);
    auto CIt = MachineBasicBlock::iterator(Consumer);
    auto BBEnd = Producer->getParent()->end();
    bool Found = false;
    for (auto It = std::next(PIt); It != BBEnd && It != CIt; ++It) {
      if (clobbersCCFlag(*It, TRI, *Flag)) {
        NeedsBridge = true;
        Found = true;
        break;
      }
    }
    if (!Found && !NeedsBridge)
      return false;  // No intervening clobber — Phase 1a's hasSideEffects
                     // adjacency is enough.  Caller's implicit-use
                     // of CarryIn on the new pseudo is still wanted.
  }

  // Producer-side byte (emit ToBytePseudo only once per producer; cache
  // the byte vreg).
  Register ByteVReg;
  auto CacheIt = BridgedByteFor.find(Producer);
  if (CacheIt != BridgedByteFor.end()) {
    ByteVReg = CacheIt->second;
  } else {
    // Defensive fallback: if the very next MI after Producer is already
    // the right ToBytePseudo (from a prior call that for some reason
    // didn't populate the cache — shouldn't happen, but be safe), reuse
    // its dest vreg.
    auto AfterProducer = std::next(MachineBasicBlock::iterator(*Producer));
    MachineBasicBlock &PMBB = *Producer->getParent();
    if (AfterProducer != PMBB.end() &&
        AfterProducer->getOpcode() == ToBytePseudo) {
      ByteVReg = AfterProducer->getOperand(0).getReg();
    } else {
      // Fresh: allocate byte vreg and emit ToBytePseudo.
      //
      // MaterializeCC_C_to_byte / MaterializeCC_V_to_byte have no
      // explicit input — CC.C / CC.V is read via the pseudo's
      // `Uses = [C]` / `Uses = [V]`.  CarryIn is attached as an
      // *implicit* use so it doesn't count against the pseudo's
      // explicit operand list (verifier rejects extras on non-
      // variadic MIs) but its live range still extends across the
      // bridge for regalloc bookkeeping.  Bug #307 — without the
      // implicit-use bookkeeping, the phantom_carry vreg's spill
      // decisions cascaded incorrectly (imaxabs / llabs failures
      // at -Og-hd6309-mame).
      ByteVReg = MRI.createVirtualRegister(&MC6809::ABcRegClass);
      MachineIRBuilder ProducerB(PMBB, AfterProducer);
      auto MtoB = ProducerB.buildInstr(ToBytePseudo)
                      .addDef(ByteVReg)
                      .addUse(CarryIn, RegState::Implicit);
      constrainSelectedInstRegOperands(*MtoB, TII, TRI, RBI);
    }
    BridgedByteFor[Producer] = ByteVReg;
  }

  // Consumer-side restore: emit ToCCPseudo immediately before this
  // Consumer. Each consumer needs its own restore.
  MachineIRBuilder ConsumerB(Consumer);
  auto BtoC = ConsumerB.buildInstr(ToCCPseudo)
                  .addUse(ByteVReg);
  constrainSelectedInstRegOperands(*BtoC, TII, TRI, RBI);

  // Bug #307 round 2 (2026-05-18): signal to the caller that the
  // bridge fired AND requires dropping the phantom_carry implicit-
  // use on the new pseudo.  This is only correct for CROSS-BB
  // bridges — where Carry would otherwise have to survive a call /
  // an unknown CC-clobbering region.  In those cases keeping the
  // implicit-use would force regalloc to keep the phantom_carry
  // vreg alive across the bridge — which is impossible (phantom
  // physregs are caller-clobbered, phantom vregs have no spill
  // storage) and triggers `loadStoreRegisterStaticStackSlot`'s
  // "Unexpected virtual register class" crash (or pre-fix the
  // verifier rejected `COPY_CC_PLACEHOLDER` from copyPhysReg's
  // fallback).
  //
  // Narrowed gate (2026-05-19): for SAME-BB CC-clobber bridges
  // (Producer and Consumer in the same MBB, intervening CC clobber
  // between them), Carry CAN stay alive in phantom physregs across
  // the bridge — there's no call involved.  The full bench at
  // 65bb11c2d5ec showed that dropping the implicit-use in the
  // same-BB-clobber case spread Bug #309's failure family to all
  // non-LTO HD6309-mame levels (~140 regressed tests).  Returning
  // `true` only for cross-BB bridges keeps the imaxabs/llabs fix
  // (the original Bug #307 round 2 manifest is cross-BB) while
  // preserving the implicit-use's role for same-BB CC-clobber
  // bridges (DCE protection, regalloc liveness signal, etc.).
  return CrossBB;
}

} // namespace

#define GET_GLOBALISEL_IMPL
#include "MC6809GenGlobalISel.inc"
#include "llvm/Support/FormatVariadic.h"
#undef GET_GLOBALISEL_IMPL

MC6809InstructionSelector::MC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI)
    : STI(STI), TII(*STI.getInstrInfo()), TRI(*STI.getRegisterInfo()), RBI(RBI),
#define GET_GLOBALISEL_PREDICATES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_PREDICATES_INIT
#define GET_GLOBALISEL_TEMPORARIES_INIT
#include "MC6809GenGlobalISel.inc"
#undef GET_GLOBALISEL_TEMPORARIES_INIT
{
}

// Defined below; used by selectLSIndexedIndirect.
static bool shouldFoldMemAccess(const MachineInstr &Dst, const MachineInstr &Src,
                                AAResults *AA);

/// Select a "register plus signed immediate offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMImmediate(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_CONSTANT) {
    return {{
        [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
    }};
  }

  return std::nullopt;
}

/// Select a "frame_index plus offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_LOAD || RootDef->getOpcode() == TargetOpcode::G_STORE) {
    MachineInstr *FrameDef = MRI.getVRegDef(RootDef->getOperand(1).getReg());
    if (FrameDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(FrameDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
      }};
    }
  }

  // Register-base memory fold (P3a): fold a single-use load of [base+const] or
  // [base] into an arith/compare consumer, so e.g. `*p == x` -> `cmp ,p` and
  // `p[k] == x` -> `cmp k,p` instead of materialising the loaded value into an
  // accumulator first (the consumer's _Mem pseudo does the load itself). The
  // base pointer stays in the index register it already occupies, and the
  // accumulator that would have held the value is freed. Guarded by OneUse (so
  // the standalone load is actually eliminated) + shouldFoldMemAccess (no
  // aliasing store / call / ordered access between the load and the consumer) --
  // the same safety regime the load/store indirect fold uses. Compare consumers
  // (G_ICMP) fold too: expandCompareIdx materializes a spilled index base and
  // (like the load expander) omits the offset operand for the o0 form, and the
  // compare-branch's Compare_*_Mem is re-expanded through the same path -- so
  // `*p == x` -> `cmp ,p`, `p[k] == x` -> `cmp k,p`.
  if (RootDef->getOpcode() == TargetOpcode::G_LOAD &&
      MRI.hasOneNonDBGUse(Root.getReg()) &&
      shouldFoldMemAccess(*Root.getParent(), *RootDef, AA)) {
    MachineInstr *AddrDef = MRI.getVRegDef(RootDef->getOperand(1).getReg());
    if (AddrDef && AddrDef->getOpcode() == TargetOpcode::G_PTR_ADD) {
      MachineInstr *OffDef = MRI.getVRegDef(AddrDef->getOperand(2).getReg());
      if (OffDef && OffDef->getOpcode() == TargetOpcode::G_CONSTANT)
        return {{
            [=](MachineInstrBuilder &MIB) { MIB.add(AddrDef->getOperand(1)); },
            [=](MachineInstrBuilder &MIB) { MIB.add(OffDef->getOperand(1)); },
        }};
    } else if (AddrDef &&
               AddrDef->getOpcode() != TargetOpcode::G_FRAME_INDEX &&
               AddrDef->getOpcode() != TargetOpcode::G_GLOBAL_VALUE) {
      // Plain register pointer: cmp/add ,p
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
      }};
    }
  }

  return std::nullopt;
}

/// Select a "high word unmerged frame index plus offset" address.
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectAMUnmergeIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (RootDef->getOpcode() == TargetOpcode::G_UNMERGE_VALUES) {
    // Big-endian: operand 0 = lo part, operand 1 = hi part.
    // Lo part is at the HIGHER address: base + element_size.
    // Hi part is at base + 0.
    //Register SrcReg = RootDef->getOperand(2).getReg();
    unsigned EltSize = MRI.getType(Root.getReg()).getSizeInBytes();
    bool IsLo = Root.getReg() == RootDef->getOperand(0).getReg();
    unsigned mergeoffset = IsLo ? EltSize : 0;
    MachineInstr *LoadDef = MRI.getVRegDef(RootDef->getOperand(2).getReg());
    if (LoadDef->getOpcode() == TargetOpcode::G_LOAD) {
      MachineInstr *FrameDef = MRI.getVRegDef(LoadDef->getOperand(1).getReg());
      if (FrameDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
        return {{
            [=](MachineInstrBuilder &MIB) { MIB.add(FrameDef->getOperand(1)); },
            [=](MachineInstrBuilder &MIB) { MIB.addImm(mergeoffset); },
        }};
      }
    }
  }

  return std::nullopt;
}

/// Select a "register plus signed immediate offset" address for a target load/store instruction
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectLSIndexedImmOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  auto RootDef = MRI.getVRegDef(Root.getReg());

  if (RootDef->getOpcode() == TargetOpcode::G_PTR_ADD) {
    auto OffsetDef = MRI.getVRegDef(RootDef->getOperand(2).getReg());
    if (OffsetDef->getOpcode() == TargetOpcode::G_CONSTANT) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.add(OffsetDef->getOperand(1)); },
      }};
    }
  }

  return std::nullopt;
}

/// Select a "base register plus register offset" address (6809 accumulator-
/// offset indexed: ld/st d,x or b,x). Matches (G_PTR_ADD base, off) where off is
/// a runtime value, yielding (base, off) so a[i] selects the register-offset
/// form directly -- both loads AND stores, pre-RA. Returns:
///  - an 8-bit (ACC8) offset register when off is a single-use G_SEXT of an i8
///    (the 6809 A/B offset is hardware sign-extended, so this skips the explicit
///    SEX -- e.g. signed a[i] -> `ldb b,x` rather than `sex; ldb d,x`);
///  - otherwise the full 16-bit (ACC16) offset register (correct for a zero-
///    extended unsigned index or a native i16 value -> `ld d,x`).
/// Constant offsets fall through (handled by selectLSIndexedImmOffset).
InstructionSelector::ComplexRendererFns
MC6809InstructionSelector::selectLSIndexedRegOffset(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();
  if (!Root.isReg())
    return std::nullopt;
  MachineInstr *RootDef = MRI.getVRegDef(Root.getReg());
  if (!RootDef || RootDef->getOpcode() != TargetOpcode::G_PTR_ADD)
    return std::nullopt;
  Register OffReg = RootDef->getOperand(2).getReg();
  MachineInstr *OffDef = MRI.getVRegDef(OffReg);
  if (!OffDef)
    return std::nullopt;
  // Constants are the imm matcher's job.
  if (OffDef->getOpcode() == TargetOpcode::G_CONSTANT)
    return std::nullopt;
  // Sign-extend gate: a sole-use G_SEXT of an i8 rides the hardware-sign-
  // extended 8-bit accumulator offset directly (ld b,x), eliding the explicit
  // SEX. This is the one register-offset load the post-RA foldLEAIntoLoad pass
  // cannot produce.
  //
  // We deliberately do NOT handle the 16-bit (D) offset case here: selecting
  // `Load(idx, D-offset)` keeps the index live in D across the load, so the
  // result (a sub-reg of D) clobbers the still-live offset -- regalloc does not
  // always reconcile this (a memmem miscompile) -- and tying up D raises loop
  // pressure (qsort/memmove timeouts). foldLEAIntoLoad already lowers the
  // unsigned/native case to `ld d,x` correctly and with better pressure (the
  // index dies at the leax), so leave those to it: return nullopt.
  if (OffDef->getOpcode() == TargetOpcode::G_SEXT) {
    Register X = OffDef->getOperand(1).getReg();
    if (MRI.getType(X) == LLT::scalar(8) && MRI.hasOneNonDBGUse(OffReg) &&
        MRI.hasOneNonDBGUse(X)) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.addReg(X); },
      }};
    }
  }
  return std::nullopt;
}

/// Select a "FrameIndex + immediate offset" address for a target load/store instruction
InstructionSelector::ComplexRendererFns MC6809InstructionSelector::selectLSFrameIndex(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();

  if (!Root.isReg())
    return std::nullopt;

  auto RootDef = MRI.getVRegDef(Root.getReg());

  if (RootDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
    return {{
        [=](MachineInstrBuilder &MIB) { MIB.add(RootDef->getOperand(1)); },
        [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
    }};
  }

  return std::nullopt;
}

/// Select an indirect-indexed address: the consumer's pointer operand is itself
/// a pointer that was loaded from memory at [idx + offset]. The 6809 indexed-
/// indirect mode (`ld [offset,idx]`) reads that pointer and derefs through it in
/// one instruction, so the explicit pointer-load -- and the scarce index
/// register it would occupy -- both vanish. Returns the *inner* load's
/// (idx, offset) for the indirect _MemIndirect pseudo.
InstructionSelector::ComplexRendererFns
MC6809InstructionSelector::selectLSIndexedIndirect(MachineOperand &Root) const {
  MachineRegisterInfo &MRI = Root.getParent()->getParent()->getParent()->getRegInfo();
  if (!Root.isReg())
    return std::nullopt;

  // The address operand must be a pointer that was itself loaded from memory...
  MachineInstr *InnerLoad = MRI.getVRegDef(Root.getReg());
  if (!InnerLoad || InnerLoad->getOpcode() != TargetOpcode::G_LOAD)
    return std::nullopt;
  // ...used only here, so folding the load away is a win, not a duplicate...
  if (!MRI.hasOneNonDBGUse(Root.getReg()))
    return std::nullopt;
  // ...and safe to re-read at the consumer (no aliasing store, call, or ordered
  // access between the inner load and this access -- same AA test the
  // single-fold path uses).
  if (!shouldFoldMemAccess(*Root.getParent(), *InnerLoad, AA))
    return std::nullopt;

  // Match the inner load's address: [base + const], else plain [base] (off 0).
  MachineInstr *AddrDef = MRI.getVRegDef(InnerLoad->getOperand(1).getReg());
  if (AddrDef && AddrDef->getOpcode() == TargetOpcode::G_PTR_ADD) {
    MachineInstr *OffDef = MRI.getVRegDef(AddrDef->getOperand(2).getReg());
    if (OffDef && OffDef->getOpcode() == TargetOpcode::G_CONSTANT) {
      return {{
          [=](MachineInstrBuilder &MIB) { MIB.add(AddrDef->getOperand(1)); },
          [=](MachineInstrBuilder &MIB) { MIB.add(OffDef->getOperand(1)); },
      }};
    }
    return std::nullopt;
  }
  // Pointer-to-pointer in a stack slot: pass the frame index through (exactly as
  // selectLSFrameIndex does) so eliminateFrameIndex + the expander produce the
  // U-relative indirect `ld [n,u]` directly -- no leax/leay to materialise the
  // slot address, and the index register stays free. (The frame index lives in
  // the base operand; PEI folds its resolved offset into the indirect offset.)
  if (AddrDef && AddrDef->getOpcode() == TargetOpcode::G_FRAME_INDEX) {
    return {{
        [=](MachineInstrBuilder &MIB) { MIB.add(AddrDef->getOperand(1)); },
        [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
    }};
  }
  // A global-address pointer would need the extended-indirect `[sym]` form (a
  // separate Sym expansion path); leave it for the non-indirect _Sym handling.
  if (AddrDef && AddrDef->getOpcode() == TargetOpcode::G_GLOBAL_VALUE)
    return std::nullopt;
  // Plain register-based pointer: ld [,idx].
  return {{
      [=](MachineInstrBuilder &MIB) { MIB.add(InnerLoad->getOperand(1)); },
      [=](MachineInstrBuilder &MIB) { MIB.addImm(0); },
  }};
}

static bool shouldFoldMemAccess(const MachineInstr &Dst, const MachineInstr &Src, AAResults *AA) {
  // assert(Src.mayLoadOrStore());

  // For now, don't attempt to fold across basic block boundaries.
  if (Dst.getParent() != Src.getParent())
    return false;

  //if ((*Src.memoperands_begin())->isVolatile())
  //  return false;

  // Look for intervening instructions that cannot be folded across.
  for (const MachineInstr &I : make_range(std::next(MachineBasicBlock::const_iterator(Src)), MachineBasicBlock::const_iterator(Dst))) {
    if (I.isCall() || I.hasUnmodeledSideEffects())
      return false;
    if (I.mayLoadOrStore()) {
      if (Src.hasOrderedMemoryRef() || I.hasOrderedMemoryRef())
        return false;
      if (I.mayAlias(AA, Src, /*UseTBAA=*/true))
        return false;
      // Note: Dst may be a store, indicating that the whole sequence is a RMW
      // operation.
      if (I.mayAlias(AA, Dst, /*UseTBAA=*/true))
        return false;
    }
  }

  return true;
}

/// Try to extract a byte constant from a G_UNMERGE_VALUES of a
/// G_CONSTANT.  Returns true and fills `Val` when the pattern matches.
///
///   %wide = G_CONSTANT i16 <const>
///   %lo, %hi = G_UNMERGE_VALUES %wide
///
/// For big-endian MC6809: part[0] (lo) is `const & 0xFF`,
/// part[1] (hi) is `(const >> 8) & 0xFF`.
static bool getUnmergedByteConstant(const MachineRegisterInfo &MRI,
                                    Register Reg, int64_t &Val) {
  const MachineInstr *Unmerge =
      getOpcodeDef(MC6809::G_UNMERGE_VALUES, Reg, MRI);
  if (!Unmerge)
    return false;
  // The source of the unmerge is the last operand.
  Register SrcReg = Unmerge->getOperand(Unmerge->getNumOperands() - 1).getReg();
  auto SrcConst = getIConstantVRegValWithLookThrough(SrcReg, MRI);
  if (!SrcConst)
    return false;
  int64_t WideVal = SrcConst->Value.getSExtValue();
  bool IsLow = (Reg == Unmerge->getOperand(0).getReg());
  Val = IsLow ? (WideVal & 0xFF) : ((WideVal >> 8) & 0xFF);
  return true;
}

struct FoldedLdIdx_match {
  const MachineInstr &Tgt;
  MachineOperand &Ptr;
  MachineOperand &Offset;
  AAResults *AA;

  bool match(const MachineRegisterInfo &MRI, Register Reg) {
    const MachineInstr *Unmerge;
    const MachineInstr *Load;
    const MachineInstr *FrameIndex;
    Unmerge = getOpcodeDef(MC6809::G_UNMERGE_VALUES, Reg, MRI);
    if (Unmerge) {
      Load = getOpcodeDef(MC6809::G_LOAD, Unmerge->getOperand(2).getReg(), MRI);
      if (Load) {
        FrameIndex = getOpcodeDef(MC6809::G_FRAME_INDEX, Load->getOperand(1).getReg(), MRI);
        if (FrameIndex) {
          Ptr = FrameIndex->getOperand(1);
          const LLT Ty = MRI.getType(Unmerge->getOperand(2).getReg());
          const unsigned TySize = Ty.getSizeInBits();
          // MC6809 is big-endian: part[0] is the LOW half (LSB),
          // which lives at the HIGHER offset in memory.
          bool IsLow = (Reg == Unmerge->getOperand(0).getReg());
          if (TySize == 32) {
            Offset = MachineOperand::CreateImm(IsLow ? 2 : 0);
          } else if (TySize == 16) {
            Offset = MachineOperand::CreateImm(IsLow ? 1 : 0);
          } else
            llvm_unreachable("Impossible unmerge size");
          return true;
        }
      }
    }
    Load = getOpcodeDef(MC6809::G_LOAD, Reg, MRI);
    if (Load) {
      FrameIndex = getOpcodeDef(MC6809::G_FRAME_INDEX, Load->getOperand(1).getReg(), MRI);
      if (FrameIndex) {
        if (!shouldFoldMemAccess(Tgt, *FrameIndex, AA))
          return false;
        Ptr = FrameIndex->getOperand(1);
        Offset = MachineOperand::CreateImm(0);
        return true;
      }
    }
    return false;
  }
};

inline FoldedLdIdx_match m_FoldedLdIdx(const MachineInstr &Tgt, MachineOperand &Ptr, MachineOperand &Offset, AAResults *AA) {
  return {Tgt, Ptr, Offset, AA};
}

// Returns the widest register class that can contain values of a given type.
// Used to ensure that every virtual register gets some register class by the
// time register allocation completes.
static const TargetRegisterClass &getRegClassForType(LLT Ty) {
  if (Ty == LLT::pointer(0, 16)) {
    return MC6809::INDEX16RegClass;
  } else {
    switch (Ty.getSizeInBits()) {
    default:
      llvm_unreachable("Invalid type size.");
    case 1:
      // i1 values share the byte pool with i8; the byte's LSB carries
      // the boolean.
      return MC6809::ACC8RegClass;
    case 8:
      return MC6809::ACC8RegClass;
    case 16:
      return MC6809::ACC16RegClass;
    case 32:
      return MC6809::ACC32RegClass;
    }
  }
}

// Version that checks register bank for s16 values — returns INDEX16
// for values in the index bank (e.g., args/returns passed in X).
static const TargetRegisterClass &getRegClassForTypeOnBank(
    LLT Ty, const RegisterBank *RB) {
  if (Ty.getSizeInBits() == 16 && RB &&
      RB->getID() == MC6809::INDEXRegBankID) {
    return MC6809::INDEX16RegClass;
  }
  return getRegClassForType(Ty);
}
// Fuse `%adv = G_PTR_ADD %base, ±N` with an adjacent same-pointer
// `Load/Store_i*_Mem(p, 0)` of access-size N into an auto-increment / -decrement
// post-modify pseudo: *p++ (PostInc: access *base, base += N) and *--p (PreDec:
// base -= N, access *base). Runs before selectImpl, so the sibling access has
// already been selected (it is later in program order, hence visited first by
// the bottom-up selector) and can be safely erased.
bool MC6809InstructionSelector::tryFusePostModify(MachineInstr &MI) const {
  struct PM {
    unsigned Mem, PostInc, PreDec;
    bool IsLoad;
    unsigned Size;
  };
  static const PM Table[] = {
      {MC6809::Load_i8_Mem, MC6809::Load_i8_PostInc, MC6809::Load_i8_PreDec, true, 1},
      {MC6809::Load_i16_Mem, MC6809::Load_i16_PostInc, MC6809::Load_i16_PreDec, true, 2},
      {MC6809::Load_iPtr_Mem, MC6809::Load_iPtr_PostInc, MC6809::Load_iPtr_PreDec, true, 2},
      {MC6809::Store_i8_Mem, MC6809::Store_i8_PostInc, MC6809::Store_i8_PreDec, false, 1},
      {MC6809::Store_i16_Mem, MC6809::Store_i16_PostInc, MC6809::Store_i16_PreDec, false, 2},
      {MC6809::Store_iPtr_Mem, MC6809::Store_iPtr_PostInc, MC6809::Store_iPtr_PreDec, false, 2},
  };

  Register Dst = MI.getOperand(0).getReg();
  Register Base = MI.getOperand(1).getReg();
  auto COff = getIConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), *MRI);
  if (!COff)
    return false;
  int64_t Off = COff->Value.getSExtValue();
  bool IsInc = Off > 0;
  unsigned Size = std::abs((int)Off);
  if (Size != 1 && Size != 2)
    return false;

  // Both forms address Base: *p++ accesses Base at offset 0 and advances +size;
  // *--p accesses Base at offset -size and advances -size (the loop-strength
  // reducer lowers *--p to Load(p, -size) + p -= size, not Load(p-size, 0)).
  int64_t AccessOff = IsInc ? 0 : Off;
  auto Match = [&](MachineInstr *A) -> const PM * {
    if (!A)
      return nullptr;
    for (const PM &E : Table) {
      if (A->getOpcode() != E.Mem)
        continue;
      if (E.Size != Size)
        return nullptr;
      if (!A->getOperand(1).isReg() || A->getOperand(1).getReg() != Base)
        return nullptr;
      const MachineOperand &OffMO = A->getOperand(2);
      int64_t OffVal;
      if (OffMO.isImm())
        OffVal = OffMO.getImm();
      else if (OffMO.isCImm())
        OffVal = OffMO.getCImm()->getSExtValue();
      else
        return nullptr;
      if (OffVal != AccessOff)
        return nullptr;
      return &E;
    }
    return nullptr;
  };

  // Find the fusible access by use list (adjacency is unnecessary: the access
  // stays put and only the pure-arithmetic advance is folded into it). Base's
  // only non-debug users must be MI and that access.
  MachineInstr *Access = nullptr;
  for (MachineInstr &U : MRI->use_nodbg_instructions(Base)) {
    if (&U == &MI)
      continue;
    if (Access)
      return false;
    Access = &U;
  }
  const PM *E = Match(Access);
  if (!E)
    return false;

  // The fused op redefines the advanced pointer Dst at the access site, so no
  // non-debug instruction between MI and the access may use Dst — otherwise that
  // use is no longer dominated (e.g. the -O0 va_list pattern stores the advanced
  // pointer back before the load). Debug uses are fine.
  if (Access->getParent() != MI.getParent())
    return false;
  MachineBasicBlock *BB = MI.getParent();
  bool AccessAfterMI = false;
  for (auto It = std::next(MachineBasicBlock::iterator(MI)); It != BB->end(); ++It)
    if (&*It == Access) {
      AccessAfterMI = true;
      break;
    }
  auto UsesDstBetween = [&](MachineBasicBlock::iterator From,
                            MachineBasicBlock::iterator To) {
    for (auto It = std::next(From); It != BB->end() && It != To; ++It) {
      if (It->isDebugInstr())
        continue;
      for (const MachineOperand &MO : It->operands())
        if (MO.isReg() && MO.getReg() == Dst)
          return true;
    }
    return false;
  };
  if (AccessAfterMI ? UsesDstBetween(MI.getIterator(), Access->getIterator())
                    : UsesDstBetween(Access->getIterator(), MI.getIterator()))
    return false;

  unsigned FusedOpc = IsInc ? E->PostInc : E->PreDec;
  MachineBasicBlock &MBBlk = *Access->getParent();
  const DebugLoc &DL = Access->getDebugLoc();
  Register Val = Access->getOperand(0).getReg();
  MachineInstrBuilder MIB;
  if (E->IsLoad)
    MIB = BuildMI(MBBlk, *Access, DL, TII.get(FusedOpc))
              .addDef(Val)
              .addDef(Dst)   // ptr_out (tied to ptr_in)
              .addReg(Base); // ptr_in
  else
    MIB = BuildMI(MBBlk, *Access, DL, TII.get(FusedOpc))
              .addDef(Dst)   // ptr_out (tied to ptr_in)
              .addReg(Val)   // stored value
              .addReg(Base); // ptr_in
  for (MachineMemOperand *MMO : Access->memoperands())
    MIB.addMemOperand(MMO);
  MRI->setRegClass(Base, &MC6809::INDEX16RegClass);
  MRI->setRegClass(Dst, &MC6809::INDEX16RegClass);
  Access->eraseFromParent();
  MI.eraseFromParent();
  constrainSelectedInstRegOperands(*MIB, TII, TRI, RBI);
  return true;
}

bool MC6809InstructionSelector::select(MachineInstr &MI) {
  assert(MI.getParent() && "Instruction should be in a basic block!");
  assert(MI.getParent()->getParent() && "Instruction should be in a function!");

  MBB = MI.getParent();
  MF = MBB->getParent();
  MRI = &MF->getRegInfo();

  // isPreISelOpcode is stolen from llvm-mos. Methinks it means "not a GlobalISel opcode".
  if (!MI.isPreISelOpcode()) {
    // Ensure that target-independent pseudos like COPY have register classes.
    constrainGenericOp(MI);
    return true;
  }

  // Intercept i16 constant shifts before selectImpl (no i16 shift pattern).
  // selectShift16 emits the native carry chain on both CPUs: ASLD/LSRD/ASRD on
  // hd6309, ASLB+ROLA / LSRA+RORB / ASRA+RORB (or the in-memory chain) on 6809.
  switch (MI.getOpcode()) {
  case TargetOpcode::G_SHL:
  case TargetOpcode::G_LSHR:
  case TargetOpcode::G_ASHR:
    if (MRI->getType(MI.getOperand(0).getReg()) == LLT::scalar(16))
      return selectShift16(MI);
    break;
  default:
    break;
  }

  // Bug #208: intercept variable-arity G_MERGE_VALUES → s32 before
  // selectImpl. The TableGen-imported pattern matches `s32 ← MERGE(s8, s8,
  // s8, s8)` and emits a REG_SEQUENCE that places the first two byte
  // operands at AQ's `sub_lo_word`/`sub_hi_word` slots — silently dropping
  // operands 3 and 4. Under HD6309 the legalizer narrows i32 add to a
  // 4-byte UADDO/UADDE chain whose merge takes this shape; the dropped
  // operands trigger DCE of the upstream byte-2/3 adds and the matching
  // post-call sret HIGH-half load, miscompiling every i32 add of a value
  // returned via sret (strtol's `acc * base + digit` accumulator being
  // the case that surfaced this bug).
  if (MI.getOpcode() == TargetOpcode::G_MERGE_VALUES &&
      MRI->getType(MI.getOperand(0).getReg()) == LLT::scalar(32) &&
      MI.getNumOperands() == 5)
    return selectMergeValues(MI);

  // Intercept G_ZEXT s1→s8 when the source is a phantom-carry vreg.
  // G_ZEXT is otherwise pattern-selected and reads the allocated
  // byte-LSB — which for a phantom-carry source is the parent byte's
  // LSB (garbage, because the real value is in CC.C / CC.V).  Route
  // through MaterializeCC_C_to_byte / MaterializeCC_V_to_byte.
  //
  // CRITICAL: insert the materialisation IMMEDIATELY AFTER the
  // phantom producer, NOT at the G_ZEXT use site.  CC.C / CC.V are
  // clobbered by any arithmetic or memory store between producer and
  // use (e.g. STD sets V=0).  Capturing the flag into a byte right
  // after the producer is the only way to plumb the value across
  // intervening ops.  The materialised byte vreg is then live from
  // producer to use, and any CC-clobbering instructions in between
  // don't affect correctness.
  if (MI.getOpcode() == TargetOpcode::G_ZEXT) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(1)) {
      if (auto Flag = getPhantomCarryFlag(SrcReg, *MRI, &CarryFlagOf)) {
        // Locate the producer MI by walking COPY/G_FREEZE (same chain
        // getPhantomCarryFlag walked). Insert the materialisation right
        // after it.
        Register Cur = SrcReg;
        MachineInstr *ProdDef = nullptr;
        while (Cur.isVirtual()) {
          ProdDef = MRI->getVRegDef(Cur);
          if (!ProdDef) break;
          unsigned Op = ProdDef->getOpcode();
          if (Op == TargetOpcode::G_FREEZE || Op == TargetOpcode::COPY) {
            Cur = ProdDef->getOperand(1).getReg();
            continue;
          }
          break;
        }
        if (ProdDef) {
          // Emit MaterializeCC_C_to_byte / MaterializeCC_V_to_byte —
          // the pseudo reads CC.C / CC.V directly via Uses=[C] / [V]
          // (no explicit operand).  hasSideEffects=1 on producer and
          // freeze, plus same-BB placement right after the producer,
          // keeps CC live across the read.
          unsigned Opc = (*Flag == MC6809::C)
                             ? MC6809::MaterializeCC_C_to_byte
                             : MC6809::MaterializeCC_V_to_byte;
          MRI->setRegClass(DstReg, &MC6809::ABcRegClass);
          MachineBasicBlock &ProdMBB = *ProdDef->getParent();
          auto InsertIt = std::next(MachineBasicBlock::iterator(ProdDef));
          MachineIRBuilder B(ProdMBB, InsertIt);
          auto I = B.buildInstr(Opc).addDef(DstReg);
          constrainSelectedInstRegOperands(*I, TII, TRI, RBI);
          MI.eraseFromParent();
          return true;
        }
      }
      // Non-phantom G_ZEXT s1→s8: rewrite as `AND_i8_Imm $dst, $src, 1`
      // tied (dst==src).  Expands post-RA to a single `AND[A|B] #1`,
      // which is the correct i1→i8 zero-extension (mask off all but
      // the LSB).  ABc is the narrowest class that satisfies both
      // AND_i8_Imm and downstream consumers like ZEX16Implicit.
      //
      // Producing `AND[A|B] #1` unconditionally can leave a redundant
      // mask at -O0 when the source was already known-LSB-only (a
      // freshly-emitted G_ICMP result, say).  -O1+ instruction-level
      // optimisers fold it.  AND #1 against an already-masked value
      // is idempotent so chaining never breaks semantics.
      MRI->setRegClass(DstReg, &MC6809::ABcRegClass);
      MRI->setRegClass(SrcReg, &MC6809::ABcRegClass);
      MI.setDesc(TII.get(MC6809::AND_i8_Imm));
      MI.getOperand(1).setIsUse();
      MI.tieOperands(0, 1);
      MI.addOperand(MachineOperand::CreateImm(1));
      return true;
    }
    // G_ZEXT i8->i16: physreg-$ad + COPY (2026-06-28). ZEX16Implicit produces
    // the result in physical AD (CLRA zeros AA, AB=src preserved); the COPY
    // moves it to the dst vreg, coalescing to AD when free (no spill / no a[i]
    // churn) or lowering to one STD when contended. See ZEX16Implicit in
    // MC6809InstrLogical.td for why this beats the old vreg-dst + implicit-AA
    // form (self-interference -> $spill_d0 -> MaterializeSpills churn).
    if (DstTy == LLT::scalar(16) && SrcTy == LLT::scalar(8)) {
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MRI->setRegClass(DstReg, &MC6809::ADcRegClass);
      MachineIRBuilder B(MI);
      B.buildCopy(Register(MC6809::AB), SrcReg);
      B.buildInstr(MC6809::ZEX16Implicit);
      B.buildCopy(DstReg, Register(MC6809::AD));
      MI.eraseFromParent();
      return true;
    }
  }

  // Bug #311 Phase 1 step 1.2 (2026-05-20): intercept G_SEXT s1→s8.
  // The TableGen pattern for SEX8Implicit was retired because TableGen
  // rejects `(i1 ACC8:$src)` when both i1 and i8 live on the same
  // class.  Emit SEX8Implicit directly here.  Source class is ACC8
  // (LSB carries the i1); destination class is ACC8 too.
  if (MI.getOpcode() == TargetOpcode::G_SEXT) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(1)) {
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      MachineIRBuilder B(MI);
      auto I = B.buildInstr(MC6809::SEX8Implicit)
                   .addDef(DstReg)
                   .addUse(SrcReg);
      constrainSelectedInstRegOperands(*I, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
    // G_SEXT i8->i16: physreg-$ad + COPY (2026-06-28), mirroring the G_ZEXT
    // i8->i16 case above. SEX16Implicit sign-extends AB into AA, producing AD;
    // the COPY moves it to the dst vreg (coalesces to AD when free).
    if (DstTy == LLT::scalar(16) && SrcTy == LLT::scalar(8)) {
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MRI->setRegClass(DstReg, &MC6809::ADcRegClass);
      MachineIRBuilder B(MI);
      B.buildCopy(Register(MC6809::AB), SrcReg);
      B.buildInstr(MC6809::SEX16Implicit);
      B.buildCopy(DstReg, Register(MC6809::AD));
      MI.eraseFromParent();
      return true;
    }
  }

  // Intercept G_ICMP with an s1 result.  The SetIfCondPat / TestPat
  // SDAG patterns match `(i8 (anyext (i1 (setcc ...))))` and rely on
  // an enclosing G_ANYEXT; at -O0 GISel emits a bare G_ICMP s1 with
  // no anyext wrap, leaving the pattern unfired.
  //
  // Manual lowering mirrors the SetIfCondPat shape:
  //   Compare_iN_{Imm,Reg}  → CC
  //   ConditionalImm(cc, CC, 1, 0) → ACC8 dst (LSB carries the bool).
  if (MI.getOpcode() == TargetOpcode::G_ICMP) {
    Register DstReg = MI.getOperand(0).getReg();
    LLT DstTy = MRI->getType(DstReg);
    if (DstTy == LLT::scalar(1)) {
      auto PredToCC = [](CmpInst::Predicate Pred) -> std::optional<unsigned> {
        switch (Pred) {
        case CmpInst::ICMP_EQ:  return MC6809CC::EQ;
        case CmpInst::ICMP_NE:  return MC6809CC::NE;
        case CmpInst::ICMP_UGT: return MC6809CC::HI;
        case CmpInst::ICMP_UGE: return MC6809CC::HS;
        case CmpInst::ICMP_ULT: return MC6809CC::LO;
        case CmpInst::ICMP_ULE: return MC6809CC::LS;
        case CmpInst::ICMP_SGT: return MC6809CC::GT;
        case CmpInst::ICMP_SGE: return MC6809CC::GE;
        case CmpInst::ICMP_SLT: return MC6809CC::LT;
        case CmpInst::ICMP_SLE: return MC6809CC::LE;
        default: return std::nullopt;
        }
      };
      auto Pred = (CmpInst::Predicate)MI.getOperand(1).getPredicate();
      auto CCOpt = PredToCC(Pred);
      if (CCOpt) {
        Register LhsReg = MI.getOperand(2).getReg();
        Register RhsReg = MI.getOperand(3).getReg();
        LLT OpTy = MRI->getType(LhsReg);
        // Pick width-specific Compare opcodes.  Detect a G_CONSTANT
        // RHS so we emit Compare_iN_Imm (CMPB/D #imm) instead of
        // Compare_iN_Reg (which would PSHS+CMPS on plain 6809).
        unsigned CmpRegOpc = 0, CmpImmOpc = 0;
        const TargetRegisterClass *LhsRC = nullptr;
        if (OpTy == LLT::scalar(8)) {
          CmpRegOpc = MC6809::Compare_i8_Reg;
          CmpImmOpc = MC6809::Compare_i8_Imm;
          LhsRC = &MC6809::ACC8RegClass;
        } else if (OpTy == LLT::scalar(16)) {
          CmpRegOpc = MC6809::Compare_i16_Reg;
          CmpImmOpc = MC6809::Compare_i16_Imm;
          LhsRC = &MC6809::ACC16RegClass;
        }
        if (CmpRegOpc) {
          std::optional<int64_t> RhsImm;
          if (auto *RhsDef = MRI->getVRegDef(RhsReg)) {
            if (RhsDef->getOpcode() == TargetOpcode::G_CONSTANT)
              RhsImm = RhsDef->getOperand(1).getCImm()->getSExtValue();
          }
          MachineIRBuilder B(MI);
          if (!MRI->getRegClassOrNull(LhsReg))
            MRI->setRegClass(LhsReg, LhsRC);
          Register CCReg = MRI->createVirtualRegister(&MC6809::CCondRegClass);
          MachineInstrBuilder Cmp;
          if (RhsImm) {
            Cmp = B.buildInstr(CmpImmOpc)
                      .addDef(CCReg)
                      .addImm(*CCOpt)
                      .addUse(LhsReg)
                      .addImm(*RhsImm);
          } else {
            if (!MRI->getRegClassOrNull(RhsReg))
              MRI->setRegClass(RhsReg, LhsRC);
            Cmp = B.buildInstr(CmpRegOpc)
                      .addDef(CCReg)
                      .addImm(*CCOpt)
                      .addUse(LhsReg)
                      .addUse(RhsReg);
          }
          constrainSelectedInstRegOperands(*Cmp, TII, TRI, RBI);
          MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
          auto Sel = B.buildInstr(MC6809::ConditionalImm)
                         .addDef(DstReg)
                         .addImm(*CCOpt)
                         .addUse(CCReg)
                         .addImm(1)
                         .addImm(0);
          constrainSelectedInstRegOperands(*Sel, TII, TRI, RBI);
          MI.eraseFromParent();
          return true;
        }
      }
    }
  }

  // Intercept G_ANYEXT s1→s8 before selectImpl — selectImpl creates a COPY
  // that leaves the destination without a register class (dangling MI issue).
  if (MI.getOpcode() == TargetOpcode::G_ANYEXT) {
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(1)) {
      // Phantom-carry source: the s1 carry-out vreg of an
      // AddSetCarry* / SubSetCarry* family pseudo is a SCHEDULING
      // PHANTOM — the real value lives in CC.C (or CC.V for
      // overflow), not in the byte-LSB the regalloc allocated.  A
      // plain COPY here would read whichever byte regalloc happened
      // to pick (unrelated to CC.C).
      //
      // Emit MaterializeCC_C_to_byte / MaterializeCC_V_to_byte —
      // post-RA they expand to
      //   C:  LDB #0 ; ADCB #0     (reads CC.C → 0/1 byte)
      //   V:  TFR CC,B ; LSRB ; ANDB #1
      // and `hasSideEffects=1` on producer + materialise keeps the
      // flag alive across the gap.
      if (auto Flag = getPhantomCarryFlag(SrcReg, *MRI, &CarryFlagOf)) {
        MRI->setRegClass(DstReg, &MC6809::ABcRegClass);
        MachineIRBuilder B(MI);
        unsigned Opc = (*Flag == MC6809::C)
                           ? MC6809::MaterializeCC_C_to_byte
                           : MC6809::MaterializeCC_V_to_byte;
        auto I = B.buildInstr(Opc).addDef(DstReg);
        constrainSelectedInstRegOperands(*I, TII, TRI, RBI);
        MI.eraseFromParent();
        return true;
      }
      // Source is a real-byte i1 (G_ICMP, Load_i1_Imm, Seq/CondSet,
      // …): the byte's LSB already IS the boolean, so anyext s1→s8
      // is a plain COPY.
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MI.setDesc(TII.get(TargetOpcode::COPY));
      return true;
    }
  }

  // Intercept G_CONSTANT s1.  The normal llc pipeline widens
  // G_CONSTANT i1 → G_CONSTANT i8 in a pre-isel combiner, so this
  // shape never reaches the selector via llc.  LLD's LTO link-time
  // codegen pipeline does NOT run that widener — bare G_CONSTANT s1
  // arrives at the selector intact at LTO.  Lower it to Load_i8_Imm
  // with the LSB carrying the boolean (Bug #312 was the LTO link
  // failure that surfaced this gap).
  if (MI.getOpcode() == TargetOpcode::G_CONSTANT) {
    Register DstReg = MI.getOperand(0).getReg();
    if (MRI->getType(DstReg) == LLT::scalar(1)) {
      int64_t Val = MI.getOperand(1).getCImm()->getSExtValue() & 1;
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      MachineIRBuilder B(MI);
      auto I = B.buildInstr(MC6809::Load_i8_Imm)
                   .addDef(DstReg)
                   .addImm(Val);
      constrainSelectedInstRegOperands(*I, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  // Bug #144 follow-on: intercept G_CONSTANT s32 with all-debug uses
  // BEFORE selectImpl matches it to Load_i32_Imm (which produces an
  // ACC32 vreg from the 1-reg {AQ} class — overflows at -Og). The
  // value can be recovered from the original source; FAKE_USE alone
  // doesn't carry information.
  if (MI.getOpcode() == TargetOpcode::G_CONSTANT) {
    Register DstReg = MI.getOperand(0).getReg();
    if (MRI->getType(DstReg) == LLT::scalar(32)) {
      bool AllUsesAreDebug = true;
      for (auto &Use : MRI->use_instructions(DstReg)) {
        if (!Use.isDebugInstr() && Use.getOpcode() != TargetOpcode::FAKE_USE) {
          AllUsesAreDebug = false;
          break;
        }
      }
      if (AllUsesAreDebug) {
        SmallVector<MachineInstr *, 4> Uses;
        for (auto &U : MRI->use_instructions(DstReg))
          Uses.push_back(&U);
        for (MachineInstr *Use : Uses)
          Use->eraseFromParent();
        MI.eraseFromParent();
        return true;
      }
    }
  }

  // Bug #178: select G_LOAD / G_STORE on a p1 (addrspace(1) = direct
  // page) global to the DP-mode opcode (LDAd/STAd/LDDd/STDd) with the
  // global symbol as the addr8 operand. The MC layer emits a single-
  // byte fixup; the linker assigns the global a DP address ($00-$FF).
  // The pattern is:
  //
  //   %addr:_(p1) = G_GLOBAL_VALUE @g
  //   %v:_(s8) = G_LOAD %addr        →     $aa = LDAd @g; %v = COPY $aa
  //   G_STORE %v:_(s8), %addr        →     $aa = COPY %v; STAd @g
  //
  // Width selects opcode and accumulator: s8 → LDAd/STAd/$aa,
  // s16 → LDDd/STDd/$ad. We mutate the G_GLOBAL_VALUE only if it has
  // no other uses (always true for a use-once temporary today; future
  // pointer-arithmetic on DP pointers would need to relax this).
  if (MI.getOpcode() == TargetOpcode::G_LOAD ||
      MI.getOpcode() == TargetOpcode::G_STORE) {
    bool IsLoad = MI.getOpcode() == TargetOpcode::G_LOAD;
    Register AddrReg = MI.getOperand(IsLoad ? 1 : 1).getReg();
    LLT AddrTy = MRI->getType(AddrReg);
    // Fold a G_GLOBAL_VALUE address directly into the load/store as a deferred
    // _Sym pseudo. The value register is left in its broad class so regalloc
    // chooses it; the post-RA expander (expandLoadSym / expandStoreSym) emits
    // the concrete reg-specific opcode AND the addressing mode — direct page /
    // extended / PC-relative — from the global's address space and the
    // relocation model. Keeping register selection post-RA avoids the forced
    // accumulator (and the TFR it costs when the value wants another register)
    // that a concrete LDDe/LDAd here would impose, while still collapsing the
    // address-materialise + indexed-zero pair into one op at selection.
    if (AddrTy.isPointer() &&
        (AddrTy.getAddressSpace() == 1 || AddrTy.getAddressSpace() == 0)) {
      bool IsDP = AddrTy.getAddressSpace() == 1;
      // Walk to the G_GLOBAL_VALUE producer.
      MachineInstr *GV = MRI->getVRegDef(AddrReg);
      while (GV && (GV->getOpcode() == TargetOpcode::COPY ||
                    GV->getOpcode() == TargetOpcode::G_FREEZE)) {
        Register Src = GV->getOperand(1).getReg();
        if (!Src.isVirtual()) break;
        GV = MRI->getVRegDef(Src);
      }
      bool HaveGV = GV && GV->getOpcode() == TargetOpcode::G_GLOBAL_VALUE;
      if (!HaveGV) {
        if (IsDP) {
          // A computed direct-page address — a runtime-indexed __directpage
          // object. Direct mode needs a constant 8-bit operand, so form the
          // object's full 16-bit address (the linker-provided direct-page base
          // __dp_base_addr plus the 8-bit in-page offset) and use ordinary
          // indexed addressing.
          Register ValReg = MI.getOperand(0).getReg();
          LLT ValTy = MRI->getType(ValReg);
          unsigned MemOpc;
          const TargetRegisterClass *ValRC;
          if (ValTy == LLT::scalar(8)) {
            MemOpc = IsLoad ? MC6809::Load_i8_Mem : MC6809::Store_i8_Mem;
            ValRC = &MC6809::ACC8RegClass;
          } else if (ValTy == LLT::scalar(16)) {
            MemOpc = IsLoad ? MC6809::Load_i16_Mem : MC6809::Store_i16_Mem;
            ValRC = &MC6809::ACC16RegClass;
          } else {
            return false; // only byte/word direct-page objects
          }
          MachineBasicBlock &MBB = *MI.getParent();
          const DebugLoc &DL = MI.getDebugLoc();
          // Trace the computed p1 address back to its underlying integer (the
          // 8-bit in-page offset). Sourcing from that leaves the p1
          // G_INTTOPTR/COPY chain dead — 8-bit index-bank pointers have no
          // register class — and the selector drops it as trivially dead.
          Register InPageOff = AddrReg;
          for (;;) {
            MachineInstr *D = MRI->getVRegDef(InPageOff);
            if (!D)
              break;
            if (D->isCopy() && D->getOperand(1).getReg().isVirtual()) {
              InPageOff = D->getOperand(1).getReg();
              continue;
            }
            if (D->getOpcode() == TargetOpcode::G_INTTOPTR) {
              InPageOff = D->getOperand(1).getReg();
              continue;
            }
            break;
          }
          if (MRI->getType(InPageOff) != LLT::scalar(8))
            return false;
          // The 8-bit in-page offset zero-extended via physical AD, then copied
          // out (physreg-$ad ZEX16Implicit form): COPY $ab=off; ZEX16Implicit
          // (CLRA -> $ad = 0:off); Off16 = COPY $ad.
          BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), Register(MC6809::AB))
              .addReg(InPageOff);
          BuildMI(MBB, MI, DL, TII.get(MC6809::ZEX16Implicit));
          Register Off16 = MRI->createVirtualRegister(&MC6809::ACC16RegClass);
          BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), Off16).addReg(MC6809::AD);
          // The direct-page base address into an index register.
          Register Base = MRI->createVirtualRegister(&MC6809::INDEX16RegClass);
          auto BaseMIB = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_iPtr_Imm), Base)
                             .addExternalSymbol("__dp_base_addr");
          // Full address = base + in-page offset (LEAX D,X — no ABX).
          Register Full = MRI->createVirtualRegister(&MC6809::INDEX16RegClass);
          auto LeaMIB = BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Reg16), Full)
                            .addReg(Base)
                            .addReg(Off16);
          // Rewrite the load/store as a zero-offset indexed access.
          MRI->setRegClass(ValReg, ValRC);
          MI.getOperand(1).setReg(Full);
          MI.setDesc(TII.get(MemOpc));
          MI.addOperand(MachineOperand::CreateImm(0));
          constrainSelectedInstRegOperands(*BaseMIB, TII, TRI, RBI);
          constrainSelectedInstRegOperands(*LeaMIB, TII, TRI, RBI);
          constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
          return true;
        }
        // AS==0 without a G_GLOBAL_VALUE producer — fall through to
        // selectImpl which handles indexed addressing via Load_*_Mem.
        goto skip_globalvalue_fold;
      }
      Register ValReg = MI.getOperand(0).getReg();
      LLT ValTy = MRI->getType(ValReg);
      unsigned Opc;
      const TargetRegisterClass *ValRC;
      if (ValTy == LLT::scalar(8)) {
        Opc = IsLoad ? MC6809::Load_i8_Sym : MC6809::Store_i8_Sym;
        ValRC = &MC6809::ACC8RegClass;
      } else if (ValTy == LLT::scalar(16)) {
        Opc = IsLoad ? MC6809::Load_i16_Sym : MC6809::Store_i16_Sym;
        ValRC = &MC6809::ACC16RegClass;
      } else if (ValTy.isPointer() && ValTy.getSizeInBits() == 16) {
        Opc = IsLoad ? MC6809::Load_iPtr_Sym : MC6809::Store_iPtr_Sym;
        ValRC = &MC6809::INDEX16RegClass;
      } else if (ValTy == LLT::scalar(32) && STI.has6309()) {
        Opc = IsLoad ? MC6809::Load_i32_Sym : MC6809::Store_i32_Sym;
        ValRC = &MC6809::AQcRegClass;
      } else {
        if (IsDP)
          return false; // unsupported width (i32 on plain 6809 etc.)
        goto skip_globalvalue_fold;
      }
      // Build the deferred pseudo: value operand + the global symbol. The value
      // reg keeps its broad class (ValRC) so regalloc places it; the addressing
      // mode and concrete opcode are chosen post-RA from the global itself.
      const MachineOperand &GVOp = GV->getOperand(1);
      Register GVDef = GV->getOperand(0).getReg();
      MachineInstrBuilder MIB =
          IsLoad ? BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(Opc), ValReg)
                       .add(GVOp)
                 : BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(Opc))
                       .addReg(ValReg)
                       .add(GVOp);
      if (!MRI->getRegClassOrNull(ValReg))
        MRI->setRegClass(ValReg, ValRC);
      constrainSelectedInstRegOperands(*MIB, TII, TRI, RBI);
      MI.eraseFromParent();
      // Erase the G_GLOBAL_VALUE if it has no remaining uses.
      if (MRI->use_empty(GVDef))
        GV->eraseFromParent();
      return true;
    }
  }
skip_globalvalue_fold:

  // Bug #322 (2026-05-22): intercept G_MUL / G_UMULH / G_SMULH on i8
  // BEFORE selectImpl matches the TableGen MulPat_i8 / MulHUPat_i8 /
  // MulHSPat_i8 patterns.  Those autogen-emit
  // `MERGE_LOHI_i16 ACC8:$b, ACC8:$a` with the inputs in their
  // existing classes — if both inputs happen to be in ABc (a common
  // shape: EXTRACT_LO_i16 + PHI-of-ACC8 / SEX8Implicit outputs),
  // constrainSelectedInstRegOperands can't narrow `$a` from ABc to
  // AAc (disjoint sibling sub-classes) and the verifier rejects.
  //
  // Manifest at -Og-hd6309-mame: test-fread-fwrite fails to BUILD
  // with `Expected a AAc register, but got a ABc register`.  Other
  // tiers don't hit this because the producer-side class assignments
  // come out differently under their pass pipelines.
  //
  // Fix: emit the MERGE+MUL+EXTRACT chain manually with explicit
  // narrowing COPYs.  Bypasses the autogen pattern for the i8
  // multiply shapes; semantically identical otherwise.
  if (MI.getOpcode() == TargetOpcode::G_MUL ||
      MI.getOpcode() == TargetOpcode::G_UMULH ||
      MI.getOpcode() == TargetOpcode::G_SMULH) {
    Register DstReg = MI.getOperand(0).getReg();
    if (MRI->getType(DstReg) == LLT::scalar(8)) {
      Register Lo = MI.getOperand(1).getReg();
      Register Hi = MI.getOperand(2).getReg();
      MachineIRBuilder Builder(MI);
      Lo = narrowToClass(Builder, MRI, Lo, &MC6809::ABcRegClass);
      Hi = narrowToClass(Builder, MRI, Hi, &MC6809::AAcRegClass);
      Register Merged = MRI->createVirtualRegister(&MC6809::ADcRegClass);
      Register Prod = MRI->createVirtualRegister(&MC6809::ADcRegClass);
      auto MergeMI = Builder.buildInstr(MC6809::MERGE_LOHI_i16)
                         .addDef(Merged).addUse(Lo).addUse(Hi);
      auto MulMI = Builder.buildInstr(MC6809::MUL_D)
                       .addDef(Prod).addUse(Merged);
      unsigned ExtractOp = (MI.getOpcode() == TargetOpcode::G_MUL)
                               ? MC6809::EXTRACT_LO_i16
                               : MC6809::EXTRACT_HI_i16;
      auto ExtractMI =
          Builder.buildInstr(ExtractOp).addDef(DstReg).addUse(Prod);
      constrainSelectedInstRegOperands(*MergeMI, TII, TRI, RBI);
      constrainSelectedInstRegOperands(*MulMI, TII, TRI, RBI);
      constrainSelectedInstRegOperands(*ExtractMI, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  // Fuse a pointer advance with an adjacent same-base load/store into an
  // auto-increment / auto-decrement access (*p++ / *--p) before selectImpl
  // turns the G_PTR_ADD into a plain LEA.
  if (MI.getOpcode() == TargetOpcode::G_PTR_ADD && tryFusePostModify(MI))
    return true;

  if (selectImpl(MI, *CoverageInfo))
    return true;

  switch (MI.getOpcode()) {
  default:
    return false;

  case TargetOpcode::G_TRUNC: {
    // Hand-select G_TRUNC when the imported pattern doesn't match (e.g.,
    // when imaginary registers in ACC16 cause a synthesized 'accum' class).
    // trunc i16→i8: emit the EXTRACT_LO_i16 pseudo, which expands post-RA
    // (bug #118 Layer 1, approach b). This avoids the sub_lo_byte COPY
    // edge that the coalescer would tighten into regalloc failures.
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(16)) {
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      MRI->setRegClass(SrcReg, &MC6809::ADcRegClass);
      MI.setDesc(TII.get(MC6809::EXTRACT_LO_i16));
      constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
      return true;
    }
    // Bug #161: trunc i32→i8 — chain via Extract16_i32_lo pseudo
    // (handles both AQ and SPILL_Q sources via its post-RA
    // expansion: TFR W,D for AQ, LDD slot+2,$su for SPILL_Q*N)
    // followed by EXTRACT_LO_i16 on the resulting AD vreg.
    //
    // Bug #302 redesign Phase 2 (2026-05-17): switched from
    // EXTRACT_LO_word_i32 to Extract16_i32_lo (Phase 1's parallel
    // replacement pseudo).  Same operand shape, same post-RA
    // expansion semantics; the rename is preparation for Phase 3+,
    // which drops AQ's sub-register hierarchy and rewrites the
    // expansion to be sub-reg-independent.
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(32)) {
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      MRI->setRegClass(SrcReg, &MC6809::ACC32RegClass);
      MachineIRBuilder Builder(MI);
      Register WordLo = MRI->createVirtualRegister(&MC6809::ADcRegClass);
      auto WordExt = Builder.buildInstr(MC6809::Extract16_i32_lo)
                        .addDef(WordLo)
                        .addUse(SrcReg);
      constrainSelectedInstRegOperands(*WordExt, TII, TRI, RBI);
      auto Ext = Builder.buildInstr(MC6809::EXTRACT_LO_i16)
                     .addDef(DstReg)
                     .addUse(WordLo);
      constrainSelectedInstRegOperands(*Ext, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
    // trunc i8→i1: extract bit 0 with AND #1. We can't use COPY with
    // sub_lsb because the register coalescer eliminates it, losing the
    // bit extraction (e.g., XOR -1 gives 0xFE which tests as non-zero).
    if (DstTy == LLT::scalar(1) && SrcTy == LLT::scalar(8)) {
      // Rewrite: %dst = G_TRUNC %src → %dst = AND_i8_Imm %src(tied), 1
      // The two-address pass will insert a COPY if dst != src.
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MI.setDesc(TII.get(MC6809::AND_i8_Imm));
      MI.getOperand(1).setIsUse();
      MI.tieOperands(0, 1);
      MI.addOperand(MachineOperand::CreateImm(1));
      return true;
    }
    return false;
  }

  case TargetOpcode::G_ANYEXT: {
    // anyext i1→i8: byte-to-byte COPY.  i1 lives in ACC8 with bit 0
    // carrying the value, so the high 7 bits are already the right
    // shape for an anyext (don't-care).
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    LLT DstTy = MRI->getType(DstReg);
    LLT SrcTy = MRI->getType(SrcReg);
    if (DstTy == LLT::scalar(8) && SrcTy == LLT::scalar(1)) {
      MRI->setRegClass(DstReg, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(SrcReg))
        MRI->setRegClass(SrcReg, &MC6809::ACC8RegClass);
      MI.setDesc(TII.get(TargetOpcode::COPY));
      return true;
    }
    return false;
  }

  case TargetOpcode::G_CONSTANT: {
    // Pointer constants (e.g., NULL) aren't covered by imported patterns
    // because they have p0 type, not s16. Hand-select to LDX #imm.
    Register DstReg = MI.getOperand(0).getReg();
    LLT DstTy = MRI->getType(DstReg);
    // Bug #144 follow-on: G_CONSTANT s32 selects to Load_i32_Imm with
    // ACC32 dst (1-reg class {AQ}). At -Og some constants are kept
    // alive only by FAKE_USE for debug observability. Materialising
    // them in AQ overflows the class. If there are no real users
    // (only FAKE_USE / DBG_VALUE), delete the constant and its
    // debug consumers — there is no information loss because the
    // value can be recovered from the original source.
    if (DstTy == LLT::scalar(32)) {
      bool AllUsesAreDebug = true;
      for (auto &Use : MRI->use_instructions(DstReg)) {
        if (!Use.isDebugInstr() && Use.getOpcode() != TargetOpcode::FAKE_USE) {
          AllUsesAreDebug = false;
          break;
        }
      }
      if (AllUsesAreDebug) {
        SmallVector<MachineInstr *, 4> Uses;
        for (auto &U : MRI->use_instructions(DstReg))
          Uses.push_back(&U);
        for (MachineInstr *Use : Uses)
          Use->eraseFromParent();
        MI.eraseFromParent();
        return true;
      }
    }
    if (!DstTy.isPointer())
      return false;
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);
    MI.setDesc(TII.get(MC6809::Load_iPtr_Imm));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_GLOBAL_VALUE: {
    // Load the address of a global into an index register.
    Register DstReg = MI.getOperand(0).getReg();
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);

    // Bug #197: under -fPIC / -fPIE, materialise the address PC-relatively
    // via LEAX label,PCR. The MC layer emits R_MC6809_PCREL_16 against the
    // global at the postbyte's offset slot; the linker resolves it at
    // static-link time so the resulting bytes are correct regardless of
    // the program's load address.
    //
    // Non-PIC keeps the absolute LDX #addr form (R_MC6809_ADDR_16) for
    // smallest code where the binary is hard-coded to a single load
    // address — the pre-#197 default behaviour, byte-identical to before.
    //
    // A writable OS9 global lives in the module's data area, reached SU-relative
    // (SU is OS9's data-base register) with OS9-specific target flags. The
    // deferred PC-relative pseudo below cannot express that, so materialise it
    // directly here: LEAX sym,SU then copy into the destination index register.
    if (MF->getTarget().isPositionIndependent() &&
        MF->getTarget().getTargetTriple().isOSOS9()) {
      MachineOperand &GlobalOp = MI.getOperand(1);
      unsigned OS9Flags = getOS9WritableGlobalTargetFlags(GlobalOp.getGlobal());
      if (OS9Flags != MC6809::MO_NO_FLAGS) {
        MachineOperand OS9GlobalOp = MachineOperand::CreateGA(
            GlobalOp.getGlobal(), GlobalOp.getOffset(), OS9Flags);
        if (!MBB->isLiveIn(MC6809::SU))
          MBB->addLiveIn(MC6809::SU);
        BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(MC6809::LEAXi_o16))
            .add(OS9GlobalOp)
            .addReg(MC6809::SU);
        BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(TargetOpcode::COPY), DstReg)
            .addReg(MC6809::IX);
        MI.eraseFromParent();
        return true;
      }
    }

    // Every remaining case selects a deferred pseudo whose index register the
    // allocator picks: Load_iPtr_Imm (absolute LDX/LDY/LDU #addr) for the static
    // model, Lea_iPtr_Sym (LEA{X,Y,U} sym,pc) for PIC/PIE — including read-only
    // OS9 globals, which are PC-reachable within the module. The address is
    // never pinned to IX, so there is no forced COPY/TFR when it wants IY/IU.
    //
    // A direct-page (addrspace 1) global is the exception: its value is the
    // 8-bit in-page offset, which is fixed at link time and so already
    // position-independent. The PC-relative form would combine the DP-offset
    // relocation with PCR (garbage), so always use the immediate form for it.
    const MachineOperand &GVMO = MI.getOperand(1);
    bool IsDPGlobal = GVMO.isGlobal() &&
                      GVMO.getGlobal()->getAddressSpace() == MC6809::AS_DirectPage;
    MI.setDesc(TII.get((MF->getTarget().isPositionIndependent() && !IsDPGlobal)
                           ? MC6809::Lea_iPtr_Sym
                           : MC6809::Load_iPtr_Imm));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_LOAD: {
    // Pointer (p0) loads aren't covered by imported patterns.
    Register DstReg = MI.getOperand(0).getReg();
    if (!MRI->getType(DstReg).isPointer())
      return false;
    Register AddrReg = MI.getOperand(1).getReg();
    MRI->setRegClass(DstReg, &MC6809::INDEX16RegClass);
    // If the address is a frame index, fold it directly into the load (the
    // scalar load patterns do this via selectAMIndexedImmOffset). Otherwise a
    // separate LEA_Ptr_Imm materialises the frame address pointlessly.
    if (MachineInstr *FI = getOpcodeDef(TargetOpcode::G_FRAME_INDEX, AddrReg, *MRI)) {
      MI.getOperand(1).ChangeToFrameIndex(FI->getOperand(1).getIndex(),
                                          FI->getOperand(1).getOffset());
    } else {
      MRI->setRegClass(AddrReg, &MC6809::INDEX16RegClass);
    }
    MI.setDesc(TII.get(MC6809::Load_iPtr_Mem));
    MI.addOperand(MachineOperand::CreateImm(0));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_STORE: {
    // Pointer (p0) stores aren't covered by imported patterns.
    Register ValReg = MI.getOperand(0).getReg();
    if (!MRI->getType(ValReg).isPointer())
      return false;
    Register AddrReg = MI.getOperand(1).getReg();
    MRI->setRegClass(ValReg, &MC6809::INDEX16RegClass);
    // Fold a frame-index address directly into the store, as for G_LOAD above.
    if (MachineInstr *FI = getOpcodeDef(TargetOpcode::G_FRAME_INDEX, AddrReg, *MRI)) {
      MI.getOperand(1).ChangeToFrameIndex(FI->getOperand(1).getIndex(),
                                          FI->getOperand(1).getOffset());
    } else {
      MRI->setRegClass(AddrReg, &MC6809::INDEX16RegClass);
    }
    MI.setDesc(TII.get(MC6809::Store_iPtr_Mem));
    MI.addOperand(MachineOperand::CreateImm(0));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_BRJT: {
    // Branch via jump table. G_BRJT %table_ptr, %jump-table.N, %index
    // Select to BranchJumpTable pseudo (expanded post-RA to PIC sequence).
    // The index is constrained to ADc (D + SPILL_D variants) — see the
    // BranchJumpTable definition in MC6809InstrLogical.td for why. This
    // was bug #68 (was using ACC16, which let regalloc pick IX/IY and
    // the asm-printer expansion blindly shifted D regardless).
    Register IdxReg = MI.getOperand(2).getReg();
    unsigned JTI = MI.getOperand(1).getIndex();
    MRI->setRegClass(IdxReg, &MC6809::ADcRegClass);
    BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(MC6809::BranchJumpTable))
        .addReg(IdxReg)
        .addJumpTableIndex(JTI);
    MI.eraseFromParent();
    return true;
  }

  case TargetOpcode::G_BRCOND: {
    // G_BRCOND %cond(s1), %bb.target
    // When the condition is an s1 in the accum bank (not FLAGS), the imported
    // patterns can't handle it. Emit a single fused TestBranch_i8_Reg, NOT
    // a separate Test_i8_Reg + ConditionalLongBranchRelative pair — PHI
    // elimination would put COPYs between them, and any COPY that gets
    // spilled (becoming STX/STY/STD) clobbers CC and breaks the branch
    // (was bug #59).
    Register CondReg = MI.getOperand(0).getReg();
    MachineBasicBlock *TargetMBB = MI.getOperand(1).getMBB();

    // BRCOND-on-phantom-carry direct branch.
    //
    // If the s1 condition is the carry-out of an unsigned
    // add/sub-with-carry-out (G_USUBO/G_UADDO/G_USUBE/G_UADDE, either
    // generic or already selected to a SubSetCarry / AddSetCarry
    // pseudo), branch directly on CC.C instead of going through the
    // materialise-CC-to-byte + test-byte path.  The phantom carry's
    // "real" value lives in CC.C; reading the byte the regalloc
    // allocated would yield garbage (the byte may be a sub-reg of
    // some unrelated value).
    //
    // Restrictions:
    //   - The producer must be in the same MBB as the BRCOND, with no
    //     CC-clobbering instructions between (anything that defs CC).
    //   - Unsigned ops only — their s1 corresponds to the C flag
    //     (1 = borrow / carry).  G_SSUBO/G_SADDO use V; not handled
    //     here yet.
    //
    // Producer enumeration is shared with G_ANYEXT s1→s8; see
    // getPhantomCarryFlag at the top of this file.

    // Walk through G_FREEZE / COPY to find the real defining instruction.
    // `freeze i1 %cmp` is commonly inserted by InstCombine between an
    // icmp and a brcond consumer; treating it as a barrier would defeat
    // the optimisation below.
    Register WalkReg = CondReg;
    MachineInstr *CondDef = MRI->getVRegDef(WalkReg);
    while (CondDef && (CondDef->getOpcode() == TargetOpcode::G_FREEZE ||
                       CondDef->getOpcode() == TargetOpcode::COPY)) {
      Register NextReg = CondDef->getOperand(1).getReg();
      if (!NextReg.isVirtual())
        break;
      WalkReg = NextReg;
      CondDef = MRI->getVRegDef(WalkReg);
    }

    // Map a condition code to the set of CC sub-flags it actually reads.
    // The conditional branch only cares about these bits; intervening
    // instructions are free to modify other flag bits without invalidating
    // the optimisation.
    //
    // 6809 condition encoding (per datasheet):
    //   HS/CC = !C       LO/CS = C
    //   HI    = !C & !Z  LS    = C | Z
    //   PL    = !N       MI    = N
    //   VC    = !V       VS    = V
    //   NE    = !Z       EQ    = Z
    //   GE    = !(N^V)   LT    = N^V
    //   GT    = !Z & !(N^V)   LE = Z | (N^V)
    //   RA    = always   RN    = never
    auto FlagsReadBy = [](int64_t cc) {
      SmallVector<MCPhysReg, 3> Out;
      switch (cc) {
      case MC6809CC::CC: case MC6809CC::CS:
        Out.push_back(MC6809::C); break;
      case MC6809CC::HI: case MC6809CC::LS:
        Out.push_back(MC6809::C); Out.push_back(MC6809::Z); break;
      case MC6809CC::EQ: case MC6809CC::NE:
        Out.push_back(MC6809::Z); break;
      case MC6809CC::VC: case MC6809CC::VS:
        Out.push_back(MC6809::V); break;
      case MC6809CC::PL: case MC6809CC::MI:
        Out.push_back(MC6809::N); break;
      case MC6809CC::GE: case MC6809CC::LT:
        Out.push_back(MC6809::N); Out.push_back(MC6809::V); break;
      case MC6809CC::GT: case MC6809CC::LE:
        Out.push_back(MC6809::N); Out.push_back(MC6809::V);
        Out.push_back(MC6809::Z); break;
      default: break; // RA/RN/INVALID — no flags consulted
      }
      return Out;
    };

    // Helper: walk from `From` (exclusive) up to BRCOND (exclusive) and
    // check that no instruction between modifies any of the CC bits the
    // branch will consult. Returns true if safe to use the live CC.
    auto CCBitsSurvive = [&](MachineInstr *From, ArrayRef<MCPhysReg> Bits) {
      for (auto It = std::next(MachineBasicBlock::iterator(From)),
                End = MachineBasicBlock::iterator(&MI);
           It != End; ++It) {
        for (MCPhysReg Bit : Bits)
          if (It->modifiesRegister(Bit, &TRI))
            return false;
      }
      return true;
    };

    // BRCOND-on-setcc direct branch.
    //
    // If the s1 condition is a `ConditionalImm cc, Compare/Test, 1, 0`
    // (the shape every `setcc` pattern emits to materialise an i1
    // result), bypass the diamond-CFG materialisation and branch
    // directly on CC using the same condition code that
    // ConditionalImm was about to test.
    //
    // Without this, the asm pattern is:
    //
    //   cmpd ,s              ; Compare sets CC
    //   <ConditionalImm diamond: lda #0/#1 to materialise the byte>
    //   ldd #trueval         ; setup of any later D-typed value (PHI, etc.)
    //                        ;   — this clobbers A, killing the byte's LSB
    //   tsta                 ; reads A, now garbage
    //   lbne <target>        ; branch never taken
    //
    // The materialised byte lives in an ACC8 reg whose LSB carries
    // the boolean; any intervening i16 def into AD destroys it.
    // Bypassing materialisation entirely sidesteps the whole class
    // of regalloc collisions.
    //
    // Operand layout of ConditionalImm:
    //   op0: ACC8 def (the materialised 0/1 byte)
    //   op1: condcode imm (CC)
    //   op2: CCond use (from Compare/Test)
    //   op3: i1 imm (true value, conventionally 1)
    //   op4: i1 imm (false value, conventionally 0)
    //
    // We only fire when (true,false) == (1,0); other orderings would invert
    // the branch sense and aren't worth the complication right now.
    // Map a CmpInst predicate to the matching MC6809 condition code.
    auto PredToCC = [](CmpInst::Predicate Pred) -> std::optional<unsigned> {
      switch (Pred) {
      case CmpInst::ICMP_EQ:  return MC6809CC::EQ;
      case CmpInst::ICMP_NE:  return MC6809CC::NE;
      case CmpInst::ICMP_UGT: return MC6809CC::HI;
      case CmpInst::ICMP_UGE: return MC6809CC::HS;
      case CmpInst::ICMP_ULT: return MC6809CC::LO;
      case CmpInst::ICMP_ULE: return MC6809CC::LS;
      case CmpInst::ICMP_SGT: return MC6809CC::GT;
      case CmpInst::ICMP_SGE: return MC6809CC::GE;
      case CmpInst::ICMP_SLT: return MC6809CC::LT;
      case CmpInst::ICMP_SLE: return MC6809CC::LE;
      default: return std::nullopt;
      }
    };

    // Bug #114 optimization: if the s1 condition is the result of a same-MBB
    // G_ICMP whose only consumer is this BRCOND, bypass the materialise-then-
    // test path entirely. Emit `Compare_*` (or `Test_*`) then `LBlbc <cc>`
    // directly, picking the cc from the icmp's predicate. The classic
    // diamond-CFG materialisation (lda #0 / lda #1 / tsta / lbne) is broken
    // because any intervening i16 def into AD clobbers the bool LSB sitting
    // inside AALSB or ABLSB — see #114 in the bug tracker for the full
    // analysis and a 3-line repro.
    //
    // Conditions to fire:
    //   - CondDef is G_ICMP in the same MBB as the BRCOND.
    //   - The G_ICMP's s1 result has exactly one non-debug use (the path
    //     from icmp to brcond may go through G_FREEZE or COPY, which we
    //     already walked through above to find CondDef).
    //   - The predicate is one we can map to an MC6809 CC.
    //   - Operand types are scalar i8 or i16 (other widths legalize away).
    //   - Operands are register/register; complex addressing modes fall
    //     through to the existing path.
    //   - Operand banks are ACCUM (so ACC8/ACC16 register classes apply;
    //     INDEX/STACK banks need the ptr/STACK16 variants and aren't
    //     covered yet — those would be needed for pointer comparisons,
    //     which currently route through a different ICMP form).
    // Bug #156: walk the FREEZE/COPY chain from CondReg back to the G_ICMP
    // and require EVERY vreg along the way to have exactly one non-debug
    // use. The original `hasOneNonDBGUse(CondDef def)` check missed the
    // case where the chain head (e.g. G_ICMP %12) has one user (a FREEZE)
    // but the FREEZE result %13 has multiple users (e.g. cross-BB BRCONDs).
    // Per-iteration use counts are racy because earlier fusions in the
    // same selection batch delete the chain head; counting the WHOLE chain
    // up front is the robust check. Multi-consumer chains fall through to
    // the materialise+TestBranch_i8_Reg path.
    auto ChainAllSingleUse = [&]() {
      Register R = CondReg;
      while (true) {
        if (!MRI->hasOneNonDBGUse(R))
          return false;
        MachineInstr *D = MRI->getVRegDef(R);
        if (!D)
          return false;
        if (D == CondDef)
          return true;
        if (D->getOpcode() != TargetOpcode::G_FREEZE &&
            D->getOpcode() != TargetOpcode::COPY)
          return false;
        Register Next = D->getOperand(1).getReg();
        if (!Next.isVirtual())
          return false;
        R = Next;
      }
    };
    // Helper to detect a constant RHS (generic G_CONSTANT or already-
    // selected Load_i{8,16,Ptr}_Imm), walking through COPY/G_FREEZE.
    auto FindImmRhs = [&](Register R) -> std::optional<int64_t> {
      Register Cur = R;
      while (true) {
        if (auto C = getIConstantVRegValWithLookThrough(Cur, *MRI))
          return C->Value.getSExtValue();
        MachineInstr *D = MRI->getVRegDef(Cur);
        if (!D) return std::nullopt;
        unsigned Op = D->getOpcode();
        if (Op == TargetOpcode::COPY || Op == TargetOpcode::G_FREEZE) {
          Register N = D->getOperand(1).getReg();
          if (!N.isVirtual()) return std::nullopt;
          Cur = N;
          continue;
        }
        if (Op == MC6809::Load_i8_Imm || Op == MC6809::Load_i16_Imm ||
            Op == MC6809::Load_iPtr_Imm)
          return D->getOperand(1).getImm();
        return std::nullopt;
      }
    };

    // Immediate-RHS compare→branch fusion, multi-use-tolerant and
    // cross-MBB-tolerant.
    //
    // When G_BRCOND's CondDef is a G_ICMP whose RHS is a constant, we
    // can emit CompareBranch_i{8,16}_Imm EVEN IF:
    //   - the G_ICMP has other users (select, etc.) — duplicating an
    //     immediate compare costs a few bytes and no CPU overhead, and
    //     the other users will get their own comparison from the
    //     unselected G_ICMP,
    //   - the G_ICMP is in a predecessor MBB — we emit a FRESH compare
    //     at the BRCOND site, so CC surviving from CondDef's MBB is
    //     irrelevant. The G_ICMP's LHS must still be live into MBB
    //     which it is by construction (or the BRCOND couldn't consume
    //     the G_ICMP's result).
    //
    // Both relaxations are load-bearing for bug #158's post-libcall
    // compare: the G_ICMP ends up one MBB back and has two consumers
    // (select for flags masking, and our BRCOND).
    if (CondDef && CondDef->getOpcode() == TargetOpcode::G_ICMP) {
      auto Pred = (CmpInst::Predicate)CondDef->getOperand(1).getPredicate();
      Register Lhs = CondDef->getOperand(2).getReg();
      Register Rhs = CondDef->getOperand(3).getReg();
      LLT Ty = MRI->getType(Lhs);
      auto CCOpt = PredToCC(Pred);
      const RegisterBank *LBank = RBI.getRegBank(Lhs, *MRI, TRI);
      bool LhsAccum = LBank &&
                      LBank->getID() == MC6809::ACCUMRegBankID;
      unsigned CmpBrImmOpc = 0;
      const TargetRegisterClass *RC = nullptr;
      if (LhsAccum && CCOpt) {
        if (Ty == LLT::scalar(8)) {
          CmpBrImmOpc = MC6809::CompareBranch_i8_Imm;
          RC = &MC6809::ACC8RegClass;
        } else if (Ty == LLT::scalar(16)) {
          CmpBrImmOpc = MC6809::CompareBranch_i16_Imm;
          RC = &MC6809::ACC16RegClass;
        }
      }
      if (CmpBrImmOpc && CCOpt) {
        if (auto Imm = FindImmRhs(Rhs);
            Imm && RBI.constrainGenericRegister(Lhs, *RC, *MRI)) {
          BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(CmpBrImmOpc))
              .addImm(*CCOpt)
              .addReg(Lhs)
              .addImm(*Imm)
              .addMBB(TargetMBB);
          // Erase the G_ICMP only if it has no remaining users after
          // we've taken ours (the BRCOND below). Otherwise leave it for
          // the select / other consumer to emit their own comparison.
          Register ICmpDst = CondDef->getOperand(0).getReg();
          MI.eraseFromParent();
          if (MRI->use_nodbg_empty(ICmpDst))
            CondDef->eraseFromParent();
          return true;
        }
      }
    }

    if (CondDef && CondDef->getOpcode() == TargetOpcode::G_ICMP &&
        CondDef->getParent() == MBB &&
        ChainAllSingleUse()) {
      auto Pred = (CmpInst::Predicate)CondDef->getOperand(1).getPredicate();
      Register Lhs = CondDef->getOperand(2).getReg();
      Register Rhs = CondDef->getOperand(3).getReg();
      LLT Ty = MRI->getType(Lhs);
      auto CCOpt = PredToCC(Pred);
      const RegisterBank *LBank = RBI.getRegBank(Lhs, *MRI, TRI);
      const RegisterBank *RBank = RBI.getRegBank(Rhs, *MRI, TRI);
      bool BothAccum = LBank && RBank &&
                       LBank->getID() == MC6809::ACCUMRegBankID &&
                       RBank->getID() == MC6809::ACCUMRegBankID;
      unsigned PushOpc = 0;
      unsigned CmpBrPullOpc = 0;
      const TargetRegisterClass *RC = nullptr;
      if (BothAccum && CCOpt) {
        if (Ty == LLT::scalar(8)) {
          PushOpc = MC6809::Push_i8;
          CmpBrPullOpc = MC6809::CompareBranch_i8_Pull;
          RC = &MC6809::ACC8RegClass;
        } else if (Ty == LLT::scalar(16)) {
          PushOpc = MC6809::Push_i16;
          CmpBrPullOpc = MC6809::CompareBranch_i16_Pull;
          RC = &MC6809::ACC16RegClass;
        }
      }
      if (CmpBrPullOpc && CCOpt) {
        // Constrain operand classes for the CompareBranch pseudo. G_ICMP
        // operands are bank-selected but may not yet have a register class
        // pinned, so we use constrainGenericRegister.
        if (RBI.constrainGenericRegister(Lhs, *RC, *MRI) &&
            RBI.constrainGenericRegister(Rhs, *RC, *MRI)) {
          // Follow the same shape the tablegen `_Push_Pull` pattern uses
          // for non-HD6309: push RHS, then CompareBranch_*_Pull reads it
          // off the stack (the `Reg` variant would expand to CMPR which
          // is HD6309-only).
          Register Pushed =
              MRI->createVirtualRegister(&MC6809::STACK16RegClass);
          BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(PushOpc), Pushed)
              .addReg(Rhs);
          // CompareBranch_*_Pull operand layout: cc, src, idx(STACK16), tgt.
          BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(CmpBrPullOpc))
              .addImm(*CCOpt)
              .addReg(Lhs)
              .addReg(Pushed)
              .addMBB(TargetMBB);
          // Remove G_ICMP first (its only use was this BRCOND) and then BRCOND.
          CondDef->eraseFromParent();
          MI.eraseFromParent();
          return true;
        }
      }
    }

    if (CondDef && CondDef->getOpcode() == MC6809::ConditionalImm &&
        CondDef->getParent() == MBB) {
      int64_t CC = CondDef->getOperand(1).getImm();
      int64_t TrueVal = CondDef->getOperand(3).getImm();
      int64_t FalseVal = CondDef->getOperand(4).getImm();
      if (TrueVal == 1 && FalseVal == 0 &&
          CCBitsSurvive(CondDef, FlagsReadBy(CC))) {
        // Bug #158 re-fusion: when the ConditionalImm's CCond source is
        // a Compare_i{8,16}_Imm in the same MBB, re-fuse into a
        // CompareBranch_i{8,16}_Imm that post-RA expands to an adjacent
        // CMP+LBcc pair. Without this, regalloc can slot CC-clobbering
        // spills between the Compare and the LBlbc emitted below,
        // silently taking the wrong branch.
        //
        // CCBitsSurvive already checks CC-clobbering instructions
        // CURRENTLY between CondDef and the BRCOND — but regalloc
        // inserts STY/LDY spills AFTER selection, which CCBitsSurvive
        // can't foresee. Fusion eliminates the gap by construction.
        Register CCondReg = CondDef->getOperand(2).getReg();
        MachineInstr *CmpDef = MRI->getVRegDef(CCondReg);
        if (CmpDef && CmpDef->getParent() == MBB) {
          unsigned CmpOpc = CmpDef->getOpcode();
          unsigned FusedOpc = 0;
          if (CmpOpc == MC6809::Compare_i8_Imm)
            FusedOpc = MC6809::CompareBranch_i8_Imm;
          else if (CmpOpc == MC6809::Compare_i16_Imm)
            FusedOpc = MC6809::CompareBranch_i16_Imm;
          if (FusedOpc) {
            // Compare_i*_Imm operand layout: (cc_out CCond, cc imm,
            // src reg, imm). CompareBranch_i*_Imm layout: (cc imm,
            // src reg, imm, branch target).
            int64_t CmpCC = CmpDef->getOperand(1).getImm();
            Register Src = CmpDef->getOperand(2).getReg();
            int64_t Imm = CmpDef->getOperand(3).getImm();
            // Re-fusion picks the ConditionalImm's CC (i.e. what the
            // branch actually consumes), not the Compare's CC — they're
            // usually the same but not guaranteed identical.
            (void)CmpCC;
            BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(FusedOpc))
                .addImm(CC)
                .addReg(Src)
                .addImm(Imm)
                .addMBB(TargetMBB);
            MI.eraseFromParent();
            // Leave Compare + ConditionalImm for DCE — they feed the
            // original i1 materialisation for any OTHER user (e.g. a
            // select). If this BRCOND was the sole consumer, they'll
            // get DCE'd later.
            return true;
          }
        }
        // Bug #206 + #271 cat-1: pick the verifier-friendliest LBlbc
        // variant. _NoC for cc that doesn't read C; _OnlyC for cc that
        // reads only C (HS/CS); canonical LBlbc for cc that reads
        // multiple flags (HI/LS/GE/LT/GT/LE).
        unsigned LBlbcOpc;
        if (MC6809CC::doesNotReadCarry(CC))
          LBlbcOpc = MC6809::LBlbc_NoC;
        else if (MC6809CC::doesOnlyReadCarry(CC))
          LBlbcOpc = MC6809::LBlbc_OnlyC;
        else
          LBlbcOpc = MC6809::LBlbc;
        BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(LBlbcOpc))
            .addImm(CC)
            .addMBB(TargetMBB);
        MI.eraseFromParent();
        // Leave the now-dead ConditionalImm for DCE; it has no other users
        // because we just consumed its only consumer (the BRCOND), but other
        // passes may yet hold references.
        return true;
      }
    }

    // Dispatch into one of three CC-flag-direct branch shapes:
    //   - carry-phantom producer (G_U*ADD/SUB*O/E or AddSetCarry/...) → BCS
    //   - overflow-phantom producer (G_S*ADD/SUB*O/E or AddSetOverflow/...) → BVS
    //   - neither → fall through to TestBranch_i8_Reg
    // Bug #115 introduced the carry path; bug #147 added the overflow
    // path so __builtin_add_overflow + if() works for signed inputs.
    // Bug #152 phase 2 refactor: use getPhantomCarryFlag for the
    // producer classification. The same-MBB + CCBitsSurvive checks
    // still live here (they're consumer-site-specific; the helper is
    // purely "identify the flag a given vreg represents").
    enum { PhantomNone, PhantomCarry, PhantomOverflow } Phantom = PhantomNone;
    if (CondDef && CondDef->getParent() == MBB &&
        CondDef->getNumOperands() >= 2 &&
        CondDef->getOperand(1).isReg() &&
        CondDef->getOperand(1).getReg() == CondReg) {
      if (auto Flag = getPhantomCarryFlag(CondReg, *MRI, &CarryFlagOf)) {
        MCPhysReg CCBit = *Flag;  // MC6809::C or MC6809::V
        if (CCBitsSurvive(CondDef, {CCBit}))
          Phantom = (CCBit == MC6809::C) ? PhantomCarry : PhantomOverflow;
      }
    }

    if (Phantom != PhantomNone) {
      unsigned CC = (Phantom == PhantomCarry) ? MC6809CC::CS : MC6809CC::VS;
      // Bug #206 + #271 cat-1: PhantomCarry → CS, which reads ONLY C
      // (LBlbc_OnlyC); PhantomOverflow → VS, which reads ONLY V and
      // doesn't read C (LBlbc_NoC).
      unsigned LBlbcOpc = (Phantom == PhantomCarry) ?
          MC6809::LBlbc_OnlyC : MC6809::LBlbc_NoC;
      BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(LBlbcOpc))
          .addImm(CC)
          .addMBB(TargetMBB);
      MI.eraseFromParent();
      return true;
    }

    // TestBranch_i8_Reg's source is constrained to ACC8.  Also set
    // ACC8 on every vreg in the FREEZE/COPY chain back to the
    // producer — post-selection passes may eliminate the intermediate
    // COPYs and rewrite uses of CondReg back to an earlier vreg;
    // those earlier vregs must already be ACC8 or regalloc's
    // SplitEditor trips a class-constraint mismatch.
    MRI->setRegClass(CondReg, &MC6809::ACC8RegClass);
    {
      Register R = CondReg;
      while (true) {
        MachineInstr *Def = MRI->getVRegDef(R);
        if (!Def || (Def->getOpcode() != TargetOpcode::G_FREEZE &&
                     Def->getOpcode() != TargetOpcode::COPY))
          break;
        Register Src = Def->getOperand(1).getReg();
        if (!Src.isVirtual())
          break;
        MRI->setRegClass(Src, &MC6809::ACC8RegClass);
        R = Src;
      }
    }
    // Branch if non-zero (NE).
    BuildMI(*MBB, MI, MI.getDebugLoc(), TII.get(MC6809::TestBranch_i8_Reg))
        .addImm(MC6809CC::NE)
        .addReg(CondReg)
        .addMBB(TargetMBB);
    MI.eraseFromParent();
    return true;
  }

  case TargetOpcode::G_FRAME_INDEX:
    return selectFrameIndex(MI);
  case TargetOpcode::G_MERGE_VALUES:
    return selectMergeValues(MI);
  case TargetOpcode::G_UNMERGE_VALUES:
    return selectUnMergeValues(MI);

  case TargetOpcode::G_IMPLICIT_DEF:
  case TargetOpcode::G_PHI:
    return selectGeneric(MI);

  case TargetOpcode::G_FREEZE: {
    // G_FREEZE is a no-op (marks value as non-poison). Lower to COPY.
    // Bug #161 round 6: prefer the tighter of (dst class, src class) so
    // we don't widen ADc → ACC16 just because the other side hadn't been
    // selected yet (bottom-up isel order). The widening cascades through
    // the COPY and lands as `%X:acc16 = COPY %Y:adc` which subsequent
    // EXTRACT_LO_i16 / EXTRACT_HI_i16 (ADc-only inputs) reject as a
    // verifier-class mismatch and downstream as PostRA scheduler "Count
    // == 0" assertions.
    Register DstReg = MI.getOperand(0).getReg();
    Register SrcReg = MI.getOperand(1).getReg();
    const TargetRegisterClass *DstRC = MRI->getRegClassOrNull(DstReg);
    const TargetRegisterClass *SrcRC = MRI->getRegClassOrNull(SrcReg);
    const TargetRegisterClass *RC = nullptr;
    if (DstRC && SrcRC)
      RC = TRI.getCommonSubClass(DstRC, SrcRC);
    if (!RC)
      RC = SrcRC ? SrcRC : DstRC;
    if (!RC) {
      LLT Ty = MRI->getType(SrcReg);
      if (Ty == LLT::scalar(1))
        // i1 shares the byte pool with i8; the byte's LSB carries
        // the boolean.
        RC = &MC6809::ACC8RegClass;
      else if (Ty == LLT::scalar(8))
        RC = &MC6809::ACC8RegClass;
      else if (Ty == LLT::scalar(16))
        RC = &MC6809::ACC16RegClass;
      else if (Ty == LLT::pointer(0, 16))
        // A pointer lives in the index bank -- NOT ACC16. Lumping it with
        // scalar(16) forced the value into the accumulator bank, which then
        // needed an index->acc->index round-trip back to the index domain
        // (e.g. for an index-domain pointer compare or a pointer PHI). That
        // bank-cross copy survives RA and post-RA copy-opt mis-propagated it
        // across a kill, producing an undefined-$ix use (Bug #380).
        RC = &MC6809::INDEX16RegClass;
      else if (Ty == LLT::scalar(32))
        RC = &MC6809::ACC32RegClass;
    }
    if (RC) {
      MRI->setRegClass(DstReg, RC);
      MRI->setRegClass(SrcReg, RC);
    }
    MI.setDesc(TII.get(TargetOpcode::COPY));
    constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
    return true;
  }

  case TargetOpcode::G_ADD:
  case TargetOpcode::G_SUB: {
    // Handle INDEX-bank i16 add/sub via LEA.
    // GlobalISel puts constants in registers (not bare immediates),
    // so TableGen patterns can't match — hand-lowering required.
    // Check DstReg for INDEX bank OR INDEX16 class (intermediate results
    // in multi-add chains may already have a class from earlier selection).
    Register DstReg = MI.getOperand(0).getReg();
    const RegisterBank *RB = MRI->getRegBankOrNull(DstReg);
    bool IsIndex = (RB && RB->getID() == MC6809::INDEXRegBankID);
    if (!IsIndex) {
      const TargetRegisterClass *RC = MRI->getRegClassOrNull(DstReg);
      IsIndex = RC && MC6809::INDEX16RegClass.hasSubClassEq(RC);
    }
    if (IsIndex) {
      Register Src1 = MI.getOperand(1).getReg();
      Register Src2 = MI.getOperand(2).getReg();
      // Look through COPYs to find the defining G_CONSTANT.
      Register Src2Origin = Src2;
      MachineInstr *Src2Def = MRI->getVRegDef(Src2Origin);
      while (Src2Def && Src2Def->getOpcode() == TargetOpcode::COPY) {
        Src2Origin = Src2Def->getOperand(1).getReg();
        if (!Src2Origin.isVirtual()) break;
        Src2Def = MRI->getVRegDef(Src2Origin);
      }
      if (Src2Def && Src2Def->getOpcode() == TargetOpcode::G_CONSTANT) {
        int64_t Offset;
        if (Src2Def->getOperand(1).isCImm())
          Offset = Src2Def->getOperand(1).getCImm()->getSExtValue();
        else
          Offset = Src2Def->getOperand(1).getImm();
        if (MI.getOpcode() == TargetOpcode::G_SUB)
          Offset = -Offset;
        auto LEA = BuildMI(*MI.getParent(), MI, MI.getDebugLoc(),
                           TII.get(MC6809::LEAPtrAdd_Imm), DstReg)
                       .addReg(Src1)
                       .addImm(Offset);
        constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        MI.eraseFromParent();
        return true;
      }

      // INDEX + INDEX (non-constant): route through ACCUM (D).
      // The 6809 has no "add two index registers" instruction.
      // Rewrite: COPY src1→D, COPY src2→D2, Push D2, Add/Sub_i16_Pull, COPY D→result.
      {
        MachineBasicBlock &MBB = *MI.getParent();
        DebugLoc DL = MI.getDebugLoc();
        bool IsSub = (MI.getOpcode() == TargetOpcode::G_SUB);

        LLT s16 = LLT::scalar(16);

        // Constrain the original INDEX-bank operands to INDEX16 register class.
        RBI.constrainGenericRegister(Src1, MC6809::INDEX16RegClass, *MRI);
        RBI.constrainGenericRegister(Src2, MC6809::INDEX16RegClass, *MRI);
        RBI.constrainGenericRegister(DstReg, MC6809::INDEX16RegClass, *MRI);

        if (!IsSub) {
          // ADD: copy one operand to D, then LEAX D,X (single instruction).
          Register AccSrc = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccSrc, &MC6809::ACC16RegClass);
          BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), AccSrc).addReg(Src2);
          auto LEA = BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Reg16), DstReg)
              .addReg(Src1)
              .addReg(AccSrc);
          constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        } else {
          // SUB: compute -b as (0 - b), then LEAX D,X.
          // Trace Src2 back to its defining load to get the memory operand,
          // so we can SUBD directly from memory: LDD #0; SUBD offset,S; LEAX D,X.
          Register AccZero = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccZero, &MC6809::ACC16RegClass);
          Register AccNegB = MRI->createGenericVirtualRegister(s16);
          MRI->setRegClass(AccNegB, &MC6809::ACC16RegClass);

          // Find the memory source of Src2 by looking through COPYs.
          Register Src2Origin = Src2;
          MachineInstr *Src2Load = MRI->getVRegDef(Src2Origin);
          while (Src2Load && Src2Load->getOpcode() == TargetOpcode::COPY) {
            Src2Origin = Src2Load->getOperand(1).getReg();
            if (!Src2Origin.isVirtual()) break;
            Src2Load = MRI->getVRegDef(Src2Origin);
          }

          if (Src2Load && Src2Load->getOpcode() == MC6809::Load_i16_Mem) {
            // D = 0, then SUBD directly from memory.
            auto Zero = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Imm), AccZero)
                .addImm(0);
            constrainSelectedInstRegOperands(*Zero, TII, TRI, RBI);
            auto SubMem = BuildMI(MBB, MI, DL, TII.get(MC6809::Sub_i16_Mem), AccNegB)
                .addReg(AccZero)
                .add(Src2Load->getOperand(1))  // index register
                .add(Src2Load->getOperand(2)); // offset
            constrainSelectedInstRegOperands(*SubMem, TII, TRI, RBI);
          } else {
            // Push b FIRST (before loading 0 into D), then subtract.
            Register AccSrc2 = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(AccSrc2, &MC6809::ACC16RegClass);
            Register StackReg = MRI->createGenericVirtualRegister(s16);
            MRI->setRegClass(StackReg, &MC6809::STACK16RegClass);
            BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), AccSrc2).addReg(Src2);
            auto Push = BuildMI(MBB, MI, DL, TII.get(MC6809::Push_i16), StackReg)
                .addReg(AccSrc2);
            constrainSelectedInstRegOperands(*Push, TII, TRI, RBI);
            // NOW load 0 (after b is safely on the stack).
            auto Zero = BuildMI(MBB, MI, DL, TII.get(MC6809::Load_i16_Imm), AccZero)
                .addImm(0);
            constrainSelectedInstRegOperands(*Zero, TII, TRI, RBI);
            auto Sub = BuildMI(MBB, MI, DL, TII.get(MC6809::Sub_i16_Pull), AccNegB)
                .addReg(AccZero)
                .addReg(StackReg);
            constrainSelectedInstRegOperands(*Sub, TII, TRI, RBI);
          }

          // LEA: result = src1 + (-b).
          auto LEA = BuildMI(MBB, MI, DL, TII.get(MC6809::LEAPtrAdd_Reg16), DstReg)
              .addReg(Src1)
              .addReg(AccNegB);
          constrainSelectedInstRegOperands(*LEA, TII, TRI, RBI);
        }
        MI.eraseFromParent();
        return true;
      }
    }
    return false;  // Fall through to selectImpl for ACCUM-bank (ADDD/SUBD).
  }

  // G_ICMP: INDEX-bank CMPX/CMPY selection deferred — requires post-RA
  // peephole to replace TFR X,D + CMPD with CMPX. The compare opcode
  // maps (CompareImmediateOpcode) are ready with IX/IY/SU/SS entries.

  case TargetOpcode::G_SADDO:
  case TargetOpcode::G_UADDO:
    return selectAddO(MI);

  case TargetOpcode::G_USUBO:
  case TargetOpcode::G_SSUBO:
    return selectSubO(MI);

  case TargetOpcode::G_SADDE:
  case TargetOpcode::G_UADDE:
    return selectAddE(MI);

  case TargetOpcode::G_USUBE:
  case TargetOpcode::G_SSUBE:
    return selectSubE(MI);

  // G_MUL/G_UMULH/G_SMULH i8: handled by TableGen patterns via
  // REG_SEQUENCE + MUL_D + EXTRACT_SUBREG (no hand-lowering needed).

  case MC6809::G_SHLE:
  case MC6809::G_LSHRE:
    return selectShiftExtend(MI);

  case TargetOpcode::G_SHL:
  case TargetOpcode::G_LSHR:
  case TargetOpcode::G_ASHR:
    // i16 shifts on 6809: no native instruction. Hand-select to byte pairs
    // (constant: LSL_i16_Reg loop, variable: libcall).
    if (!STI.has6309() && MRI->getType(MI.getOperand(0).getReg()) == LLT::scalar(16))
      return selectShift16(MI);
    break;
  }
  return false;
}

bool MC6809InstructionSelector::selectFrameIndex(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);
  auto Instr = Builder.buildInstr(MC6809::LEA_Ptr_Imm).add(MI.getOperand(0))
                   .add(MI.getOperand(1))
                   .addImm(0);
  constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectMergeValues(MachineInstr &MI) {
  MachineIRBuilder Builder(MI);

  Register Dst = MI.getOperand(0).getReg();
  Register Lo = MI.getOperand(1).getReg();
  Register Hi = MI.getOperand(2).getReg();

  auto LoConst = getIConstantVRegValWithLookThrough(Lo, *MRI);
  auto HiConst = getIConstantVRegValWithLookThrough(Hi, *MRI);
  const unsigned Size = MRI->getType(Dst).getSizeInBits();

  // Bug #144 / #144 follow-on: detect debug-only consumers of wide
  // (s32 or s64) merge results and rewrite them to operate on the
  // input pieces directly, so we never need an ACC32 ({AQ}, 1 reg)
  // or ACC64 (no regclass at all) vreg. Without this, -Og leaves
  // these debug-shaped chains alive and the back-end either
  // overflows AQ (s32 case) or hits "VReg has no regclass after
  // selection" (s64 case in __ultoa_invert et al.).
  bool WideAllDebug = false;
  if (Size == 32 || Size == 64) {
    WideAllDebug = true;
    for (auto &Use : MRI->use_instructions(Dst)) {
      if (!Use.isDebugInstr() && Use.getOpcode() != TargetOpcode::FAKE_USE) {
        WideAllDebug = false;
        break;
      }
    }
  }
  if (WideAllDebug) {
    // Collect all input vregs; G_MERGE_VALUES has variable arity.
    // For s32: 2 × s16 inputs. For s64: 4 × s16 inputs (or 2 × s32
    // post-narrowScalar). Each fragment carries the input's bit
    // width and offset within Dst.
    SmallVector<Register, 4> InRegs;
    SmallVector<unsigned, 4> InWidths;
    unsigned BitOffset = 0;
    for (unsigned I = 1; I < MI.getNumOperands(); ++I) {
      Register R = MI.getOperand(I).getReg();
      InRegs.push_back(R);
      InWidths.push_back(MRI->getType(R).getSizeInBits());
    }
    LLVMContext &Ctx = MF->getFunction().getContext();
    SmallVector<MachineInstr *, 4> Uses;
    for (auto &U : MRI->use_instructions(Dst))
      Uses.push_back(&U);
    for (MachineInstr *Use : Uses) {
      if (Use->getOpcode() == TargetOpcode::FAKE_USE) {
        // FAKE_USE keeps the value live; reference each input piece
        // so none dies prematurely.
        auto NewFU = BuildMI(*Use->getParent(), Use, Use->getDebugLoc(),
                             TII.get(TargetOpcode::FAKE_USE));
        for (Register R : InRegs)
          NewFU.addUse(R);
        Use->eraseFromParent();
        continue;
      }
      assert(Use->isDebugValue());
      const DILocalVariable *Var = Use->getDebugVariable();
      const DebugLoc &DL = Use->getDebugLoc();
      auto MakeFragment = [&](unsigned OffsetBits, unsigned WidthBits) {
        const DIExpression *OldExpr = Use->getDebugExpression();
        SmallVector<uint64_t, 4> Ops;
        if (OldExpr)
          Ops.append(OldExpr->elements_begin(), OldExpr->elements_end());
        return DIExpression::appendOpsToArg(
            DIExpression::get(Ctx, Ops),
            {dwarf::DW_OP_LLVM_fragment, OffsetBits, WidthBits}, 0,
            /*StackValue=*/false);
      };
      BitOffset = 0;
      for (unsigned I = 0; I < InRegs.size(); ++I) {
        BuildMI(*Use->getParent(), Use, DL, TII.get(TargetOpcode::DBG_VALUE))
            .addReg(InRegs[I]).addReg(0).addMetadata(Var)
            .addMetadata(MakeFragment(BitOffset, InWidths[I]));
        BitOffset += InWidths[I];
      }
      Use->eraseFromParent();
    }
    MI.eraseFromParent();
    return true;
  }

  if (LoConst && HiConst) {
    if (Size == 16) {
      uint64_t Val = HiConst->Value.getZExtValue() << 8 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load_i16_Imm).addDef(Dst).addImm(Val);
      Instr->addImplicitDefUseOperands(*MF);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    } else if (Size == 32) {
      uint64_t Val = HiConst->Value.getZExtValue() << 16 | LoConst->Value.getZExtValue();
      auto Instr = Builder.buildInstr(MC6809::Load_i32_Imm).addDef(Dst).addImm(Val);
      Instr->addImplicitDefUseOperands(*MF);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    }
    MI.eraseFromParent();
    return true;
  }
  if (Size == 16) {
    // s8×2 → s16: emit MERGE_LOHI_i16 pseudo, which expands post-RA and
    // avoids sub-register COPY edges between ACC8 and ACC16 in the vreg
    // graph (bug #118 Layer 1, approach b).
    MRI->setRegClass(Dst, &MC6809::ADcRegClass);
    if (!MRI->getRegClassOrNull(Lo))
      MRI->setRegClass(Lo, &MC6809::ACC8RegClass);
    if (!MRI->getRegClassOrNull(Hi))
      MRI->setRegClass(Hi, &MC6809::ACC8RegClass);
    // Bug #319 (2026-05-21, refined 2026-05-22): MERGE_LOHI_i16's $lo
    // input is ABc, $hi is AAc.  When the operand vreg's class is
    // broader (typically ACC8, from upstream BIT1-elim widening of
    // ConditionalImm / SEX8Implicit / ZEX8Implicit), the autogen
    // constrainSelectedInstRegOperands narrow can fail — verifier
    // rejects the ACC8 operand.
    //
    // Stage 1's `hasFakeUse(MI)` gate was too tight: at -Og without
    // -fextend-lifetimes there's no FAKE_USE, yet the bug still
    // manifests (32 residual verifier hits at -Og tiers after Stage
    // 1+2 — confirmed 2026-05-22).  Better gate: emit a narrowing
    // COPY only when the operand has multiple uses.  Multi-use
    // means another consumer also constrains the vreg's class, so
    // constrainSelectedInstRegOperands' best-effort narrow can fail
    // to find a class compatible with ALL uses.  Single-use is
    // narrowable cleanly so the COPY would be redundant (and breaks
    // -O0 lit asm CHECK strings, as observed 2026-05-22).
    //
    // Bug #322 (2026-05-22): also force a COPY when the operand's
    // class is DISJOINT from the target class (e.g. ABc when we
    // need AAc, or vice versa).  `getCommonSubClass` returns null
    // when no register can satisfy both, which means
    // constrainSelectedInstRegOperands has no class to constrain
    // to and will fail.  This is the test-fread-fwrite shape: i1
    // ZEX bytes get assigned to ABc (the natural class for
    // SEX8Implicit / ZEX*Implicit outputs), but both bytes land
    // in ABc for the SAME MERGE_LOHI_i16 — one as $lo (ABc, OK)
    // and one as $hi (needs AAc, mismatch).
    //
    // Earlier attempt to use a broader "always COPY if class
    // mismatch" gate regressed non-Og tiers because it inserted
    // COPYs even for the broader→narrower single-use case where
    // constrain DID work (and the redundant COPY broke -O0 lit
    // CHECK strings).  The `getCommonSubClass` check is the
    // surgical predicate: it ONLY fires when constrain can't
    // succeed, preserving the working single-use narrow path.
    auto narrowIfNeeded = [&](Register R,
                              const TargetRegisterClass *RC) -> Register {
      const TargetRegisterClass *Cur = MRI->getRegClassOrNull(R);
      if (Cur == RC)
        return R;
      if (!MRI->hasOneUse(R))
        return narrowToClass(Builder, MRI, R, RC);
      // Single-use: check if constrain can succeed.
      if (!Cur || TRI.getCommonSubClass(Cur, RC))
        return R;
      // Disjoint classes — constrain can't bridge, must COPY.
      return narrowToClass(Builder, MRI, R, RC);
    };
    Lo = narrowIfNeeded(Lo, &MC6809::ABcRegClass);
    Hi = narrowIfNeeded(Hi, &MC6809::AAcRegClass);
    auto Merge = Builder.buildInstr(MC6809::MERGE_LOHI_i16)
                     .addDef(Dst)
                     .addUse(Lo)
                     .addUse(Hi);
    constrainSelectedInstRegOperands(*Merge, TII, TRI, RBI);
  } else {
    // Real i32 consumers (HD6309 ALU ops, Store_i32, etc.): emit
    // REG_SEQUENCE with word sub-regs on AQ. The debug-only case
    // is handled by the WideAllDebug rewrite above, so by here
    // the consumers are real and need a materialised wide vreg.
    //
    // G_MERGE_VALUES is variable-arity: legalizer narrowing of i32 add
    // chains produces (s32) ← MERGE(s8 b0, s8 b1, s8 b2, s8 b3), while
    // 2-input MERGE(s16 lo, s16 hi) → s32 also occurs naturally.
    // AQ's only sub-regs are sub_lo_word / sub_hi_word (i16), so for
    // 4 × s8 we first pair the bytes into i16s via MERGE_LOHI_i16 and
    // then REG_SEQUENCE the two i16s into AQ. Bug #208: the previous
    // code unconditionally took operands 1..2 as Lo/Hi, silently
    // dropping operands 3..4 of a 4-byte chain — every i32 add
    // including the post-__mulsi3 sret-return + digit accumulation in
    // strtol lost its high 16 bits.
    Register Lo16 = Lo, Hi16 = Hi;
    if (MI.getNumOperands() == 5) {
      Register B0 = MI.getOperand(1).getReg();
      Register B1 = MI.getOperand(2).getReg();
      Register B2 = MI.getOperand(3).getReg();
      Register B3 = MI.getOperand(4).getReg();
      if (!MRI->getRegClassOrNull(B0))
        MRI->setRegClass(B0, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(B1))
        MRI->setRegClass(B1, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(B2))
        MRI->setRegClass(B2, &MC6809::ACC8RegClass);
      if (!MRI->getRegClassOrNull(B3))
        MRI->setRegClass(B3, &MC6809::ACC8RegClass);
      // Bug #319 (2026-05-21) / Bug #322 (2026-05-22): same
      // narrowIfNeeded gate as the s8×2→s16 shape above.  Force a
      // COPY when the operand has multiple uses (constrain can't
      // satisfy all consumers) OR when the operand's class is
      // disjoint from the target class (constrain has no common
      // subclass to fall back to — e.g. ABc when we need AAc).
      auto narrowIfNeeded = [&](Register R,
                                const TargetRegisterClass *RC) -> Register {
        const TargetRegisterClass *Cur = MRI->getRegClassOrNull(R);
        if (Cur == RC)
          return R;
        if (!MRI->hasOneUse(R))
          return narrowToClass(Builder, MRI, R, RC);
        if (!Cur || TRI.getCommonSubClass(Cur, RC))
          return R;
        return narrowToClass(Builder, MRI, R, RC);
      };
      B0 = narrowIfNeeded(B0, &MC6809::ABcRegClass);
      B1 = narrowIfNeeded(B1, &MC6809::AAcRegClass);
      B2 = narrowIfNeeded(B2, &MC6809::ABcRegClass);
      B3 = narrowIfNeeded(B3, &MC6809::AAcRegClass);
      Lo16 = MRI->createVirtualRegister(&MC6809::ADcRegClass);
      Hi16 = MRI->createVirtualRegister(&MC6809::ADcRegClass);
      auto MLo = Builder.buildInstr(MC6809::MERGE_LOHI_i16)
                     .addDef(Lo16).addUse(B0).addUse(B1);
      auto MHi = Builder.buildInstr(MC6809::MERGE_LOHI_i16)
                     .addDef(Hi16).addUse(B2).addUse(B3);
      constrainSelectedInstRegOperands(*MLo, TII, TRI, RBI);
      constrainSelectedInstRegOperands(*MHi, TII, TRI, RBI);
    }
    // Bug #302 redesign Phase 2 (2026-05-17): switched from
    // REG_SEQUENCE-with-sub_lo_word/sub_hi_word destination-assembly
    // to Build32_i16i16 (Phase 1's parallel replacement pseudo).
    // The REG_SEQUENCE pattern was the structural root cause of
    // Bug #302: writes to a vreg's sub-word slots leaked
    // intersection-class demand from downstream consumers into
    // the destination vreg, producing `acc32_with_sub_lsb` =
    // {AQ} only collapses that broke vfprintf's LTO codegen.
    // Build32_i16i16's opaque ACC32 destination has no sub-word
    // constraint, eliminating the leak.
    Register DstAcc32 = Dst;
    MRI->setRegClass(DstAcc32, &MC6809::ACC32RegClass);
    // Bug #319 Stage 2 (2026-05-21): Build32_i16i16 takes ADc:$lo
    // and ADc:$hi.  If either is in the broader ACC16 (the typical
    // producer's output class), FAKE_USE at -Og blocks the
    // constraint-shrink and the verifier complains.
    if (hasFakeUse(MI)) {
      Lo16 = narrowToClass(Builder, MRI, Lo16, &MC6809::ADcRegClass);
      Hi16 = narrowToClass(Builder, MRI, Hi16, &MC6809::ADcRegClass);
    }
    auto Build = Builder.buildInstr(MC6809::Build32_i16i16)
                     .addDef(DstAcc32)
                     .addUse(Lo16)
                     .addUse(Hi16);
    constrainSelectedInstRegOperands(*Build, TII, TRI, RBI);
  }
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectAddO(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  // Bug #147: G_SADDO's s1 result represents the V (signed-overflow)
  // flag, while G_UADDO's represents C (carry). The MC expansion is
  // identical (same ADDA/ADDB/ADDD), but selectBrCond needs to dispatch
  // BVS vs BCS based on which flag the s1 phantom maps to. Carry the
  // signedness in the pseudo opcode by emitting AddSetOverflow_* for
  // signed and AddSetCarry_* for unsigned.
  bool IsSigned = MI.getOpcode() == TargetOpcode::G_SADDO;
  auto PickOpc = [IsSigned](unsigned CarryOpc, unsigned OverflowOpc) {
    return IsSigned ? OverflowOpc : CarryOpc;
  };
  // Bug #311 Phase 2: i32 arm.  HD6309 native ADDW+ADCD chain runs the
  // i32 carry inside the pseudo — the carry-out vreg is still a
  // PHANTOM_CARRY scheduling marker, same as i8/i16.
  auto PickByWidth = [DstSize](unsigned Opc8, unsigned Opc16, unsigned Opc32) {
    switch (DstSize) {
    case 8:  return Opc8;
    case 16: return Opc16;
    case 32: return Opc32;
    default: llvm_unreachable("Unexpected DstSize for AddO/SubO/AddE/SubE");
    }
  };

  // Bug #186 follow-up Phase 1a (2026-04-28): instead of emitting a
  // separate IMPLICIT_DEF placeholder for the IR carry vreg, attach
  // an implicit-def of CarryOut DIRECTLY onto the SetCarry/SetOverflow
  // pseudo (via .addDef(CarryOut, RegState::ImplicitDefine) below).
  // The vreg's def is then literally the producing MI; no MI can
  // slide between the vreg-def and the CC.C/V write because they ARE
  // the same instruction. Combined with honest Defs=[NZ,V,C] on
  // Compare/Test/BitTest (Phase 2), regalloc can track CC.C across
  // intervening clobbers without the bug #184 byte-intermediate
  // workaround. The CarryFlagOf[] cache is still populated for the
  // cross-BB scenario where COPY/G_FREEZE renames the vreg between
  // producer and consumer; getPhantomCarryFlag walks the chain and
  // recognizes the producing pseudo's opcode directly.
  CarryFlagOf[CarryOut] = IsSigned ? MC6809::V : MC6809::C;
  if (!MRI->getRegClassOrNull(CarryOut))
    // Phantom-carry vregs live in PHANTOM_CARRY — a disjoint regalloc
    // pool with synthetic 1-bit physregs (PHANTOM_CARRY_0..7).  Keeping
    // them off the byte pool means they don't compete with real byte
    // results during regalloc (which used to force spills and cost
    // +47-60% codegen size on i32/i64 add chains).  The phantom
    // physreg has no hardware backing; AsmPrinter never emits it
    // (only explicit operands cross into MC).  See
    // MC6809PhantomCarryGuard.cpp for the late-pass safety net.
    MRI->setRegClass(CarryOut, &MC6809::PHANTOM_CARRYRegClass);

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_GCst(ValReg))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarry_i8_Imm,  MC6809::AddSetOverflow_i8_Imm),
        PickOpc(MC6809::AddSetCarry_i16_Imm, MC6809::AddSetOverflow_i16_Imm),
        PickOpc(MC6809::AddSetCarry_i32_Imm, MC6809::AddSetOverflow_i32_Imm));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .addImm(Value)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  // Fold G_UNMERGE_VALUES of G_CONSTANT into an immediate operand.
  // This catches the byte halves of an i16 constant that was split
  // by the legalizer for an i32 carry chain.
  {
    Register UnmReg;
    int64_t ByteVal;
    Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(UnmReg), m_Reg(Reg))) ||
              mi_match(Dst, *MRI, m_GUAddO(m_Reg(UnmReg), m_Reg(Reg)));
    if (Success && getUnmergedByteConstant(*MRI, Reg, ByteVal)) {
      Opcode = PickByWidth(
          PickOpc(MC6809::AddSetCarry_i8_Imm,  MC6809::AddSetOverflow_i8_Imm),
          PickOpc(MC6809::AddSetCarry_i16_Imm, MC6809::AddSetOverflow_i16_Imm),
          PickOpc(MC6809::AddSetCarry_i32_Imm, MC6809::AddSetOverflow_i32_Imm));
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addUse(UnmReg)
                       .addImm(ByteVal)
                       .addDef(CarryOut, RegState::ImplicitDefine);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA))));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarry_i8_Mem,  MC6809::AddSetOverflow_i8_Mem),
        PickOpc(MC6809::AddSetCarry_i16_Mem, MC6809::AddSetOverflow_i16_Mem),
        PickOpc(MC6809::AddSetCarry_i32_Mem, MC6809::AddSetOverflow_i32_Mem));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSAddO(m_Reg(LHS), m_Reg(RHS))) ||
            mi_match(Dst, *MRI, m_GUAddO(m_Reg(LHS), m_Reg(RHS)));
  if (Success) {
    // Bug #319 Stage 2 (2026-05-21): mirror selectAddE — i8 _Reg
    // variant requires ABc operands; FAKE_USE at -Og blocks the
    // automatic narrow.  See file-top helpers.
    if (DstSize == 8 && hasFakeUse(MI)) {
      LHS = narrowToClass(Builder, MRI, LHS, &MC6809::ABcRegClass);
      RHS = narrowToClass(Builder, MRI, RHS, &MC6809::ABcRegClass);
    }
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarry_i8_Reg,  MC6809::AddSetOverflow_i8_Reg),
        PickOpc(MC6809::AddSetCarry_i16_Reg, MC6809::AddSetOverflow_i16_Reg),
        PickOpc(MC6809::AddSetCarry_i32_Reg, MC6809::AddSetOverflow_i32_Reg));
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addUse(LHS)
                .addUse(RHS)
                .addDef(CarryOut, RegState::ImplicitDefine);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  return false;
}

// Select G_USUBO/G_SSUBO (i16 subtract producing carry-out) into one
// of the SubSetCarry_i*_{Imm,Mem,Reg} pseudos.
//
// SUB IS NON-COMMUTATIVE — bug #61 history (now fixed)
// ====================================================
// This function used to match BOTH operand orders when one operand
// was a constant:
//
//   m_GUSubO(m_Reg, m_GCst)   // (reg − const)
//   m_GUSubO(m_GCst, m_Reg)   // (const − reg)   ← danger
//
// On a successful match, both shapes were lowered into
// `SubSetCarry_i*_Imm reg, const`, which means "compute reg − const".
// For the (reg, const) shape that's correct. For the (const, reg)
// shape it SILENTLY SWAPPED THE OPERANDS, computing `reg − const`
// when the IR asked for `const − reg`. The swap was invisible at
// the source level — `0 − x` (an honest negation) was being
// compiled as `x − 0` (a no-op).
//
// The bug was discovered while adding labs to the picolibc test:
// labs(-3) returned -3 instead of 3 because the negation step was
// silently elided.
//
// Fix
// ---
// Don't match the (const, reg) order at all. Let those cases fall
// through to the (reg, reg) match below — the constant becomes a
// vreg via G_CONSTANT and operand order is then preserved by the
// reg-reg form. Add and AddO can still match (const, reg) safely
// because addition IS commutative; only Sub variants need this
// careful treatment.
//
// The fix has a small code-quality cost: previously a (const − x)
// pattern compiled to one instruction (`subd #const`), now it
// compiles to two (`ldd #const; sub_reg_reg`) plus some extra
// stack for the materialized constant. The autogenerated CHECK
// lines in `test/CodeGen/MC6809/sub.ll` and `if.ll` were
// regenerated via `update_llc_test_checks.py` to match.
//
// A future optimization could re-match the (const, reg) shape via
// a Negate post-step (similar to how `MC6809ArithRevPat` handles
// plain Sub) and recover the lost code quality without
// reintroducing the swap bug.
//
// Dependency on bug #63
// ---------------------
// This fix depended on bug #63 being fixed first. Bug #63 was the
// "two distinct ACC spills both materialize through $ad" issue in
// MaterializeSpills. Without #63 fixed, removing the (const, reg)
// matches here would route `0 − x` through the (reg, reg) path,
// which then hits the multi-spill clobber. After #63's fix, the
// (reg, reg) path works correctly.
bool MC6809InstructionSelector::selectSubO(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;

  // Bug #147: see selectAddO above for the rationale of dispatching on
  // signedness. SSUBO's s1 represents V (signed-overflow); USUBO's s1
  // represents C (carry/borrow).
  bool IsSigned = MI.getOpcode() == TargetOpcode::G_SSUBO;
  auto PickOpc = [IsSigned](unsigned CarryOpc, unsigned OverflowOpc) {
    return IsSigned ? OverflowOpc : CarryOpc;
  };
  // Bug #311 Phase 2: i32 arm (HD6309 native SUBW+SBCD).
  auto PickByWidth = [DstSize](unsigned Opc8, unsigned Opc16, unsigned Opc32) {
    switch (DstSize) {
    case 8:  return Opc8;
    case 16: return Opc16;
    case 32: return Opc32;
    default: llvm_unreachable("Unexpected DstSize for AddO/SubO/AddE/SubE");
    }
  };

  // Bug #186 follow-up Phase 1a (2026-04-28): see selectAddO above.
  CarryFlagOf[CarryOut] = IsSigned ? MC6809::V : MC6809::C;
  if (!MRI->getRegClassOrNull(CarryOut))
    // Phantom-carry vregs live in PHANTOM_CARRY — a disjoint regalloc
    // pool with synthetic 1-bit physregs (PHANTOM_CARRY_0..7).  Keeping
    // them off the byte pool means they don't compete with real byte
    // results during regalloc (which used to force spills and cost
    // +47-60% codegen size on i32/i64 add chains).  The phantom
    // physreg has no hardware backing; AsmPrinter never emits it
    // (only explicit operands cross into MC).  See
    // MC6809PhantomCarryGuard.cpp for the late-pass safety net.
    MRI->setRegClass(CarryOut, &MC6809::PHANTOM_CARRYRegClass);

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  // Match `(reg − const)` only — never `(const − reg)`. See the
  // function-level comment for why.
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_GCst(ValReg))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_GCst(ValReg)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarry_i8_Imm,  MC6809::SubSetOverflow_i8_Imm),
        PickOpc(MC6809::SubSetCarry_i16_Imm, MC6809::SubSetOverflow_i16_Imm),
        PickOpc(MC6809::SubSetCarry_i32_Imm, MC6809::SubSetOverflow_i32_Imm));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .addImm(Value)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  // Fold G_UNMERGE_VALUES of G_CONSTANT into an immediate operand.
  // Only match (reg - unmerged_const), never the reverse — SUB is
  // non-commutative (see bug #61 comments above).
  {
    Register UnmReg;
    int64_t ByteVal;
    Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(UnmReg), m_Reg(Reg))) ||
              mi_match(Dst, *MRI, m_GUSubO(m_Reg(UnmReg), m_Reg(Reg)));
    if (Success && getUnmergedByteConstant(*MRI, Reg, ByteVal)) {
      Opcode = PickByWidth(
          PickOpc(MC6809::SubSetCarry_i8_Imm,  MC6809::SubSetOverflow_i8_Imm),
          PickOpc(MC6809::SubSetCarry_i16_Imm, MC6809::SubSetOverflow_i16_Imm),
          PickOpc(MC6809::SubSetCarry_i32_Imm, MC6809::SubSetOverflow_i32_Imm));
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addUse(UnmReg)
                       .addImm(ByteVal)
                       .addDef(CarryOut, RegState::ImplicitDefine);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  // Match `(reg − mem)` only — never `(mem − reg)`. Same
  // non-commutative reasoning as for the (reg, const) form above.
  // The (mem, reg) shape would have to materialize the loaded value
  // into a vreg and route through the (reg, reg) match below.
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA))));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarry_i8_Mem,  MC6809::SubSetOverflow_i8_Mem),
        PickOpc(MC6809::SubSetCarry_i16_Mem, MC6809::SubSetOverflow_i16_Mem),
        PickOpc(MC6809::SubSetCarry_i32_Mem, MC6809::SubSetOverflow_i32_Mem));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine)
                     .cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSSubO(m_Reg(LHS), m_Reg(RHS))) ||
            mi_match(Dst, *MRI, m_GUSubO(m_Reg(LHS), m_Reg(RHS)));
  if (Success) {
    // Bug #319 Stage 2 (2026-05-21): mirror selectAddO — see helpers.
    if (DstSize == 8 && hasFakeUse(MI)) {
      LHS = narrowToClass(Builder, MRI, LHS, &MC6809::ABcRegClass);
      RHS = narrowToClass(Builder, MRI, RHS, &MC6809::ABcRegClass);
    }
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarry_i8_Reg,  MC6809::SubSetOverflow_i8_Reg),
        PickOpc(MC6809::SubSetCarry_i16_Reg, MC6809::SubSetOverflow_i16_Reg),
        PickOpc(MC6809::SubSetCarry_i32_Reg, MC6809::SubSetOverflow_i32_Reg));
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addUse(LHS)
                .addUse(RHS)
                .addDef(CarryOut, RegState::ImplicitDefine);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
  );

  return false;
}

bool MC6809InstructionSelector::selectAddE(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Carry;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  assert((DstSize == 8 || DstSize == 16 || DstSize == 32) &&
         "Only 8 / 16 / 32-bit add-with-carry exists");
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;
  // Bug #311 Phase 2: i32 arm (HD6309 EXG-D,W + ADCB/ADCA + EXG + ADCD).
  auto PickByWidth = [DstSize](unsigned Opc8, unsigned Opc16, unsigned Opc32) {
    switch (DstSize) {
    case 8:  return Opc8;
    case 16: return Opc16;
    case 32: return Opc32;
    default: llvm_unreachable("Unexpected DstSize for AddO/SubO/AddE/SubE");
    }
  };

  // Bug #184 + bug #186 follow-up Phase 2a: cross-BB AND same-BB
  // (with intervening CC.C-clobber) carry-in freezing/restoring.
  // See ensureCarryChainIntegrity comment for the rationale.
  // Bug #307 round 2 (2026-05-18): when the bridge fires, the new
  // pseudo we build below must NOT carry the phantom_carry implicit-
  // use — otherwise regalloc keeps the phantom_carry vreg alive
  // across the bridge and triggers cross-call spill that has no
  // valid lowering.  See ensureCarryChainIntegrity's return-value
  // comment.
  bool Bridged =
      ensureCarryChainIntegrity(MI, MI.getOperand(4).getReg(), *MRI, TII,
                                TRI, RBI, &CarryFlagOf, BridgedByteFor);

  // Bug #147: G_SADDE's s1 result is V (signed-overflow); G_UADDE's is C.
  bool IsSigned = MI.getOpcode() == TargetOpcode::G_SADDE;
  auto PickOpc = [IsSigned](unsigned CarryOpc, unsigned OverflowOpc) {
    return IsSigned ? OverflowOpc : CarryOpc;
  };

  // Bug #186 follow-up Phase 1a (2026-04-28): see selectAddO above.
  CarryFlagOf[CarryOut] = IsSigned ? MC6809::V : MC6809::C;
  if (!MRI->getRegClassOrNull(CarryOut))
    // Phantom-carry vregs live in PHANTOM_CARRY — a disjoint regalloc
    // pool with synthetic 1-bit physregs (PHANTOM_CARRY_0..7).  Keeping
    // them off the byte pool means they don't compete with real byte
    // results during regalloc (which used to force spills and cost
    // +47-60% codegen size on i32/i64 add chains).  The phantom
    // physreg has no hardware backing; AsmPrinter never emits it
    // (only explicit operands cross into MC).  See
    // MC6809PhantomCarryGuard.cpp for the late-pass safety net.
    MRI->setRegClass(CarryOut, &MC6809::PHANTOM_CARRYRegClass);

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    // Constant-carry fold: if the carry-IN is a known constant
    // (still-G_CONSTANT or already-selected Load_i1_Imm), the
    // SetCarryUse pseudo's `Uses = [C]` is a runtime hole — no
    // producer of CC.C upstream, so the read is verifier-undefined
    // AND functionally wrong (ADCB consumes hardware CC.C, which
    // has no connection to the constant's value).  Fold the
    // constant carry into the immediate operand and emit the
    // AddSetCarry sibling (no `Use`).  The carry-OUT vreg is still
    // produced for downstream chain users.
    int64_t CarryVal = 0;
    bool CarryIsConst = false;
    if (auto CC = getIConstantVRegSExtVal(Carry, *MRI)) {
      CarryVal = *CC & 1;
      CarryIsConst = true;
    } else if (MachineInstr *CarryDef = MRI->getVRegDef(Carry)) {
      if (CarryDef->getOpcode() == MC6809::Load_i1_Imm &&
          CarryDef->getOperand(1).isImm()) {
        CarryVal = CarryDef->getOperand(1).getImm() & 1;
        CarryIsConst = true;
      }
    }
    if (CarryIsConst) {
      Opcode = PickByWidth(
          PickOpc(MC6809::AddSetCarry_i8_Imm, MC6809::AddSetOverflow_i8_Imm),
          PickOpc(MC6809::AddSetCarry_i16_Imm, MC6809::AddSetOverflow_i16_Imm),
          PickOpc(MC6809::AddSetCarry_i32_Imm, MC6809::AddSetOverflow_i32_Imm));
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addUse(Reg)
                       .addImm(Value + CarryVal)
                       .addDef(CarryOut, RegState::ImplicitDefine);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarryUse_i8_Imm,  MC6809::AddSetOverflowUse_i8_Imm),
        PickOpc(MC6809::AddSetCarryUse_i16_Imm, MC6809::AddSetOverflowUse_i16_Imm),
        PickOpc(MC6809::AddSetCarryUse_i32_Imm, MC6809::AddSetOverflowUse_i32_Imm));
    // Keep the carry-IN vreg as an implicit-use so DCE can't remove
    // the upstream SetCarry when its non-carry byte result is dead
    // (e.g. multi-byte multiply chains where only the high half is
    // kept).  Without this, the implicit $c use here dangles (no
    // upstream define) and the verifier rejects the MIR.
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .addImm(Value)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  // Fold G_UNMERGE_VALUES of G_CONSTANT into an immediate operand.
  {
    Register UnmReg;
    int64_t ByteVal;
    Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(UnmReg), m_Reg(Reg), m_Reg(Carry))) ||
              mi_match(Dst, *MRI, m_GUAddE(m_Reg(UnmReg), m_Reg(Reg), m_Reg(Carry)));
    if (Success && getUnmergedByteConstant(*MRI, Reg, ByteVal)) {
      Opcode = PickByWidth(
          PickOpc(MC6809::AddSetCarryUse_i8_Imm,  MC6809::AddSetOverflowUse_i8_Imm),
          PickOpc(MC6809::AddSetCarryUse_i16_Imm, MC6809::AddSetOverflowUse_i16_Imm),
          PickOpc(MC6809::AddSetCarryUse_i32_Imm, MC6809::AddSetOverflowUse_i32_Imm));
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addUse(UnmReg)
                       .addImm(ByteVal)
                       .addDef(CarryOut, RegState::ImplicitDefine);
      if (!Bridged)
        Instr.addUse(Carry, RegState::Implicit);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarryUse_i8_Mem,  MC6809::AddSetOverflowUse_i8_Mem),
        PickOpc(MC6809::AddSetCarryUse_i16_Mem, MC6809::AddSetOverflowUse_i16_Mem),
        PickOpc(MC6809::AddSetCarryUse_i32_Mem, MC6809::AddSetOverflowUse_i32_Mem));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    Instr.cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarryUse_i8_Mem,  MC6809::AddSetOverflowUse_i8_Mem),
        PickOpc(MC6809::AddSetCarryUse_i16_Mem, MC6809::AddSetOverflowUse_i16_Mem),
        PickOpc(MC6809::AddSetCarryUse_i32_Mem, MC6809::AddSetOverflowUse_i32_Mem));
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    Instr.cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUAddE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry)));
  if (Success) {
    // Bug #319 (2026-05-21): i8 _Reg variant requires ABc operands.
    // Producer may be in ACC8 (broader); at -Og with -fextend-
    // lifetimes the FAKE_USE blocks the narrow, tripping the
    // verifier.  See file-top hasFakeUse / narrowToClass helpers.
    if (DstSize == 8 && hasFakeUse(MI)) {
      LHS = narrowToClass(Builder, MRI, LHS, &MC6809::ABcRegClass);
      RHS = narrowToClass(Builder, MRI, RHS, &MC6809::ABcRegClass);
    }
    Opcode = PickByWidth(
        PickOpc(MC6809::AddSetCarryUse_i8_Reg,  MC6809::AddSetOverflowUse_i8_Reg),
        PickOpc(MC6809::AddSetCarryUse_i16_Reg, MC6809::AddSetOverflowUse_i16_Reg),
        PickOpc(MC6809::AddSetCarryUse_i32_Reg, MC6809::AddSetOverflowUse_i32_Reg));
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addUse(LHS)
                .addUse(RHS)
                .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
      );

  return false;
}

// Select G_USUBE/G_SSUBE (i16 subtract-with-borrow, the upper-half
// continuation of an i32 sub) into one of the SubSetCarryUse_i*_*
// pseudos. Same shape as selectSubO above, but with an extra
// `carry_in` input that comes from the lo half's carry-out.
//
// Bug #61 fix applies here too — see selectSubO for the full
// historical write-up. Only the (reg, const) and (reg, mem) shapes
// are matched; (const, reg) and (mem, reg) fall through to the
// (reg, reg) match below.
bool MC6809InstructionSelector::selectSubE(MachineInstr &MI) {

  MachineIRBuilder Builder(MI);
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut, Carry;
  LLT DstTy = MRI->getType(Dst);
  const auto DstSize = DstTy.getSizeInBits();
  assert((DstSize == 8 || DstSize == 16 || DstSize == 32) &&
         "Only 8 / 16 / 32-bit sub-with-borrow exists");
  bool Success;
  MachineInstrBuilder Instr;
  MachineInstr *Load;
  Register Reg;
  unsigned Opcode = 0;
  // Bug #311 Phase 2: i32 arm.
  auto PickByWidth = [DstSize](unsigned Opc8, unsigned Opc16, unsigned Opc32) {
    switch (DstSize) {
    case 8:  return Opc8;
    case 16: return Opc16;
    case 32: return Opc32;
    default: llvm_unreachable("Unexpected DstSize for AddO/SubO/AddE/SubE");
    }
  };

  CarryOut = MI.getOperand(1).getReg();

  // Bug #184 + bug #186 follow-up Phase 2a: cross-BB AND same-BB
  // (with intervening CC.C-clobber) carry-in freezing/restoring.
  // See ensureCarryChainIntegrity comment for the rationale.
  // Bug #307 round 2 (2026-05-18): when the bridge fires, the new
  // pseudo we build below must NOT carry the phantom_carry implicit-
  // use — same reasoning as selectAddE.
  bool Bridged =
      ensureCarryChainIntegrity(MI, MI.getOperand(4).getReg(), *MRI, TII,
                                TRI, RBI, &CarryFlagOf, BridgedByteFor);

  // Bug #147: G_SSUBE's s1 is V (signed-overflow); G_USUBE's is C.
  bool IsSigned = MI.getOpcode() == TargetOpcode::G_SSUBE;
  auto PickOpc = [IsSigned](unsigned CarryOpc, unsigned OverflowOpc) {
    return IsSigned ? OverflowOpc : CarryOpc;
  };

  // Bug #186 follow-up Phase 1a (2026-04-28): see selectAddO above.
  CarryFlagOf[CarryOut] = IsSigned ? MC6809::V : MC6809::C;
  if (!MRI->getRegClassOrNull(CarryOut))
    // Phantom-carry vregs live in PHANTOM_CARRY — a disjoint regalloc
    // pool with synthetic 1-bit physregs (PHANTOM_CARRY_0..7).  Keeping
    // them off the byte pool means they don't compete with real byte
    // results during regalloc (which used to force spills and cost
    // +47-60% codegen size on i32/i64 add chains).  The phantom
    // physreg has no hardware backing; AsmPrinter never emits it
    // (only explicit operands cross into MC).  See
    // MC6809PhantomCarryGuard.cpp for the late-pass safety net.
    MRI->setRegClass(CarryOut, &MC6809::PHANTOM_CARRYRegClass);

  std::optional<ValueAndVReg> ValReg;
  int64_t Value;
  // Match `(reg − const, carry_in)` only — never `(const − reg, ...)`.
  // See selectSubO for the full bug #61 history.
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_GCst(ValReg), m_Reg(Carry)));
  if (Success) {
    Value = ValReg->Value.getSExtValue();
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarryUse_i8_Imm,  MC6809::SubSetOverflowUse_i8_Imm),
        PickOpc(MC6809::SubSetCarryUse_i16_Imm, MC6809::SubSetOverflowUse_i16_Imm),
        PickOpc(MC6809::SubSetCarryUse_i32_Imm, MC6809::SubSetOverflowUse_i32_Imm));
    // Keep the carry-IN vreg as an implicit-use
    // (see selectAddE for the full rationale).
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .addImm(Value)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  // Fold G_UNMERGE_VALUES of G_CONSTANT into an immediate operand.
  // Only match (reg - unmerged_const, carry_in) — same non-commutative
  // reasoning as selectSubO.
  {
    Register UnmReg;
    int64_t ByteVal;
    Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(UnmReg), m_Reg(Reg), m_Reg(Carry))) ||
              mi_match(Dst, *MRI, m_GUSubE(m_Reg(UnmReg), m_Reg(Reg), m_Reg(Carry)));
    if (Success && getUnmergedByteConstant(*MRI, Reg, ByteVal)) {
      Opcode = PickByWidth(
          PickOpc(MC6809::SubSetCarryUse_i8_Imm,  MC6809::SubSetOverflowUse_i8_Imm),
          PickOpc(MC6809::SubSetCarryUse_i16_Imm, MC6809::SubSetOverflowUse_i16_Imm),
          PickOpc(MC6809::SubSetCarryUse_i32_Imm, MC6809::SubSetOverflowUse_i32_Imm));
      Instr = Builder.buildInstr(Opcode)
                       .addDef(Dst)
                       .addUse(UnmReg)
                       .addImm(ByteVal)
                       .addDef(CarryOut, RegState::ImplicitDefine);
      if (!Bridged)
        Instr.addUse(Carry, RegState::Implicit);
      constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
      MI.eraseFromParent();
      return true;
    }
  }

  MachineOperand Ptr = MachineOperand::CreateReg(0, false);
  MachineOperand Offset = MachineOperand::CreateReg(0, false);
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarryUse_i8_Mem,  MC6809::SubSetOverflowUse_i8_Mem),
        PickOpc(MC6809::SubSetCarryUse_i16_Mem, MC6809::SubSetOverflowUse_i16_Mem),
        PickOpc(MC6809::SubSetCarryUse_i32_Mem, MC6809::SubSetOverflowUse_i32_Mem));
    // Carry-IN implicit-use (see selectAddE).
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    Instr.cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GSSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(Reg), m_all_of(m_MInstr(Load), m_FoldedLdIdx(MI, Ptr, Offset, AA)), m_Reg(Carry)));
  if (Success) {
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarryUse_i8_Mem,  MC6809::SubSetOverflowUse_i8_Mem),
        PickOpc(MC6809::SubSetCarryUse_i16_Mem, MC6809::SubSetOverflowUse_i16_Mem),
        PickOpc(MC6809::SubSetCarryUse_i32_Mem, MC6809::SubSetOverflowUse_i32_Mem));
    // Carry-IN implicit-use (see selectAddE).
    Instr = Builder.buildInstr(Opcode)
                     .addDef(Dst)
                     .addUse(Reg)
                     .add(Ptr)
                     .add(Offset)
                     .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    Instr.cloneMemRefs(*Ptr.getParent());
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  Register LHS, RHS;
  Success = mi_match(Dst, *MRI, m_GSSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry))) ||
            mi_match(Dst, *MRI, m_GUSubE(m_Reg(LHS), m_Reg(RHS), m_Reg(Carry)));
  if (Success) {
    // Bug #319 (2026-05-21): mirror selectAddE — see helper docs.
    if (DstSize == 8 && hasFakeUse(MI)) {
      LHS = narrowToClass(Builder, MRI, LHS, &MC6809::ABcRegClass);
      RHS = narrowToClass(Builder, MRI, RHS, &MC6809::ABcRegClass);
    }
    Opcode = PickByWidth(
        PickOpc(MC6809::SubSetCarryUse_i8_Reg,  MC6809::SubSetOverflowUse_i8_Reg),
        PickOpc(MC6809::SubSetCarryUse_i16_Reg, MC6809::SubSetOverflowUse_i16_Reg),
        PickOpc(MC6809::SubSetCarryUse_i32_Reg, MC6809::SubSetOverflowUse_i32_Reg));
    // Carry-IN implicit-use (see selectAddE).
    Instr = Builder.buildInstr(Opcode)
                .addDef(Dst)
                .addUse(LHS)
                .addUse(RHS)
                .addDef(CarryOut, RegState::ImplicitDefine);
    if (!Bridged)
      Instr.addUse(Carry, RegState::Implicit);
    constrainSelectedInstRegOperands(*Instr, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  LLVM_DEBUG(
      (void)MRI->getVRegDef(MI.getOperand(2).getReg());
      (void)MRI->getVRegDef(MI.getOperand(3).getReg());
      );

  return false;
}

bool MC6809InstructionSelector::selectUnMergeValues(MachineInstr &MI) {
  // Operand layout: [def0, def1, ..., defN-1, src]. The original code
  // unconditionally took the first 3 regs as (Lo, Hi, Src), which is
  // wrong for the 4-way s32 → 4×s8 case (5 operands, src at index 4).
  // Bug #161: detect the s32 → 4×s8 case explicitly. Picolibc's i32
  // path (now reachable after the -mcpu=hd6309 typo fix in 013cdad0)
  // routinely produces this shape via the legalizer's narrow chain,
  // and the selector hand-path needs to cover it.
  unsigned NumDefs = MI.getNumOperands() - 1;
  Register Src = MI.getOperand(NumDefs).getReg();
  LLT SrcTy = MRI->getType(Src);

  MachineIRBuilder Builder(MI);

  // s32 → 4×s8: chain via two s16 sub-register extracts (lo_word /
  // hi_word) followed by EXTRACT_LO_i16 / EXTRACT_HI_i16 on each.
  if (SrcTy == S32 && NumDefs == 4) {
    Register D0 = MI.getOperand(0).getReg();
    Register D1 = MI.getOperand(1).getReg();
    Register D2 = MI.getOperand(2).getReg();
    Register D3 = MI.getOperand(3).getReg();
    MRI->setRegClass(Src, &MC6809::ACC32RegClass);
    for (Register R : {D0, D1, D2, D3})
      if (!MRI->getRegClassOrNull(R))
        MRI->setRegClass(R, &MC6809::ACC8RegClass);

    // Two intermediate s16 vregs: low word (D0/D1 source) and high word
    // (D2/D3 source).  Bug #302 redesign Phase 2 (2026-05-17): switched
    // from EXTRACT_LO/HI_word_i32 to Extract16_i32_lo/hi (Phase 1's
    // parallel replacement pseudos).  Same operand shape and post-RA
    // expansion semantics; preparation for Phase 3+.
    Register WordLo = MRI->createVirtualRegister(&MC6809::ADcRegClass);
    Register WordHi = MRI->createVirtualRegister(&MC6809::ADcRegClass);
    auto LoWord = Builder.buildInstr(MC6809::Extract16_i32_lo).addDef(WordLo).addUse(Src);
    auto HiWord = Builder.buildInstr(MC6809::Extract16_i32_hi).addDef(WordHi).addUse(Src);
    constrainSelectedInstRegOperands(*LoWord, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*HiWord, TII, TRI, RBI);

    auto E0 = Builder.buildInstr(MC6809::EXTRACT_LO_i16).addDef(D0).addUse(WordLo);
    auto E1 = Builder.buildInstr(MC6809::EXTRACT_HI_i16).addDef(D1).addUse(WordLo);
    auto E2 = Builder.buildInstr(MC6809::EXTRACT_LO_i16).addDef(D2).addUse(WordHi);
    auto E3 = Builder.buildInstr(MC6809::EXTRACT_HI_i16).addDef(D3).addUse(WordHi);
    constrainSelectedInstRegOperands(*E0, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*E1, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*E2, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*E3, TII, TRI, RBI);
    MI.eraseFromParent();
    return true;
  }

  // Otherwise we expect the original 3-operand (1 src + 2 dsts) shape.
  Register Lo = MI.getOperand(0).getReg();
  Register Hi = MI.getOperand(1).getReg();
  assert((SrcTy == S16 || SrcTy == S32) && "The Src of G_UNMERGE_VALUES must be S16 or S32");

  if (SrcTy == S16) {
    // s16 → s8×2: emit EXTRACT_LO_i16 + EXTRACT_HI_i16 pseudos instead of
    // sub-register COPYs (bug #118 Layer 1, approach b).
    MRI->setRegClass(Src, &MC6809::ADcRegClass);
    if (!MRI->getRegClassOrNull(Lo))
      MRI->setRegClass(Lo, &MC6809::ACC8RegClass);
    if (!MRI->getRegClassOrNull(Hi))
      MRI->setRegClass(Hi, &MC6809::ACC8RegClass);
    auto LoExt = Builder.buildInstr(MC6809::EXTRACT_LO_i16)
                     .addDef(Lo)
                     .addUse(Src);
    auto HiExt = Builder.buildInstr(MC6809::EXTRACT_HI_i16)
                     .addDef(Hi)
                     .addUse(Src);
    constrainSelectedInstRegOperands(*LoExt, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*HiExt, TII, TRI, RBI);
  } else {
    // s32 → s16×2: Bug #302 redesign Phase 2 (2026-05-17) — switched
    // from EXTRACT_LO/HI_word_i32 to Extract16_i32_lo/hi (Phase 1's
    // parallel replacement pseudos).  Same operand shape and post-RA
    // expansion semantics.
    MRI->setRegClass(Src, &MC6809::ACC32RegClass);
    if (!MRI->getRegClassOrNull(Lo))
      MRI->setRegClass(Lo, &MC6809::ADcRegClass);
    if (!MRI->getRegClassOrNull(Hi))
      MRI->setRegClass(Hi, &MC6809::ADcRegClass);
    auto LoWord = Builder.buildInstr(MC6809::Extract16_i32_lo).addDef(Lo).addUse(Src);
    auto HiWord = Builder.buildInstr(MC6809::Extract16_i32_hi).addDef(Hi).addUse(Src);
    constrainSelectedInstRegOperands(*LoWord, TII, TRI, RBI);
    constrainSelectedInstRegOperands(*HiWord, TII, TRI, RBI);
  }
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectShiftExtend(MachineInstr &MI) {
  // Select G_SHLE (shift-left-extend) and G_LSHRE (logical-shift-right-extend).
  // These are single-bit shifts with carry in/out, produced by the shift
  // decomposition in legalizeShiftRotate for constant shifts.
  //
  // G_SHLE → LSL_i8_Reg (= ASL, shift left by 1)
  // G_LSHRE + carry from ICMP (sign test) → ASR_i8_Reg (arithmetic shift right)
  // G_LSHRE + carry = 0/undef → LSR_i8_Reg (logical shift right)
  Register Dst = MI.getOperand(0).getReg();
  Register CarryOut = MI.getOperand(1).getReg();
  Register Src = MI.getOperand(2).getReg();
  Register CarryIn = MI.getOperand(3).getReg();

  LLT Ty = MRI->getType(Dst);
  if (Ty != LLT::scalar(8))
    return false; // Only handle s8 for now

  // Determine the shift instruction based on carry_in source:
  // - Constant 0 / undef: first in chain → ASL/LSR/ASR
  // - From G_ICMP: ASHR sign test → ASR
  // - From another G_SHLE/G_LSHRE: carry chain → ROL/ROR
  MachineInstr *CarryDef = MRI->getVRegDef(CarryIn);
  bool IsCarryChain = CarryDef &&
      (CarryDef->getOpcode() == MC6809::G_SHLE ||
       CarryDef->getOpcode() == MC6809::G_LSHRE);

  unsigned ShiftOpc;
  if (MI.getOpcode() == MC6809::G_SHLE) {
    ShiftOpc = IsCarryChain ? MC6809::ROL_i8_Reg : MC6809::LSL_i8_Reg;
  } else {
    if (CarryDef && CarryDef->getOpcode() == TargetOpcode::G_ICMP)
      ShiftOpc = MC6809::ASR_i8_Reg;
    else if (IsCarryChain)
      ShiftOpc = MC6809::ROR_i8_Reg;
    else
      ShiftOpc = MC6809::LSR_i8_Reg;
  }

  MachineIRBuilder Builder(MI);
  auto Shift = Builder.buildInstr(ShiftOpc)
      .addDef(Dst)
      .addUse(Src)
      .addImm(1);  // shift amount = 1
  constrainSelectedInstRegOperands(*Shift, TII, TRI, RBI);

  // The carry output is implicitly in CC. Mark it as dead if unused,
  // or create a COPY from the C flag if used.
  if (MRI->use_empty(CarryOut)) {
    // Carry not used — nothing to do, it's implicit in CC
  } else {
    // Carry is used by the next shift in the chain.
    // For single-byte shifts, each iteration is independent (same constant
    // carry_in), so the carry_out is never actually consumed. But mark it
    // as defined to satisfy the register allocator.
    Builder.buildUndef(CarryOut);
  }

  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectShift16(MachineInstr &MI) {
  // Hand-select i16 shifts on 6809 into byte-pair operations.
  // SHL: ASLB + ROLA (carry from lo propagates to hi)
  // LSHR: LSRA + RORB (carry from hi propagates to lo)
  // ASHR: ASRA + RORB (sign-preserving shift of hi, carry to lo)
  Register DstReg = MI.getOperand(0).getReg();
  Register SrcReg = MI.getOperand(1).getReg();
  Register AmtReg = MI.getOperand(2).getReg();

  // Only handle constant shift amount
  auto Amt = getIConstantVRegValWithLookThrough(AmtReg, *MRI);
  if (!Amt)
    return false;

  uint64_t ShiftAmt = Amt->Value.getZExtValue();
  if (ShiftAmt == 0) {
    // Shift by 0 = copy
    MachineIRBuilder Builder(MI);
    Builder.buildCopy(DstReg, SrcReg);
    MI.eraseFromParent();
    return true;
  }

  unsigned ShiftOpc;
  switch (MI.getOpcode()) {
  case TargetOpcode::G_SHL:  ShiftOpc = MC6809::LSL_i16_Reg; break;
  case TargetOpcode::G_LSHR: ShiftOpc = MC6809::LSR_i16_Reg; break;
  case TargetOpcode::G_ASHR: ShiftOpc = MC6809::ASR_i16_Reg; break;
  default: return false;
  }

  MachineBasicBlock &MBB = *MI.getParent();
  const DebugLoc &DL = MI.getDebugLoc();

  // Copy source into the D register and shift it by ShiftAmt with a SINGLE
  // count-carrying pseudo (operand $val = the count). The post-RA expander
  // emits ShiftAmt single-bit shifts in place. Emitting one pseudo (rather than
  // ShiftAmt separate ones) means a spilled value is loaded/stored once around
  // the whole shift instead of per bit -- and lets foldMemoryOperandImpl fold
  // the spill into an in-memory asl/rol chain on the stack slot.
  //
  // Use ADc (D + spill slots) -- the class LSL/LSR/ASR_i16_Reg are defined with
  // (dst tied to src). ADc is spillable, and ADc ⊆ ACC16 so any consumer that
  // accepts ACC16 also accepts the result.
  Register Cur = MRI->createVirtualRegister(&MC6809::ADcRegClass);
  BuildMI(MBB, MI, DL, TII.get(TargetOpcode::COPY), Cur).addReg(SrcReg);
  auto &Shift = *BuildMI(MBB, MI, DL, TII.get(ShiftOpc), DstReg)
      .addReg(Cur)
      .addImm(ShiftAmt);
  constrainSelectedInstRegOperands(Shift, TII, TRI, RBI);
  MRI->setRegClass(DstReg, &MC6809::ADcRegClass);
  MI.eraseFromParent();
  return true;
}

bool MC6809InstructionSelector::selectGeneric(MachineInstr &MI) {
  unsigned Opcode;
  switch (MI.getOpcode()) {
  default:
    llvm_unreachable("Select Generic - Unexpected opcode.");
  case TargetOpcode::G_IMPLICIT_DEF:
    Opcode = MC6809::IMPLICIT_DEF;
    break;
  case TargetOpcode::G_PHI:
    Opcode = MC6809::PHI;
    break;
  }
  MI.setDesc(TII.get(Opcode));
  MI.addImplicitDefUseOperands(*MF);
  // Establish any tied operands and known register classes.
  constrainSelectedInstRegOperands(MI, TII, TRI, RBI);
  // Make sure that the outputs have register classes.
  constrainGenericOp(MI);
  return true;
}

// Ensures that any virtual registers defined by this operation are given a
// register class. Otherwise, it's possible for chains of generic operations
// (PHI, COPY, etc.) to circularly define virtual registers in such a way that
// they never actually receive a register class. Since every virtual register
// is defined exactly once, making sure definitions are constrained suffices.
void MC6809InstructionSelector::constrainGenericOp(MachineInstr &MI) {
  MachineRegisterInfo &MRI = MI.getMF()->getRegInfo();
  for (MachineOperand &Op : MI.all_defs()) {
    if (Op.getReg().isPhysical() || MRI.getRegClassOrNull(Op.getReg()))
      continue;
    LLT Ty = MRI.getType(Op.getReg());
    const RegisterBank *RB = MRI.getRegBankOrNull(Op.getReg());
    constrainOperandRegClass(Op, getRegClassForTypeOnBank(Ty, RB));
  }
}

void MC6809InstructionSelector::constrainOperandRegClass(MachineOperand &RegMO, const TargetRegisterClass &RegClass) {
  MachineInstr &MI = *RegMO.getParent();
  RegMO.setReg(llvm::constrainOperandRegClass(*MF, TRI, *MRI, TII, RBI, MI, RegClass, RegMO));
}

InstructionSelector *llvm::createMC6809InstructionSelector(const MC6809TargetMachine &TM, MC6809Subtarget &STI, MC6809RegisterBankInfo &RBI) { return new MC6809InstructionSelector(TM, STI, RBI); }
