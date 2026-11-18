//===-- MC6809ValueTracker.cpp - Which register holds which memory -------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#include "MC6809ValueTracker.h"

#include "MC6809FrameLowering.h"
#include "MC6809InstrInfo.h"
#include "MCTargetDesc/MC6809MCTargetDesc.h"

#include "llvm/ADT/STLExtras.h"
#include "llvm/Analysis/ValueTracking.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstr.h"
#include "llvm/CodeGen/MachineMemOperand.h"
#include "llvm/CodeGen/PseudoSourceValue.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/CodeGen/TargetSubtargetInfo.h"
#include "llvm/IR/GlobalValue.h"
#include "llvm/IR/Instructions.h"

using namespace llvm;

/// Bug #366: true if MI may write the function's own stack frame. Used to
/// invalidate frame-slot index mirrors for a store through a frame-address
/// pointer (`std n,y` where Y holds a frame-local address), which the
/// (base,offset) slot tracker — keyed on the index base, not SU/SS — cannot
/// relate to a frame slot.
///
/// CONSERVATIVE BY DEFAULT: a store may write the frame UNLESS its MMO proves
/// otherwise. Only two footprints are provably non-frame (and so safe to skip,
/// preserving the reload-elimination win):
///   - a non-Stack/FixedStack PseudoSourceValue that does not may-alias the
///     frame (GOT / JumpTable / ConstantPool / GlobalValue / CallEntry), or
///   - an IR Value whose underlying object is a GlobalValue.
/// Everything else — a Stack/FixedStack PSV, a no-MMO store, or a bare IR Value
/// that is (or derives from) an alloca — may be the frame. Inlined locals like
/// `long_options`/`ap`/`data` keep an `into %ir.NAME` alloca MMO post-LTO and
/// are NOT lowered to a Stack/FixedStack PSV; classifying those as non-frame
/// was the Bug #366 miscompile (test-getopt -flto: stale optarg pointer).
bool MC6809ValueTracker::mayStoreToStackFrame(const MachineInstr &MI,
                                 const MachineFrameInfo &MFI) {
  if (!MI.mayStore())
    return false;
  if (MI.memoperands_empty())
    return true; // unknown footprint — conservative
  for (const MachineMemOperand *MMO : MI.memoperands()) {
    if (!MMO->isStore())
      continue;
    if (const PseudoSourceValue *PSV = MMO->getPseudoValue()) {
      if (PSV->isStack() || PSV->kind() == PseudoSourceValue::FixedStack ||
          PSV->mayAlias(&MFI))
        return true; // frame / spill / may-alias-frame PSV
      continue;      // GOT / JumpTable / ConstantPool / GlobalValue / CallEntry
    }
    const Value *V = MMO->getValue();
    if (!V)
      return true; // neither PSV nor IR value — unknown, conservative
    if (!isa<GlobalValue>(getUnderlyingObject(V)))
      return true; // alloca / inttoptr / opaque pointer — may be the frame
  }
  return false;
}

/// The spill slot an access is to: the frame index every memory operand of MI
/// names, when that index is a spill slot; INT_MIN otherwise. A spill slot's
/// address is never taken, so nothing but an access naming its frame index can
/// read or write it -- and a store to it cannot reach anything else.
int MC6809ValueTracker::spillSlotOf(const MachineInstr &MI,
                                     const MachineFrameInfo &MFI) {
  if (MI.memoperands_empty())
    return INT_MIN;
  int FI = INT_MIN;
  for (const MachineMemOperand *MMO : MI.memoperands()) {
    const auto *FS =
        dyn_cast_or_null<FixedStackPseudoSourceValue>(MMO->getPseudoValue());
    if (!FS || !MFI.isSpillSlotObjectIndex(FS->getFrameIndex()) ||
        (FI != INT_MIN && FI != FS->getFrameIndex()))
      return INT_MIN;
    FI = FS->getFrameIndex();
  }
  return FI;
}

/// A store to a direct-page imaginary register (`std <rs0`): compiler-private
/// memory no pointer reaches.
static bool storesImaginaryReg(const MachineInstr &MI) {
  for (const MachineOperand &MO : MI.operands())
    if (MO.isReg() && MO.getReg().isPhysical() &&
        (MC6809::Imag8RegClass.contains(MO.getReg()) ||
         MC6809::Imag16RegClass.contains(MO.getReg())))
      return true;
  return false;
}

MC6809ValueTracker::MC6809ValueTracker(const MachineFunction &MF,
                                       const TargetRegisterInfo &TRI)
    : MF(MF), MFI(MF.getFrameInfo()), TRI(TRI) {}

int MC6809ValueTracker::holderWidth(Register R) {
  return MC6809InstrInfo::getValueRegBytes(R);
}

// A location is the frame if its memory operand says so (stack or
// fixed-stack, or an alloca), or, with no memory operand, if it is addressed
// from a frame base.
bool MC6809ValueTracker::isFrameLocation(const MachineInstr &Est,
                                         Register Base) const {
  if (Est.memoperands_empty())
    return static_cast<const MC6809FrameLowering &>(
               *MF.getSubtarget().getFrameLowering())
        .isFrameBase(MF, Base);
  for (const MachineMemOperand *MMO : Est.memoperands()) {
    if (const PseudoSourceValue *PSV = MMO->getPseudoValue()) {
      if (!(PSV->isStack() || PSV->kind() == PseudoSourceValue::FixedStack))
        return false;
      continue;
    }
    const Value *V = MMO->getValue();
    if (!V || !isa<AllocaInst>(getUnderlyingObject(V)))
      return false;
  }
  return true;
}

MC6809ValueTracker::Mirror *MC6809ValueTracker::find(const Key &K) {
  for (auto &E : Entries)
    if (E.first == K)
      return &E.second;
  return nullptr;
}

void MC6809ValueTracker::set(const Key &K, Register Reg, MachineInstr &Est,
                             bool SourceKilled) {
  // An imaginary register is compiler-private direct-page memory: only an
  // access naming it (or an overlapping one) reaches it.
  bool IsImaginary = MC6809::Imag8RegClass.contains(K.Base) ||
                     MC6809::Imag16RegClass.contains(K.Base);
  int SpillFI = IsImaginary ? INT_MIN : spillSlotOf(Est, MFI);
  bool IsFrame =
      !IsImaginary && SpillFI == INT_MIN && isFrameLocation(Est, K.Base);
  Mirror New{Reg, SourceKilled, SpillFI, IsFrame, IsImaginary, &Est};
  if (Mirror *M = find(K))
    *M = New;
  else
    Entries.push_back({K, New});
}

void MC6809ValueTracker::clearHolder(Register R) {
  for (auto &E : Entries)
    if (E.second.Reg.isValid() && TRI.regsOverlap(E.second.Reg, R))
      E.second.Reg = Register();
}

void MC6809ValueTracker::clearBase(Register R) {
  for (auto &E : Entries)
    if (E.first.Base.isValid() && TRI.regsOverlap(E.first.Base, R))
      E.second.Reg = Register();
  llvm::erase_if(Pending, [&](const PendingStore &P) {
    return TRI.regsOverlap(P.K.Base, R);
  });
}

static bool isImaginary(Register R) {
  return MC6809::Imag8RegClass.contains(R) ||
         MC6809::Imag16RegClass.contains(R);
}

void MC6809ValueTracker::addPendingStore(const Key &K, int Width,
                                         MachineInstr &Store) {
  bool Imag = isImaginary(K.Base);
  Pending.push_back(
      {K, Width, &Store, Imag ? INT_MIN : spillSlotOf(Store, MFI), Imag});
}

void MC6809ValueTracker::takeCoveredStores(
    Register Base, int Off, int Width, SmallVectorImpl<MachineInstr *> &Dead) {
  llvm::erase_if(Pending, [&](const PendingStore &P) {
    if (P.K.Base != Base || P.K.Offset < Off ||
        P.K.Offset + P.Width > Off + Width)
      return false;
    Dead.push_back(P.Store);
    for (auto &E : Entries)
      if (E.second.Est == P.Store) {
        E.second.Est = nullptr;
        E.second.Reg = Register();
      }
    return true;
  });
}

void MC6809ValueTracker::readRange(Register Base, int Off, int Width) {
  llvm::erase_if(Pending, [&](const PendingStore &P) {
    return P.K.Base == Base && P.K.Offset < Off + Width &&
           Off < P.K.Offset + P.Width;
  });
}

void MC6809ValueTracker::readAlias(const MachineInstr &MI, bool KeyedOnOwnBase,
                                   Register OwnBase) {
  int LoadFI = spillSlotOf(MI, MFI);
  llvm::erase_if(Pending, [&](const PendingStore &P) {
    if (KeyedOnOwnBase && P.K.Base == OwnBase)
      return false; // readRange decides
    if (P.SpillFI != INT_MIN && LoadFI != INT_MIN)
      return LoadFI == P.SpillFI;
    if (P.IsImaginary && !MI.memoperands_empty() &&
        all_of(MI.memoperands(), [](const MachineMemOperand *MMO) {
          return MMO->getPseudoValue() || MMO->getValue();
        }) &&
        !namesOverlapping(MI, P.K.Base))
      return false;
    return true;
  });
}

void MC6809ValueTracker::storeOverlap(Register Base, int Off, int Width,
                                      Register StoredReg) {
  for (auto &E : Entries) {
    if (E.first.Base != Base || !E.second.Reg.isValid())
      continue;
    int M = E.first.Offset;
    int WS = holderWidth(E.second.Reg);
    if (M + WS <= Off || Off + Width <= M)
      continue; // disjoint byte ranges
    // The store writes this register's own bytes back where they were
    // (`std 4,u` over a mirror of A at 4,u): the mirror stays. Only at that
    // exact position -- `stq 2,u` writes W, not D, over 4,u.
    if (StoredReg.isValid()) {
      int Pos = MC6809InstrInfo::getSubRegByteOffset(TRI, StoredReg,
                                                     E.second.Reg);
      if (Pos >= 0 && M == Off + Pos)
        continue;
    }
    E.second.Reg = Register();
  }
}

bool MC6809ValueTracker::namesOverlapping(const MachineInstr &MI,
                                          Register R) const {
  for (const MachineOperand &MO : MI.operands())
    if (MO.isReg() && MO.getReg().isPhysical() &&
        TRI.regsOverlap(MO.getReg(), R))
      return true;
  return false;
}

void MC6809ValueTracker::storeAlias(const MachineInstr &MI,
                                    bool IndexedOnOwnBase, Register OwnBase) {
  int StoreFI = spillSlotOf(MI, MFI);
  bool StorePrivate = StoreFI != INT_MIN || storesImaginaryReg(MI);
  bool StoreMayHitFrame = !StorePrivate && mayStoreToStackFrame(MI, MFI);
  for (auto &E : Entries) {
    if (!E.second.Reg.isValid() ||
        (IndexedOnOwnBase && E.first.Base == OwnBase))
      continue;
    // An imaginary register goes on a store naming it (or one overlapping
    // it), or on any store whose footprint is not known to be private.
    bool Drop = E.second.IsImaginary
                    ? !StorePrivate || namesOverlapping(MI, E.first.Base)
                : E.second.SpillFI != INT_MIN ? StoreFI == E.second.SpillFI
                : E.second.IsFrame            ? StoreMayHitFrame
                                              : !StorePrivate;
    if (Drop)
      E.second.Reg = Register();
  }
}

void MC6809ValueTracker::clearKillsToPriorDef(MachineInstr &MI, Register Reg,
                                              const TargetRegisterInfo &TRI) {
  MachineBasicBlock &MBB = *MI.getParent();
  for (auto It = std::next(MI.getReverseIterator()); It != MBB.rend(); ++It) {
    bool DefsReg = false;
    for (MachineOperand &MO : It->operands()) {
      if (!MO.isReg() || !MO.getReg().isPhysical() ||
          !TRI.regsOverlap(MO.getReg(), Reg))
        continue;
      if (MO.isUse() && MO.isKill())
        MO.setIsKill(false);
      if (MO.isDef())
        DefsReg = true;
    }
    // Kills before the definition belong to another live range; the
    // definition itself is live now.
    if (DefsReg) {
      It->clearRegisterDeads(Reg);
      break;
    }
  }
}
