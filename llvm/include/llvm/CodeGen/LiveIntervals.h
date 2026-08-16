//===- LiveIntervals.h - Live Interval Analysis -----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file This file implements the LiveInterval analysis pass.  Given some
/// numbering of each the machine instructions (in this implemention depth-first
/// order) an interval [i, j) is said to be a live interval for register v if
/// there is no instruction with number j' > j such that v is live at j' and
/// there is no instruction with number i' < i such that v is live at i'. In
/// this implementation intervals can have holes, i.e. an interval might look
/// like [1,20), [50,65), [1000,1001).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_CODEGEN_LIVEINTERVALS_H
#define LLVM_CODEGEN_LIVEINTERVALS_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/IndexedMap.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/LiveInterval.h"
#include "llvm/CodeGen/LiveIntervalCalc.h"
#include "llvm/CodeGen/MachineBasicBlock.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/MachinePassManager.h"
#include "llvm/CodeGen/SlotIndexes.h"
#include "llvm/CodeGen/TargetRegisterInfo.h"
#include "llvm/MC/LaneBitmask.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
#include <cstdint>
#include <utility>

namespace llvm {

LLVM_ABI extern cl::opt<bool> UseSegmentSetForPhysRegs;

class BitVector;
class MachineBlockFrequencyInfo;
class MachineDominatorTree;
class MachineFunction;
class MachineInstr;
class MachineRegisterInfo;
class ProfileSummaryInfo;
class raw_ostream;
class TargetInstrInfo;
class VirtRegMap;

class LiveIntervals {
  friend class LiveIntervalsAnalysis;
  friend class LiveIntervalsWrapperPass;

public:
  /// MC6809 Bug #290: hook for register allocators to react to
  /// spiller-driven physreg clobbers added mid-pass.  Bug #256 Fix B's
  /// `addInstructionDefsToRegUnits` propagates a new MI's implicit-defs
  /// into the affected regunits' cached LiveRanges; vregs already
  /// assigned to physregs containing those regunits whose live range
  /// crosses the new MI's slot now have an invalid assignment.
  /// Greedy needs to re-evaluate them, but greedy's main loop only
  /// processes unassigned vregs.  This delegate is fired once per
  /// (Unit, Slot) where a new dead-def was recorded so the allocator
  /// can re-enqueue the affected assigned vregs through its own
  /// selectOrSplit pipeline.
  class ClobberDelegate {
  public:
    virtual ~ClobberDelegate() = default;
    /// Called when a new dead-def has been recorded in the cached
    /// regunit LiveRange for \p Unit at slot \p Pos by an MI \p MI
    /// that the spiller (or any other mid-pass mutator) just
    /// inserted via addInstructionDefsToRegUnits.  Implementations
    /// should walk LiveRegMatrix's LIU at Unit and re-enqueue any
    /// assigned vregs whose live range covers Pos.
    virtual void clobberPropagatedToRegUnit(MachineInstr &MI,
                                            MCRegUnit Unit,
                                            SlotIndex Pos) = 0;
  };

private:
  MachineFunction *MF = nullptr;
  MachineRegisterInfo *MRI = nullptr;
  const TargetRegisterInfo *TRI = nullptr;
  const TargetInstrInfo *TII = nullptr;
  SlotIndexes *Indexes = nullptr;
  MachineDominatorTree *DomTree = nullptr;
  std::unique_ptr<LiveIntervalCalc> LICalc;

  /// Special pool allocator for VNInfo's (LiveInterval val#).
  VNInfo::Allocator VNInfoAllocator;

  /// Live interval pointers for all the virtual registers.
  IndexedMap<LiveInterval *, VirtReg2IndexFunctor> VirtRegIntervals;

  /// Sorted list of instructions with register mask operands. Always use the
  /// 'r' slot, RegMasks are normal clobbers, not early clobbers.
  SmallVector<SlotIndex, 8> RegMaskSlots;

  /// This vector is parallel to RegMaskSlots, it holds a pointer to the
  /// corresponding register mask.  This pointer can be recomputed as:
  ///
  ///   MI = Indexes->getInstructionFromIndex(RegMaskSlot[N]);
  ///   unsigned OpNum = findRegMaskOperand(MI);
  ///   RegMaskBits[N] = MI->getOperand(OpNum).getRegMask();
  ///
  /// This is kept in a separate vector partly because some standard
  /// libraries don't support lower_bound() with mixed objects, partly to
  /// improve locality when searching in RegMaskSlots.
  /// Also see the comment in LiveInterval::find().
  SmallVector<const uint32_t *, 8> RegMaskBits;

  /// For each basic block number, keep (begin, size) pairs indexing into the
  /// RegMaskSlots and RegMaskBits arrays.
  /// Note that basic block numbers may not be layout contiguous, that's why
  /// we can't just keep track of the first register mask in each basic
  /// block.
  SmallVector<std::pair<unsigned, unsigned>, 8> RegMaskBlocks;

  /// Keeps a live range set for each register unit to track fixed physreg
  /// interference.
  SmallVector<LiveRange *, 0> RegUnitRanges;

  /// MC6809 Bug #256: per-regunit version counter, bumped whenever the
  /// corresponding entry of RegUnitRanges is mutated.  Parallel to
  /// RegUnitRanges (same indexing).  Consumed by InterferenceCache so
  /// that cached LiveRange* pointers and segment cursors can be
  /// invalidated when the underlying fixed-interference range changes
  /// mid-regalloc (e.g. the spiller inserts a new MI with physreg
  /// implicit-defs).  Tag value 0 is the initial state; consumers
  /// re-fetch when the tag they cached differs from the current tag.
  SmallVector<unsigned, 0> RegUnitFixedTags;

  /// MC6809 Bug #290: optional delegate for the register allocator
  /// to react to spiller-driven physreg clobbers (see
  /// `addInstructionDefsToRegUnits`).  Set by RegAllocGreedy / RABasic
  /// at run() entry; cleared at exit.  nullptr is the default state
  /// (no notification fired — safe pre/post regalloc and outside any
  /// allocator pass).
  ClobberDelegate *ClobberCallback = nullptr;

  // Can only be created from pass manager.
  LiveIntervals() = default;
  LiveIntervals(MachineFunction &MF, SlotIndexes &SI, MachineDominatorTree &DT)
      : Indexes(&SI), DomTree(&DT) {
    analyze(MF);
  }

  LLVM_ABI void analyze(MachineFunction &MF);

  LLVM_ABI void clear();

public:
  LiveIntervals(LiveIntervals &&) = default;
  LLVM_ABI ~LiveIntervals();

  LLVM_ABI bool invalidate(MachineFunction &MF, const PreservedAnalyses &PA,
                           MachineFunctionAnalysisManager::Invalidator &Inv);

  /// Calculate the spill weight to assign to a single instruction.
  /// If \p PSI is provided the calculation is altered for optsize functions.
  LLVM_ABI static float getSpillWeight(bool isDef, bool isUse,
                                       const MachineBlockFrequencyInfo *MBFI,
                                       const MachineInstr &MI,
                                       ProfileSummaryInfo *PSI = nullptr);

  /// Calculate the spill weight to assign to a single instruction.
  /// If \p PSI is provided the calculation is altered for optsize functions.
  LLVM_ABI static float getSpillWeight(bool isDef, bool isUse,
                                       const MachineBlockFrequencyInfo *MBFI,
                                       const MachineBasicBlock *MBB,
                                       ProfileSummaryInfo *PSI = nullptr);

  LiveInterval &getInterval(Register Reg) {
    if (hasInterval(Reg))
      return *VirtRegIntervals[Reg.id()];

    return createAndComputeVirtRegInterval(Reg);
  }

  const LiveInterval &getInterval(Register Reg) const {
    return const_cast<LiveIntervals *>(this)->getInterval(Reg);
  }

  bool hasInterval(Register Reg) const {
    return VirtRegIntervals.inBounds(Reg.id()) && VirtRegIntervals[Reg.id()];
  }

  /// Interval creation.
  LiveInterval &createEmptyInterval(Register Reg) {
    assert(!hasInterval(Reg) && "Interval already exists!");
    VirtRegIntervals.grow(Reg.id());
    auto &Interval = VirtRegIntervals[Reg.id()];
    Interval = createInterval(Reg);
    return *Interval;
  }

  LiveInterval &createAndComputeVirtRegInterval(Register Reg) {
    LiveInterval &LI = createEmptyInterval(Reg);
    computeVirtRegInterval(LI);
    return LI;
  }

  LiveInterval &createAndComputeVirtRegInterval(Register Reg, bool &NeedSplit) {
    LiveInterval &LI = createEmptyInterval(Reg);
    NeedSplit = computeVirtRegInterval(LI);
    return LI;
  }

  /// Return an existing interval for \p Reg.
  /// If \p Reg has no interval then this creates a new empty one instead.
  /// Note: does not trigger interval computation.
  LiveInterval &getOrCreateEmptyInterval(Register Reg) {
    return hasInterval(Reg) ? getInterval(Reg) : createEmptyInterval(Reg);
  }

  /// Interval removal.
  void removeInterval(Register Reg) {
    auto &Interval = VirtRegIntervals[Reg];
    delete Interval;
    Interval = nullptr;
  }

  /// Given a register and an instruction, adds a live segment from that
  /// instruction to the end of its MBB.
  LLVM_ABI LiveInterval::Segment
  addSegmentToEndOfBlock(Register Reg, MachineInstr &startInst);

  /// After removing some uses of a register, shrink its live range to just
  /// the remaining uses. This method does not compute reaching defs for new
  /// uses, and it doesn't remove dead defs.
  /// Dead PHIDef values are marked as unused. New dead machine instructions
  /// are added to the dead vector. Returns true if the interval may have been
  /// separated into multiple connected components.
  LLVM_ABI bool shrinkToUses(LiveInterval *li,
                             SmallVectorImpl<MachineInstr *> *dead = nullptr);

  /// Specialized version of
  /// shrinkToUses(LiveInterval *li, SmallVectorImpl<MachineInstr*> *dead)
  /// that works on a subregister live range and only looks at uses matching
  /// the lane mask of the subregister range.
  /// This may leave the subrange empty which needs to be cleaned up with
  /// LiveInterval::removeEmptySubranges() afterwards.
  LLVM_ABI void shrinkToUses(LiveInterval::SubRange &SR, Register Reg);

  /// Extend the live range \p LR to reach all points in \p Indices. The
  /// points in the \p Indices array must be jointly dominated by the union
  /// of the existing defs in \p LR and points in \p Undefs.
  ///
  /// PHI-defs are added as needed to maintain SSA form.
  ///
  /// If a SlotIndex in \p Indices is the end index of a basic block, \p LR
  /// will be extended to be live out of the basic block.
  /// If a SlotIndex in \p Indices is jointy dominated only by points in
  /// \p Undefs, the live range will not be extended to that point.
  ///
  /// See also LiveRangeCalc::extend().
  LLVM_ABI void extendToIndices(LiveRange &LR, ArrayRef<SlotIndex> Indices,
                                ArrayRef<SlotIndex> Undefs);

  void extendToIndices(LiveRange &LR, ArrayRef<SlotIndex> Indices) {
    extendToIndices(LR, Indices, /*Undefs=*/{});
  }

  /// If \p LR has a live value at \p Kill, prune its live range by removing
  /// any liveness reachable from Kill. Add live range end points to
  /// EndPoints such that extendToIndices(LI, EndPoints) will reconstruct the
  /// value's live range.
  ///
  /// Calling pruneValue() and extendToIndices() can be used to reconstruct
  /// SSA form after adding defs to a virtual register.
  LLVM_ABI void pruneValue(LiveRange &LR, SlotIndex Kill,
                           SmallVectorImpl<SlotIndex> *EndPoints);

  /// This function should not be used. Its intent is to tell you that you are
  /// doing something wrong if you call pruneValue directly on a
  /// LiveInterval. Indeed, you are supposed to call pruneValue on the main
  /// LiveRange and all the LiveRanges of the subranges if any.
  [[maybe_unused]] void pruneValue(LiveInterval &, SlotIndex,
                                   SmallVectorImpl<SlotIndex> *) {
    llvm_unreachable(
        "Use pruneValue on the main LiveRange and on each subrange");
  }

  SlotIndexes *getSlotIndexes() const { return Indexes; }

  /// Returns true if the specified machine instr has been removed or was
  /// never entered in the map.
  bool isNotInMIMap(const MachineInstr &Instr) const {
    return !Indexes->hasIndex(Instr);
  }

  /// Returns the base index of the given instruction.
  SlotIndex getInstructionIndex(const MachineInstr &Instr) const {
    return Indexes->getInstructionIndex(Instr);
  }

  /// Returns the instruction associated with the given index.
  MachineInstr *getInstructionFromIndex(SlotIndex index) const {
    return Indexes->getInstructionFromIndex(index);
  }

  /// Return the first index in the given basic block.
  SlotIndex getMBBStartIdx(const MachineBasicBlock *mbb) const {
    return Indexes->getMBBStartIdx(mbb);
  }

  /// Return the last index in the given basic block.
  SlotIndex getMBBEndIdx(const MachineBasicBlock *mbb) const {
    return Indexes->getMBBEndIdx(mbb);
  }

  bool isLiveInToMBB(const LiveRange &LR, const MachineBasicBlock *mbb) const {
    return LR.liveAt(getMBBStartIdx(mbb));
  }

  bool isLiveOutOfMBB(const LiveRange &LR, const MachineBasicBlock *mbb) const {
    return LR.liveAt(getMBBEndIdx(mbb).getPrevSlot());
  }

  MachineBasicBlock *getMBBFromIndex(SlotIndex index) const {
    return Indexes->getMBBFromIndex(index);
  }

  void insertMBBInMaps(MachineBasicBlock *MBB) {
    Indexes->insertMBBInMaps(MBB);
    assert(unsigned(MBB->getNumber()) == RegMaskBlocks.size() &&
           "Blocks must be added in order.");
    RegMaskBlocks.push_back(std::make_pair(RegMaskSlots.size(), 0));
  }

  SlotIndex InsertMachineInstrInMaps(MachineInstr &MI) {
    return Indexes->insertMachineInstrInMaps(MI);
  }

  void InsertMachineInstrRangeInMaps(MachineBasicBlock::iterator B,
                                     MachineBasicBlock::iterator E) {
    for (MachineBasicBlock::iterator I = B; I != E; ++I)
      Indexes->insertMachineInstrInMaps(*I);
  }

  void RemoveMachineInstrFromMaps(MachineInstr &MI) {
    Indexes->removeMachineInstrFromMaps(MI);
  }

  SlotIndex ReplaceMachineInstrInMaps(MachineInstr &MI, MachineInstr &NewMI) {
    return Indexes->replaceMachineInstrInMaps(MI, NewMI);
  }

  VNInfo::Allocator &getVNInfoAllocator() { return VNInfoAllocator; }

  /// Implement the dump method.
  LLVM_ABI void print(raw_ostream &O) const;
  LLVM_ABI void dump() const;

  // For legacy pass to recompute liveness.
  void reanalyze(MachineFunction &MF) {
    clear();
    analyze(MF);
  }

  MachineDominatorTree &getDomTree() { return *DomTree; }

  /// If LI is confined to a single basic block, return a pointer to that
  /// block.  If LI is live in to or out of any block, return NULL.
  LLVM_ABI MachineBasicBlock *intervalIsInOneMBB(const LiveInterval &LI) const;

  /// Returns true if VNI is killed by any PHI-def values in LI.
  /// This may conservatively return true to avoid expensive computations.
  LLVM_ABI bool hasPHIKill(const LiveInterval &LI, const VNInfo *VNI) const;

  /// Add kill flags to any instruction that kills a virtual register.
  LLVM_ABI void addKillFlags(const VirtRegMap *);

  /// Call this method to notify LiveIntervals that instruction \p MI has been
  /// moved within a basic block. This will update the live intervals for all
  /// operands of \p MI. Moves between basic blocks are not supported.
  ///
  /// \param UpdateFlags Update live intervals for nonallocatable physregs.
  LLVM_ABI void handleMove(MachineInstr &MI, bool UpdateFlags = false);

  /// Update intervals of operands of all instructions in the newly
  /// created bundle specified by \p BundleStart.
  ///
  /// \param UpdateFlags Update live intervals for nonallocatable physregs.
  ///
  /// Assumes existing liveness is accurate.
  /// \pre BundleStart should be the first instruction in the Bundle.
  /// \pre BundleStart should not have a have SlotIndex as one will be assigned.
  LLVM_ABI void handleMoveIntoNewBundle(MachineInstr &BundleStart,
                                        bool UpdateFlags = false);

  /// Update live intervals for instructions in a range of iterators. It is
  /// intended for use after target hooks that may insert or remove
  /// instructions, and is only efficient for a small number of instructions.
  ///
  /// OrigRegs is a vector of registers that were originally used by the
  /// instructions in the range between the two iterators.
  ///
  /// Currently, the only changes that are supported are simple removal
  /// and addition of uses.
  LLVM_ABI void repairIntervalsInRange(MachineBasicBlock *MBB,
                                       MachineBasicBlock::iterator Begin,
                                       MachineBasicBlock::iterator End,
                                       ArrayRef<Register> OrigRegs);

  // Register mask functions.
  //
  // Machine instructions may use a register mask operand to indicate that a
  // large number of registers are clobbered by the instruction.  This is
  // typically used for calls.
  //
  // For compile time performance reasons, these clobbers are not recorded in
  // the live intervals for individual physical registers.  Instead,
  // LiveIntervalAnalysis maintains a sorted list of instructions with
  // register mask operands.

  /// Returns a sorted array of slot indices of all instructions with
  /// register mask operands.
  ArrayRef<SlotIndex> getRegMaskSlots() const { return RegMaskSlots; }

  /// Returns a sorted array of slot indices of all instructions with register
  /// mask operands in the basic block numbered \p MBBNum.
  ArrayRef<SlotIndex> getRegMaskSlotsInBlock(unsigned MBBNum) const {
    std::pair<unsigned, unsigned> P = RegMaskBlocks[MBBNum];
    return getRegMaskSlots().slice(P.first, P.second);
  }

  /// Returns an array of register mask pointers corresponding to
  /// getRegMaskSlots().
  ArrayRef<const uint32_t *> getRegMaskBits() const { return RegMaskBits; }

  /// Returns an array of mask pointers corresponding to
  /// getRegMaskSlotsInBlock(MBBNum).
  ArrayRef<const uint32_t *> getRegMaskBitsInBlock(unsigned MBBNum) const {
    std::pair<unsigned, unsigned> P = RegMaskBlocks[MBBNum];
    return getRegMaskBits().slice(P.first, P.second);
  }

  /// Test if \p LI is live across any register mask instructions, and
  /// compute a bit mask of physical registers that are not clobbered by any
  /// of them.
  ///
  /// Returns false if \p LI doesn't cross any register mask instructions. In
  /// that case, the bit vector is not filled in.
  LLVM_ABI bool checkRegMaskInterference(const LiveInterval &LI,
                                         BitVector &UsableRegs);

  // Register unit functions.
  //
  // Fixed interference occurs when MachineInstrs use physregs directly
  // instead of virtual registers. This typically happens when passing
  // arguments to a function call, or when instructions require operands in
  // fixed registers.
  //
  // Each physreg has one or more register units, see MCRegisterInfo. We
  // track liveness per register unit to handle aliasing registers more
  // efficiently.

  /// Return the live range for register unit \p Unit. It will be computed if
  /// it doesn't exist.
  LiveRange &getRegUnit(MCRegUnit Unit) {
    LiveRange *LR = RegUnitRanges[static_cast<unsigned>(Unit)];
    if (!LR) {
      // Compute missing ranges on demand.
      // Use segment set to speed-up initial computation of the live range.
      RegUnitRanges[static_cast<unsigned>(Unit)] = LR =
          new LiveRange(UseSegmentSetForPhysRegs);
      computeRegUnitRange(*LR, Unit);
      // MC6809 Bug #256: bump the version tag so any consumer that
      // cached a previous nullptr or stale pointer for this regunit
      // re-fetches before its next interference query.
      ++RegUnitFixedTags[static_cast<unsigned>(Unit)];
    }
    return *LR;
  }

  /// Return the live range for register unit \p Unit if it has already been
  /// computed, or nullptr if it hasn't been computed yet.
  LiveRange *getCachedRegUnit(MCRegUnit Unit) {
    return RegUnitRanges[static_cast<unsigned>(Unit)];
  }

  const LiveRange *getCachedRegUnit(MCRegUnit Unit) const {
    return RegUnitRanges[static_cast<unsigned>(Unit)];
  }

  /// MC6809 Bug #256: Return the current version tag for the
  /// fixed-interference range of regunit \p Unit.  Consumers should
  /// cache this alongside any cached `LiveRange*` for the unit and
  /// re-fetch when the tag they observe differs from the current value.
  unsigned getRegUnitFixedTag(MCRegUnit Unit) const {
    return RegUnitFixedTags[static_cast<unsigned>(Unit)];
  }

  /// MC6809 Bug #290: register a delegate to be notified when
  /// `addInstructionDefsToRegUnits` propagates a new dead-def into a
  /// cached regunit LiveRange.  The register allocator (RAGreedy /
  /// RABasic) registers itself here so it can re-enqueue already-
  /// assigned vregs whose physreg is now invalidated by the new
  /// clobber.  Pass nullptr (or call clearClobberDelegate) to
  /// disable notification.
  void setClobberDelegate(ClobberDelegate *D) { ClobberCallback = D; }
  void clearClobberDelegate() { ClobberCallback = nullptr; }

  /// Remove computed live range for register unit \p Unit. Subsequent uses
  /// should rely on on-demand recomputation.
  void removeRegUnit(MCRegUnit Unit) {
    delete RegUnitRanges[static_cast<unsigned>(Unit)];
    RegUnitRanges[static_cast<unsigned>(Unit)] = nullptr;
    // MC6809 Bug #256: bump version tag — any consumer holding the
    // deleted pointer must invalidate before its next access.
    ++RegUnitFixedTags[static_cast<unsigned>(Unit)];
  }

  /// Remove associated live ranges for the register units associated with \p
  /// Reg. Subsequent uses should rely on on-demand recomputation.  \note This
  /// method can result in inconsistent liveness tracking if multiple phyical
  /// registers share a regunit, and should be used cautiously.
  void removeAllRegUnitsForPhysReg(MCRegister Reg) {
    for (MCRegUnit Unit : TRI->regunits(Reg))
      removeRegUnit(Unit);
  }

  /// Remove value numbers and related live segments starting at position
  /// \p Pos that are part of any liverange of physical register \p Reg or one
  /// of its subregisters.
  LLVM_ABI void removePhysRegDefAt(MCRegister Reg, SlotIndex Pos);

  /// MC6809 Bug #256: walk the physreg def operands of \p MI (explicit
  /// AND implicit) and record a dead-def at MI's slot in each affected
  /// regunit's `LiveRange`, but only if that regunit's range is already
  /// cached.  If the range hasn't been computed yet, lazy computation
  /// will pick up the def via `LICalc::createDeadDefs` walking
  /// `MRI->def_operands`, so no explicit action is required.  The
  /// per-regunit version tag is bumped on every modification so that
  /// `InterferenceCache::Entry` invalidates its cached `Fixed` pointer
  /// at next interference query.
  ///
  /// Pre-condition: \p MI has already been inserted via
  /// `InsertMachineInstrInMaps` so its SlotIndex is valid.
  ///
  /// Spiller-facing API: called by `InlineSpiller::insertReload` and
  /// `insertSpill` so spills with physreg implicit-defs (e.g. MC6809's
  /// `SpillLoad_i32_Mem` clobber set) are visible to the greedy
  /// allocator's interference query.
  LLVM_ABI void addInstructionDefsToRegUnits(MachineInstr &MI);

  /// MC6809 Bug #256: symmetric counterpart to
  /// `addInstructionDefsToRegUnits`.  Remove any value at MI's slot
  /// from each affected regunit's cached `LiveRange`, bumping the
  /// per-regunit version tag.  Must be called BEFORE
  /// `RemoveMachineInstrFromMaps` (the SlotIndex must still resolve).
  ///
  /// Spiller-facing API: called from `LRE_WillEraseInstruction` on the
  /// three `LiveRangeEdit::Delegate` implementers (`RAGreedy`,
  /// `RABasic`, `HoistSpillHelper`) so any spiller-introduced physreg
  /// def is retracted before the MI vanishes — otherwise orphan
  /// VNInfo defs would survive at slots whose MI was erased
  /// (`-verify-machineinstrs` rejects this state).
  LLVM_ABI void removeInstructionDefsFromRegUnits(MachineInstr &MI);

  /// Remove value number and related live segments of \p LI and its subranges
  /// that start at position \p Pos.
  LLVM_ABI void removeVRegDefAt(LiveInterval &LI, SlotIndex Pos);

  /// Split separate components in LiveInterval \p LI into separate intervals.
  LLVM_ABI void
  splitSeparateComponents(LiveInterval &LI,
                          SmallVectorImpl<LiveInterval *> &SplitLIs);

  /// For live interval \p LI with correct SubRanges construct matching
  /// information for the main live range. Expects the main live range to not
  /// have any segments or value numbers.
  LLVM_ABI void constructMainRangeFromSubranges(LiveInterval &LI);

private:
  /// Compute live intervals for all virtual registers.
  void computeVirtRegs();

  /// Compute RegMaskSlots and RegMaskBits.
  void computeRegMasks();

  /// Walk the values in \p LI and check for dead values:
  /// - Dead PHIDef values are marked as unused.
  /// - Dead operands are marked as such.
  /// - Completely dead machine instructions are added to the \p dead vector
  ///   if it is not nullptr.
  /// Returns true if any PHI value numbers have been removed which may
  /// have separated the interval into multiple connected components.
  bool computeDeadValues(LiveInterval &LI,
                         SmallVectorImpl<MachineInstr *> *dead);

  LLVM_ABI static LiveInterval *createInterval(Register Reg);

  void printInstrs(raw_ostream &O) const;
  void dumpInstrs() const;

  void computeLiveInRegUnits();
  LLVM_ABI void computeRegUnitRange(LiveRange &, MCRegUnit Unit);
  LLVM_ABI bool computeVirtRegInterval(LiveInterval &);

  using ShrinkToUsesWorkList = SmallVector<std::pair<SlotIndex, VNInfo *>, 16>;
  void extendSegmentsToUses(LiveRange &Segments, ShrinkToUsesWorkList &WorkList,
                            Register Reg, LaneBitmask LaneMask);

  /// Helper function for repairIntervalsInRange(), walks backwards and
  /// creates/modifies live segments in \p LR to match the operands found.
  /// Only full operands or operands with subregisters matching \p LaneMask
  /// are considered.
  void repairOldRegInRange(MachineBasicBlock::iterator Begin,
                           MachineBasicBlock::iterator End,
                           const SlotIndex endIdx, LiveRange &LR, Register Reg,
                           LaneBitmask LaneMask = LaneBitmask::getAll());

  class HMEditor;
};

class LiveIntervalsAnalysis : public AnalysisInfoMixin<LiveIntervalsAnalysis> {
  friend AnalysisInfoMixin<LiveIntervalsAnalysis>;
  LLVM_ABI static AnalysisKey Key;

public:
  using Result = LiveIntervals;
  LLVM_ABI Result run(MachineFunction &MF,
                      MachineFunctionAnalysisManager &MFAM);
};

class LiveIntervalsPrinterPass
    : public RequiredPassInfoMixin<LiveIntervalsPrinterPass> {
  raw_ostream &OS;

public:
  explicit LiveIntervalsPrinterPass(raw_ostream &OS) : OS(OS) {}
  LLVM_ABI PreservedAnalyses run(MachineFunction &MF,
                                 MachineFunctionAnalysisManager &MFAM);
};

class LLVM_ABI LiveIntervalsWrapperPass : public MachineFunctionPass {
  LiveIntervals LIS;

public:
  static char ID;

  LiveIntervalsWrapperPass();

  void getAnalysisUsage(AnalysisUsage &AU) const override;
  void releaseMemory() override { LIS.clear(); }

  /// Pass entry point; Calculates LiveIntervals.
  bool runOnMachineFunction(MachineFunction &) override;

  /// Implement the dump method.
  void print(raw_ostream &O, const Module * = nullptr) const override {
    LIS.print(O);
  }

  LiveIntervals &getLIS() { return LIS; }
};

} // end namespace llvm

#endif
