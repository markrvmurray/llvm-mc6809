//===-- MC6809ValueTracker.h - Which register holds which memory -*- C++ -*-=//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// A per-basic-block model, for the post-RA passes, of "register R holds the
// value of memory location L", with one set of rules for what invalidates it.
// A location is an indexed slot -- a base register and a byte offset -- or a
// direct-page imaginary register (keyed by that register, offset 0). Each
// mirror records how wide the holder is, the store that set it up (so a
// client can delete it if it is never read), and what the memory is -- a
// spill slot, a frame location, or anything a pointer may reach -- which
// decides which other stores may alias it.
//
// The tracker keeps the model; clients decide what to track and what to do
// with it (delete a reload, turn it into a transfer, delete a dead store).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_MC6809VALUETRACKER_H
#define LLVM_LIB_TARGET_MC6809_MC6809VALUETRACKER_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/CodeGen/Register.h"
#include <climits>

namespace llvm {

class MachineFrameInfo;
class MachineFunction;
class MachineInstr;
class TargetRegisterInfo;

class MC6809ValueTracker {
public:
  struct Key {
    Register Base;
    int Offset;
    bool operator==(const Key &O) const {
      return Base == O.Base && Offset == O.Offset;
    }
  };

  struct Mirror {
    Register Reg;                     // the holder; invalid once stale
    MachineInstr *Store = nullptr;    // the store that set it up, if any
    bool WasRead = false;             // the location was read since
    bool SourceKilled = false;        // that store killed its source
    int SpillFI = INT_MIN;            // spill slot, or INT_MIN
    bool IsFrame = false;             // frame location (not a spill slot)
    bool IsImaginary = false;         // a direct-page imaginary register
    MachineInstr *Est = nullptr;      // the access that set it up
  };

  MC6809ValueTracker(const MachineFunction &MF, const TargetRegisterInfo &TRI);

  /// Forget everything (a call, a branch, a return).
  void clear() { Entries.clear(); }

  /// The mirror at K, if any (possibly stale: check Reg).
  Mirror *find(const Key &K);

  /// Record that Reg holds the value at K, set up by Est (whose memory
  /// operand says what the memory is) and, if a store, by Store. Returns the
  /// mirror K had before (by value; Reg invalid if none), for a client to
  /// judge the earlier store dead.
  Mirror set(const Key &K, Register Reg, MachineInstr *Store,
             const MachineInstr &Est, bool SourceKilled = false);

  void markRead(const Key &K);
  void markAllRead();

  /// A def or kill of R: no mirror is held in R any more.
  void clearHolder(Register R);
  /// A def of R: a mirror based on R names other memory now.
  void clearBase(Register R);

  /// A store of Width bytes at Base+Off overwrote those bytes: drop every
  /// overlapping mirror on Base -- except one whose holder's bytes StoredReg
  /// wrote back to their own position.
  void storeOverlap(Register Base, int Off, int Width, Register StoredReg);

  /// Any store MI: drop every mirror it may alias but is not an indexed
  /// store on that mirror's own base (those go through storeOverlap). A
  /// spill-slot mirror goes only on a store to that slot; a frame mirror on
  /// a store that may write the frame; an imaginary-register mirror on a
  /// store naming that register; any other mirror on any store -- except
  /// that stores to a spill slot or an imaginary register drop nothing else.
  void storeAlias(const MachineInstr &MI, bool IndexedOnOwnBase,
                  Register OwnBase);

  /// The width in bytes of a mirror held in R.
  static int holderWidth(Register R);

  /// True if MI may write the stack frame (unknown footprint counts).
  static bool mayStoreToStackFrame(const MachineInstr &MI,
                                   const MachineFrameInfo &MFI);
  /// The spill slot every memory operand of MI names, or INT_MIN.
  static int spillSlotOf(const MachineInstr &MI, const MachineFrameInfo &MFI);

  /// A reload of Reg is being deleted, so Reg's value now reaches its uses
  /// from the definition before it: clear every kill of Reg (or a register
  /// overlapping it) between that definition and MI, and the definition's
  /// dead flag. Both only lengthen a live range, so they are always safe.
  static void clearKillsToPriorDef(MachineInstr &MI, Register Reg,
                                   const TargetRegisterInfo &TRI);

  /// All mirrors, for client walks.
  SmallVectorImpl<std::pair<Key, Mirror>> &entries() { return Entries; }

private:
  const MachineFunction &MF;
  const MachineFrameInfo &MFI;
  const TargetRegisterInfo &TRI;
  SmallVector<std::pair<Key, Mirror>, 8> Entries;

  bool isFrameLocation(const MachineInstr &Est, Register Base) const;
  bool namesOverlapping(const MachineInstr &MI, Register R) const;
};

} // namespace llvm

#endif
