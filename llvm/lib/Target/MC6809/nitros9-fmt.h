//===-- nitros9-fmt.h - OS-9 / NitrOS-9 module-format constants -*- C++ -*-===//
//
// Part of the LLVM Project, under Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Single source of truth for the constants needed to emit and validate OS-9
// (NitrOS-9) program modules from the MC6809 backend.
//
// All values are cross-checked against the canonical references:
//   ~/Documents/NitrOS-9/nitros9/defs/os9.d                          (spec)
//   ~/Documents/NitrOS-9/nitros9/level1/modules/kernel/fcrc.asm      (CRC alg)
//   ~/Documents/NitrOS-9/nitros9/level1/modules/kernel/fvmodul.asm   (validation)
//   ~/Documents/lwtools/lwlink/output.c                              (emission)
//
// lwtools' do_output_os9() in lwlink/output.c is the authoritative
// reference implementation for OS-9 module emission; CRC::step() below
// is identical to its os9crc() byte-for-byte.  The trailing 3 CRC bytes
// of an emitted module equal the bitwise complement of the running CRC
// over the body+name, per lwlink/output.c:559-561.
//
// Bug #163, Phase 1 (see llvm/docs/MC6809-OS9.md).
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_LIB_TARGET_MC6809_NITROS9_FMT_H
#define LLVM_LIB_TARGET_MC6809_NITROS9_FMT_H

#include <cstdint>

// The identifiers below intentionally mirror the Microware OS-9 spec names
// (F$Exit, I$Read, S$Kill, E$BMCRC ...) translated to C++.  Renaming them to
// LLVM camelCase would lose the cross-reference value, so the
// readability-identifier-naming check is suppressed for the constants block.
// NOLINTBEGIN(readability-identifier-naming)

namespace llvm {
namespace MC6809 {
namespace OS9 {

// =============================================================================
// Module header layout
// =============================================================================
//
// All multi-byte fields are BIG-ENDIAN.  The 13-byte header below is followed
// by 4 bytes of program-module-specific fields (M$Exec, M$Mem) for a total of
// 17 bytes before the body proper.  Driver / device modules add further
// type-specific bytes.
//
//   offset  size  field            equivalent os9.d label
//   ------  ----  ---------------- ---------------------------
//   0       2     sync bytes       M$ID1 ($87), M$ID2 ($CD)
//   2       2     module size      M$Size  (incl. header + body + 3-byte CRC)
//   4       2     name offset      M$Name  (from module start)
//   6       1     type / language  M$Type  (high nibble: type, low: lang)
//   7       1     attr / revision  M$Revs  (high nibble: attr, low: rev)
//   8       1     header parity    M$Parity (XOR of bytes [0..8] == $FF)
//   9       2     exec offset      M$Exec  (program entry, from module start)
//   11      2     mem size         M$Mem   (process data area to allocate)
//   ...     ...   module body
//   end-3   3     24-bit CRC

namespace Header {

constexpr unsigned OffsetSync       = 0;
constexpr unsigned OffsetSize       = 2;
constexpr unsigned OffsetName       = 4;
constexpr unsigned OffsetTypeLang   = 6;
constexpr unsigned OffsetAttrRev    = 7;
constexpr unsigned OffsetParity     = 8;
constexpr unsigned OffsetExec       = 9;
constexpr unsigned OffsetMem        = 11;

// Total size of the universal header (through parity byte).
constexpr unsigned UniversalSize    = 9;

// Program modules add M$Exec and M$Mem after the universal header.
constexpr unsigned ProgramHeaderSize = 13;

// Trailing CRC size (always 3 bytes).
constexpr unsigned CRCSize          = 3;

// Maximum module size dictated by 16-bit size field.
constexpr unsigned MaxModuleSize    = 0xFFFF;

} // namespace Header

// =============================================================================
// Sync bytes (M$ID1, M$ID2 in os9.d:821-822)
// =============================================================================

constexpr uint8_t SyncByte0 = 0x87;
constexpr uint8_t SyncByte1 = 0xCD;
constexpr uint16_t Sync     = 0x87CD;

// =============================================================================
// Type / Language byte (high nibble = type, low nibble = language)
// =============================================================================
//
// Values from os9.d:847-853 and surrounding region.  The high-nibble type is
// what the OS-9 loader uses to dispatch (program vs. device-driver vs. ...);
// the low-nibble language tells the loader what kind of code it's looking at
// (which matters chiefly for 6809-vs-6309 module compatibility).

namespace Type {

// High-nibble values, pre-shifted into bits 4..7 (i.e., (Type::Prgrm | lang)
// is the raw byte value).
constexpr uint8_t Prgrm  = 0x10; // user program module
constexpr uint8_t Sbrtn  = 0x20; // subroutine module
constexpr uint8_t Multi  = 0x30; // multi-module
constexpr uint8_t Data   = 0x40; // data module
constexpr uint8_t TrapLib = 0xB0; // user trap library  (Float09.bin is one)
constexpr uint8_t Systm  = 0xC0; // system module
constexpr uint8_t FlMgr  = 0xD0; // file manager
constexpr uint8_t Drivr  = 0xE0; // device driver
constexpr uint8_t Devic  = 0xF0; // device descriptor

} // namespace Type

namespace Language {

// Low-nibble values.
constexpr uint8_t Objct    = 0x01; // 6809 native object code
constexpr uint8_t ICode    = 0x02; // Basic09 intermediate code
constexpr uint8_t PCode    = 0x03; // Pascal P-code
constexpr uint8_t CCode    = 0x04; // C intermediate code
constexpr uint8_t CblCode  = 0x05; // Cobol I-code
constexpr uint8_t FrtnCode = 0x06; // Fortran I-code
constexpr uint8_t Obj6309  = 0x07; // 6309 native code (NitrOS-9 extension)

} // namespace Language

// Convenience: assemble a Type/Language byte.
constexpr uint8_t makeTypeLang(uint8_t Type, uint8_t Lang) {
  return Type | (Lang & 0x0F);
}

// =============================================================================
// Attribute / Revision byte (high nibble = attributes, low = revision)
// =============================================================================
//
// Bits, per os9.d:859-874.  Recommended default for emitted program modules:
//   ReEnt | ModNat | rev=0  =>  $A0
// Re-entrant native code, revision zero.  Bumping the revision lets a newer
// module supersede an older one with the same name in the module directory.

namespace Attribute {

// High-nibble bit values.
constexpr uint8_t ReEnt    = 0x80; // re-entrant (multiple processes can share)
constexpr uint8_t ModProt  = 0x40; // memory-protected (Level-2 only)
constexpr uint8_t ModNat   = 0x20; // native code (not interpreted)
constexpr uint8_t BufWrits = 0x10; // buffered writes (device modules)
// BufReads bit doesn't appear in os9.d; bit 0x10 may have multiple meanings
// in different module-type contexts.

constexpr uint8_t RevisionMask = 0x0F;

// Recommended default for our program-module emit:
constexpr uint8_t DefaultProgramAttr = ReEnt | ModNat;

} // namespace Attribute

// =============================================================================
// CRC-24 algorithm constants
// =============================================================================
//
// The OS-9 CRC-24 polynomial is 0x800063 (Microware spec).  The canonical
// implementation is at nitros9/level1/modules/kernel/fcrc.asm and the
// validation at nitros9/level1/modules/kernel/fvmodul.asm:223-244.
//
// The algorithm is initialised to 0xFFFFFF.  For a VALID module, processing
// the entire module (including the trailing 3 CRC bytes) leaves the running
// state at exactly ValidMagic.
//
// For EMISSION: process body (everything before the trailing 3 CRC bytes),
// take the bitwise complement of the resulting state, and write those 3
// bytes as the module's CRC tail.  This is the equivalent of the standard
// "CRC of data || CRC == 0" property, but in OS-9's specific encoding.

namespace CRC {

constexpr uint32_t Init       = 0xFFFFFF;
constexpr uint32_t ValidMagic = 0x800FE3;   // CRCCon1<<16 | CRCCon23
constexpr uint32_t Polynomial = 0x800063;   // for reference (LFSR formulation)

// Per-byte step.  Direct translation of fcrc.asm CRCAlgo.
//   crc:  current 24-bit state (in low 24 bits of uint32_t)
//   b:    next input byte
// Returns: updated 24-bit state.
inline uint32_t step(uint32_t Crc, uint8_t B) {
  uint8_t C0 = (Crc >> 16) & 0xFF;
  uint8_t C1 = (Crc >> 8) & 0xFF;
  uint8_t C2 = Crc & 0xFF;
  B ^= C0;
  C0 = C1;
  C1 = C2;
  C1 ^= (B >> 7) & 0xFF;
  C2 = (B << 1) & 0xFF;
  C1 ^= (B >> 2) & 0xFF;
  C2 ^= (B << 6) & 0xFF;
  B = (B ^ (B << 1)) & 0xFF;
  B = (B ^ (B << 2)) & 0xFF;
  B = (B ^ (B << 4)) & 0xFF;
  if (B & 0x80) {
    C0 ^= 0x80;
    C2 ^= 0x21;
  }
  return (uint32_t(C0) << 16) | (uint32_t(C1) << 8) | C2;
}

} // namespace CRC

// =============================================================================
// Header parity
// =============================================================================
//
// The parity byte (offset 8) is chosen so that XOR-ing the first 9 bytes of
// the header together yields 0xFF.  See ChkMHPar at fvmodul.asm:209-221.
//
// At emission time:
//   parity = 0xFF ^ B0 ^ B1 ^ B2 ^ B3 ^ B4 ^ B5 ^ B6 ^ B7
//
// At validation time:
//   header is valid iff (B0 ^ B1 ^ ... ^ B8) == 0xFF.

inline uint8_t computeHeaderParity(const uint8_t Header[Header::UniversalSize - 1]) {
  uint8_t Acc = 0xFF;
  for (unsigned I = 0; I < Header::UniversalSize - 1; ++I)
    Acc ^= Header[I];
  return Acc;
}

// =============================================================================
// OS-9 syscall numbers
// =============================================================================
//
// SWI2 + FCB <code> dispatches a syscall.  These are the ones a hosted
// picolibc runtime needs.  Full enumeration is in os9.d:152-307.

namespace Syscall {

// System calls (F$xxx).  Numbers are the FCB byte following the SWI2.
constexpr uint8_t F_Link     = 0x00; // link to in-memory module
constexpr uint8_t F_Load     = 0x01; // load module from file
constexpr uint8_t F_UnLink   = 0x02; // decrement module link count
constexpr uint8_t F_Fork     = 0x03; // create child process
constexpr uint8_t F_Wait     = 0x04; // wait for child
constexpr uint8_t F_Chain    = 0x05; // replace current process with new module
constexpr uint8_t F_Exit     = 0x06; // terminate process
constexpr uint8_t F_Mem      = 0x07; // set process memory size
constexpr uint8_t F_Send     = 0x08; // send signal to process
constexpr uint8_t F_Icpt     = 0x09; // install signal-intercept handler
constexpr uint8_t F_Sleep    = 0x0A; // pause for N ticks
constexpr uint8_t F_SPrior   = 0x0B; // set process priority
constexpr uint8_t F_ID       = 0x0C; // get process ID / user-ID
constexpr uint8_t F_SUser    = 0x0D; // set user-ID
constexpr uint8_t F_Time     = 0x15; // get system time (Y/M/D/H/M/S)
constexpr uint8_t F_STime    = 0x16; // set system time
constexpr uint8_t F_TLink    = 0x17; // link by trap-handler vector

// I/O calls (I$xxx).  Path numbers are 1-based (path 0 is invalid; paths 1/2/3
// are conventionally stdin/stdout/stderr inherited from the parent shell).
constexpr uint8_t I_Attach   = 0x80; // attach to device
constexpr uint8_t I_Detach   = 0x81; // detach from device
constexpr uint8_t I_Dup      = 0x82; // duplicate path number
constexpr uint8_t I_Create   = 0x83; // create file
constexpr uint8_t I_Open     = 0x84; // open file
constexpr uint8_t I_MakDir   = 0x85; // make directory
constexpr uint8_t I_ChgDir   = 0x86; // change directory
constexpr uint8_t I_Delete   = 0x87; // delete file
constexpr uint8_t I_Seek     = 0x88; // seek path
constexpr uint8_t I_Read     = 0x89; // raw read from path
constexpr uint8_t I_Write    = 0x8A; // raw write to path
constexpr uint8_t I_ReadLn   = 0x8B; // read line (CR-terminated)
constexpr uint8_t I_WritLn   = 0x8C; // write line (appends CR)
constexpr uint8_t I_GetStt   = 0x8D; // get path status
constexpr uint8_t I_SetStt   = 0x8E; // set path status
constexpr uint8_t I_Close    = 0x8F; // close path

} // namespace Syscall

// =============================================================================
// Error codes (subset)
// =============================================================================
//
// Returned in the B register when CC.C is set after a syscall.  Full list at
// os9.d:380+.  picolibc's errno mapping will translate these to POSIX errnos.

namespace Error {

constexpr uint8_t E_BPNum   = 0x01; // bad path number
constexpr uint8_t E_BPNam   = 0x02; // bad path name
constexpr uint8_t E_PNNF    = 0x03; // path name not found
constexpr uint8_t E_SLF     = 0x04; // segment list full
constexpr uint8_t E_CEF     = 0x05; // creating existing file
constexpr uint8_t E_IBA     = 0x06; // illegal block address
constexpr uint8_t E_HangUp  = 0x07; // carrier lost
constexpr uint8_t E_MNF     = 0x08; // module not found
constexpr uint8_t E_DelSP   = 0x09; // tried to delete stack pointer
constexpr uint8_t E_IPrcID  = 0x0A; // illegal process id
constexpr uint8_t E_BMCRC   = 0x0F; // bad module CRC
constexpr uint8_t E_USigP   = 0x10; // un-installed signal pending
constexpr uint8_t E_NES     = 0x11; // non-existing segment
constexpr uint8_t E_FNA     = 0x12; // file not accessible
constexpr uint8_t E_BPAdr   = 0x13; // bad page address
constexpr uint8_t E_BPath   = 0x14; // bad path
constexpr uint8_t E_BMode   = 0x15; // bad mode
constexpr uint8_t E_DirFul  = 0x16; // directory full
constexpr uint8_t E_MemFul  = 0x17; // memory full
constexpr uint8_t E_DNE     = 0xC9; // does not exist

} // namespace Error

// =============================================================================
// Signal numbers (subset)
// =============================================================================
//
// Used by F$Send and the F$Icpt handler.  See os9.d:325-338.

namespace Signal {

constexpr uint8_t S_Kill   = 0x00; // un-interceptable kill
constexpr uint8_t S_Wake   = 0x01; // wake up
constexpr uint8_t S_Abort  = 0x02; // keyboard-abort
constexpr uint8_t S_Intrpt = 0x03; // keyboard-interrupt
constexpr uint8_t S_Window = 0x04; // window change / hangup

} // namespace Signal

} // namespace OS9
} // namespace MC6809
} // namespace llvm

// NOLINTEND(readability-identifier-naming)

#endif // LLVM_LIB_TARGET_MC6809_NITROS9_FMT_H
