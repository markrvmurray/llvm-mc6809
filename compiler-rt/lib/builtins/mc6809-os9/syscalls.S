//===-- syscalls.S - OS-9 / NitrOS-9 syscall stubs for picolibc ----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Bug #163 Phase 2: minimal syscall layer.
//
// Three core syscalls are landed here; this is enough for printf /
// putchar / fputs (writes) and getchar / fread from stdin (reads),
// plus clean process termination.  Follow-up commits will land:
//
//   - _sbrk         needs a saved data-area-base; will land with the
//                   CRT/syscall layer that wires direct-page slots
//                   reserved for OS-9 runtime state.
//   - _gettimeofday needs F$Time (returns Y/M/D/H/M/S in a buffer) +
//                   a Julian-day conversion to UNIX timestamp.
//
// LLVM-MC6809 C-ABI summary (per CC_MC6809 / RetCC_MC6809 in
// MC6809CallingConv.td):
//
//   Pointers       → IX (first one)
//   i16 (non-ptr)  → IX (first one; subsequent → stack)
//   i8 / i1        → B  (first one; subsequent → stack)
//   else / spilled → stack at offset N,s after the return address
//
//   Return: i16/ptr → IX,  i8 → B
//
// OS-9 syscall ABI (per llvm/docs/MC6809-OS9.md §A.2):
//
//   Entry registers vary per syscall — see each stub below.
//   Exit:  CC.C clear = success (return values in registers per call)
//          CC.C set   = error, errno is in B
//
// errno wiring: NOT yet plumbed.  At this Phase 2 PoC, errors return
// -1 in X (per picolibc's syscall convention) but the actual error
// number is discarded.  Picolibc's errno will be wired in the
// Phase 4 picolibc-on-OS9 commit.
//
//===----------------------------------------------------------------------===//

	.section .text,"ax",@progbits


//===----------------------------------------------------------------------===//
// void _exit(int status) __attribute__((noreturn))
//
//   C-ABI:  status   → X
//   OS-9:   F$Exit   → B = status byte
//
// F$Exit never returns.  We don't bother with the int-return register.
//===----------------------------------------------------------------------===//

	.globl	_exit
	.type	_exit,@function
_exit:
	tfr	x,d			// D = status; B is low byte already
	swi2
	.byte	0x06			// F$Exit
	// never reached; trap if it ever returns
	bra	.
	.size	_exit, .-_exit


//===----------------------------------------------------------------------===//
// int _write(int fd, const char *buf, int n)
//
//   C-ABI in:   X = fd,  2,s = buf,  4,s = n
//   OS-9 entry: A = path#,  X = buf,  Y = nbytes
//   OS-9 exit:  CC.C set = error (B = errno); else Y = bytes written
//   Return:     X = bytes written (success) or -1 (error)
//===----------------------------------------------------------------------===//

	.globl	_write
	.type	_write,@function
_write:
	tfr	x,d			// D = fd
	tfr	b,a			// A = fd low byte = path #
	ldx	2,s			// X = buf
	ldy	4,s			// Y = nbytes
	swi2
	.byte	0x8A			// I$Write
	bcs	.Lwrite_err
	tfr	y,x			// X = bytes written (return value)
	rts
.Lwrite_err:
	ldx	#-1
	rts
	.size	_write, .-_write


//===----------------------------------------------------------------------===//
// int _read(int fd, void *buf, int n)
//
//   C-ABI in:   X = fd,  2,s = buf,  4,s = n
//   OS-9 entry: A = path#,  X = buf,  Y = nbytes
//   OS-9 exit:  CC.C set = error (B = errno); else Y = bytes read
//   Return:     X = bytes read (success) or -1 (error)
//===----------------------------------------------------------------------===//

	.globl	_read
	.type	_read,@function
_read:
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	2,s			// X = buf
	ldy	4,s			// Y = nbytes
	swi2
	.byte	0x89			// I$Read
	bcs	.Lread_err
	tfr	y,x
	rts
.Lread_err:
	ldx	#-1
	rts
	.size	_read, .-_read


//===----------------------------------------------------------------------===//
// NitrOS-9-native aliases.
//
// The OS-9 ecosystem refers to these syscalls as `F$Exit`, `I$Read`,
// `I$Write` — same address, just the native name.  Ported NitrOS-9
// source can use these names verbatim.  Requires the C consumer to
// have `-fdollars-in-identifiers` enabled (the mc6809-unknown-os9
// driver auto-enables it; see clang/lib/Driver/ToolChains/
// MC6809Toolchain.cpp).
//===----------------------------------------------------------------------===//

	.globl	F$Exit
	.set	F$Exit, _exit
	.globl	I$Write
	.set	I$Write, _write
	.globl	I$Read
	.set	I$Read, _read
