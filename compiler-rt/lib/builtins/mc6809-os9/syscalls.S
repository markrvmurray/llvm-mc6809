//===-- syscalls.S - OS-9 / NitrOS-9 syscall stubs for picolibc ----------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// Minimal OS-9 syscall layer.
//
// LLVM-MC6809 C-ABI summary (per CC_MC6809 / RetCC_MC6809 in
// MC6809CallingConv.td):
//
//   Pointers       -> IX (first one)
//   i16 (non-ptr)  -> IX (first one; subsequent -> stack)
//   i8 / i1        -> B  (first one; subsequent -> stack)
//   else / spilled -> stack at offset N,s after the return address
//
//   Return: i16/ptr -> IX,  i8 -> B
//
// OS-9 syscall ABI (per llvm/docs/MC6809-OS9.md):
//
//   Entry registers vary per syscall.
//   Exit: CC.C clear = success, CC.C set = error with errno in B.
//
// Error helper conventions:
//
//   _osret/_oserr   Direct OS-9 wrapper convention used by the
//                   Kreider/CMOC library layer: success returns 0,
//                   failure returns the OS-9 error code and stores it in
//                   the per-process runtime errno slot.
//   _sysret/_os9err POSIX-style C wrapper convention: success returns 0 or
//                   a syscall-specific value, failure returns -1 and stores
//                   the OS-9 error code in the runtime errno slot.
//
//===----------------------------------------------------------------------===//

	.set	I_Seek,   0x88
	.set	I_Read,   0x89
	.set	I_Write,  0x8A
	.set	I_ReadLn, 0x8B
	.set	I_WritLn,0x8C
	.set	E_EOF,    0xD3

	.section .text,"ax",@progbits


//===----------------------------------------------------------------------===//
// Shared error helpers.
//===----------------------------------------------------------------------===//

	.globl	_oserr
	.type	_oserr,@function
_oserr:
	clra				// D = OS-9 error byte from B
	pshs	d			// Save errno while computing its U-relative slot.
	ldd	#__errno_offset
	leax	d,u
	puls	d
	std	,x
	tfr	d,x			// Direct OS-9 wrappers return errno.
	rts
	.size	_oserr, .-_oserr

	.globl	_osret
	.type	_osret,@function
_osret:
	bcs	_oserr
	ldx	#0
	rts
	.size	_osret, .-_osret

	.globl	_os9err
	.type	_os9err,@function
_os9err:
	clra				// D = OS-9 error byte from B
	pshs	d			// Save errno while computing its U-relative slot.
	ldd	#__errno_offset
	leax	d,u
	puls	d
	std	,x
	ldx	#-1
	rts
	.size	_os9err, .-_os9err

	.globl	_sysret
	.type	_sysret,@function
_sysret:
	bcs	_os9err
	ldx	#0
	rts
	.size	_sysret, .-_sysret


//===----------------------------------------------------------------------===//
// _exit / F$Exit now live in syscalls.c (inline-asm C).
//===----------------------------------------------------------------------===//


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
	.byte	I_Write			// I$Write
	bcs	_os9err
	tfr	y,x			// X = bytes written
	rts
	.size	_write, .-_write


//===----------------------------------------------------------------------===//
// int _read(int fd, void *buf, int n)
//
//   C-ABI in:   X = fd,  2,s = buf,  4,s = n
//   OS-9 entry: A = path#,  X = buf,  Y = nbytes
//   OS-9 exit:  CC.C set = error (B = errno); else Y = bytes read
//   Return:     X = bytes read (success), 0 on EOF, or -1 (error)
//===----------------------------------------------------------------------===//

	.globl	_read
	.type	_read,@function
_read:
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	2,s			// X = buf
	ldy	4,s			// Y = nbytes
	swi2
	.byte	I_Read			// I$Read
	bcs	.Lread_err
	tfr	y,x
	rts
.Lread_err:
	cmpb	#E_EOF			// EOF is a successful zero-byte read.
	bne	_os9err
	ldx	#0
	rts
	.size	_read, .-_read


//===----------------------------------------------------------------------===//
// Direct OS-9 I/O helpers from the Kreider/CMOC library shape.
//
// int _os_read(int fd, void *buf, int *countp)
// int _os_readln(int fd, void *buf, int *countp)
// int _os_write(int fd, const void *buf, int *countp)
// int _os_writeln(int fd, const void *buf, int *countp)
//
//   C-ABI in:   X = fd,  2,s = buf,  4,s = countp
//   Return:     X = 0 on success, OS-9 error code on failure
//   Side effect: *countp is updated with the actual byte count on success.
//
// These are intentionally the low-level OS-9 convention.  Higher-level
// POSIX-style wrappers (_read/_write) continue to return byte counts or -1.
//===----------------------------------------------------------------------===//

	.globl	_os_read
	.type	_os_read,@function
_os_read:
	pshs	y
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	4,s			// X = buf
	ldy	[6,s]			// Y = requested byte count
	swi2
	.byte	I_Read			// I$Read
	bcs	.Los_read_err
.Los_read_save:
	ldx	6,s			// X = countp
	sty	,x
	puls	y
	ldx	#0
	rts
.Los_read_err:
	cmpb	#E_EOF
	bne	.Los_read_oserr
	ldy	#0
	bra	.Los_read_save
.Los_read_oserr:
	puls	y
	lbra	_oserr
	.size	_os_read, .-_os_read

	.globl	_os_readln
	.type	_os_readln,@function
_os_readln:
	pshs	y
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	4,s			// X = buf
	ldy	[6,s]			// Y = requested byte count
	swi2
	.byte	I_ReadLn		// I$ReadLn
	bcs	.Los_read_err
	bra	.Los_read_save
	.size	_os_readln, .-_os_readln

	.globl	_os_write
	.type	_os_write,@function
_os_write:
	pshs	y
	ldy	[6,s]			// Y = requested byte count
	beq	.Los_write_success
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	4,s			// X = buf
	swi2
	.byte	I_Write			// I$Write
	bcs	.Los_write_err
	ldx	6,s			// X = countp
	sty	,x
.Los_write_success:
	puls	y
	ldx	#0
	rts
.Los_write_err:
	puls	y
	lbra	_oserr
	.size	_os_write, .-_os_write

	.globl	_os_writeln
	.type	_os_writeln,@function
_os_writeln:
	pshs	y
	ldy	[6,s]			// Y = requested byte count
	beq	.Los_writeln_success
	tfr	x,d
	tfr	b,a			// A = fd
	ldx	4,s			// X = buf
	swi2
	.byte	I_WritLn		// I$WritLn
	bcs	.Los_writeln_err
	ldx	6,s			// X = countp
	sty	,x
.Los_writeln_success:
	puls	y
	ldx	#0
	rts
.Los_writeln_err:
	puls	y
	lbra	_oserr
	.size	_os_writeln, .-_os_writeln


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

	.globl	I$Write
	.set	I$Write, _write
	.globl	I$Read
	.set	I$Read, _read
