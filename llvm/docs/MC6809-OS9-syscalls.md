# OS-9 system calls: what a program can reach from C

Every system call NitrOS-9 defines, with a box per level.  The point of the
table is to be filled in as the work happens, so it lists everything rather
than only what looked useful at the time.

| mark | meaning |
|------|---------|
| `[x]` | wrapped **and exercised** by a test that would fail if it broke |
| `[ ]` | not done, or wrapped but never exercised (the note says which) |
| ✗ | not available at that level, or not a call a program can make |

**On the Level 1 column**: the shims themselves are level-agnostic -- an
unticked L1 box beside a ticked L2 one means "never run there", not "not
written".  Nothing has been run on Level 1 yet.

Codes are the byte after `SWI2`.  The source of truth for names and numbers
is `defs/os9.d` in the NitrOS-9 tree.

## Process, memory and time

| Code | Call | What it is | L1 | L2 | Notes |
|------|------|-----------|:--:|:--:|-------|
| $00 | F$Link | link to a module already in memory | [ ] | [x] | `__os9_link`; the FP module |
| $01 | F$Load | load a module from a file and link it | [ ] | [x] | `__os9_load` |
| $02 | F$UnLink | drop a link to a module | [ ] | [x] | `__os9_unlink`; `_exit` gives back FPO9 |
| $03 | F$Fork | start a new process | [ ] | [ ] | wanted for `system()`, `popen()` |
| $04 | F$Wait | wait for a child to die | [ ] | [ ] | with F$Fork |
| $05 | F$Chain | replace this process with another module | [ ] | [ ] | the shape of `execve()` |
| $06 | F$Exit | terminate, with a status | [ ] | [x] | `_exit` |
| $07 | F$Mem | change the data area size | [ ] | [x] | the heap claim; live `sbrk` still wants it |
| $08 | F$Send | send a signal to a process | [ ] | [ ] | half of `kill()` |
| $09 | F$Icpt | install a signal handler | [ ] | [ ] | half of `signal()` |
| $0A | F$Sleep | suspend for a time | [ ] | [ ] | shim in `os9.h`, never exercised |
| $0B | F$SSpd | suspend a process | [ ] | [ ] | |
| $0C | F$ID | this process's ID, user number and priority | [ ] | [x] | `getpid()` |
| $0D | F$SPrior | set a process's priority | [ ] | [ ] | |
| $0E | F$SSWI | set a software-interrupt vector | [ ] | [ ] | |
| $0F | F$PErr | print an error message | [ ] | [ ] | the kernel's `perror()` |
| $10 | F$PrsNam | parse a path name | [ ] | [ ] | |
| $11 | F$CmpNam | compare two names, OS-9 rules | [ ] | [ ] | `__os9_name_eq` does this in C today |
| $12 | F$SchBit | search an allocation bitmap | [ ] | [ ] | |
| $13 | F$AllBit | allocate in a bitmap | [ ] | [ ] | |
| $14 | F$DelBit | deallocate in a bitmap | [ ] | [ ] | |
| $15 | F$Time | the current date and time | [ ] | [x] | `gettimeofday()` |
| $16 | F$STime | set the date and time | [ ] | [ ] | |
| $17 | F$CRC | compute an OS-9 CRC | [ ] | [ ] | |
| $18 | F$GPrDsc | copy a process descriptor | ✗ | [ ] | Level 2 only |
| $19 | F$GBlkMp | copy the system block map | ✗ | [ ] | Level 2 only |
| $1A | F$GModDr | copy the module directory | ✗ | [ ] | Level 2 only |
| $1B | F$CpyMem | copy memory from another task | ✗ | [ ] | Level 2 only |
| $1C | F$SUser | set the user ID | ✗ | [ ] | Level 2 only |
| $1D | F$UnLoad | unlink a module by name | ✗ | [ ] | Level 2 only |
| $1E | F$Alarm | set an alarm | ✗ | [ ] | Level 2 only |
| $21 | F$NMLink | link without mapping | ✗ | [ ] | Level 2 only |
| $22 | F$NMLoad | load without mapping | ✗ | [ ] | Level 2 only |
| $23 | F$Debug | drop into the system debugger | [ ] | [ ] | nothing a library should call |
| $25 | F$TPS | ticks per second | ✗ | [ ] | shim in `os9.h`, never exercised |
| $26 | F$TimAlm | per-process alarm | ✗ | [ ] | Level 2 only |
| $56 | F$XTime | time including fractions of a second | ✗ | [ ] | shim exists; wanted for `clock()`, `times()` -- but see the note below on whether a program may call it |

## I/O

| Code | Call | What it is | L1 | L2 | Notes |
|------|------|-----------|:--:|:--:|-------|
| $80 | I$Attach | attach a device | [ ] | [ ] | |
| $81 | I$Detach | detach a device | [ ] | [ ] | |
| $82 | I$Dup | duplicate a path number | [ ] | [ ] | `dup()`; shim exists, never exercised |
| $83 | I$Create | create a file | [ ] | [x] | `open()` with O_CREAT |
| $84 | I$Open | open a file or directory | [ ] | [x] | `open()`, `opendir()`, `access()` |
| $85 | I$MakDir | make a directory | [ ] | [x] | `mkdir()` |
| $86 | I$ChgDir | change the working or execution directory | [ ] | [x] | `chdir()` |
| $87 | I$Delete | delete a file or directory | [ ] | [x] | `unlink()`, `rmdir()` |
| $88 | I$Seek | set the file position (X:U) | [ ] | [x] | `lseek()`; asm stub, U is the data base |
| $89 | I$Read | read bytes | [ ] | [x] | `read()` |
| $8A | I$Write | write bytes | [ ] | [x] | `write()` |
| $8B | I$ReadLn | read a line, with editing | [ ] | [ ] | shim exists, never exercised |
| $8C | I$WritLn | write a line, with editing | [ ] | [ ] | shim exists, never exercised |
| $8D | I$GetStt | get path status | [ ] | [x] | `fstat()`, `isatty()`, `statvfs()`, `getcwd()` |
| $8E | I$SetStt | set path status | [ ] | [x] | `ftruncate()` via SS.Size |
| $8F | I$Close | close a path | [ ] | [x] | `close()` |
| $90 | I$DeletX | delete from the execution directory | [ ] | [ ] | |
| $91 | I$ModDsc | modify a device descriptor in memory | [ ] | [ ] | |

### Status calls worth their own boxes

`I$GetStt`/`I$SetStt` are a family; these are the codes used or wanted.

| Code | Status call | What it is | L1 | L2 | Notes |
|------|-------------|-----------|:--:|:--:|-------|
| $00 | SS.Opt | the 32-byte path options | [ ] | [x] | device type, for `isatty()` and `fstat()` |
| $02 | SS.Size | file size; set truncates or extends | [ ] | [x] | X:U pair |
| $05 | SS.Pos | current position | [ ] | [ ] | `lseek(SEEK_CUR)` uses it |
| $06 | SS.EOF | at end of file? | [ ] | [ ] | |
| $0E | SS.DevNm | the device a path is on | [ ] | [x] | `getcwd()`, `statvfs()` |
| $0F | SS.FD | the file descriptor sector | [ ] | [x] | `fstat()` dates, attributes, link count |
| $10 | SS.Ticks | lockout duration | [ ] | [ ] | |
| $11 | SS.Lock | lock or release a record | [ ] | [ ] | |
| $20 | SS.FDInf | file descriptor info | [ ] | [ ] | |
| $26 | SS.DSize | disk size (RBF) | [ ] | [ ] | `statvfs()` reads the ident sector instead |

## System-state calls -- not callable from a program

$27-$33 exist at both levels; $34-$57 and $70-$71 are Level 2 only.  These
are the kernel's own service requests: F$VIRQ, F$SRqMem, F$SRtMem, F$IRQ,
F$IOQu, F$AProc, F$NProc, F$VModul, F$Find64, F$All64, F$Ret64, F$SSvc,
F$IODel, F$SLink, F$Boot, F$BtMem, F$GProcP, F$Move, F$AllRAM, F$AllImg,
F$DelImg, F$SetImg, F$FreeLB, F$FreeHB, F$AllTsk, F$DelTsk, F$SetTsk,
F$ResTsk, F$RelTsk, F$DATLog, F$DATTmp, F$LDAXY, F$LDAXYP, F$LDDDXY,
F$LDABX, F$STABX, F$AllPrc, F$DelPrc, F$ELink, F$FModul, F$MapBlk,
F$ClrBlk, F$DelRAM, F$GCMDir, F$AlHRAM, F$ReBoot, F$CRCMod, F$VBlock,
F$RegDmp, F$NVRAM.

| | L1 | L2 |
|---|:--:|:--:|
| all of the above | ✗ | ✗ |

They run in system state, on the kernel's stack and its data.  A program
that calls one either gets an error or takes the machine down with it.

**F$XTime ($56) is the open question.**  It is listed with the user calls
above because that is where the plan for `clock()` and `times()` puts it,
but it sits in this reserved range, and nothing in the whole NitrOS-9 tree
calls it -- no command, no module.  Whether a program in user state can
reach it at all is therefore unverified, and worth a five-line probe before
any work is planned on top of it.

## Filling this in

Tick a box only when something would fail if the call broke -- a test in
`compiler-rt/test/mc6809-os9-runtime/`, or a picolibc test that reaches it
through the C function named in the notes.  A wrapper in `<os9.h>` with
nothing calling it is not a tick; several of those are here already, and
they are marked as such.
