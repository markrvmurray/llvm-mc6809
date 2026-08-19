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
written".  What is ticked there comes from two suites: the runtime cases, green at
both levels (18/18), which cover start-up, the module calls and the
console; and the picolibc suite, which now runs at Level 1 too
(`NITROS9_LEVEL=1`) and reached 102 OK there.  What is still unticked
either was not reached by a passing Level 1 test, or belongs to a test
that does not fit -- Level 1 has no DAT, so 162 of them are too big for
the one 64K space it shares with the system.

Codes are the byte after `SWI2`.  The source of truth for names and numbers
is `defs/os9.d` in the NitrOS-9 tree.

## Process, memory and time

| Code | Call | What it is | L1 | L2 | Notes |
|------|------|-----------|:--:|:--:|-------|
| $00 | F$Link | link to a module already in memory | [x] | [x] | `__os9_link`; verified with the module already in memory -- on a fresh boot this fails and F$Load does the work |
| $01 | F$Load | load a module from a file and link it | [x] | [x] | `__os9_load` |
| $02 | F$UnLink | drop a link to a module | [x] | [x] | `__os9_unlink`; `_exit` gives back FPO9, checked with `mdir` at both levels |
| $03 | F$Fork | start a new process | [ ] | [ ] | wanted for `system()`, `popen()` |
| $04 | F$Wait | wait for a child to die | [ ] | [ ] | with F$Fork |
| $05 | F$Chain | replace this process with another module | [ ] | [ ] | the shape of `execve()` |
| $06 | F$Exit | terminate, with a status | [x] | [x] | `_exit` |
| $07 | F$Mem | change the data area size | [x] | [x] | the heap claim; live `sbrk` still wants it |
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
| $15 | F$Time | the current date and time | [x] | [x] | `gettimeofday()` |
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
| $83 | I$Create | create a file | [x] | [x] | `open()` with O_CREAT |
| $84 | I$Open | open a file or directory | [x] | [x] | `open()`, `opendir()`, `access()`; L1 via the runtime suite |
| $85 | I$MakDir | make a directory | [ ] | [x] | `mkdir()` |
| $86 | I$ChgDir | change the working or execution directory | [ ] | [x] | `chdir()` |
| $87 | I$Delete | delete a file or directory | [x] | [x] | `unlink()`, `rmdir()` |
| $88 | I$Seek | set the file position (X:U) | [x] | [x] | `lseek()`; asm stub, U is the data base |
| $89 | I$Read | read bytes | [x] | [x] | `read()` |
| $8A | I$Write | write bytes | [x] | [x] | `write()`; every runtime case prints through it |
| $8B | I$ReadLn | read a line, with editing | [ ] | [ ] | shim exists, never exercised |
| $8C | I$WritLn | write a line, with editing | [ ] | [ ] | shim exists, never exercised |
| $8D | I$GetStt | get path status | [x] | [x] | `fstat()`, `isatty()`, `statvfs()`, `getcwd()` |
| $8E | I$SetStt | set path status | [ ] | [x] | `ftruncate()` via SS.Size |
| $8F | I$Close | close a path | [x] | [x] | `close()`; L1 via the runtime suite |
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

These run in system state, on the kernel's stack and its data.  A program
that calls one either gets an error or takes the machine down with it, so
every box is crossed; they are listed for completeness, and so that nobody
has to work out for themselves which calls a program may actually use.

$56 (F$XTime) belongs to this range too but is listed with the user calls
above, because the plan for `clock()` and `times()` rests on it -- see the
note there about whether a program may call it at all.

| Code | Call | What it is | L1 | L2 | Notes |
|------|------|-----------|:--:|:--:|-------|
| $27 | F$VIRQ | install or delete a virtual IRQ | ✗ | ✗ | system state |
| $28 | F$SRqMem | system memory request | ✗ | ✗ | system state |
| $29 | F$SRtMem | system memory return | ✗ | ✗ | system state |
| $2A | F$IRQ | enter the IRQ polling table | ✗ | ✗ | system state |
| $2B | F$IOQu | enter the I/O queue | ✗ | ✗ | system state |
| $2C | F$AProc | enter the active process queue | ✗ | ✗ | system state |
| $2D | F$NProc | start the next process | ✗ | ✗ | system state |
| $2E | F$VModul | validate a module | ✗ | ✗ | system state |
| $2F | F$Find64 | find a process or path descriptor | ✗ | ✗ | system state |
| $30 | F$All64 | allocate a process or path descriptor | ✗ | ✗ | system state |
| $31 | F$Ret64 | return a process or path descriptor | ✗ | ✗ | system state |
| $32 | F$SSvc | initialise the service request table | ✗ | ✗ | system state |
| $33 | F$IODel | delete an I/O module | ✗ | ✗ | system state |
| $34 | F$SLink | system link | ✗ | ✗ | Level 2 only, system state |
| $35 | F$Boot | bootstrap the system | ✗ | ✗ | Level 2 only, system state |
| $36 | F$BtMem | bootstrap memory request | ✗ | ✗ | Level 2 only, system state |
| $37 | F$GProcP | get a process pointer | ✗ | ✗ | Level 2 only, system state |
| $38 | F$Move | move data, low bound first | ✗ | ✗ | Level 2 only, system state |
| $39 | F$AllRAM | allocate RAM blocks | ✗ | ✗ | Level 2 only, system state |
| $3A | F$AllImg | allocate image RAM blocks | ✗ | ✗ | Level 2 only, system state |
| $3B | F$DelImg | deallocate image RAM blocks | ✗ | ✗ | Level 2 only, system state |
| $3C | F$SetImg | set a process DAT image | ✗ | ✗ | Level 2 only, system state |
| $3D | F$FreeLB | get a free low block | ✗ | ✗ | Level 2 only, system state |
| $3E | F$FreeHB | get a free high block | ✗ | ✗ | Level 2 only, system state |
| $3F | F$AllTsk | allocate a task number | ✗ | ✗ | Level 2 only, system state |
| $40 | F$DelTsk | deallocate a task number | ✗ | ✗ | Level 2 only, system state |
| $41 | F$SetTsk | set a task's DAT registers | ✗ | ✗ | Level 2 only, system state |
| $42 | F$ResTsk | reserve a task number | ✗ | ✗ | Level 2 only, system state |
| $43 | F$RelTsk | release a task number | ✗ | ✗ | Level 2 only, system state |
| $44 | F$DATLog | DAT block/offset to logical address | ✗ | ✗ | Level 2 only, system state |
| $45 | F$DATTmp | make a temporary DAT image (obsolete) | ✗ | ✗ | Level 2 only, system state |
| $46 | F$LDAXY | load A from [X,[Y]] | ✗ | ✗ | Level 2 only, system state |
| $47 | F$LDAXYP | load A from [X+,[Y]] | ✗ | ✗ | Level 2 only, system state |
| $48 | F$LDDDXY | load D from [D+X,[Y]] | ✗ | ✗ | Level 2 only, system state |
| $49 | F$LDABX | load A from 0,X in task B | ✗ | ✗ | Level 2 only, system state |
| $4A | F$STABX | store A at 0,X in task B | ✗ | ✗ | Level 2 only, system state |
| $4B | F$AllPrc | allocate a process descriptor | ✗ | ✗ | Level 2 only, system state |
| $4C | F$DelPrc | deallocate a process descriptor | ✗ | ✗ | Level 2 only, system state |
| $4D | F$ELink | link by module directory entry | ✗ | ✗ | Level 2 only, system state |
| $4E | F$FModul | find a module directory entry | ✗ | ✗ | Level 2 only, system state |
| $4F | F$MapBlk | map a specific block | ✗ | ✗ | Level 2 only, system state |
| $50 | F$ClrBlk | clear a specific block | ✗ | ✗ | Level 2 only, system state |
| $51 | F$DelRAM | deallocate RAM blocks | ✗ | ✗ | Level 2 only, system state |
| $52 | F$GCMDir | pack the module directory | ✗ | ✗ | Level 2 only, system state |
| $53 | F$AlHRAM | allocate high RAM blocks | ✗ | ✗ | Level 2 only, system state |
| $54 | F$ReBoot | reboot, or drop to RSDOS | ✗ | ✗ | Level 2 only, system state |
| $55 | F$CRCMod | toggle or report CRC checking | ✗ | ✗ | Level 2 only, system state |
| $57 | F$VBlock | verify modules in a block and add them | ✗ | ✗ | Level 2 only, system state |
| $70 | F$RegDmp | debugging register dump | ✗ | ✗ | Level 2 only, system state |
| $71 | F$NVRAM | read or write battery-backed RAM | ✗ | ✗ | Level 2 only, system state |

## Filling this in

Tick a box only when something would fail if the call broke -- a test in
`compiler-rt/test/mc6809-os9-runtime/`, or a picolibc test that reaches it
through the C function named in the notes.  A wrapper in `<os9.h>` with
nothing calling it is not a tick; several of those are here already, and
they are marked as such.
