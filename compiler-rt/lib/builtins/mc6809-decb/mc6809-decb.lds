/*
 * mc6809-decb.lds — lld linker script for CoCo Disk Extended Color BASIC
 * machine-language binaries (-target mc6809-unknown-decb).
 *
 * OUTPUT_FORMAT(decb-program) tells ld.lld to wrap the flat body in the
 * 10-byte DECB binary envelope:
 *
 *   [ lld prepends ]  [ this script outputs ]  [ lld appends ]
 *   | $00 len(2) la(2)| flat body (.text + ..) | $FF $00 $00 ea(2) |
 *
 * where la = load address (VMA of first ALLOC section) and
 *       ea = exec address (entry-point symbol _start).
 *
 * DECB program startup (per CoCo BASIC ROM LOADM/EXEC convention):
 *   - LOADM reads the preamble to determine load address and length,
 *     copies the body to RAM at the load address.
 *   - EXEC reads the postamble exec address and JSRs to it.
 *   - The stack pointer (S) is already set by BASIC; _start can call
 *     main and RTS back to BASIC without further setup.
 *
 * Default load address is $3F00 — a common choice for short CoCo
 * user programs that return to BASIC.  Override with:
 *   -Wl,--defsym=__load_addr=0xNNNN  (not yet wired; see below)
 *
 * Layout:
 *   .text.startup — _start must be first (exec address == load address)
 *   .text         — remaining code
 *   .rodata       — read-only data (string literals, etc.)
 *   .data         — initialised data (copied from ROM image at startup
 *                   by crt0 if present; for simple programs this is empty)
 *   .bss          — zero-initialised data (NOLOAD)
 */

OUTPUT_FORMAT(decb-program)
ENTRY(_start)

SECTIONS {
    .text 0x3f00 : {
        *(.text.startup .text.startup.*)
        *(.text .text.*)
        *(.rodata .rodata.*)
    }

    .data : ALIGN(2) {
        *(.data .data.*)
    }

    .bss (NOLOAD) : ALIGN(2) {
        *(.bss .bss.*)
        *(COMMON)
    }

    /DISCARD/ : {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.ARM.*)
    }
}
