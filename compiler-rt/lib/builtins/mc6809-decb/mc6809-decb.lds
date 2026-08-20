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
 *   .dp           — one 256-byte page at the load address: the compiler's
 *                   imaginary registers below 0x18, then .dp.data and
 *                   .dp.bss.  crt0 puts its address in DP
 *   .text.startup — _start, which is the exec address
 *   .text         — remaining code
 *   .rodata       — read-only data (string literals, etc.)
 *   .data         — initialised data (copied from ROM image at startup
 *                   by crt0 if present; for simple programs this is empty)
 *   .bss          — zero-initialised data (NOLOAD)
 *
 * crt0 gets __dp_base_addr, __dp_bss_start/_end and __bss_start/_end from
 * here.  .data needs no copy: LOADM has already put it in RAM.
 */

OUTPUT_FORMAT(decb-program)
ENTRY(_start)

/* The direct page is the page the program is loaded into, so the whole of it
 * -- the compiler's scratch registers included -- is part of the image LOADM
 * copies.  `__dp_base` is the page number, as picolibc's script means it;
 * `__dp_base_addr` is the address crt0 puts in DP. */
__dp_base = 0x3f;
__dp_base_addr = __dp_base << 8;

/* The compiler's imaginary registers, at the bottom of the page: the same
 * layout and the same names as picolibc.ld, with user direct-page objects
 * from 0x18 on. */
PROVIDE(__rc0  = __dp_base_addr + 0x00);
PROVIDE(__rc1  = __dp_base_addr + 0x01);
PROVIDE(__rc2  = __dp_base_addr + 0x02);
PROVIDE(__rc3  = __dp_base_addr + 0x03);
PROVIDE(__rc4  = __dp_base_addr + 0x04);
PROVIDE(__rc5  = __dp_base_addr + 0x05);
PROVIDE(__rc6  = __dp_base_addr + 0x06);
PROVIDE(__rc7  = __dp_base_addr + 0x07);
PROVIDE(__rs0  = __dp_base_addr + 0x08);
PROVIDE(__rs1  = __dp_base_addr + 0x0A);
PROVIDE(__rs2  = __dp_base_addr + 0x0C);
PROVIDE(__rs3  = __dp_base_addr + 0x0E);
PROVIDE(__rs0hi = __rs0); PROVIDE(__rs0lo = __rs0 + 1);
PROVIDE(__rs1hi = __rs1); PROVIDE(__rs1lo = __rs1 + 1);
PROVIDE(__rs2hi = __rs2); PROVIDE(__rs2lo = __rs2 + 1);
PROVIDE(__rs3hi = __rs3); PROVIDE(__rs3lo = __rs3 + 1);
PROVIDE(__rc8  = __dp_base_addr + 0x10);
PROVIDE(__rc9  = __dp_base_addr + 0x11);
PROVIDE(__rc10 = __dp_base_addr + 0x12);
PROVIDE(__rc11 = __dp_base_addr + 0x13);
PROVIDE(__rc12 = __dp_base_addr + 0x14);
PROVIDE(__rc13 = __dp_base_addr + 0x15);
PROVIDE(__rc14 = __dp_base_addr + 0x16);
PROVIDE(__rc15 = __dp_base_addr + 0x17);

SECTIONS {
    /* The page itself.  What is below 0x18 is scratch, so whatever LOADM
     * leaves there will do; the image simply starts with it. */
    .dp __dp_base_addr : {
        . = __dp_base_addr + 0x18;
        *(.dp.data .dp.data.*)
        *(.dp.rodata .dp.rodata.*)
        __dp_bss_start = .;
        *(.dp.bss .dp.bss.*)
        __dp_bss_end = .;
        . = __dp_base_addr + 256;
    }

    .text : {
        *(.text.startup .text.startup.*)
        *(.text .text.*)
        *(.rodata .rodata.*)
    }

    /* There is no ROM here: LOADM copies the whole file into RAM, so
     * initialised data arrives where it belongs and nothing copies it.  Only
     * the zeroed areas need work, and only because they are not in the file. */
    .data : ALIGN(2) {
        *(.data .data.*)
    }

    .bss (NOLOAD) : ALIGN(2) {
        __bss_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_end = .;
    }

    /* BASIC set the stack before it EXECed; what is left above the program is
     * where a heap would go if a library wants one. */
    PROVIDE(__heap_start = .);

    /DISCARD/ : {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.ARM.*)
    }
}
