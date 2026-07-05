/* link.ld — linker script for MC6809 execution tests.
 *
 * usim09batch (after commit 78a1fa4, "pico-thing alignment" — Bug #358;
 * the IO slab moved down from $FFD0-$FFD2 to $FFC4-$FFCC) presents a
 * single unified RAM region with a small IO slab + vector ROM at the top.
 *
 * Even though the emulated machine no longer has a ROM/RAM split, we
 * keep a virtual code/data partition in this script so test inputs
 * that assume $0100-area is free data memory (e.g. arithmetic.s's
 * `sta $0100`) keep working. Data/.bss/heap live low; code lives
 * high. The runtime initialises the stack just below the code region
 * so push-down doesn't collide with code bytes.
 *
 * Bug #181 (2026-04-26): the previous DRAM=$0000-$BFFF / CRAM=$C003-
 * $FFCF split capped code at ~16 KB. codegen-picolibc.s now needs
 * ~18.5 KB. Since the hardware no longer has a ROM/RAM split, the
 * legacy 16 KB code cap is artificial — we just need DRAM to cover
 * the $0100-area data tests. Shifting the boundary to $4000 gives
 * data tests 16 KB (more than enough) and code ~48 KB.
 */

MEMORY {
    /* Data partition: $0000-$3FFF (16 KB). Covers .directpage at
     * $00-$FF, the $0100-area scratch that arithmetic.s and friends
     * write to, plus .data/.bss/heap. */
    DRAM (rwx) : ORIGIN = 0x0000, LENGTH = 0x4000
    /* Code partition: $4000-$FFC3 (~48 KB). Top end stops before the
     * IO slab at $FFC4-$FFCC (ACIA at $FFC4/$FFC5, halt at $FFCC). */
    CRAM (rwx) : ORIGIN = 0x4000, LENGTH = 0xBFC4
    VEC  (r)   : ORIGIN = 0xFFF0, LENGTH = 0x0010
}

/* IO device symbols — the single source of truth for the memory map.
   IR under test (codegen-picolibc.ll) references these as externals
   instead of hardcoding addresses inline (bug #102). Change the map
   here and everything follows; no per-test patching required.
   Bug #358: ACIA + halt moved down from $FFD0-$FFD2 to $FFC4-$FFCC. */
__acia_status = 0xFFC4;
__acia_data   = 0xFFC5;
__halt        = 0xFFCC;

SECTIONS {
    /* Imaginary registers — direct page (DP=0, addresses 0x00-0xFF).
       8-bit (RC) and 16-bit (RS) pools are non-overlapping. */
    .directpage (NOLOAD) : {
        /* 8-bit pool: RC0..RC7 at $00-$07 */
        __rc0 = .; . += 1;
        __rc1 = .; . += 1;
        __rc2 = .; . += 1;
        __rc3 = .; . += 1;
        __rc4 = .; . += 1;
        __rc5 = .; . += 1;
        __rc6 = .; . += 1;
        __rc7 = .; . += 1;
        /* 16-bit pool: RS0..RS3 at $08-$0F. Byte sub-register aliases:
           big-endian, HI at the base address, LO at base+1. */
        __rs0 = .; . += 2;
        __rs1 = .; . += 2;
        __rs2 = .; . += 2;
        __rs3 = .; . += 2;
        __rs0hi = __rs0; __rs0lo = __rs0 + 1;
        __rs1hi = __rs1; __rs1lo = __rs1 + 1;
        __rs2hi = __rs2; __rs2lo = __rs2 + 1;
        __rs3hi = __rs3; __rs3lo = __rs3 + 1;
        /* Remaining (for future expansion) */
        __rc8 = .; . += 1; __rc9 = .; . += 1;
        __rc10 = .; . += 1; __rc11 = .; . += 1;
        __rc12 = .; . += 1; __rc13 = .; . += 1;
        __rc14 = .; . += 1; __rc15 = .; . += 1;
        __rs4 = .; . += 2; __rs5 = .; . += 2;
        __rs6 = .; . += 2; __rs7 = .; . += 2;
        /* Dedicated byte-ALU scratch (path (c) of
           emit6809RegByteFromMem — replaces PSHS/op/LEAS triple). */
        __scratch = .; . += 1;
        *(.directpage)
        /* Direct-page zero-init and data: a static-stack frame placed in the
           direct page, and any __attribute__((directpage)) object, land here so
           their `<sym` / mc6809_8(sym) references resolve within page 0. */
        *(.dp.bss .dp.bss.*)
        *(.dp.data .dp.data.*)
        *(.dp.rodata .dp.rodata.*)
    } > DRAM

    .ram (NOLOAD) : {
        *(.ram)
    } > DRAM

    .rom : {
        *(.rom)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
    } > CRAM

    .data : {
        __data_start = .;
        *(.data)
        *(.data.*)
        __data_end = .;
    } > DRAM

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        __bss_end = .;
    } > DRAM

    .vectors : {
        *(.vectors)
    } > VEC
}
