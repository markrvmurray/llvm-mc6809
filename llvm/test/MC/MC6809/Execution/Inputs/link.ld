/* link.ld — linker script for MC6809 execution tests.
 *
 * usim09batch (after commit fa2af824, "relocate IO and vector ROM to
 * bottom of address space") presents a single unified RAM region with
 * a small IO slab + vector ROM at the top.
 *
 * Even though the emulated machine no longer has a ROM/RAM split, we
 * keep the legacy code/data partition virtually in this script so the
 * test inputs (many of which assume $0100-area is free data memory,
 * e.g. arithmetic.s's `sta $0100`) keep working. Code lives high
 * ($C003-$FFCF, mirroring the old ROM range), data/.bss/heap live
 * low ($0000-$BFFF). The runtime initialises the stack just below
 * the code region so push-down doesn't collide with code bytes.
 */

MEMORY {
    DRAM (rwx) : ORIGIN = 0x0000, LENGTH = 0xC000
    /* Code partition: $C003-$FFCF. Top end stops before the IO slab
     * at $FFD0-$FFEF (ACIA at $FFD0/$FFD1, halt at $FFD2). */
    CRAM (rwx) : ORIGIN = 0xC003, LENGTH = 0x3FCD
    VEC  (r)   : ORIGIN = 0xFFF0, LENGTH = 0x0010
}

/* IO device symbols — the single source of truth for the memory map.
   IR under test (codegen-picolibc.ll) references these as externals
   instead of hardcoding addresses inline (bug #102). Change the map
   here and everything follows; no per-test patching required. */
__acia_status = 0xFFD0;
__acia_data   = 0xFFD1;
__halt        = 0xFFD2;

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
        /* 16-bit pool: RS0..RS3 at $08-$0F */
        __rs0 = .; . += 2;
        __rs1 = .; . += 2;
        __rs2 = .; . += 2;
        __rs3 = .; . += 2;
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
