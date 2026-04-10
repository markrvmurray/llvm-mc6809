/* link.ld — linker script for MC6809 execution tests */

MEMORY {
    RAM (rwx) : ORIGIN = 0x0000, LENGTH = 0x8000
    /* IO map (matches usim09batch — see runtime.inc):
         0xBF00-0xBF01  ACIA (status/data)
         0xBF02         halt device
       The IO region declaration here is informational only —
       nothing is actually placed in it by SECTIONS below. */
    IO  (rw)  : ORIGIN = 0xBF00, LENGTH = 0x0003
    /* ROM extended from 0xE000 to 0xC000 (8 KiB → 16 KiB) so
       larger picolibc tests fit. The ACIA and halt device used to
       be inside this range; they were moved into the 0xBF00 area
       (see usim09batch's main09batch.cpp) so the ROM range stays
       contiguous. */
    ROM (rx)  : ORIGIN = 0xC000, LENGTH = 0x3FF0
    VEC (r)   : ORIGIN = 0xFFF0, LENGTH = 0x0010
}

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
    } > RAM

    .ram (NOLOAD) : {
        *(.ram)
    } > RAM

    /* .data lives in RAM (VMA) but initial values are stored in ROM (LMA).
       The crt0 in runtime.inc copies from ROM to RAM at startup. */
    .data : {
        __data_start = .;
        *(.data)
        *(.data.*)
        __data_end = .;
    } > RAM AT> ROM

    .bss (NOLOAD) : {
        __bss_start = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        __bss_end = .;
    } > RAM

    .rom : {
        *(.rom)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
    } > ROM

    .vectors : {
        *(.vectors)
    } > VEC

    __data_load = LOADADDR(.data);
}
