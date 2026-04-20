/* link.ld — linker script for MC6809 execution tests */

MEMORY {
    RAM (rwx) : ORIGIN = 0x0000, LENGTH = 0xC000
    /* IO devices overlay 0xC000-0xC002 (matches usim09batch):
         0xC000-0xC001  ACIA (status/data)
         0xC002         Halt/exit device (written value = exit code)
       Code starts at 0xC003 to avoid the IO overlay. */
    ROM (rx)  : ORIGIN = 0xC003, LENGTH = 0x3FED
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

    .rom : {
        *(.rom)
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata.*)
    } > ROM

    /* .data lives in RAM (VMA) but initial values are stored in ROM (LMA)
       AFTER the code. The crt0 copies from ROM to RAM at startup. */
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

    .vectors : {
        *(.vectors)
    } > VEC

    __data_load = LOADADDR(.data);
}
