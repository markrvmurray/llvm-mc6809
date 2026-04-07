/* link.ld — linker script for MC6809 execution tests */

MEMORY {
    RAM (rwx) : ORIGIN = 0x0000, LENGTH = 0x8000
    IO  (rw)  : ORIGIN = 0xBF00, LENGTH = 0x0100
    ROM (rx)  : ORIGIN = 0xE000, LENGTH = 0x1FF0
    VEC (r)   : ORIGIN = 0xFFF0, LENGTH = 0x0010
}

SECTIONS {
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
