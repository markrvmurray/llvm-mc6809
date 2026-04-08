/* link.ld — linker script for MC6809 execution tests */

MEMORY {
    RAM (rwx) : ORIGIN = 0x0000, LENGTH = 0x8000
    IO  (rw)  : ORIGIN = 0xBF00, LENGTH = 0x0100
    ROM (rx)  : ORIGIN = 0xE000, LENGTH = 0x1FF0
    VEC (r)   : ORIGIN = 0xFFF0, LENGTH = 0x0010
}

SECTIONS {
    /* Imaginary registers — direct page (DP=0, addresses 0x00-0xFF).
       Each __rsN is a 2-byte pointer, __rcN are the individual bytes.
       RC0 is the low byte of RS0, RC1 is the high byte. */
    .directpage (NOLOAD) : {
        __rc0 = .; __rc1 = . + 1; __rs0 = .; . += 2;
        __rc2 = .; __rc3 = . + 1; __rs1 = .; . += 2;
        __rc4 = .; __rc5 = . + 1; __rs2 = .; . += 2;
        __rc6 = .; __rc7 = . + 1; __rs3 = .; . += 2;
        __rc8 = .; __rc9 = . + 1; __rs4 = .; . += 2;
        __rc10 = .; __rc11 = . + 1; __rs5 = .; . += 2;
        __rc12 = .; __rc13 = . + 1; __rs6 = .; . += 2;
        __rc14 = .; __rc15 = . + 1; __rs7 = .; . += 2;
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
