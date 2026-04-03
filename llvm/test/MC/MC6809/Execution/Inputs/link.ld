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

    .data : {
        *(.data)
        *(.data.*)
    } > RAM

    .bss (NOLOAD) : {
        *(.bss)
        *(.bss.*)
    } > RAM

    .rom : {
        *(.rom)
    } > ROM

    .vectors : {
        *(.vectors)
    } > VEC
}
