/* A cut-down OS-9 program-module script: the body layout, and the data-area
   layout as U-relative offsets, exactly the symbols ld.lld's OS-9 relocation
   code reads. .dp.data starts at U+$20, .data at U+$100. */
OUTPUT_FORMAT(os9-program-module)
ENTRY(_start)
SECTIONS {
    .text 0 : { *(.text .text.*) }
    .rodata : { *(.rodata .rodata.*) }
    .dp.data : {
        __dp_data_image_start = .;
        *(.dp.data .dp.data.*)
        __dp_data_image_end = .;
    }
    .data : {
        __data_image_start = .;
        *(.data .data.*)
        __data_image_end = .;
    }
    .os9_reloc : { KEEP(*(.os9_reloc)) }
    .dp.bss (NOLOAD) : {
        __dp_bss_image_start = .;
        *(.dp.bss .dp.bss.*)
        __dp_bss_image_end = .;
    }
    .bss (NOLOAD) : {
        __bss_image_start = .;
        *(.bss .bss.*)
        __bss_image_end = .;
    }
    __dp_data_start = 0x20;
    __dp_bss_start  = __dp_data_start + (__dp_data_image_end - __dp_data_image_start);
    __data_start    = 0x100;
    __bss_start     = __data_start + (__data_image_end - __data_image_start);
}
