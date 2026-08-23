/*
 * mc6809-os9.lds — lld linker script for OS-9 program modules.
 *
 * Output: an OS-9 program module (OUTPUT_FORMAT(os9-program-module) makes
 * ld.lld wrap the flat body laid out below in the 13-byte header, the fcs
 * name and the 3-byte CRC trailer; --oformat=binary on the command line
 * gives the bare body for tools/os9-link instead).
 *
 * Program startup (llvm/docs/MC6809-OS9.md):
 *   - The kernel loads the module, allocates a separate M$Mem-byte process
 *     data area, sets U = data-area base and DP = U >> 8.
 *   - The kernel copies nothing and zeroes nothing: the CRT does both.
 *   - S = U + M$Mem (the stack is the last thing in the data area).
 *   - PC = module base + M$Exec, which is _start, first in .text.
 *
 * Two address spaces meet here.  The link's VMA space is the MODULE BODY:
 * .text, .rodata and the read-only images of the initialised data live at
 * their body offsets, and code reaches them PC-relatively.  Everything a
 * process WRITES lives in the DATA AREA, addressed from U (and, for the
 * first page, from DP), and its layout is a set of U-relative offsets that
 * this script defines as symbols:
 *
 *   U +
 *   $0000  __rc0..__rc7, __rs0..__rs3, __rc8..__rc15   the compiler's
 *          direct-page imaginary registers, $00-$17 (the bare-metal map)
 *   $0018  .dp.data copy       initialised __directpage objects
 *          .dp.bss  (zeroed)   zero-initialised __directpage objects and
 *                              anything else the compiler puts in page 0
 *          ---- all of this must end at U + $0100 at the latest: it is
 *               the direct page, reached with 1-byte addresses via
 *               DP = U>>8, which is why U must be 256-byte aligned (OS-9
 *               allocates data areas in whole pages)
 *   __data_start   .data copy
 *   __bss_start    .bss (zeroed)
 *   __heap_start   heap
 *   __stack_start  stack; S = U + __mem_size
 *
 * The images of .dp.data and .data sit in the body (__dp_data_image_*,
 * __data_image_*); .dp.bss and .bss are NOLOAD and their body-space `.` is
 * meaningless -- what matters is the U-relative *_start symbols.  ld.lld
 * maps a body address inside any of the four regions to its U-relative
 * offset (R_MC6809_OS9_OFFSET_16 / _8, and the words in the copied .data
 * that point into the data area) from exactly these symbols, so a new
 * region needs only its image and start symbols here.
 *
 * Overridable via -Wl,--defsym=:
 *   __heap_size        (default 0x0400 = 1024 B) the static heap pool,
 *                      used when the kernel grants no more memory
 *   __stack_size       (default 0x0200 = 512 B)
 *   __os9_mem_request  (default 0x4000 = 16 KB) the total data-area size
 *                      the start-up code asks the kernel for (F$Mem); the
 *                      pages granted above the stack become the heap.  0
 *                      disables the request.
 */

OUTPUT_FORMAT(os9-program-module)
ENTRY(_start)

PROVIDE(__heap_size  = 0x0400);
PROVIDE(__stack_size = 0x0200);
PROVIDE(__os9_mem_request = 0x4000);

/*
 * The imaginary registers.  Same map as bare metal: RC0-7 at $00, RS0-3 at
 * $08 (big-endian halves HI/LO), RC8-15 at $10.  They are the process's own
 * (they live in its data area), and the compiler reaches them by direct
 * addressing, `<__rs0`, whose one-byte operand is exactly this value.
 */
__rc0  = 0x00; __rc1  = 0x01; __rc2  = 0x02; __rc3  = 0x03;
__rc4  = 0x04; __rc5  = 0x05; __rc6  = 0x06; __rc7  = 0x07;
__rs0  = 0x08; __rs1  = 0x0A; __rs2  = 0x0C; __rs3  = 0x0E;
__rs0hi = __rs0; __rs0lo = __rs0 + 1;
__rs1hi = __rs1; __rs1lo = __rs1 + 1;
__rs2hi = __rs2; __rs2lo = __rs2 + 1;
__rs3hi = __rs3; __rs3lo = __rs3 + 1;
__rc8  = 0x10; __rc9  = 0x11; __rc10 = 0x12; __rc11 = 0x13;
__rc12 = 0x14; __rc13 = 0x15; __rc14 = 0x16; __rc15 = 0x17;
__dp_start = 0x18;

SECTIONS {
    /*
     * Body starts at byte 0.  _start MUST be first so the exec offset is
     * always exactly 13 (sizeof OS-9 header).  CRT files put `_start` in
     * `.text.startup`.
     */
    .text 0x0000 : {
        *(.text.startup .text.startup.*)
        *(.text .text.*)
    }

    .rodata : ALIGN(2) {
        *(.rodata .rodata.*)
        *(.dp.rodata .dp.rodata.*)
    }

    /*
     * Images of the initialised data, in the body.  The CRT copies them into
     * the data area at startup: .dp.data to U + __dp_data_start, .data to
     * U + __data_start.
     */
    .dp.data : ALIGN(1) {
        __dp_data_image_start = .;
        *(.dp.data .dp.data.*)
        __dp_data_image_end = .;
    }
    __dp_data_size = __dp_data_image_end - __dp_data_image_start;

    .data : ALIGN(2) {
        __data_image_start = .;
        *(.data .data.*)
        /*
         * Constructor and destructor tables: code pointers, which in a
         * module placed at run time can only live in the data area (copied
         * and rebased by the CRT like every pointer in .data).  The bounds
         * are data-area offsets, as __libc_init_array reaches them.
         */
        . = ALIGN(2);
        __preinit_array_start = .;
        KEEP(*(.preinit_array))
        __preinit_array_end = .;
        __init_array_start = .;
        KEEP(*(SORT_BY_INIT_PRIORITY(.init_array.*) SORT_BY_INIT_PRIORITY(.ctors.*)))
        KEEP(*(.init_array .ctors))
        __init_array_end = .;
        __fini_array_start = .;
        KEEP(*(SORT_BY_INIT_PRIORITY(.fini_array.*) SORT_BY_INIT_PRIORITY(.dtors.*)))
        KEEP(*(.fini_array .dtors))
        KEEP(*(.fini_array*))
        __fini_array_end = .;
        __data_image_end = .;
    }
    __data_size = __data_image_end - __data_image_start;

    /*
     * Runtime relocation records for pointers held in the copied .data.
     * Each entry is (kind, U-relative offset of the word); ld.lld has already
     * rewritten the word itself to a module-body offset (kind 1) or a
     * data-area offset (kind 2), so the CRT adds the module base or U.
     */
    .os9_reloc : ALIGN(1) {
        __os9_reloc_start = .;
        KEEP(*(.os9_reloc))
        __os9_reloc_end = .;
    }
    __os9_reloc_size = __os9_reloc_end - __os9_reloc_start;

    /*
     * Zero-initialised regions: not in the file; their sizes contribute to
     * M$Mem.  Their body-space `.` is irrelevant -- the U-relative *_start
     * symbols below are the layout.
     */
    .dp.bss (NOLOAD) : ALIGN(1) {
        __dp_bss_image_start = .;
        *(.dp.bss .dp.bss.*)
        __dp_bss_image_end = .;
    }
    __dp_bss_size = __dp_bss_image_end - __dp_bss_image_start;

    .bss (NOLOAD) : ALIGN(2) {
        __bss_image_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_image_end = .;
    }
    __bss_size = __bss_image_end - __bss_image_start;

    .heap (NOLOAD) : ALIGN(2) {
        __heap_image_start = .;
        . += __heap_size;
        __heap_image_end = .;
    }

    .stack (NOLOAD) : ALIGN(2) {
        __stack_image_start = .;
        . += __stack_size;
        __stack_image_end = .;
    }

    /* The data area, as U-relative offsets. */
    __dp_data_start = __dp_start;
    __dp_bss_start  = __dp_data_start + __dp_data_size;
    __dp_end        = __dp_bss_start + __dp_bss_size;
    __data_start    = ALIGN(__dp_end, 2);
    __bss_start     = __data_start + __data_size;
    __heap_start    = __bss_start + __bss_size;
    __stack_start   = __heap_start + __heap_size;
    __mem_size      = __stack_start + __stack_size;

    /DISCARD/ : {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.ARM.*)
    }
}

ASSERT(__dp_end <= 0x100,
       "OS-9: direct-page objects (.dp.data + .dp.bss) do not fit in the first page of the data area")
