/*
 * mc6809-os9.lds — lld linker script for OS-9 program modules.
 *
 * Bug #163 Phase 2 (proof of concept).
 *
 * Output: a FLAT binary holding only the OS-9 module BODY (no header,
 * no CRC).  The body is then wrapped by `os9-link` (tools/os9-link),
 * which prepends the 13-byte header and appends the fcs name + 3-byte
 * CRC trailer.
 *
 *   [ os9-link prepends ] [ this script outputs ] [ os9-link appends ]
 *   |   13-byte header  |  flat body (.text +   |  fcs name + CRC   |
 *   |  sync,size,parity |  .rodata + .data)     |  trailer ($800FE3)|
 *
 * OS-9 program startup (per llvm/docs/MC6809-OS9.md §A.3):
 *   - Kernel loads the module into memory, allocates a separate
 *     `M$Mem`-byte process data area, sets U = data-area base.
 *   - Kernel does NOT copy .data automatically — the CRT must.
 *   - Kernel sets S = U + M$Mem - 1 (stack at top of data area).
 *   - Kernel jumps to (module_base + M$Exec).
 *
 * Layout responsibilities:
 *   .text     — code; first byte MUST be `_start` (so exec offset = 13).
 *   .rodata   — read-only data; lives in the module body, accessed PCR.
 *   .data     — initialised data IMAGE in the body; CRT copies to U+0.
 *   .bss      — NOLOAD; CRT zeros at U + __data_size.
 *   .runtime  — NOLOAD; private compiler-rt process storage.
 *   .heap     — NOLOAD; reserved after runtime storage for sbrk-style growth.
 *   .stack    — NOLOAD; reserved after heap, top = S = U + M$Mem.
 *
 * Symbols exported (used by CRT and by build-system to pass --mem to
 * os9-link):
 *   __data_image_start, __data_image_end  — addresses in module body
 *                                            (CRT references via PCR).
 *   __data_size                            — bytes to copy at startup.
 *   __bss_size                             — bytes to zero at startup.
 *   __errno_offset                         — U-relative runtime errno slot.
 *   __heap_start, __heap_size              — heap region (U-relative).
 *   __stack_start, __stack_size            — stack region (U-relative).
 *   __mem_size                             — total M$Mem allocation.
 *
 * Overridable via -Wl,--defsym=:
 *   __heap_size   (default 0x0400 = 1024 B)
 *   __stack_size  (default 0x0200 = 512 B)
 */

/*
 * OUTPUT_FORMAT(os9-program-module) tells ld.lld to wrap the flat body
 * we lay out below in the 13-byte OS-9 header + fcs name + 3-byte CRC
 * trailer.  The clang driver auto-passes --os9-name; the writer reads
 * __mem_size (defined at the end of this script) for M$Mem.
 *
 * The legacy two-step path (--oformat=binary + tools/os9-link) is still
 * supported — pass --oformat=binary on the command line to override.
 */
OUTPUT_FORMAT(os9-program-module)
ENTRY(_start)

PROVIDE(__heap_size  = 0x0400);
PROVIDE(__stack_size = 0x0200);

SECTIONS {
    /*
     * Body starts at byte 0.  _start MUST be first so the exec_offset
     * passed to os9-link is always exactly 13 (sizeof OS-9 header).
     * The .text.startup linker-input section is the GCC/clang
     * convention for startup code; CRT files should put `_start` in
     * `.text.startup` (or `.text.startup._start`).
     */
    .text 0x0000 : {
        *(.text.startup .text.startup.*)
        *(.text .text.*)
    }

    .rodata : ALIGN(2) {
        *(.rodata .rodata.*)
    }

    /*
     * .data image: appears in the body so the kernel reads it from
     * disk along with code, but the CRT is responsible for copying
     * it into the runtime data area.
     */
    .data : ALIGN(2) {
        __data_image_start = .;
        *(.data .data.*)
        __data_image_end = .;
    }
    __data_size = __data_image_end - __data_image_start;

    /*
     * .bss is NOLOAD: not in the file, but its size contributes to
     * M$Mem.  Layout is U-relative at runtime; the file `.` counter
     * is irrelevant for these sections.
     */
    .bss (NOLOAD) : ALIGN(2) {
        __bss_image_start = .;
        *(.bss .bss.*)
        *(COMMON)
        __bss_image_end = .;
    }
    __bss_size = __bss_image_end - __bss_image_start;

    .runtime (NOLOAD) : ALIGN(2) {
        __runtime_image_start = .;
        __errno_image_start = .;
        . += 2;
        __errno_image_end = .;
        __runtime_image_end = .;
    }
    __runtime_size = __runtime_image_end - __runtime_image_start;

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

    /*
     * U-relative offsets (CRT consumes via immediate `#`):
     *   data lives at offset 0 in the process data area
     *   bss starts immediately after the data image
     *   runtime storage starts after bss
     *   heap starts after runtime storage
     *   stack starts after heap; S = U + __mem_size
     */
    __data_start  = 0;
    __bss_start   = __data_size;
    __runtime_start = __data_size + __bss_size;
    __errno_offset = __runtime_start;
    __heap_start  = __runtime_start + __runtime_size;
    __stack_start = __heap_start + __heap_size;
    __mem_size    = __stack_start + __stack_size;

    /DISCARD/ : {
        *(.comment)
        *(.note .note.*)
        *(.eh_frame .eh_frame_hdr)
        *(.ARM.*)
    }
}
