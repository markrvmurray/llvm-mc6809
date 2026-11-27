# OS-9 Runtime Porting Notes

Compiler-rt owns startup and POSIX support. The pinned
[nitros9-c-libs](https://github.com/nitros9project/nitros9-c-libs) checkout
owns OS calls: its `lib/*.as`, `os9/include/os9.h` and `include/os9raw.h`
are built/installed by this directory's CMake rules. Do not add local
syscall implementations or copies of those headers.

## Native calls

The shared selector places LLVM's first word/pointer and integer result
in X, uses U for the data base, and preserves registers used by each kernel
service. Calling-convention selection occurs inside the assembly body at
build time; there is no runtime ABI bridge. X:U seek/status and module calls
preserve the original assembly in the shared library.

Ordinary `_os_*` calls return zero or an OS-9 error code and set errno on
failure. Read/write calls update the actual transfer count. Read EOF is a
successful zero-byte read. Raw create reports an existing file; Unix
replacement/truncation belongs in creat/create above this layer.

Picolibc's `libos/os9` supplies POSIX policies and errno translation. The
out-of-line `_read`/`_write` functions here retain their byte-count/-1 API.
`_exit` performs runtime cleanup before calling the shared native exit entry.

## Build and provenance

Python 3 and lwasm are required. CMake fetches `nitros9-c-libs.revision`;
`NITROS9_C_LIBS_SOURCE_DIR` overrides the checkout for development and
`NITROS9_LWASM` selects the assembler. Installed `nitros9-os-calls.json`
records the requested revision and actual source hashes.

Validate a new shared call with the library's three-compiler kernel probe,
then rebuild this runtime and picolibc. Keep writable state in the process
area addressed from U/DP, never in the reentrant module body. Keep
compiler-specific startup and arithmetic primitives in compiler-rt.
