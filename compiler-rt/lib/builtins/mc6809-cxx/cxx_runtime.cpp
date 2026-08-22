//===-- cxx_runtime.cpp - what C++ needs beyond the C library -------------===//
//
// Part of LLVM-MC6809, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// There is no C++ standard library on this target and there is not going to
// be one: libc++ is far larger than a 64K address space.  What there is, is
// the language over the C library -- classes, templates, virtual functions,
// objects with constructors -- and that needs a handful of symbols the
// compiler emits calls to.  This is those symbols and nothing else.
//
// Not here, and not planned: exceptions and RTTI.  Unwinding wants tables
// and an unwinder on a machine where a hello world is already 7K, so the
// driver refuses -fexceptions and -frtti rather than letting a program fail
// at link time with a mangled name nobody can read.
//
// __cxa_atexit and __dso_handle come from picolibc, so a static object's
// destructor needs nothing from here.
//
//===----------------------------------------------------------------------===//

// Freestanding on purpose: this is built with the compiler-rt build, before
// there is a C library to include a header from.  Three declarations are all
// it needs, and they are the C library's own.
typedef __SIZE_TYPE__ size_t;

extern "C" {
void *malloc(size_t);
void free(void *);
void abort(void) __attribute__((noreturn));
}

namespace std {
struct nothrow_t {
  explicit nothrow_t() = default;
};
extern const nothrow_t nothrow;
const nothrow_t nothrow{};
} // namespace std

// The allocating forms.  A failed new returns null rather than throwing:
// there are no exceptions here, and a program that cannot have the memory
// finds out by looking, as it would in C.
void *operator new(size_t size) { return malloc(size ? size : 1); }
void *operator new[](size_t size) { return ::operator new(size); }
void *operator new(size_t size, const std::nothrow_t &) noexcept {
  return ::operator new(size);
}
void *operator new[](size_t size, const std::nothrow_t &) noexcept {
  return ::operator new(size);
}

void operator delete(void *p) noexcept { free(p); }
void operator delete[](void *p) noexcept { free(p); }
void operator delete(void *p, const std::nothrow_t &) noexcept { free(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept { free(p); }
// The sized forms, which is what a virtual destructor calls.
void operator delete(void *p, size_t) noexcept { free(p); }
void operator delete[](void *p, size_t) noexcept { free(p); }

extern "C" {

// Called if a pure virtual function is somehow reached -- during a base
// class's constructor, usually.  There is nothing sensible to do and
// returning would run the call anyway, so stop.
void __cxa_pure_virtual(void) { abort(); }

// The same, for a deleted virtual.
void __cxa_deleted_virtual(void) { abort(); }

// Function-local statics.  One thread, no interrupts to worry about within
// a construction: the guard byte says whether the object is there yet.
int __cxa_guard_acquire(unsigned char *guard) { return !*guard; }
void __cxa_guard_release(unsigned char *guard) { *guard = 1; }
void __cxa_guard_abort(unsigned char *) {}

} // extern "C"
