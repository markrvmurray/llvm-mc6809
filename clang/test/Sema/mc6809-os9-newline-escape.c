// RUN: %clang_cc1 -triple mc6809-unknown-os9 -fos9-newline -DOS9 -fsyntax-only -verify=os9 %s
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -fos9-newline -DOS9 -fsyntax-only -pedantic -verify=pedantic %s
// RUN: %clang_cc1 -triple mc6809-unknown-os9 -fsyntax-only -verify=lf %s

// '\l' is an escape only with -fos9-newline, where it is an extension like
// '\e'; elsewhere it is an unknown escape that means 'l'.

// os9-no-diagnostics

char lf = '\l'; // pedantic-warning {{use of non-standard escape character '\l'}} \
                // lf-warning {{unknown escape sequence '\l'}}

#ifdef OS9
_Static_assert('\n' == 13, "'\\n' is a carriage return");
_Static_assert('\r' == 13, "'\\r' is a carriage return");
_Static_assert('\l' == 10, "'\\l' is a line feed"); // pedantic-warning {{use of non-standard escape character '\l'}}
_Static_assert(sizeof("\n") == 2, "one character");
#else
_Static_assert('\n' == 10, "'\\n' is a line feed");
_Static_assert('\l' == 'l', "'\\l' is not an escape"); // lf-warning {{unknown escape sequence '\l'}}
#endif
