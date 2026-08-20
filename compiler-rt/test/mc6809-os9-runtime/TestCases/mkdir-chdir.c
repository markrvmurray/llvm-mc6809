// REQUIRES: mc6809-os9-runtime
// RUN: %run_os9_case %s OS9
// CFLAGS: -fno-builtin
//
// I$MakDir, I$ChgDir and I$Delete on a directory.  Making the directory is
// only half of it: the test writes a file with a plain relative name, so if
// the change of directory did not take, the file lands somewhere else and
// the delete of the now-not-empty directory fails.
//
// OS9: makdir=0 chdir=0 wrote=5 read=hello chdir-back=0 from-parent=0 rmfile=0
#include <os9.h>

static void put(const char *s) {
  int n = 0;
  while (s[n] != 0)
    n++;
  _os_write(1, s, &n);
}

static void putn(long v) {
  char b[10];
  int i = 0;
  if (v < 0) { put("-"); v = -v; }
  do { b[i++] = (char)('0' + (int)(v % 10)); v /= 10; } while (v);
  while (i--) { int one = 1; _os_write(1, &b[i], &one); }
}

static void field(const char *name, long v) {
  put(name); put("="); putn(v); put(" ");
}

int main(void) {
  int fd, err, n;
  char buf[8];

  // Leave nothing of an earlier run behind: a directory that still has the
  // file in it cannot be deleted, and the test would report the wrong thing.
  (void)_os_delete("subdir/f.txt");

  err = _os_makdir("subdir", 0xBF);     // owner and public, read/write/exec
  field("makdir", err);
  if (err) { put("\n"); return 1; }

  err = _os_chgdir("subdir", OS9_READ);
  field("chdir", err);
  if (err) { put("\n"); return 1; }

  // A relative name, so this only lands in the new directory if the change
  // of directory actually happened.
  err = _os_create("f.txt", OS9_WRITE, 0x03, &fd);
  if (err) { put("create="); putn(err); put("\n"); return 1; }
  n = 5;
  err = _os_write(fd, "hello", &n);
  _os_close(fd);
  field("wrote", err == 0 ? n : -err);

  err = _os_open("f.txt", OS9_READ, &fd);
  if (err) { put("open="); putn(err); put("\n"); return 1; }
  for (n = 0; n < 8; n++) buf[n] = 0;
  n = 5;
  err = _os_read(fd, buf, &n);
  _os_close(fd);
  buf[n > 0 ? n : 0] = 0;
  put("read="); put(err == 0 ? buf : "ERR"); put(" ");

  err = _os_chgdir("..", OS9_READ);
  field("chdir-back", err);

  // Reached from the parent, under the directory's name: the file is only
  // there if the change of directory took, and only reachable by this name
  // if we are back where we started.
  err = _os_open("subdir/f.txt", OS9_READ, &fd);
  field("from-parent", err);
  if (err == 0)
    _os_close(fd);

  err = _os_delete("subdir/f.txt");
  field("rmfile", err);
  put("\n");

  // The directory itself is left behind: OS-9 will not delete one until its
  // directory attribute has been cleared, which is a job for rmdir() and is
  // tracked on its own.  The disk is made afresh for each run, so nothing
  // accumulates.
  return 0;
}
