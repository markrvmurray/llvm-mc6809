/* Standalone reproducer for Bug #127a.
 *
 * Mimics picolibc's vsnprintf -> __file_str_put structure:
 *   - outer fn builds an on-stack struct with a function ptr + pos/end ptrs
 *   - calls fn ptr 9 times to write 'M'*8 + '\n' into a 20-byte buffer
 *   - callback does: if (s->pos != s->end) *s->pos++ = c;
 *
 * Expected output:  buf=4d,4d,4d,4d,4d,4d,4d,4d,0a,3f,3f,3f,3f,3f,3f,3f,
 * Bug symptom:      buf=3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,3f,
 */

#include <stdio.h>
#include <string.h>

struct file;
typedef int (*put_t)(char, struct file *);

struct file {
    unsigned char unget[2];   /* offset 0..1 */
    unsigned char flags;      /* offset 2     */
    put_t         put;        /* offset 3..4 */
    void         *get;        /* offset 5..6 */
    void         *flush;      /* offset 7..8 */
};                            /* sizeof = 9   */

struct file_str {
    struct file f;            /* offset 0..8  */
    char *pos;                /* offset 9..10 */
    char *end;                /* offset 11..12 */
};

static int
my_put(char c, struct file *stream)
{
    struct file_str *s = (struct file_str *)stream;
    if (s->pos != s->end)
        *s->pos++ = c;
    return (unsigned char)c;
}

static int
my_emit(char *buf, int n, const char *src, int len)
{
    struct file_str fs = {
        .f = { .flags = 2, .put = my_put },
        .pos = buf,
        .end = buf + n,
    };
    int i;
    for (i = 0; i < len; i++)
        (*fs.f.put)(src[i], &fs.f);
    (*fs.f.put)('\n', &fs.f);
    if (n)
        *fs.pos = '\0';
    return i + 1;
}

const char m[8] = { 'M','M','M','M','M','M','M','M' };

int
main(void)
{
    char buf[20];
    int  n, i;

    for (i = 0; i < 20; i++) buf[i] = '?';
    n = my_emit(buf, sizeof buf, m, 8);

    printf("n=%d buf=", n);
    for (i = 0; i < 16; i++)
        printf("%02x,", (unsigned char)buf[i]);
    printf("\n");

    if (memcmp(buf, "MMMMMMMM\n", 9) != 0) {
        printf("FAIL\n");
        return 1;
    }
    printf("PASS\n");
    return 0;
}
