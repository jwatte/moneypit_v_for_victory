#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../stb/stb_image.h"
#include "yuv.h"


static inline unsigned char cl255(float f) {
    if (f < 0) return 0;
    if (f > 255) return 255;
    return (unsigned char)floorf(f);
}

int main(int argc, char const *argv[]) {
    if (argc != 2 || argv[1][0] == '-') {
        fprintf(stderr, "usage: mkyuv inputfile.png\n");
        fprintf(stderr, "generates inputfile.yuv\n");
        exit(1);
    }
    int x0, y0, n0;
    unsigned char *d = stbi_load(argv[1], &x0, &y0, &n0, 3);
    if (!d) {
        fprintf(stderr, "%s: could not load image\n", argv[1]);
        exit(2);
    }
    if ((x0 & 1) || (y0 & 1)) {
        fprintf(stderr, "%s: odd dimension %dx%d doesn't work\n", argv[1], x0, y0);
        exit(2);
    }
    unsigned char *oot = (unsigned char *)malloc(x0 * y0 * 6 / 4);
    rgb_to_yuv(oot, d, x0, y0);
    char *buf = (char *)malloc(strlen(argv[1]) + 5);
    memcpy(buf, argv[1], strlen(argv[1]));
    char *dot = strrchr(buf, '.');
    if (!dot) dot = buf + strlen(buf);
    strcpy(dot, ".yuv");
    FILE *f = fopen(buf, "wb");
    if (!f) {
        fprintf(stderr, "%s: cannot create\n", buf);
        exit(3);
    }
    if (x0 * y0 * 6 / 4 != (int)fwrite(oot, 1, x0 * y0 * 6 / 4, f)) {
        fprintf(stderr, "%s: short write\n", buf);
        exit(3);
    }
    fclose(f);
    return 0;
}

