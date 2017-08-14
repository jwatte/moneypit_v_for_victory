#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../stb/stb_image_write.h"
#include "yuv.h"

#define FMT_RGB 0
#define FMT_YUV 1
#define FMT_GRAY 2
#define FMT_GRAU 3
#define FMT_GRAV 4

int main(int argc, char const *argv[]) {
    if (!argv[1] || !strcmp(argv[1], "--help")) {
        fprintf(stderr, "mkpng: create PNG files from RAW files\n");
        fprintf(stderr, "mkpng input.raw [yuv|gray|grau|grav] [width [height [bpp [output.png [yuv]]]]]\n");
        fprintf(stderr, "default output is 'raw.png'; default size 320x240x3 rgb\n");
        exit(1);
    }
    char const *inname = "raw";
    if (argv[1]) {
        inname = argv[1];
        ++argv;
        --argc;
    }
    int format = FMT_RGB;
    if (argv[1]) {
        if (!strcmp(argv[1], "yuv")) {
            format = FMT_YUV;
        } else if (!strcmp(argv[1], "gray")) {
            format = FMT_GRAY;
        } else if (!strcmp(argv[1], "grau")) {
            format = FMT_GRAU;
        } else if (!strcmp(argv[1], "grav")) {
            format = FMT_GRAV;
        } else {
            goto skip_fmt;
        }
        ++argv;
        --argc;
skip_fmt:
        ;
    }
    int width = 320;
    if (argv[1]) {
        width = atoi(argv[1]);
        ++argv;
        --argc;
        if (width < 1) {
            fprintf(stderr, "illegal width: %s\n", argv[0]);
            exit(1);
        }
    }
    int height = 240;
    if (argv[1]) {
        height = atoi(argv[1]);
        ++argv;
        --argc;
        if (height < 1) {
            fprintf(stderr, "illegal height: %s\n", argv[0]);
            exit(1);
        }
    }
    int bpp = 3;
    if (argv[1]) {
        bpp = atoi(argv[1]);
        ++argv;
        --argc;
        if (bpp < 1 || bpp > 4) {
            fprintf(stderr, "illegal bpp: %s\n", argv[0]);
            exit(1);
        }
    }
    char const *outname = "raw.png";
    if (argv[1]) {
        outname = argv[1];
        ++argv;
        --argc;
    }
    if (argv[1]) {
        if (!strcmp(argv[1], "yuv")) {
            format = FMT_YUV;
        } else if (!strcmp(argv[1], "gray")) {
            format = FMT_GRAY;
        } else if (!strcmp(argv[1], "grau")) {
            format = FMT_GRAU;
        } else if (!strcmp(argv[1], "grav")) {
            format = FMT_GRAV;
        } else if (!strcmp(argv[1], "rgb")) {
            format = FMT_RGB;
        }
        else {
            fprintf(stderr, "illegal format: %s\n", argv[1]);
            exit(1);
        }
        ++argv;
        --argc;
    }
    char const *formatNames[] = {
        "rgb", "yuv", "gray", "grau", "grav"
    };
    fprintf(stderr, "input=%s\n", inname);
    fprintf(stderr, "width=%d\n", width);
    fprintf(stderr, "height=%d\n", height);
    fprintf(stderr, "bpp=%d\n", bpp);
    fprintf(stderr, "output=%s\n", outname);
    fprintf(stderr, "format=%s\n", formatNames[format]);
    FILE *f = fopen(inname, "rb");
    if (!f) {
        fprintf(stderr, "%s: cannot read\n", inname);
        exit(2);
    }
    char *d = (char *)malloc(width * height * bpp);
    //  this may do a short read for YUV inputs
    fread(d, 1, width * height * bpp, f);
    fclose(f);
    if (format == FMT_YUV) {
        char *s = (char *)malloc(width * height * 6 / 4);
        memcpy(s, d, width * height * 6 / 4);
        yuv_to_rgb((unsigned char const *)s, (unsigned char *)d, width, height);
    } else if (format == FMT_GRAY) {
        bpp = 1;
    } else if (format == FMT_GRAU) {
        bpp = 1;
        d += width * height;
        width /= 2;
        height /= 2;
    } else if (format == FMT_GRAV) {
        bpp = 1;
        d += width * height * 5 / 4;
        width /= 2;
        height /= 2;
    }
    int ok = stbi_write_png(outname, width, height, bpp, d, 0);
    if (!ok) {
        fprintf(stderr, "%s: failed to write\n", outname);
        return 1;
    }
    return 0;
}
