#include "detect_inner.h"
#include "../stb/stb_image.h"
#include "../stb/stb_image_write.h"
#include "settings.h"
#include "project.h"
#include "queue.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>


void crop_center(unsigned char *buf, int inx, int iny, int outx, int outy, int bpp) {
    int rowbytes = inx * bpp;
    int dstbytes = outx * bpp;
    unsigned char *s = buf + (iny - outy) * rowbytes + (inx - outx) * bpp;
    for (int r = 0; r < outy; ++r) {
        memmove(buf, s, dstbytes);
        buf += dstbytes;
        s += rowbytes;
    }
}

char const *squarename = NULL;
char const *dumpname = NULL;

int main(int argc, char const *argv[]) {
    load_settings("camcam");
    read_analyzer_settings();
    if (argv[1] && !strcmp(argv[1], "dump")) {
        if (argc < 4) {
            goto usage;
        }
        dumpname = argv[2];
        argv += 2;
        argc -= 2;
    }
    if (argv[1] && !strcmp(argv[1], "square")) {
        if (argc < 4) {
            goto usage;
        }
        squarename = argv[2];
        argv += 2;
        argc -= 2;
    }
    if (argc != 2 || argv[1][0] == '-') {
usage:
        fprintf(stderr, "usage: mkdetect [dump output.png] input.{png,yuv}\n");
        exit(1);
    }
    int x = 0, y = 0, n = 0;
    unsigned char *buf = 0;
    if (!strchr(argv[1], '.')) {
        fprintf(stderr, "%s: unknown file extension\n", argv[1]);
        exit(1);
    }
    if (!strcmp(strrchr(argv[1], '.'), ".yuv")) {
        FILE *f = fopen(argv[1], "rb");
        fseek(f, 0, 2);
        unsigned long l = ftell(f);
        rewind(f);
        buf = (unsigned char *)malloc(l);
        if (l != fread(buf, 1, l, f)) {
            free(buf);
            buf = NULL;
        }
        fclose(f);
        int npix = l * 2 / 3;
        //  assume 4 : 3 ratio
        //  w * (w * 3 / 4) = npix
        //  w * w = npix * 4 / 3
        //  w = sqrtf(npis * 4 / 3)
        n = 1;
        x = (int)floorf(sqrtf(npix * 4 / 3));
        y = npix / x;
        fprintf(stderr, "%s: assuming %dx%d size\n", argv[1], x, y);
    } else {
        buf = stbi_load(argv[1], &x, &y, &n, 1);
    }
    if (!buf) {
        fprintf(stderr, "%s: could not load image\n", argv[1]);
        exit(2);
    }
    if (x < PROC_WIDTH || y < PROC_HEIGHT) {
        fprintf(stderr, "%s: must be at least %dx%d pixels\n", argv[1], PROC_WIDTH, PROC_HEIGHT);
        exit(2);
    }
    if (x > PROC_WIDTH || y > PROC_HEIGHT) {
        crop_center(buf, x, y, PROC_WIDTH, PROC_HEIGHT, 1);
        fprintf(stderr, "%s: cropping from %dx%d to %dx%d\n", argv[1], x, y, PROC_WIDTH, PROC_HEIGHT);
    }
    unsigned char *an = (unsigned char *)malloc(PROC_WIDTH * PROC_HEIGHT);
    detect_color_inner(buf, an, PROC_WIDTH, PROC_HEIGHT);
    if (dumpname) {
        if (!stbi_write_png(dumpname, PROC_WIDTH, PROC_HEIGHT, 1, an, 0)) {
            fprintf(stderr, "%s: could not write file\n", dumpname);
            exit(3);
        }
    }
    DetectOutput output = { 0 };
    Frame *f = new Frame(PROJECT_WIDTH * PROJECT_HEIGHT);
    f->width_ = PROJECT_WIDTH;
    f->height_ = PROJECT_HEIGHT;
    determine_steering(an, x, y, f, &output);
    if (squarename) {
        unsigned char *sqproj = f->data_;
        int sw = PROJECT_WIDTH;
        int sh = PROJECT_HEIGHT;
        for (unsigned char *ptr = sqproj, *end = sqproj + sw*sh; ptr != end; ++ptr) {
            if (*ptr == MISSING_DATA) {
                *ptr = 66;
            }
        }
        stbi_write_png(squarename, sw, sh, 1, sqproj, 0);
    }

    if (output.num_clusters) {
        fprintf(stderr, "steer=%.2f\n", output.steer);
        fprintf(stderr, "drive=%.2f\n", output.drive);
        fprintf(stderr, "num_clusters=%d\n", output.num_clusters);
        for (int i = 0; i != output.num_clusters; ++i) {
            Cluster const &c = output.clusters[i];
            fprintf(stderr, "cluster %d  x=(%d-%d) y=(%d-%d) count=%d label=%d\n",
                    i, c.minx, c.maxx, c.miny, c.maxy, c.count, c.label);
        }
    }

    return 0;
}
