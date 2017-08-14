#include <stdio.h>
#include <stdlib.h>

unsigned char Y[320*240];
unsigned char u[160*120];
unsigned char v[160*120];

#define COLORGAIN 0.25f

int main() {
    for (int y = 0; y < 15; ++y) {
        for (int x = 0; x < 20; ++x) {
            unsigned char yy = (unsigned char)((rand() & 0x7f) + 64);
            unsigned char uu = (unsigned char)(((rand() & 0x7f) - 64) * COLORGAIN + 128);
            unsigned char vv = (unsigned char)(((rand() & 0x7f) - 64) * COLORGAIN + 128);
            if ((rand() & 255) < 80) {
                yy = 180;
                uu = 128-35;
                vv = 128+10;
            } else {
                yy = 0;
                uu = 64;
                vv = 64;
            }
            for (int r = 0; r < 16; ++r) {
                for (int c = 0; c < 16; ++c) {
                    Y[(y*16+r)*320 + (x*16)+c] = yy;
                }
            }
            for (int r = 0; r < 8; ++r) {
                for (int c = 0; c < 8; ++c) {
                    u[(y*8+r)*160 + (x*8)+c] = uu;
                    v[(y*8+r)*160 + (x*8)+c] = vv;
                }
            }
        }
    }
    if (320*240 != fwrite(Y, 1, 320*240, stdout)) {
        fprintf(stderr, "Y: short write\n");
        return 1;
    }
    if (160*120 != fwrite(u, 1, 160*120, stdout)) {
        fprintf(stderr, "u: short write\n");
        return 1;
    }
    if (160*120 != fwrite(v, 1, 160*120, stdout)) {
        fprintf(stderr, "v: short write\n");
        return 1;
    }
    return 0;
}

