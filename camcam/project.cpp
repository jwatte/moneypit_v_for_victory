#include "project.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
//  apt install libglm-dev
#define GLM_FORCE_RADIANS 1
#include <glm/fwd.hpp>
#include <glm/vec3.hpp>
#include <glm/mat4x4.hpp>
//#include <glm/trigonometric.hpp>
#include <glm/gtc/matrix_transform.hpp>


using namespace glm;


int make_project_data(
        struct ProjectParameters const *inParams,
        struct ProjectData **outData)
{
    if (inParams->inWidth < 2 || inParams->inHeight < 2 || inParams->outWidth < 2 || inParams->outHeight < 2) {
        return -1;
    }
    if (inParams->desiredOutResolution < 1e-3f) {
        return -1;
    }
    if (inParams->angledDownRadians < 0.0f) {
        return -1;
    }
    if (inParams->widthRadians < 1e-3f) {
        return -1;
    }
    if (inParams->heightOfCamera < 1e-3) {
        return -1;
    }

    *outData = NULL;
    float heightOverWidth = float(inParams->inHeight)/float(inParams->inWidth);

    float fovy = 2.0f * atanf(tanf(inParams->widthRadians * 0.5f) * heightOverWidth);
    mat4x4 proj(perspective(fovy, 1.0f / heightOverWidth, inParams->heightOfCamera * 0.5f, inParams->heightOfCamera * 100.0f));
    //  Default is Y-up; I want Z-up, so rotate by -pi/2
    mat4x4 view(rotate(mat4x4(1.0f), inParams->angledDownRadians - (float(M_PI) * 0.5f), vec3(1.0f, 0.0f, 0.0f))
        * translate(mat4x4(1.0f), vec3(0.0f, 0.0f, -inParams->heightOfCamera)));

    float vpWidth = inParams->inWidth;
    float vpHeight = vpWidth * heightOverWidth;
    vec4 viewport(0, 0, vpWidth, vpHeight);

    //  unproject from screen center-bottom to world vector
    vec3 ll(normalize(unProject(vec3(vpWidth * 0.5f, 0, 1.0f), view, proj, viewport)));
    if (ll.z > -1e-5) {
        return -2;
    }
    //  project world vector to ground plane
    ll = vec3(0.0f, 0.0f, inParams->heightOfCamera) + (ll * (inParams->heightOfCamera / -ll.z));
    float baseY = ll.y;

    ProjectData *ret = (ProjectData *)malloc(sizeof(ProjectData)
        + sizeof(float) * inParams->outHeight * 3);
    ret->yPerScanline = (float *)&ret[1];
    ret->xPerScanline = ret->yPerScanline + inParams->outHeight;
    ret->incrementPerScanline = ret->xPerScanline + inParams->outHeight;

    float leftX = -inParams->outWidth * inParams->desiredOutResolution * 0.5f;
    for (int y = 0; y < inParams->outHeight; ++y) {
        float wY = baseY + y * inParams->desiredOutResolution;
        vec3 win(project(vec3(leftX, wY, 0.0f), view, proj, viewport));
        int outy = inParams->outHeight - y - 1;
        ret->yPerScanline[outy] = vpHeight - win.y;
        ret->xPerScanline[outy] = win.x;
        ret->incrementPerScanline[outy] = 2.0f * (vpWidth * 0.5f - win.x) / inParams->outWidth;
        //  todo: fit a polynomial to the distortion and get the derivative
    }
    ret->resolution = inParams->desiredOutResolution;
    ret->nearY = baseY;
    ret->width = inParams->outWidth;
    ret->height = inParams->outHeight;
    ret->inHeight = inParams->inHeight;
    ret->inWidth = inParams->inWidth;

    *outData = ret;
    return 0;
}           

void free_project_data(
        struct ProjectData *freeData) {
    free(freeData);
}




void project_bitmap(
        struct ProjectData const *inData,
        unsigned char const *src,
        unsigned char *dst,
        int bpp)
{
    int idh = inData->height;
    int idw = inData->width;
    for (int y = 0; y < idh; ++y) {
        int yy = inData->yPerScanline[y];
        unsigned char *scanline = dst + y * idw * bpp;
        if (yy < 0 || yy >= inData->inHeight) {
            memset(scanline, MISSING_DATA, idw * bpp);
            continue;
        }
        unsigned char const *srcline = src + yy * inData->inWidth * bpp;
        float xx = inData->xPerScanline[y];
        float xd = inData->incrementPerScanline[y];
        for (int x = 0; x < idw; ++x) {
            int xxi = (int)xx;
            if (xxi >= inData->inWidth) {
                memset(scanline, MISSING_DATA, (idw - x) * bpp);
                break;
            }
            if (xxi < 0) {
                for (int j = 0; j != bpp; ++j) {
                    scanline[j] = MISSING_DATA;
                }
            } else {
                int ix = xxi * bpp;
                for (int j = 0; j != bpp; ++j) {
                    scanline[j] = srcline[ix];
                }
            }
            scanline += bpp;
            xx += xd;
        }
    }
}


