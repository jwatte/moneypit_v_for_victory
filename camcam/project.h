#if !defined(project_h)
#define project_h

#if !defined(PROJECT_EXPORT)
#if defined(__cplusplus)
#define PROJECT_EXPORT extern "C"
#else
#define PROJECT_EXPORT
#endif
#endif

/* functions for projecting a camera picture onto a horizontal ground plane */

struct ProjectParameters {
    float heightOfCamera;
    float widthRadians;
    float angledDownRadians;
    float desiredOutResolution;
    int inWidth;
    int inHeight;
    int outWidth;
    int outHeight;
};

struct ProjectData {
    float resolution;
    float nearY;
    int inWidth;
    int inHeight;
    int width;
    int height;
    float *yPerScanline;
    float *xPerScanline;
    float *incrementPerScanline;
    //  TODO: derivatives to compensate for distortion
};

#define MISSING_DATA 0x02

/* First, fill out your camera parameters, and bake some 
 * pre-calculated data for generating output images.
 */
PROJECT_EXPORT int make_project_data(
        struct ProjectParameters const *inParams,
        struct ProjectData **outData);

PROJECT_EXPORT void free_project_data(
        struct ProjectData *freeData);

/* Then pass in an input image, and generate the corresponding
 * flat-projected output image.
 */
PROJECT_EXPORT void project_bitmap(
        struct ProjectData const *inData,
        unsigned char const *src,
        unsigned char *dst,
        int bpp);

#endif  //  project_h
