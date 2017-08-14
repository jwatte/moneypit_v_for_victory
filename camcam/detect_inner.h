#if !defined(detect_inner_h)
#define detect_inner_h

#if defined(__cplusplus)
#if !defined(DETECTINNER_EXPORT)
#define DETECTINNER_EXPORT extern "C"
#endif
#else
#if !defined(DETECTINNER_EXPORT)
#define DETECTINNER_EXPORT
#endif
#endif

#define PROC_WIDTH 320
#define PROC_HEIGHT 240
#define PROJECT_WIDTH 128
#define PROJECT_HEIGHT 128

extern float detect_ygain;
extern float detect_cgain;
extern float detect_d2;
extern float detect_ycenter;
extern float detect_ucenter;
extern float detect_vcenter;

struct Frame;

struct Cluster {
    int minx;
    int maxx;
    int miny;
    int maxy;
    int count;
    unsigned char label;
};
struct DetectOutput {
    float steer;
    float drive;
    int num_clusters;
    Cluster const *clusters;
};
DETECTINNER_EXPORT int determine_steering(unsigned char const *analyze_output, int width, int height, struct Frame *frame, DetectOutput *out);
DETECTINNER_EXPORT void detect_color_inner(unsigned char const *bptr, unsigned char *dcls, int width, int height);
DETECTINNER_EXPORT void read_analyzer_settings();
DETECTINNER_EXPORT unsigned char *get_sqproj(int *ow, int *oh);
DETECTINNER_EXPORT unsigned char *get_sqproj_work(int *ow, int *oh);
DETECTINNER_EXPORT int paint_clusters(unsigned char *buf, int w, int h, Cluster const *cl, int ncl);

struct Gains {
    float speed_gain;
    float turn_gain;
    float turn_squared_gain;
};
DETECTINNER_EXPORT void steer_get_gains(Gains &oGains);
DETECTINNER_EXPORT void steer_set_gains(Gains const &iGains);


#endif  //  detect_inner_h

