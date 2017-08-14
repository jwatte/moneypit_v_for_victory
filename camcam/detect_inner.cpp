#include "detect_inner.h"
#include "settings.h"
#include "project.h"
#include "queue.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include "../stb/stb_image_write.h"


float detect_ygain = 1.0f;
float detect_cgain = 3;
float detect_d2 = 1600;
float detect_ycenter = 180;
float detect_ucenter = -28;
float detect_vcenter = 13;


#define MAX_HEIGHT PROC_HEIGHT

static float speed_gain = 1.0f;
static float turn_gain = 0.15f;
static float turn_squared_gain = 0.1f;
static bool complainedNoClusters = false;

static int dwidth = 0;
static int dheight = 0;
static ProjectData *dproject;

#define PROJECT_RESOLUTION 0.67f
#define CAMERA_HEIGHT 25.0f
#define CAMERA_WIDTH_RADIANS (60.0f * 3.1415927f / 180.0f)
#define ANGLED_DOWN_RADIANS (25.0f * 3.1415927f / 180.0f)

#define INTERCEPT_GAIN 1.0f
#define SLOPE_GAIN 0.2f


#define MIN_CLUSTER_SIZE 8
#define MAX_CLUSTERS 255
Cluster g_clusters[MAX_CLUSTERS];

unsigned char flat_map[PROJECT_WIDTH * PROJECT_HEIGHT];
unsigned char flat_work[PROJECT_WIDTH * PROJECT_HEIGHT];

unsigned char *get_sqproj(int *ow, int *oh) {
    *ow = PROJECT_WIDTH;
    *oh = PROJECT_HEIGHT;
    return flat_map;
}

unsigned char *get_sqproj_work(int *ow, int *oh) {
    *ow = PROJECT_WIDTH;
    *oh = PROJECT_HEIGHT;
    return flat_work;
}

void steer_get_gains(Gains &oGains) {
    oGains.speed_gain = speed_gain;
    oGains.turn_gain = turn_gain;
    oGains.turn_squared_gain = turn_squared_gain;
}

void steer_set_gains(Gains const &iGains) {
    speed_gain = iGains.speed_gain;
    turn_gain = iGains.turn_gain;
    turn_squared_gain = iGains.turn_squared_gain;
    set_setting_float("speed_gain", speed_gain);
    set_setting_float("turn_gain", turn_gain);
    set_setting_float("turn_squared_gain", turn_squared_gain);
}

int paint_clusters(unsigned char *buf, int w, int h, Cluster const *cl, int ncl) {
    for (int cc = 0; cc != ncl; ++cc) {
        if (cl->miny < 0 || cl->maxy >= w || cl->minx < 0 || cl->maxx >= h) {
            return -1;
        }
        if (cl->label) {
            for (int y = cl->miny; y <= cl->maxy; ++y) {
                for (int x = cl->minx; x <= cl->maxx; ++x) {
                    if (buf[w*y+x] == 0) {
                        buf[w*y+x] = 32+cl->label*3;
                    }
                }
            }
        }
        ++cl;
    }
    return 0;
}

//  build connected clusters
int detect_clusters(
        unsigned char label,
        unsigned char const *input,
        int width,
        int height,
        unsigned char *work_area,
        Cluster *output,
        int output_count,
        int min_size,
        int *out_errors)
{
    int num_clusters = 0;
    int num_errors = 0;
    Cluster *clust = NULL;
    unsigned char cur_index = 0;
    unsigned char const *src = input;
    unsigned char *ptr = work_area;
    for (int r = 0; r < height; ++r) {
        clust = NULL;
        for (int c = 0; c < width; ++c) {
            unsigned char top_index = (r > 0) ? ptr[-width] : 0;
            if (*src == label) {
                //  I'm not already in a run-length cluster
                if (!clust) {
                    if (top_index) {
                        cur_index = top_index;
                        clust = &output[cur_index-1];
                    } else {
                        for (int i = num_clusters-1; i != -1; --i) {
                            if (!output[i].label) {
                                clust = &output[i];
                                break;
                            }
                        }
                        if (!clust) {
                            if(num_clusters < output_count) {
                                clust = &output[num_clusters];
                                ++num_clusters;
                                clust->label = num_clusters;
                            }
                        }
                        if (clust) {
                            cur_index = clust - output + 1;
                            clust->minx = c;
                            clust->maxx = c;
                            clust->miny = r;
                            clust->maxy = r;
                            clust->count = 0;
                            clust->label = cur_index;
                        } else {
                            fprintf(stderr, "Too many indifidual clusters: %d\n", num_clusters);
                            ++num_errors;
                            cur_index = 0;
                        }
                    }
                }
            } else {
                clust = NULL;
                cur_index = 0;
            }
            if (clust) {
                clust->count++;
                clust->maxx = std::max(clust->maxx, c);
                clust->maxy = std::max(clust->maxy, r);
                if (top_index && (top_index != cur_index)) {
                    //  join these clusters
                    Cluster *top = &output[top_index-1];
                    top->minx = std::min(clust->minx, top->minx);
                    top->miny = std::min(clust->miny, top->miny);
                    top->maxx = std::max(clust->maxx, top->maxx);
                    top->maxy = std::max(clust->maxy, top->maxy);
                    top->count += clust->count;
                    //  remove the current cluster
                    int sc = clust->minx - c;
                    int ec = clust->maxx - c;
                    //  TODO: I could just mark these joined, and not actually 
                    //  re-write the map until at the end. For very complex, 
                    //  winding clusters, that might save some processing.
                    for (int dr = clust->miny - r; dr <= 0; ++dr) {
                        for (int dc = sc; dc <= ec; ++dc) {
                            if (ptr[dc + dr * width] == cur_index) {
                                ptr[dc + dr * width] = top_index;
                            }
                        }
                    }
                    clust->label = 0;
                    clust->count = 0;
                    clust = top;
                    cur_index = top_index;
                }
            }
            *ptr = cur_index;
            ++src;
            ++ptr;
        }
    }
    for (int i = 0; i != num_clusters; ++i) {
        if (!output[i].count || (output[i].count < min_size)) {
            output[i].label = 0;
        } else {
            output[i].label = i + 1;
        }
    }
    if (num_clusters > 0) {
        std::sort(output, &output[num_clusters], [](Cluster const &a, Cluster const &b) {
                if (a.label == 0) {
                    return false;
                }
                if (b.label == 0) {
                    return true;
                }
                int d = a.miny - b.miny;
                if (d < 0) {
                    return true;
                }
                if (d > 0) {
                    return false;
                }
                if (a.count > b.count) {
                    return true;
                }
                return false;
            });
        while (num_clusters > 0) {
            if (!output[num_clusters-1].label) {
                --num_clusters;
            } else {
                break;
            }
        }
    }
    if (out_errors) {
        *out_errors = num_errors;
    }
    return num_clusters;
}

int determine_steering(unsigned char const *analyze_output, int width, int height, Frame *flatFrame, DetectOutput *out) {

    out->steer = 0;
    out->drive = 0;

    //  project just to see if it works
    if (width != dwidth || height != dheight) {
        fprintf(stderr, "creating projection plan for %d->%d to %d->%d\n", width, height, PROJECT_WIDTH, PROJECT_HEIGHT);
        dwidth = width;
        dheight = height;
        free_project_data(dproject);
        ProjectParameters params = { 0 };
        params.heightOfCamera = CAMERA_HEIGHT;
        params.widthRadians = CAMERA_WIDTH_RADIANS;
        params.angledDownRadians = ANGLED_DOWN_RADIANS;
        params.desiredOutResolution = PROJECT_RESOLUTION;
        params.inWidth = width;
        params.inHeight = height;
        params.outWidth = PROJECT_WIDTH;
        params.outHeight = PROJECT_HEIGHT;
        if (make_project_data(&params, &dproject) || !dproject) {
            fprintf(stderr, "Could not create deprojection plan!\n");
            dproject = NULL;
            return -1;
        }
    }
    if (!dproject) {
        return -2;
    }
    unsigned char *flatOutput = flatFrame ? flatFrame->data_ : flat_map;
    project_bitmap(dproject, analyze_output, flatOutput, 1);
    int n_errors = 0;
    int n_clusters = detect_clusters(255, flatOutput, PROJECT_WIDTH, PROJECT_HEIGHT, flat_work, g_clusters, MAX_CLUSTERS, MIN_CLUSTER_SIZE, &n_errors);
    //  ignore the error of "too many clusters," if it happens at all (very unlikely)
    Cluster const *const c = g_clusters;
    out->num_clusters = n_clusters;
    out->clusters = g_clusters;
    if (!n_clusters) {
        if (!complainedNoClusters) {
            fprintf(stderr, "no clusters! cannot steer\n");
            complainedNoClusters = true;
        }
        out->steer = 0.4f;
        out->drive = 0.5f;
        return -3;
    }
    if (complainedNoClusters) {
        fprintf(stderr, "found clusters for steering again\n");
        complainedNoClusters = false;
    }

    float turn = 0.0f;
    float speed = 0.0f;

    if (n_clusters > 1) {
        //  linear regression on X as function of Y
        float sumx = 0;
        float sumy = 0;
        float weight = 0;
        float sumx2 = 0;
        float sumy2 = 0;
        float sumxy = 0;
        for (int i = 0; i != n_clusters; ++i) {
            float count = c[i].count;
            float x = (c[i].minx + c[i].maxx) * 0.5f;
            sumx += x * count;
            float y = (c[i].miny + c[i].miny) * 0.5f;
            sumy += y * count;
            sumxy += x * y * count;
            sumx2 += x * x * count;
            sumy2 += y * y * count;
            weight += count;
        }
        float div = (weight * sumy2 - sumy * sumy);
        if (fabsf(div) < 1e-3f) {
            //  horizontal? Pick the first cluster
            goto one_cluster;
        } else {
            float a = (sumx * sumy2 - sumy * sumxy) / div;
            float b = (weight * sumxy - sumy * sumx) / div;
            //  X = a + b Y
            //  turn towards intercept, and turn in direction of line
            turn = (a * 2 - PROJECT_HEIGHT) / PROJECT_HEIGHT * INTERCEPT_GAIN
                - b * SLOPE_GAIN;
            //  how far can I see?
            float topBlobY = float(c[0].miny) / PROJECT_HEIGHT;
            speed = 2.5f * (1.0f - topBlobY);
        }
    } else {
one_cluster:
        //  only one blob? turn towards it, I guess?
        turn = 2.0f * float((c[0].minx + c[0].maxx) - PROJECT_WIDTH) / PROJECT_WIDTH;
        turn = turn * (1.0f + (c[0].miny + c[0].maxy) / PROJECT_HEIGHT);
        speed = 0.7f;
    }

    turn = turn * turn * turn_squared_gain * ((turn < 0) ? -1 : 1) + turn * turn_gain;
    speed = speed * speed_gain;

    turn = turn * (1 + speed);
    if (speed < 0.2f) {
        speed = 0.2f;
    }
    out->steer = turn;
    out->drive = speed;

    return 0;
}

static inline unsigned char classify(float y, float u, float v) {
    //  Darker colors have lower U / V swing, so make
    //  some adjustments to the assumed center. Don't 
    //  adjust fully proportionally, to avoid everything 
    //  matching black.
    float brightAdjust = (y + 20) / (detect_ycenter + 20);
    float dy = (y - detect_ycenter);
    float du = (u - detect_ucenter * brightAdjust);
    float dv = (v - detect_vcenter * brightAdjust);
    float d = dy*dy*detect_ygain + du*du*detect_cgain + dv*dv*detect_cgain;
    if (d < detect_d2) {
        return 255;
    }
    return 0;
}

void detect_color_inner(unsigned char const *bptr, unsigned char *dcls, int width, int height) {
    unsigned char const *y = (unsigned char const *)bptr;
    unsigned char const *u = (unsigned char const *)(y + width * height);
    unsigned char const *v = (unsigned char const *)(u + width * height / 4);
    for (int r = 0; r < height; r += 2) {
        for (int c = 0; c < width; c += 2) {
            float u0 = (float)*u - 128.0f;
            float v0 = (float)*v - 128.0f;
            float y0 = y[0];
            float y1 = y[1];
            float y2 = y[width];
            float y3 = y[width+1];
            dcls[0] = classify(y0, u0, v0);
            dcls[1] = classify(y1, u0, v0);
            dcls[width] = classify(y2, u0, v0);
            dcls[width+1] = classify(y3, u0, v0);
            dcls += 2;
            y += 2;
            u++;
            v++;
        }
        dcls += width;
        y += width;
    }
}

void read_analyzer_settings() {
    detect_ycenter = get_setting_float("detect_ycenter", detect_ycenter);
    detect_ucenter = get_setting_float("detect_ucenter", detect_ucenter);
    detect_vcenter = get_setting_float("detect_vcenter", detect_vcenter);
    detect_ygain = get_setting_float("detect_ygain", detect_ygain);
    detect_cgain = get_setting_float("detect_cgain", detect_cgain);
    detect_d2 = get_setting_float("detect_d2", detect_d2);
    speed_gain = get_setting_float("speed_gain", speed_gain);
    turn_gain = get_setting_float("turn_gain", turn_gain);
    turn_squared_gain = get_setting_float("turn_squared_gain", turn_squared_gain);
    fprintf(stderr,
            "analyzer_settings: speed_gain=%.2f turn_gain=%.2f turn_squared_gain=%.2f ycenter=%.2f ucenter=%.2f vcenter=%.2f ygain=%.2f cgain=%.2f d2=%.0f\n",
            speed_gain, turn_gain, turn_squared_gain, detect_ycenter, detect_ucenter, detect_vcenter, detect_ygain, detect_cgain, detect_d2);
}


