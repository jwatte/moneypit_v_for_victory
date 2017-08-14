#include "detect.h"
#include "interface/mmal/mmal.h"
#include "interface/mmal/mmal_buffer.h"
#include "settings.h"
#include "queue.h"
#include "navigation.h"
#include "detect_inner.h"
#include "pipeline.h"
#include <pthread.h>
#include <stdio.h>
#include <math.h>


//  bad from the inner loop, but what can you do?
#include "../stb/stb_image_write.h"


uint64_t usspent;
int framesAnalyzed;
void *volatile browse_buffer;

static DetectOutput lastSteering;
static char const *detectDump;

FrameQueue analyzer_input_queue(2, PROC_WIDTH * PROC_HEIGHT * 6 / 4, PROC_WIDTH, PROC_HEIGHT, 2);
FrameQueue analyzer_analyzed_queue(1, PROC_WIDTH * PROC_HEIGHT, PROC_WIDTH, PROC_HEIGHT, 1);
FrameQueue flat_map_queue(1, PROJECT_WIDTH * PROJECT_HEIGHT, PROJECT_WIDTH, PROJECT_HEIGHT, 1);


int num_analyzed;
uint64_t analyze_start;
bool complainedNoSteering = false;

static unsigned char analyze_overflow[PROC_WIDTH * PROC_HEIGHT];


void detect_get_last_output(DetectOutput *oDetect) {
    *oDetect = lastSteering;
}

void detect_write_params(char const *dumpName) {
    detectDump = dumpName;
}

void analyze_data(Frame *iframe, Frame *dframe) {
    unsigned char *dcls = dframe ? dframe->data_ : analyze_overflow;
    //  turn UYV into "is yellow"
    detect_color_inner(iframe->data_, dcls, PROC_WIDTH, PROC_HEIGHT);
    DetectOutput output = { 0 };
    Frame *flatFrame = flat_map_queue.beginWrite();
    if (determine_steering(dcls, PROC_WIDTH, PROC_HEIGHT, flatFrame, &output)) {
        if (!complainedNoSteering) {
            fprintf(stderr, "Could not determine steering\n");
            complainedNoSteering = true;
        }
    } else {
        if (complainedNoSteering) {
            fprintf(stderr, "could determine steering again\n");
        }
        complainedNoSteering = false;
    }
    if (flatFrame) {
        iframe->link(flatFrame);
    }
    lastSteering = output;
    navigation_set_image(output.drive, output.steer);

    char const *dd = detectDump;
    if (dd) {
        detectDump = NULL;
        FILE *f = fopen(dd, "wb");
        if (!f) {
            fprintf(stderr, "Could not write %s\n", dd);
        } else {
            fprintf(stderr, "Writing %s\n", dd);
            fprintf(f, "speed %f turn %f num_clusters %d\n", output.drive, output.steer, output.num_clusters);
            for (int i = 0; i != output.num_clusters; ++i) {
                Cluster const *c = output.clusters + i;
                fprintf(f, "cluster %d  x(%d-%d) y(%d-%d) count %d label %d\n", i, c->minx, c->maxx, c->miny, c->maxy, c->count, c->label);
            }
            fclose(f);
        }
        if (!stbi_write_png("/tmp/debug-dcls.png", PROC_WIDTH, PROC_HEIGHT, 1, dcls, 0)) {
            fprintf(stderr, "Could not write /tmp/debug-dcls.png\n");
        } else {
            fprintf(stderr, "dump: wrote /tmp/debug-dcls.png\n");
        }
        int sw, sh;
        unsigned char const *dsq = get_sqproj(&sw, &sh);
        if (!stbi_write_png("/tmp/debug-inner-square.png", sw, sh, 1, dsq, 0)) {
            fprintf(stderr, "Could not write /tmp/debug-inner-square.png\n");
        } else {
            fprintf(stderr, "dump: wrote /tmp/debug-inner-square.png\n");
        }
        dsq = get_sqproj_work(&sw, &sh);
        if (!stbi_write_png("/tmp/debug-inner-work.png", sw, sh, 1, dsq, 0)) {
            fprintf(stderr, "Could not write /tmp/debug-inner-work.png\n");
        } else {
            fprintf(stderr, "dump: wrote /tmp/debug-inner-work.png\n");
        }
    }
}

void analyze_buffer(Pipeline *, Frame *&inFrame, Frame *&outFrame, void *) {
    uint64_t usstart = vcos_getmicrosecs64();
    void *bbd = browse_buffer;
    if (bbd) {
        memcpy(inFrame->data_, bbd, inFrame->size_);
    }
    analyze_data(inFrame, outFrame);
    if (outFrame) {
        outFrame->link(inFrame);
        inFrame = NULL;
    }
    uint64_t usstop = vcos_getmicrosecs64();
    usspent += usstop - usstart;
    ++framesAnalyzed;
    if ((framesAnalyzed >= 500) || (usspent >= 10000000)) {
        fprintf(stderr, "analysis avg: %.3f ms\n", usspent * 0.001 / framesAnalyzed);
        int stin, stout, stfl;
        analyzer_input_queue.getStats(stin, stout, stfl);
        fprintf(stderr, "input_queue: %d in, %d out, %d inflight\n", stin, stout, stfl);
        analyzer_analyzed_queue.getStats(stin, stout, stfl);
        fprintf(stderr, "analyzed_queue: %d in, %d out, %d inflight\n", stin, stout, stfl);
        flat_map_queue.getStats(stin, stout, stfl);
        fprintf(stderr, "flat_queue: %d in, %d out, %d inflight\n", stin, stout, stfl);
        framesAnalyzed = 0;
        usspent = 0;
    }
}

Pipeline analyzer_input_pipeline(analyze_buffer);

void start_analyzer() {
    read_analyzer_settings();
    fprintf(stderr, "Starting analyzer; %.2f %.2f %.2f / %.2f %.2f %.2f\n",
            detect_ycenter, detect_ucenter, detect_vcenter,
            detect_ygain, detect_cgain, detect_d2);
    analyzer_input_pipeline.connectInput(&analyzer_input_queue);
    analyzer_input_pipeline.connectOutput(&analyzer_analyzed_queue);
    analyzer_input_pipeline.start(NULL);
}

void stop_analyzer() {
    fprintf(stderr, "Stopping analyzer\n");
    analyzer_input_pipeline.stop();
    fprintf(stderr, "Analyzer stopped\n");
}

int analyzer_consume(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer) {
    uint64_t usstart = vcos_getmicrosecs64();
    if (num_analyzed == 200 || (usstart - analyze_start > 10000000)) {
        time_t t;
        time(&t);
        char buf[100];
        strftime(buf, 100, "%H:%M:%S", localtime(&t));
        fprintf(stderr, "%s: capture fps: %.1f\n", buf, float(num_analyzed) * 1e6 / float(usstart - analyze_start));
        analyze_start = 0;
        num_analyzed = 0;
    }
    if (!analyze_start) {
        analyze_start = usstart;
    }
    Frame *in = analyzer_input_queue.beginWrite();
    static int nTotal;
    static int nMissed = 0;
    ++nTotal;
    if (in) {
        ++num_analyzed;
        mmal_buffer_header_mem_lock(buffer);
        memcpy(in->data_, buffer->data + buffer->offset, in->size_);
        mmal_buffer_header_mem_unlock(buffer);
        in->endWrite();
    } else {
        ++nMissed;
        if (!(nMissed & 31)) {
            fprintf(stderr, "analyzer_consume(): %d/%d missed writes\n", nMissed, nTotal);
        }
    }
    return 0;
}


