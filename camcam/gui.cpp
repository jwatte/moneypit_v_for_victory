#include "gui.h"
#include "detect_inner.h"
#include "detect.h" //  browse_buffer and friends
#include "widget.h"
#include "serport.h"
#include "settings.h"
#include "../stb/stb_image_write.h"
#include "navigation.h"
#include "queue.h"
#include "yuv.h"

#include <GL/freeglut.h>
#include <math.h>
#include <assert.h>
#include <string.h>
#include <sys/signal.h>
#include <dirent.h>
#include <sys/stat.h>
#include <vector>
#include <unistd.h>
#include <atomic>
#include <string>
#include <list>
#include <algorithm>


#define MAX_BROWSE_FILES 300
#define WIN_WIDTH 1024
#define WIN_HEIGHT 640
#define SCREEN_WIDTH 1280
#define SCREEN_HEIGHT 800
#define SLIDER_MAX_D2 14400


extern void set_recording(bool rec);

int snapshotState;
unsigned char snapshot_yuv[PROC_WIDTH * PROC_HEIGHT * 2];
bool learning = false;
char learnPrefix[100] = "learning";
int learnIndex = 0;
uint64_t lastLearningSnapshot = 0;
static bool debugDump;

struct BrowseFile {
    public:
        BrowseFile() : yuv(NULL) {}
        ~BrowseFile() { if (yuv) free(yuv); }
        unsigned char *yuv;
        std::string path;
};

std::vector<BrowseFile *> browseFiles;
bool browseMode = false;
int browseIndex = 0;

void set_browse_mode(bool b) {
    if (browseFiles.size() == 0) {
        b = false;
    }
    browseMode = b;
    browseIndex = 0;
    browse_buffer = b ? browseFiles[browseIndex]->yuv : NULL;
}

void select_browse_file(int ix) {
    if (ix && ix == browseIndex) {
        return;
    }
    fprintf(stderr, "select_browse_file(%d)\n", ix);
    int sz = (int)browseFiles.size() - 1;
    if (sz == -1) {
        set_browse_mode(false);
        return;
    }
    browseIndex = (ix < 0) ? 0 : (ix > sz) ? sz : ix;
    browse_buffer = browseMode ? browseFiles[ix]->yuv : NULL;
    fprintf(stderr, "browse file: %s\n", browseMode ? browseFiles[ix]->path.c_str() : "none");
    if (snapshotState) {
        snapshotState = 1;
    }
}

void clear_browse_files() {
    browse_buffer = NULL;
    browseIndex = 0;
    fprintf(stderr, "clear_browse_files()\n");
    for (auto p : browseFiles) {
        delete p;
    }
    browseFiles.resize(0);
}

bool read_file(char const *path, unsigned char **odata) {
    fprintf(stderr, "read_file(%s)\n", path);
    FILE *f = fopen(path, "rb");
    if (!f) {
        return false;
    }
    fseek(f, 0, 2);
    long l = ftell(f);
    rewind(f);
    *odata = (unsigned char *)malloc(l);
    if (l != (long)fread(*odata, 1, l, f)) {
        fprintf(stderr, "%s: short read\n", path);
        free(odata);
        *odata = NULL;
        fclose(f);
        return false;
    }
    fclose(f);
    return true;
}

int read_browse_files() {
    fprintf(stderr, "read_browse_files()\n");
    DIR *d = opendir("/var/tmp/mpq");
    struct dirent *dent = NULL;
    set_browse_mode(false);
    std::vector<BrowseFile *> newList;
    while ((dent = readdir(d)) != NULL) {
        if (dent->d_name[0] == '.') {
            continue;
        }
        if (!strstr(dent->d_name, ".yuv")) {
            continue;
        }
        if (browseFiles.size() == MAX_BROWSE_FILES) {
            fprintf(stderr, "reached %d files; not reading more\n", (int)browseFiles.size());
            break;
        }
        BrowseFile *bf = new BrowseFile();
        bf->path = "/var/tmp/mpq/";
        bf->path += dent->d_name;
        if (!read_file(bf->path.c_str(), &bf->yuv)) {
            delete bf;
            fprintf(stderr, "Could not read file: %s\n", bf->path.c_str());
        } else {
            newList.push_back(bf);
        }
    }
    std::sort(newList.begin(), newList.end(), [] (BrowseFile *a, BrowseFile *b) {
            return strcmp(a->path.c_str(), b->path.c_str()) < 0;
        });
    closedir(d);
    //  make sure the capture process sees the NULL browse buffer
    usleep(100000);
    clear_browse_files();
    browseFiles = newList;
    fprintf(stderr, "done scanning %d files\n", (int)browseFiles.size());
    return (int)browseFiles.size();
}

void set_learning(bool learn) {
    if (learnIndex == 0) {
        time_t t;
        time(&t);
        strftime(learnPrefix, 100, "learn-%Y-%m-%d-%H-%M-%S", localtime(&t));
    }
    learning = learn;
}

void warnWidget(Widget *w, char const *msg) {
    if (!w->warned_) {
        fprintf(stderr, "WARN: widget '%s': %s\n", w->label_.c_str(), msg);
        w->warned_ = true;
    }
}

Widget::Widget(float x, float y, char const *label, void (*fn)(Widget *), float w, float h)
    : x_(x)
    , y_(y)
    , w_(w)
    , h_(h)
    , label_(label)
    , hilite_(false)
    , lit_(false)
    , drawBackground_(true)
    , clickable_(fn != NULL)
    , warned_(false)
    , clickedFn_(fn)
{
}

void Widget::backColor() {
    if (hilite_) {
        glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    } else if (lit_) {
        glColor4f(1.0f, 0.4f, 0.4f, 1.0f);
    } else {
        glColor4f(0.7f, 0.7f, 0.7f, 1.0f);
    }
}

void Widget::draw() {
    if (drawBackground_) {
        drawBack();
    }
    drawFront();
}

void Widget::drawBack() {
    backColor();
    glBegin(GL_QUADS);
    glVertex2f(x_ - w_, y_ - h_);
    glVertex2f(x_ + w_, y_ - h_);
    glVertex2f(x_ + w_, y_ + h_);
    glVertex2f(x_ - w_, y_ + h_);
    glEnd();
}

void Widget::drawFront() {
    if (label_.size()) {
        float fx = x_-w_+std::min(0.05f, w_*0.25f);
        float fy = y_-std::min(0.05f, h_*0.5f);
        glRasterPos2f(fx, fy);
        GLboolean b = GL_TRUE;
        glGetBooleanv(GL_CURRENT_RASTER_POSITION_VALID, &b);
        if (b != GL_TRUE) {
            warnWidget(this, "raster position invalid");
        }
        drawText(label_.c_str(), fx, fy);
    }
}

bool Widget::test(float fx, float fy) {
    return (fabsf(fx-x_) < w_) && (fabsf(fy-y_) < h_);
}

void Widget::movement(bool inside, float fx, float fy) {
    if (clickable_) {
        hilite_ = inside;
    }
}

void Widget::click(bool down) {
    if (clickable_) {
        hilite_ = down;
        if (!down) {
            clicked();
        }
    }
}

void Widget::clicked() {
    if (clickedFn_) {
        clickedFn_(this);
    }
}

TextWidget::TextWidget(float fx, float fy, char const *label, float fw, float fh)
    : Widget(fx, fy, label, NULL, fw, fh)
{
    clickable_ = false;
    drawBackground_ = false;
}

class MultiLineText : public Widget {
    public:
        enum {
            NLINES = 8
        };
        MultiLineText(float fx, float fy, float fw, float fh);
        void backColor() override;
        void draw() override;
        void add(char const *line);

        void maybe_pop();

    private:

        void addLine(char const *line);

        std::list<std::string> toDraw_;
        std::atomic<int> added_;
        char *addedList_[NLINES];
};

MultiLineText::MultiLineText(float fx, float fy, float fw, float fh)
    : Widget(fx, fy, "", NULL, fw, fh)
{
    added_ = 0;
}

void MultiLineText::backColor() {
    glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
}

void MultiLineText::draw() {
    for (int i = 0; i != NLINES; ++i) {
        char *buf = addedList_[i];
        if (buf) {
            addedList_[i] = NULL;
            addLine(buf);
            free(buf);
        }
    }
    added_ = 0;
    drawBack();
    float y = y_ + h_ - 0.07f;
    float x = x_ - w_ + 0.004f;
    for (auto const &str : toDraw_) {
        drawText(str.c_str(), x, y);
        y -= 0.07f;
    }
}

void MultiLineText::add(char const *line) {
    int ix = added_.fetch_add(1);
    if (ix >= NLINES) {
        added_.fetch_sub(1);
        return;
    }
    assert(ix >= 0 && ix < NLINES);
    addedList_[ix] = ::strdup(line);
}

void MultiLineText::maybe_pop() {
    if (!toDraw_.empty()) {
        toDraw_.pop_front();
    }
}

void MultiLineText::addLine(char const *line) {
    toDraw_.push_back(line);
    if (toDraw_.size() > NLINES) {
        toDraw_.pop_front();
    }
}

class SliderWidget : public Widget {
    public:
        SliderWidget(float x, float y, char const *l, float w, float h, void (*selfn)(Widget *) = NULL, float (*dispfn)(Widget *, float) = NULL)
            : Widget(x, y, l, selfn, w, h)
        {
            clickable_ = false;
            drawBackground_ = true;
            value_ = 0;
            disp_ = dispfn;
        }
        float value_;
        float (*disp_)(Widget *, float);
        void movement(bool inside, float fx, float fy) override {
            if (inside) {
                float d = (fx - (x_ - w_)) / (w_ * 2);
                if (d >= 0.0f && d <= 1.0f) {
                    select(d);
                }
            }
        }
        void backColor() override {
            glColor4f(0.1f, 0.1f, 0.1f, 1.0f);
        }
        void drawFront() override {
            float x1 = x_ - w_ + (w_ * 2 - 0.01f) * value_;
            glColor4f(1, 1, 1, 1);
            glBegin(GL_QUADS);
            glVertex2f(x1, y_-h_);
            glVertex2f(x1+0.01f, y_-h_);
            glVertex2f(x1+0.01f, y_+h_);
            glVertex2f(x1, y_+h_);
            glEnd();
            char val[10];
            float v = value_;
            if (disp_) {
                v = disp_(this, v);
            }
            sprintf(val, "%.2f", v);
            drawText(val, x_ - 0.5f * w_, y_ - 0.8f * h_, 0.8f, 0.8f, 0.8f, 0.8f);
            assert(!glGetError());
        }
        virtual void select(float d) {
            value_ = d;
            clicked();
        }
};


float times3Pow2Display(Widget *, float v) {
    return powf(2.0f, v * 3 - 2);
}

float squaredDisplay(Widget *, float v) {
    return v * v;
}

float d2Display(Widget *, float v) {
    return v * SLIDER_MAX_D2;
}

float colorGainDisplay(Widget *, float v) {
    return powf(10.0f, v * 2 - 1);
}

float fileDisplay(Widget *, float v) {
    return v * browseFiles.size();
}

void toggle1(Widget *);
void toggle2(Widget *);
void toggleDrawDetect(Widget *);
void toggleSnapshot(Widget *);
void toggleBrowse(Widget *);
void toggleUpdate(Widget *);
void selectFile(Widget *);
void selectD2(Widget *);
void setColorGain(Widget *);
void toggleDebugDump(Widget *);
void updateSpeedGain(Widget *);
void updateTurnGain(Widget *);
void updateTurn2Gain(Widget *);


Widget *detectButton = new Widget(0.7f, 0.85f, "Detected", toggleDrawDetect);
Widget *snapshotButton = new Widget(0.7f, 0.65f, "Snapshot", toggleSnapshot);
Widget *updateButton = new Widget(0.7f, 0.45f, "Update", toggleUpdate);
Widget *browseButton = new Widget(0.7f, 0.25f, "Browse", toggleBrowse);
Widget *set1 = new Widget(-0.85f, -0.70f, "Set 1", toggle1, 0.10f, 0.06f);
Widget *set2 = new Widget(-0.85f, -0.85f, "Set 2", toggle2, 0.10f, 0.06f);
TextWidget *colorDisplay = new TextWidget(-0.40f, -0.55f, "Color", 0.3f, 0.05f);
TextWidget *addColor1 = new TextWidget(-0.35f, -0.70f, "Add1", 0.3f, 0.06f);
TextWidget *addColor2 = new TextWidget(-0.35f, -0.85f, "Add2", 0.3f, 0.06f);
SliderWidget *fileSelect = new SliderWidget(0.5f, -0.55f, "File", 0.5f, 0.04f, selectFile, fileDisplay);
SliderWidget *detectD2Slider = new SliderWidget(0.5f, -0.65f, "Detect-d2", 0.5f, 0.04f, selectD2, d2Display);
SliderWidget *colorGainSlider = new SliderWidget(0.5f, -0.75f, "ColorGain", 0.5f, 0.04f, setColorGain, colorGainDisplay);
Widget *debugDumpBtn = new Widget(-0.5f, 0.95f, "Dump", toggleDebugDump, 0.09f, 0.05f);
TextWidget *navDisplay = new TextWidget(0.4f, 0.95f, "Nav Info", 0.3f, 0.05f);
MultiLineText *logText = new MultiLineText(0.7f, -0.15f, 0.3f, 0.3f);
SliderWidget *speedGainSlider = new SliderWidget(0.5f, -0.85f, "SpeedGain", 0.5f, 0.04f, updateSpeedGain, times3Pow2Display);
SliderWidget *turnGainSlider = new SliderWidget(0.24f, -0.95f, "SpeedGain", 0.24f, 0.04f, updateTurnGain, squaredDisplay);
SliderWidget *turn2GainSlider = new SliderWidget(0.76f, -0.95f, "SpeedGain", 0.24f, 0.04f, updateTurn2Gain, squaredDisplay);


ColorPickerWidget::ColorPickerWidget(float left, float right, float top, float bottom)
    : Widget((left+right)*0.5f, (top+bottom)*0.5f, "picker", NULL, (right-left)*0.5f, (top-bottom)*0.5f)
{
    drawBackground_ = false;
}

void ColorPickerWidget::draw() {
}

static inline float cl255f(float f) {
    if (f < 0) return 0;
    if (f > 255) return 255;
    return f;
}

void ColorPickerWidget::movement(bool inside, float fx, float fy) {
    fprintf(stderr, "ColorPickerWidget motion %d %.1f %.1f\n", inside, fx, fy);
    if (inside && (snapshotState || browseMode)) {
        if (set1->lit_ || set2->lit_) {
            float dx = (fx - (x_ - w_)) / (w_ * 2);
            float dy = 1.0f - (fy - (y_ - h_)) / (h_ * 2);
            int x = (int)(dx * PROC_WIDTH);
            int y = (int)(dy * PROC_HEIGHT);
            if (x < 0) x = 0;
            if (x >= PROC_WIDTH) x = PROC_WIDTH-1;
            if (y < 0) y = 0;
            if (y >= PROC_HEIGHT) y = PROC_HEIGHT-1;
            float ycenter = cl255f((float)snapshot_yuv[x + y * PROC_WIDTH]);
            float ucenter = (int)snapshot_yuv[PROC_WIDTH * PROC_HEIGHT + x / 2 + y / 2 * PROC_WIDTH / 2] - 128;
            float vcenter = (int)snapshot_yuv[PROC_WIDTH * PROC_HEIGHT * 5 / 4 + x / 2 + y / 2 * PROC_WIDTH / 2] - 128;
            char dst[256];
            sprintf(dst, "%.1f %.1f %.1f", ycenter, ucenter, vcenter);
            if (set1->lit_) addColor1->label_ = dst; else addColor2->label_ = dst;
        } else {
            fprintf(stderr, "No delta color is selected\n");
        }
    }
}

GLuint atex;
GLuint ctex;
GLuint stex;        //  square texture
bool porterror = false;
bool drawDetect = false;

//  todo: can I make this alias with something else, so that I 
//  use less cache?
unsigned char rgbtex[PROC_WIDTH * PROC_HEIGHT * 3];

void updateColorDisplayLabel() {
    char buf[256];
    sprintf(buf, "%.1f %.1f %.1f / %.1f %d",
            detect_ycenter, detect_ucenter, detect_vcenter,
            detect_cgain, (int)detect_d2);
    colorDisplay->label_ = buf;
    ((SliderWidget *)detectD2Slider)->value_ = std::min(detect_d2 / SLIDER_MAX_D2, 1.0f);
    ((SliderWidget *)colorGainSlider)->value_ = (log10f(std::max(detect_cgain, 0.1f)) + 1) / 2;
}

void drawTheQuad() {
    float w = PROC_WIDTH;
    if (w > 512) w = 512;
    float h = PROC_HEIGHT;
    if (h > 512) h = 512;
    float left = -0.8f;
    float top = 0.9f;
    float width = 1.2f;
    float height = 1.8f * 240 / 320;
    float right = left + width;
    float bottom = top - height;
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, h / 512.0f);
    glVertex2f(left, bottom);
    glTexCoord2f(w / 512.0f, h / 512.0f);
    glVertex2f(right, bottom);
    glTexCoord2f(w / 512.0f, 0.0f);
    glVertex2f(right, top);
    glTexCoord2f(0.0f, 0.0f);
    glVertex2f(left, top);
    glEnd();
    assert(!glGetError());
}


void drawText(char const *text, float x, float y, float r, float g, float b, float a) {
    glColor4f(r, g, b, a);
    glRasterPos2f(x, y);
    while (*text) {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, *text);
        ++text;
    }
}


void toggleDrawDetect(Widget *w) {
    drawDetect = !drawDetect;
    w->lit_ = drawDetect;
    fprintf(stderr, "toggleDrawDetect: %d\n", (int)drawDetect);
}


void toggleSnapshot(Widget *w) {
    snapshotState++;
    if (snapshotState >= 3) {
        snapshotState = 0;
        browse_buffer = NULL;
    }
    w->lit_ = snapshotState != 0;
    fprintf(stderr, "snapshot state %d\n", snapshotState);
}

void toggleBrowse(Widget *w) {
    browseMode = !browseMode;
    if (browseMode) {
        if (read_browse_files()) {
            browseMode = true;
            select_browse_file(0);
        }
    }
    browseButton->lit_ = browseMode;
    if (!browseMode) {
        browseButton->label_ = "Browse";
        set_browse_mode(false);
    } else {
        char buf[30];
        sprintf(buf, "%d files", (int)browseFiles.size());
        browseButton->label_ = buf;
    }
}

void doSave(Widget *) {
    save_settings("camcam");
    fprintf(stderr, "Save button saved settings\n");
}

void doQuit(Widget *) {
    running = false;
}

void toggle1(Widget *w) {
    set2->lit_ = false;
    set1->lit_ = !set1->lit_;
}

void toggle2(Widget *w) {
    set1->lit_ = false;
    set2->lit_ = !set2->lit_;
}

void toggleUpdate(Widget *w) {
    if (snapshotState || browseMode) {
        snapshotState = 0;
        snapshotButton->lit_ = false;
        set1->lit_ = false;
        set2->lit_ = false;
        float y0 = detect_ycenter;
        float u0 = detect_ucenter;
        float v0 = detect_vcenter;
        float y1 = 0, u1 = 0, v1 = 0;
        float y2 = 0, u2 = 0, v2 = 0;
        float num = 1.0f;
        float dy = 1.0f;
        float du = 1.0f;
        float dv = 1.0f;
        if (sscanf(addColor1->label_.c_str(), "%f %f %f", &y1, &u1, &v1) == 3) {
            dy += fabsf(y1 - y0);
            du += fabsf(u1 - u0);
            dv += fabsf(v1 - v0);
            num += 1.0f;
        }
        if (sscanf(addColor2->label_.c_str(), "%f %f %f", &y2, &u2, &v2) == 3) {
            dy += fabsf(y2 - y0);
            du += fabsf(u2 - u0);
            dv += fabsf(v2 - v0);
            if (num >= 2.0f) {
                dy += fabsf(y2 - y1);
                dv += fabsf(v2 - v1);
                du += fabsf(u2 - u1);
            }
            num += 1.0f;
        }
        detect_ycenter = (y0 + y1 + y2) / num;
        detect_ucenter = (u0 + u1 + u2) / num;
        detect_vcenter = (v0 + v1 + v2) / num;
        detect_ygain = 1.0f;
        detect_cgain = dy / (dv + du) * 4.0f;
        detect_cgain = std::max(1.0f, std::min(8.0f, detect_cgain));
        detect_d2 = ((dy / num) * (dy / num) + (du / num) * (dy / num) + (dv / num) * (dv / num)) * 1.5f;
        detect_d2 = std::min(10000.0f, std::max(900.0f, detect_d2));
        fprintf(stderr, "num = %g; Y=%.1f U=%.1f V=%.1f yg=%.1f cg=%.1f d2=%.1f\n",
                num, detect_ycenter, detect_ucenter, detect_vcenter, detect_ygain, detect_cgain, detect_d2);
        set_setting_float("detect_ycenter", detect_ycenter);
        set_setting_float("detect_ucenter", detect_ucenter);
        set_setting_float("detect_vcenter", detect_vcenter);
        set_setting_float("detect_ygain",   detect_ygain);
        set_setting_float("detect_cgain",   detect_cgain);
        set_setting_float("detect_d2",      detect_d2);
        addColor1->label_ = "";
        addColor2->label_ = "";
        updateColorDisplayLabel();
        save_settings("camcam");
    }
}

void selectFile(Widget *w) {
    int i = (int)(browseFiles.size() * ((SliderWidget *)fileSelect)->value_);
    if (browseMode) {
        select_browse_file(i);
    }
}

void selectD2(Widget *w) {
    float d2 = d2Display(w, detectD2Slider->value_);
    detect_d2 = d2;
    set_setting_float("detect_d2", detect_d2);
    updateColorDisplayLabel();
}

void setColorGain(Widget *w) {
    float f = colorGainDisplay(w, colorGainSlider->value_); 
    detect_cgain = f;
    set_setting_float("detect_cgain", detect_cgain);
    updateColorDisplayLabel();
}

void toggleDebugDump(Widget *w) {
    debugDump = true;
}

void updateSpeedGain(Widget *w) {
    Gains g;
    steer_get_gains(g);
    g.speed_gain = times3Pow2Display(w, speedGainSlider->value_);
    steer_set_gains(g);
}

void updateTurnGain(Widget *w) {
    Gains g;
    steer_get_gains(g);
    g.turn_gain = squaredDisplay(w, turnGainSlider->value_);
    steer_set_gains(g);
}

void updateTurn2Gain(Widget *w) {
    Gains g;
    steer_get_gains(g);
    g.turn_squared_gain = squaredDisplay(w, turn2GainSlider->value_);
    steer_set_gains(g);
}


/*
   float left = -0.8f;
   float top = 0.9f;
   float width = 1.2f;
   float height = 1.8f * 240 / 320;
   */
Widget *widgets[] = {
    detectButton,
    snapshotButton,
    new ColorPickerWidget(-0.8f, 0.4f, 0.9f, (0.9f - 1.8f * 240 / 320)),
    new Widget(-0.9f, 0.95f, "Quit", doQuit, 0.09f, 0.075f),
    new Widget(-0.7f, 0.95f, "Save", doSave, 0.09f, 0.075f),
    colorDisplay,
    addColor1,
    addColor2,
    set1,
    set2,
    updateButton,
    browseButton,
    fileSelect,
    detectD2Slider,
    colorGainSlider,
    debugDumpBtn,
    navDisplay,
    logText,
    speedGainSlider,
    turnGainSlider,
    turn2GainSlider,
    NULL
};

Widget *curTrackWidget;
int totalNumFrames;


void add_incoming_text(char const *text) {
    logText->add(text);
}

void display() {
    glClearColor(porterror ? 1.0f : 0.3f, porterror ? 0.0f : 0.3f, porterror ? 0.0f : 0.3f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    assert(!glGetError());

    glEnable(GL_TEXTURE_2D);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    assert(!glGetError());
    int w = PROC_WIDTH;
    if (w > 512) {
        w = 512;
    }
    int h = PROC_HEIGHT;
    if (h > 512) {
        h = 512;
    }
    Frame *dirtyFrame = analyzer_analyzed_queue.beginRead();
    if (dirtyFrame) {
        if (PROC_WIDTH > 512) {
            glPixelStorei(GL_UNPACK_ROW_LENGTH, PROC_WIDTH);
        }
        glBindTexture(GL_TEXTURE_2D, atex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_LUMINANCE, GL_UNSIGNED_BYTE, dirtyFrame->data_);
        //fprintf(stderr, "upload 0x%lx\n", (unsigned long)last_analyzed);
        assert(!glGetError());
    }
    Frame *yuvframe = dirtyFrame ? dirtyFrame->link_ : NULL;
    if (yuvframe) {
        if (snapshotState < 2) {
            yuv_to_rgb(yuvframe->data_, rgbtex, PROC_WIDTH, PROC_HEIGHT);
            glBindTexture(GL_TEXTURE_2D, ctex);
            if (PROC_WIDTH > 512) {
                glPixelStorei(GL_UNPACK_ROW_LENGTH, PROC_WIDTH * 3);
            }
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, rgbtex);
        }
        if (snapshotState == 1) {
            memcpy(snapshot_yuv, browseMode ? browseFiles[browseIndex]->yuv : yuvframe->data_, PROC_WIDTH * PROC_HEIGHT * 6 / 4);
            ++snapshotState;
            browse_buffer = snapshot_yuv;
        }
        if (learning) {
            uint64_t now = get_microseconds();
            if (now - lastLearningSnapshot >= 500000) {
                lastLearningSnapshot = now;
                char buf[100];
                ++learnIndex;
                sprintf(buf, "/var/tmp/mpq/%s-%04d.yuv", learnPrefix, learnIndex);
                FILE *f = fopen(buf, "wb");
                if (f) {
                    fwrite(yuvframe->data_, 1, PROC_WIDTH * PROC_HEIGHT * 6 / 4, f);
                    fclose(f);
                }
            }
        }
    }
    Frame *sqframe = yuvframe ? yuvframe->link_ : NULL;
    if (sqframe) {
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
        glBindTexture(GL_TEXTURE_2D, stex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, sqframe->width_, sqframe->height_,
                GL_LUMINANCE, GL_UNSIGNED_BYTE, sqframe->data_);
        assert(!glGetError());
    }
    if (debugDump && yuvframe && dirtyFrame && sqframe) {
        fprintf(stderr, "debugDump: /tmp/debug-analyzed.png\n");
        stbi_write_png("/tmp/debug-analyzed.png", PROC_WIDTH, PROC_HEIGHT, 1, dirtyFrame->data_, 0);
        fprintf(stderr, "debugDump: /tmp/debug-square.png\n");
        DetectOutput output = { 0 };
        detect_get_last_output(&output);
        paint_clusters(sqframe->data_, sqframe->width_, sqframe->height_, output.clusters, output.num_clusters);
        stbi_write_png("/tmp/debug-square.png", sqframe->width_, sqframe->height_, 1, sqframe->data_, 0);
        fprintf(stderr, "debugDump: /tmp/debug-yuv.yuv\n");
        FILE *f = fopen("/tmp/debug-yuv.yuv", "wb");
        fwrite(yuvframe->data_, 1, PROC_WIDTH * PROC_HEIGHT * 6/4, f);
        fclose(f);
        fprintf(stderr, "debugDump: /tmp/debug-rgb.png\n");
        stbi_write_png("/tmp/debug-rgb.png", PROC_WIDTH, PROC_HEIGHT, 3, rgbtex, 0);
        detect_write_params("/tmp/debug-params.txt");
        debugDump = false;
    }
    if (dirtyFrame) {
        dirtyFrame->recycle();
    }

    glBindTexture(GL_TEXTURE_2D, ctex);
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glColor4f(1, 1, 1, 1);
    drawTheQuad();

    if (drawDetect) {
        glBindTexture(GL_TEXTURE_2D, atex);
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glColor4f(1.0f, 0.0f, 1.0f, 0.5f);
        drawTheQuad();
        glDisable(GL_BLEND);
    }

    glDisable(GL_TEXTURE_2D);
    Widget **wp = widgets;
    while (*wp) {
        (*wp)->draw();
        ++wp;
    }

    if (drawDetect) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);
        glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
        glColor4f(0.0f, 1.0f, 1.0f, 0.25f);
        glBindTexture(GL_TEXTURE_2D, stex);
        glEnable(GL_TEXTURE_2D);
        glBegin(GL_QUADS);
        glTexCoord2f(0, 1);
        glVertex2f(0.45f, -0.25f*8/5);
        glTexCoord2f(1, 1);
        glVertex2f(0.95f, -0.25f*8/5);
        glTexCoord2f(1, 0);
        glVertex2f(0.95f, 0.25f*8/5);
        glTexCoord2f(0, 0);
        glVertex2f(0.45f, 0.25f*8/5);
        glEnd();
        glDisable(GL_BLEND);
        assert(!glGetError());
    }

    char datetime[50];
    time_t t;
    time(&t);
    strftime(datetime, 50, "%Y-%m-%d %H:%M:%S", localtime(&t));
    char label[100];
    ++totalNumFrames;
    sprintf(label, "%s %d\n", datetime, totalNumFrames);
    drawText(label, -0.95f, -0.95f, 0, 0, 1, 1);
    assert(!glGetError());

    glutSwapBuffers();
    glutPostRedisplay();
    assert(!glGetError());
}


bool hasShutdown = false;
uint64_t lastScroll = 0;
uint64_t lastLog = 0;
uint64_t firstNow = 0;

uint8_t prevFlags;

void idle() {
    if (!running) {
        return;
    }

    uint64_t now = get_microseconds();
    if (firstNow == 0) {
        firstNow = now;
        fprintf(stderr, "basing firstNow on %lld\n", now);
    }

    bool shouldRecord = false;
    bool shouldLearn = false;
    bool shouldNavigate = false;
    bool shouldShutdown = false;

    porterror = !has_tstate();

    if (!porterror) {
        auto p = tstate();
        if (p.flags != prevFlags) {
            fprintf(stderr, "%lld: flags go from %x to %x\n", now, (int)prevFlags, (int)p.flags);
            prevFlags = p.flags;
        }
        //  When in radio drive mode, record the path, and 
        //  keep the lights on.
        if (p.flags & FLAG_DRIVEMODE) {
            shouldRecord = true;
        }
        //  Learning mode means record a bunch of raw yuv 
        //  snapshots that can later be analyzed.
        if (p.flags & FLAG_LEARNMODE) {
            shouldLearn = true;
        }

        if (p.flags & FLAG_RACEMODE) {
            shouldNavigate = true;
        }

        if (p.flags & FLAG_SHUTDOWN) {
            shouldShutdown = true;
            shouldRecord = false;
            shouldLearn = false;
            shouldNavigate = false;
        }

        if (now - lastLog > 200000) {
            lastLog = now;
            fprintf(stderr, "%lld m1=%ld m2=%ld\n", now-firstNow, (long)p.m1, (long)p.m2);
            fflush(stderr);
        }
    }

    set_recording(shouldRecord);
    set_learning(shouldLearn || shouldNavigate);
    navigation_enable(shouldNavigate);

    if (shouldShutdown && !hasShutdown) {
        hasShutdown = true;
        fprintf(stderr, "shutdown received\n");
        sync();
        system("sudo shutdown -h now");
    }

    {
        char buf[100];
        float drive = 0;
        float turn = 0;
        navigation_get_image(drive, turn);
        sprintf(buf, "T %5.2f  D %5.2f", turn, drive);
        navDisplay->label_ = buf;
    }

    if (now - lastScroll > 5000000) {
        logText->maybe_pop();
        lastScroll = now;
    }

    Gains g;
    steer_get_gains(g);
    speedGainSlider->value_ = std::min(1.0f, std::max(0.0f, (log2f(g.speed_gain) + 2) / 3.0f));
    turnGainSlider->value_ = sqrtf(std::min(1.0f, std::max(0.0f, g.turn_gain)));
    turn2GainSlider->value_ = sqrtf(std::min(1.0f, std::max(0.0f, g.turn_squared_gain)));
}

void setstop(int) {
    running = false;
}

GLuint mktex(GLenum t, GLuint size) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, t, size, size, 0, t, GL_UNSIGNED_BYTE, NULL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    assert(!glGetError());
    return tex;
}


float upx(int x) {
    return (x - WIN_WIDTH/2.0f) / (WIN_WIDTH/2.0f);
}

float upy(int y) {
    return (WIN_HEIGHT/2.0f - y) / (WIN_HEIGHT/2.0f);
}


Widget *findWidget(int x, int y) {
    float fx = upx(x);
    float fy = upy(y);
    Widget **wp = widgets;
    while (*wp) {
        if ((*wp)->test(fx, fy)) {
            return *wp;
        }
        ++wp;
    }
    return NULL;
}

void clearTrackWidget() {
    if (curTrackWidget) {
        curTrackWidget->movement(false, 0, 0);
        curTrackWidget = NULL;
    }
}

void keyboard(unsigned char key, int x, int y) {
    if (key == 'd' || key == 'D') {
        drawDetect = !drawDetect;
    }
}

void entry(int state) {
    fprintf(stderr, "entry %d\n", state);
    if (!state) {
        clearTrackWidget();
    }
}

void motion(int x, int y) {
    Widget *w = findWidget(x, y);
    if (w != curTrackWidget) {
        if (curTrackWidget) {
            curTrackWidget->movement(false, 0, 0);
        }
    } else if (curTrackWidget) {
        curTrackWidget->movement(true, upx(x), upy(y));
    }
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        fprintf(stderr, "mouse %d,%d %d,%d\n", button, state, x, y);
        Widget *w = findWidget(x, y);
        if (state == 0) {   //  clicking down
            clearTrackWidget();
            if (w) {
                fprintf(stderr, "clicked widget '%s'\n", w->label_.c_str());
                curTrackWidget = w;
                w->click(true);
                w->movement(true, upx(x), upy(y));
            }
        } else {
            if (curTrackWidget == w) {
                if (w) {
                    w->click(false);
                }
            }
            clearTrackWidget();
        }
    }
}

void create_main_window() {
    int argc = 1;
    char camcam[] = "camcam";
    char *argv[] = { camcam, NULL };

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT);
    glutInitWindowPosition(SCREEN_WIDTH-WIN_WIDTH-5, SCREEN_HEIGHT-WIN_HEIGHT-20);
    glutCreateWindow("Cam-Cam");
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboard);
    glutEntryFunc(entry);
    glutMotionFunc(motion);
    glutMouseFunc(mouse);

    atex = mktex(GL_LUMINANCE, 512);
    ctex = mktex(GL_RGB, 512);
    stex = mktex(GL_LUMINANCE, 128);
}

void run_main_loop() {
    if (open_ser(NULL, add_incoming_text) < 0) {
        fprintf(stderr, "error opening serial port\n");
    }
    start_navigation_thread();
    updateColorDisplayLabel();
    signal(SIGINT, setstop);
    while (running) {
        glutMainLoopEvent();
        idle();
    }
    close_ser();
    signal(SIGINT, SIG_DFL);
    fprintf(stderr, "main loop exits\n");
    stop_navigation_thread();
}



