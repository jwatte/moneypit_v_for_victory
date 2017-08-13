#include "global.h"
#include "StatusTFT.h"
#include "Touch7834.h"
//#include "Adafruit_GFX.h"
//#include "Adafruit_ILI9341.h"
//#include "Fonts/FreeSans12pt7b.h"
#include "ILI9341_t3.h"
#include "font_Arial.h"
#include "PowerControl.h"
#include "Packets.h"


//#define TFT_DC 14 //  CS_A
#define TFT_DC 21 //  B3
#define TFT_CS 15 //  CS_B
#define TFT_RST 7 //  RX_3


#define GRAY        0x8410
#define DARKGRAY    0x4208
#define WHITE       0xffff
#define BLACK       0x0000
#define BRIGHTRED   0xf800
#define RED         0xe000
#define YELLOW      0xf700
#define GREEN       0x0300

#define SLIDER_BG DARKGRAY
#define SLIDER_FG WHITE

ILI9341_t3 tft(TFT_CS, TFT_DC, TFT_RST);
Touch7834 touch;
static bool forceNextDraw_;
StatusTFT StatusTFT::instance;
uint32_t lastHost_;
uint8_t nextHostVal;

class GUI {
    public:
        void init() {
            showMs_ = 0;
            hiliteButton_ = 0;
            prevHilite_ = 0;
            on_ = false;
            prevOn_ = false;
            lastTouch_ = 0;
            lastStep_ = 0;
        }
        bool on() { return on_; }
        void step(uint32_t now_ms, uint32_t dus);
        void hide();
        void show();
        void track(int16_t x, int16_t y);
        void track();
        void redraw();
        uint32_t showMs_;
        uint32_t lastTouch_;
        uint32_t lastStep_;
        uint16_t hiliteButton_;
        uint16_t prevHilite_;
        bool inPreShow_;
        bool on_;
        bool prevOn_;
};

GUI gui;

struct Button {
    char const *text;
    uint16_t left;
    uint16_t top;
    void (*func)();
    uint16_t bg;
};

void gui_clear() {
    memset(StatusTFT::instance.status.servoFlags, 0, sizeof(StatusTFT::instance.status.servoFlags));
    memset(StatusTFT::instance.status.clawFlags, 0, sizeof(StatusTFT::instance.status.clawFlags));
}

void gui_off() {
    PowerControl::requestShutdown();
}

void gui_back() {
    //  just hide the GUI
}

void gui_race() {
    StatusTFT::setMode(SM_Race);
}

void gui_learn() {
    StatusTFT::setMode(SM_Learn);
}

void gui_idle() {
    StatusTFT::setMode(SM_Idle);
}

static void draw_slider(int x, int y, int w, int h, float v, float l, float r) {
    tft.fillRect(x, y, w, h, SLIDER_BG);
    int xc = (int)floorf((v - l) / (r - l) * w);
    if (xc < 1) xc = 1;
    if (xc > w-1) xc = w-1;
    tft.fillRect(x+xc-1, y, 2, h, SLIDER_FG);
}

static Button buttons[] = {
    { "Clear",   10,    100,    &gui_clear, DARKGRAY  },
    { "Off",    210,    100,    &gui_off,   DARKGRAY  },
    { "Back",    10,    190,    &gui_back,  DARKGRAY  },
    { "Race",    10,     10,    &gui_race,  DARKGRAY  },
    { "Learn",  110,     10,    &gui_learn, DARKGRAY  },
    { "Idle",   210,     10,    &gui_idle,  DARKGRAY  },
};

void GUI::step(uint32_t now_ms, uint32_t dus) {
    int16_t x, y;
    lastStep_ = now_ms;
    if (touch.read(&x, &y)) {
        lastTouch_ = lastStep_;
        show();
        if (!inPreShow_) {
            track(x, y);
        }
    } else if (on_) {
        //  because of glitchy reads, only time out if several reads fail
        if (lastStep_ - lastTouch_ > 80) {
            if (inPreShow_) {
                inPreShow_ = false;
            } else if (now_ms - showMs_ > 5000) {
                hide();
            } else {
                track();
            }
        }
    }
    if (on_ != prevOn_) {
        prevHilite_ = 0xffff;
        hiliteButton_ = 0;
        tft.fillScreen(BLACK);
        prevOn_ = on_;
        if (!on_) {
            inPreShow_ = true;
        }
    }
    if (on_) {
        redraw();
    }
}

void GUI::show() {
    for (uint8_t i = 0; i != sizeof(buttons)/sizeof(buttons[0]); ++i) {
        if (buttons[i].func == gui_idle && StatusTFT::instance.mode() == SM_Idle) {
            buttons[i].bg = GREEN;
        } else if (buttons[i].func == gui_race && StatusTFT::instance.mode() == SM_Race) {
            buttons[i].bg = GREEN;
        } else if (buttons[i].func == gui_learn && StatusTFT::instance.mode() == SM_Learn) {
            buttons[i].bg = GREEN;
        } else { 
            buttons[i].bg = DARKGRAY;
        }
    }
    showMs_ = lastStep_;
    on_ = true;
}

void GUI::hide() {
    on_ = false;
    forceNextDraw_ = true;  //  ooh, hack!
}

#define BUTTON_WIDTH 90
#define BUTTON_HEIGHT 40

static void draw_button(Button const &btn, bool on) {
    if (on) {
        tft.fillRect(btn.left, btn.top, BUTTON_WIDTH+1, BUTTON_HEIGHT+1, WHITE);
        tft.setTextColor(BLACK, WHITE);
    } else {
        tft.fillRect(btn.left, btn.top, BUTTON_WIDTH+1, BUTTON_HEIGHT+1, btn.bg);
        tft.setTextColor(WHITE, btn.bg);
    }
    tft.setCursor(btn.left+5, btn.top+5);
    tft.print(btn.text);
    tft.setTextColor(WHITE, BLACK);
}

void GUI::track(int16_t x, int16_t y) {
    hiliteButton_ = 0;
    for (size_t i = 0; i != sizeof(buttons)/sizeof(buttons[0]); ++i) {
        Button &b = buttons[i];
        if (x >= b.left && x <= b.left + BUTTON_WIDTH && y >= b.top && y <= b.top + BUTTON_HEIGHT) {
            hiliteButton_ |= (1 << i);
        }
    }
}

void GUI::track() {
    for (size_t i = 0; i != sizeof(buttons)/sizeof(buttons[0]); ++i) {
        if (hiliteButton_ & (1 << i)) {
            (*buttons[i].func)();
            hide();
            return;
        }
    }
}

void GUI::redraw() {
    uint16_t todraw = prevHilite_ ^ hiliteButton_;
    for (size_t i = 0; i != sizeof(buttons)/sizeof(buttons[0]); ++i) {
        if (todraw & (1 << i)) {
            draw_button(buttons[i], (hiliteButton_ & (1 << i)) != 0);
        }
    }
    prevHilite_ = hiliteButton_;
}




StatusTFT::StatusTFT() {
}


void StatusTFT::force_text(char const *txt) {
    tft.fillScreen(0);
    tft.setTextColor(YELLOW);
    tft.setCursor(20, 100);
    tft.print(txt);
}

void StatusTFT::init(uint32_t now) {

    gui.init();
    memset(&status, 0, sizeof(status));
    memset(&prevStatus, 0xff, sizeof(prevStatus));
    onMs = 0;
    lastOnTime = 0;
    lastSvoTime = 0;
    mode_ = SM_Idle;

    tft.begin();
    tft.setRotation(3);
//    tft.setFont(&FreeSans12pt7b);
    tft.setFont(Arial_16);
    tft.fillScreen(0);
    tft.setTextSize(0);
    tft.setTextColor(WHITE, BLACK);

    touch.begin();
}

StatusMode StatusTFT::mode() {
    return mode_;
}

void StatusTFT::setMode(StatusMode sm) {
    SERIALUSB.print("mode ");
    SERIALUSB.println(sm, DEC);
    StatusTFT::instance.mode_ = sm;
}

template<typename T> bool diff(T const &a, T &b, bool force) {
    bool ret = a != b;
    if (ret) {
        b = a;
    }
    return ret || force;
}

static char ss[16];
//static char wmm0[12];
//static char wmm1[12];
//static char px[12];
//static char py[12];
//static char hd[8];
static char cf0[6];
static char cf1[6];
static char cf2[6];
static char sf0[4];
static char sf1[4];
static char sf2[4];
static char sf3[4];
static char vt[6];
static char md[8];
static char fp[6];
static char dm[8];

static void getTextBounds(ILI9341_t3 &tft, char const *buf, uint16_t x, uint16_t y,
    int16_t *ox, int16_t *oy, uint16_t *ow, uint16_t *oh) {
    *ox = x - 2;
    *oy = y - 4;
    *ow = strlen(buf) * 14 + 4;
    *oh = 22;
}

#define DRAW(type, arg, fmt, clr) \
void draw ## type (arg, char *buf, uint16_t x, uint16_t y) { \
    int16_t ox, oy; \
    uint16_t ow, oh; \
    getTextBounds(tft, buf, x, y, &ox, &oy, &ow, &oh); \
    tft.fillRect(ox, oy, ow, oh, BLACK); \
    sprintf fmt; \
    tft.setCursor(x, y); \
    if (clr != 0xffff) { \
        tft.setTextColor(clr, BLACK); \
        tft.print(buf); \
        tft.setTextColor(WHITE, BLACK); \
    } else { \
        tft.print(buf); \
    } \
}

DRAW(Seconds, uint32_t ms, (buf, "%ldm %02lds", ms/60000, ms/1000 % 60), WHITE)
DRAW(WheelMm, uint32_t d, (buf, "%ldmm", (int32_t)d*550/(64*50)), WHITE)    //  550mm circumference, 64*50 ticks per revolution
DRAW(Position, uint32_t p, (buf, "%.1fm", (float)p * 0.001f), WHITE)
DRAW(Heading, int16_t h, (buf, "%.1f", (float)h * 0.01f), WHITE)
DRAW(ClawFlags, uint16_t f, (buf, "%04x", f), f ? RED : WHITE)
DRAW(ServoFlags, uint8_t f, (buf, "%02x", f), f ? RED : WHITE)
DRAW(Voltage, uint8_t v, (buf, "%.1fV", v*0.1f+0.04f), v < 104 ? BRIGHTRED : v < 108 ? YELLOW : WHITE)
DRAW(Fps, uint16_t fps, (buf, "%d/s", fps), WHITE)
DRAW(MaxLoop, uint16_t dus, (buf, "%.1fms", dus*0.001f), dus == 0xffff ? RED : WHITE)
DRAW(DriveMode, uint8_t dmd, (buf, "%s", (dmd & FLAG_HOSTDRIVE) ? "Host" : (dmd & FLAG_DRIVEMODE) ? "Radio" : (dmd & FLAG_CONNECTED) ? "Idle" : "n/c"), (dmd & FLAG_HOSTDRIVE) ? GREEN : (dmd & FLAG_DRIVEMODE) ? WHITE : (dmd & FLAG_CONNECTED) ? YELLOW : RED)
DRAW(RunMode, uint8_t rm, (buf, "%s", (rm & FLAG_RACEMODE) ? "Race" : (rm & FLAG_LEARNMODE) ? "Learn" : ""), WHITE)

void StatusTFT::step(uint32_t ms, uint32_t dus) {
    onMs = ms;
    bool force = forceNextDraw_;
    if (force) {
        forceNextDraw_ = false;
    }
    //  out of phase with the 200ms update to avoid drawing everything at once
    if (((onMs + 5) / 10 != lastSvoTime / 10) || force) {
        gui.step(ms, dus);
        if (!gui.on()) {
            if (diff(status.clawFlags[0], prevStatus.clawFlags[0], force)) {
                drawClawFlags(status.clawFlags[0], cf0, 30, 45);
            }
            if (diff(status.clawFlags[1], prevStatus.clawFlags[1], force)) {
                drawClawFlags(status.clawFlags[1], cf1, 30, 115);
            }
            if (diff(status.clawFlags[2], prevStatus.clawFlags[2], force)) {
                drawClawFlags(status.clawFlags[2], cf2, 30, 185);
            }
            if (diff(status.servoFlags[0], prevStatus.servoFlags[0], force)) {
                drawServoFlags(status.servoFlags[0], sf0, 0, 10);
            }
            if (diff(status.servoFlags[1], prevStatus.servoFlags[1], force)) {
                drawServoFlags(status.servoFlags[1], sf1, 80, 10);
            }
            if (diff(status.servoFlags[2], prevStatus.servoFlags[2], force)) {
                drawServoFlags(status.servoFlags[2], sf2, 0, 215);
            }
            if (diff(status.servoFlags[3], prevStatus.servoFlags[3], force)) {
                drawServoFlags(status.servoFlags[3], sf3, 80, 215);
            }
        }
        lastSvoTime = onMs + 5;
    }
    if (force || (onMs / 200 != lastOnTime / 200)) {
        if (diff(status.voltage, prevStatus.voltage, force)) {
            drawVoltage(status.voltage, vt, 220, 190);
        }
        if (!gui.on()) {
            if (onMs / 1000 != lastOnTime / 1000) {
                drawSeconds(onMs, ss, 200, 115);
            }
            lastOnTime = onMs;
            if (diff(status.fps, prevStatus.fps, force)) {
                drawFps(status.fps, fp, 220, 215);
            }
            if (diff(status.maxloop, prevStatus.maxloop, force)) {
                drawMaxLoop(status.maxloop, md, 130, 215);
            }
            if (diff(status.flags, prevStatus.flags, force)) {
                tft.fillRect(200, 140, 100, 40, BLACK);
                drawDriveMode(status.flags, dm, 200, 140);
                drawRunMode(status.flags, dm, 200, 160);
            }
        }
        if (PowerControl::pendingShutdown()) {
            if (onMs & 256) {
                tft.setTextColor(WHITE, RED);
            } else {
                tft.setTextColor(YELLOW, BLACK);
            }
            tft.setCursor(120, 80);
            tft.print("SHUTDOWN");
            tft.setTextColor(WHITE, BLACK);
        }
    }
    if (ms - lastHost_ >= 100) {
        lastHost_ = ms;
        switch (nextHostVal) {
            case 0:
                draw_slider(160, 40, 100, 8, StatusTFT::instance.status.maxT, 0.0f, 1.0f);
                break;
            case 1:
                draw_slider(160, 50, 100, 8, StatusTFT::instance.status.hostDrive, 0.0f, 1.5f);
                break;
            case 2:
                draw_slider(160, 60, 100, 8, StatusTFT::instance.status.hostTurn, -2.0f, 2.0f);
                break;
        }
        if (++nextHostVal > 2) {
            nextHostVal = 0;
        }
    }
}

