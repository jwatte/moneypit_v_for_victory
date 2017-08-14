#include "serport.h"
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include "interface/vcos/vcos.h"
#include <list>
#include <string>
#include <assert.h>
#include <sys/time.h>


#include "CRC.h"
#include "global.h"

static int sfd = -1;
static unsigned char inbuf[512];
static int inptr;
static unsigned char outbuf[512];
static int outptr;
static int outbeg;
static void (*incoming_text_fn)(char const *);
static std::string portPath(getenv("SERIALPORT") ? getenv("SERIALPORT") : "/dev/ttyACM0");

static T2H_State tState;
uint64_t tStateTime;
static uint64_t lastStepTime_;

//  uint8_t mode
//  uint16_t turn
//  uint16_t drive
static H2T_State hState;
static uint64_t hStateTime;
static uint64_t lastAttemptOpen;
static uint8_t hStateVersion;

static uint32_t linkCount;

inline static int16_t clamp(int v, int16_t lo, int16_t hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

void ser_set_hstate(uint8_t mode, float drive, float turn) {
    hState.mode = mode ? HOST_MODE_HOSTDRIVE : HOST_MODE_RADIODRIVE;
    int16_t d = clamp((int)(drive * 10000), -30000, 30000);
    hState.drive = 32768 + d;
    int16_t t = clamp((int)(turn * 10000), -30000, 30000);
    hState.turn = 32768 + t;
}

int open_ser_inner() {
    if (sfd != -1) {
        close_ser();
        sfd = -1;
    }
    sfd = ::open(portPath.c_str(), O_RDWR | O_NONBLOCK);
    if (sfd < 0) {
        perror(("open(" + portPath + ")").c_str());
        if (portPath == "/dev/ttyACM0") {
            portPath = "/dev/ttyACM1";
        } else if (portPath == "/dev/ttyACM1") {
            portPath = "/dev/ttyACM0";
        }
        return -1;
    }
    struct termios tio;
    if (tcgetattr(sfd, &tio) < 0) {
        perror(("tcgetattr(" + portPath + ")").c_str());
error:
        close(sfd);
        sfd = -1;
        return -1;
    }
    cfmakeraw(&tio);
    cfsetspeed(&tio, B1000000);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(sfd, TCSAFLUSH, &tio) < 0) {
        perror(("tcsetattr(" + portPath + ")").c_str());
        goto error;
    }
    inptr = 0;
    outptr = 0;
    outbeg = 0;
    fprintf(stderr, "open(%s) succeeded\n", portPath.c_str());
    return 0;
}

int open_ser(char const *path, void(*textfn)(char const *)) {
    if (path != NULL) {
        portPath = path;
    }
    incoming_text_fn = textfn;
    return open_ser_inner();
}

static bool complainedAboutLostBytes = false;

static bool ser_wr(void const *data, size_t sz) {
    if (sz > sizeof(outbuf) - outptr) {
        if (!complainedAboutLostBytes) {
            fprintf(stderr, "ser_wr(): %ld bytes lost\n", (long)sz);
            complainedAboutLostBytes = true;
        }
        return false;
    }
    if (complainedAboutLostBytes) {
        fprintf(stderr, "ser_wr(): could write %ld bytes\n", (long)sz);
        complainedAboutLostBytes = false;
    }
    if (outptr == 0) {
        outbeg = 0;
    }
    if (sizeof(outbuf)-outbeg-outptr < sz) {
        memmove(&outbuf[0], &outbuf[outbeg], outptr);
        outbeg = 0;
    }
    memmove(&outbuf[outbeg], data, sz);
    outptr += sz;
    assert((size_t)outptr <= sizeof(outbuf));
    return true;
}

static void generate_hstate(uint64_t now) {
    hStateTime = now;
    unsigned char ob[100] = { 0xff, PACKET_SETINSTATE, 0 };
    Encode enc(&ob[3], 95);
    hStateVersion++;
    hState.version = hStateVersion;
    hState.visit(enc);
    ob[2] = enc.len();
    CRC16 crc(&ob[0], 3+ob[2]);
    enc.put(crc.crc_);
    assert(enc.ok());
    if (!ser_wr(ob, enc.len() + 3)) {
        fprintf(stderr, "Closing serial port because of write backlog\n");
        close_ser();
    }
}

void skip_exact(int skip) {
    assert(skip <= inptr);
    if (skip < inptr) {
        memmove(&inbuf[0], &inbuf[skip], inptr-skip);
    }
    inptr -= skip;
}

void skip_to_ff(int mskip) {
    assert(mskip > 0);
    for (int i = 1; i != mskip; ++i) {
        if ((inbuf[i] == 0xff) && ((i+1 == mskip) || (inbuf[i] != 0xff))) {
            skip_exact(i);
            return;
        }
    }
    skip_exact(mskip);
}

static void unknown(unsigned char type) {
    fprintf(stderr, "unknown packet type 0x%02x from Teensy\n", type);
}

void parse_inbuf(uint64_t now) {
again:
    if (inbuf[0] == 0xff) { //  binary packet
        if (inptr >= 5) {
            if (inptr >= 5 + inbuf[2]) {
                CRC16 crc(inbuf, 3 + inbuf[2]);
                uint16_t bc = GET2B(&inbuf[3 + inbuf[2]]);
                if (bc != crc.crc_) {
                    fprintf(stderr, "mis-matched CRC: calc 0x%04x != packet 0x%04x\n", crc.crc_, bc);
                    for (int i = 0; i != 5 + inbuf[2]; ++i) {
                        fprintf(stderr, " %02x", inbuf[i]);
                    }
                    fprintf(stderr, "\n");
                    skip_to_ff(5 + inbuf[2]);
                }
                else {
                    switch (inbuf[1]) {
                        case RESPONSE_SETOUTSTATE:
                            {
                                T2H_State st;
                                Decode dec(&inbuf[3], inbuf[2]);
                                st.visit(dec);
                                if (dec.ok()) {
                                    for (int i = 0; i != 3; ++i) {
                                        if (tState.rstatus[i] != st.rstatus[i]) {
                                            fprintf(stderr, "rclaw 0x%02x status 0x%04x\n", i+0x80, st.rstatus[i]);
                                        }
                                    }
                                    for (int i = 0; i != 4; ++i) {
                                        if (tState.dstatus[i] != st.dstatus[i]) {
                                            fprintf(stderr, "dxl 0x%02x status 0x%02x\n", i+2, st.dstatus[i]);
                                        }
                                    }
                                    tState = st;
                                    tStateTime = now;
                                }
                                else {
                                    fprintf(stderr, "Decode RESPONSE_SETOUTSTATE: decoder error with %d bytes and %d inptr\n", inbuf[2], inptr);
                                }
                            }
                            break;
                        case RESPONSE_LINKSTATS:
                            {
                                T2H_LinkStats ls;
                                Decode dec(&inbuf[3], inbuf[2]);
                                ls.visit(dec);
                                if (dec.ok()) {
                                    if (ls.numUnknown || (ls.numGood != ls.numTotal) || !(++linkCount & 15)) {
                                        fprintf(stderr, "LinkStats: %d/%d good %d InState %d Unknown\n",
                                                ls.numGood, ls.numTotal, ls.numInstate, ls.numUnknown);
                                    }
                                } else {
                                    fprintf(stderr, "LinkStats: bad packet\n");
                                }
                            }
                            break;
                        default:
                            unknown(inbuf[1]);
                            break;
                    }
                    skip_exact(5 + inbuf[2]);
                }
            }
        }
    } else {    //  go to cr/lf
        for (int i = 0; i != inptr; ++i) {
            if (inbuf[i] == 10 || inbuf[1] == 13 || inbuf[i] == 0xff) {
                if (i != 0) {
                    std::string msg(&inbuf[0], &inbuf[i]);
                    //  TODO: display the text message in the GUI
                    struct timeval tv = { 0, 0 };
                    gettimeofday(&tv, NULL);
                    time_t t = tv.tv_sec;
                    char datetime[100];
                    strftime(datetime, 100, "%H:%M:%S", localtime(&t));
                    fprintf(stdout, "%s.%03d: recv msg: %s\n", datetime, (int)tv.tv_usec/1000, msg.c_str());
                    fflush(stdout);
                    incoming_text_fn(msg.c_str());
                }
                if (inbuf[i] == 0xff) {
                    memmove(&inbuf[0], &inbuf[i], inptr-i);
                    goto again;
                }
                while ((i != inptr) && (inbuf[i] == 10 || inbuf[i] == 13)) {
                    ++i;
                }
                if (i < inptr) {
                    memmove(&inbuf[0], &inbuf[i], inptr-i);
                }
                inptr -= i;
                goto again;
            }
        }
    }
}

void poll_ser() {
    uint64_t now = vcos_getmicrosecs64();
    lastStepTime_ = now;
    if (sfd < 0) {
        outbeg = outptr = inptr = 0;
        if (now - lastAttemptOpen > 5000000) {
            lastAttemptOpen = now;
            open_ser_inner();
        }
        return;
    }
    int tr = sizeof(inbuf) - inptr;
    int nr = ::read(sfd, &inbuf[inptr], tr);
    if (nr > 0) {
        // fprintf(stderr, "poll_ser(): read(): %d\n", nr);
        inptr += nr;
        parse_inbuf(now);
    }
    if (now - hStateTime >= 20000) {
        hStateTime = now;
        generate_hstate(now);
    }
    if (outptr > 0) {
        int nw = ::write(sfd, &outbuf[outbeg], outptr);
        // fprintf(stderr, "poll_ser() write(%d): %d\n", outptr, nw);
        if (nw > 0) {
            outbeg += nw;
            outptr -= nw;
        }
    }
}

void close_ser() {
    if (sfd < 0) {
        return;
    }
    fprintf(stderr, "Closing serial port\n");
    close(sfd);
    fprintf(stderr, "Serial port closed\n");
    sfd = -1;
}

T2H_State const &tstate() {
    return tState;
}


bool has_tstate() {
    //  half a second to get a state
    return (lastStepTime_ - tStateTime) < 500000;
}


/* how not to express foreign dependencies */

#include "../mpv_teensy/Packets.cpp"
#include "../mpv_teensy/CRC.cpp"

