#include "navigation.h"
#include "serport.h"
#include "settings.h"
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdio.h>
#include <atomic>
#include <math.h>


static bool volatile navRunning = false;
static pthread_t navThread;

static float speed_max = 1.5f;
static float turn_max = 2.0f;
static std::atomic<float> speed;
static std::atomic<float> turn;
static bool navigating;


#define MAX_SLEW_RATE 0.15f

static float slewLimited = 0;
static int numNav = 0;

static float seek(float target, float prev) {
    float d = fabsf(target - prev);
    if (d < MAX_SLEW_RATE) {
        return target;
    }
    slewLimited += d - MAX_SLEW_RATE;
    if (target < prev) {
        return prev - MAX_SLEW_RATE;
    }
    return prev + MAX_SLEW_RATE;
}

void navigation_enable(bool nav) {
    navigating = nav;
}

bool navigation_get_enable() {
    return navigating;
}

void navigation_set_image(float ispeed, float iturn) {
    float fspeed = ispeed;
    float fturn = iturn;
    if (fabsf(fturn) > turn_max) {
        fturn = turn_max * (fturn > 0 ? 1 : -1);
    }
    if (fabsf(fspeed) > speed_max) {
        fspeed = speed_max * (fspeed > 0 ? 1 : -1);
    }
    if (fabsf(fspeed) < fabsf(fturn * 0.5f)) {
        fspeed = fabsf(fturn * 0.5f) * (fspeed < 0 ? -1 : 1);
    }
    speed = seek(fspeed, speed);
    turn = seek(fturn, turn);
    if (!(++numNav & 31)) {
        fprintf(stderr, "nav: speed %.2f turn %.2f slewLimit %.2f\n",
                speed.load(), turn.load(), slewLimited);
        slewLimited = 0;
    }
}

void navigation_get_image(float &ispeed, float &iturn) {
    ispeed = speed;
    iturn = turn;
}

static void *nav_fn(void *) {
    while (navRunning) {
        poll_ser();
        if (navigating) {
            ser_set_hstate(1, speed, turn);
        } else {
            ser_set_hstate(0, 0, 0);
        }
        usleep(20000);
    }
    return NULL;
}

void start_navigation_thread() {
    if (!navRunning) {
        speed_max = get_setting_float("speed_max", speed_max);
        turn_max = get_setting_float("turn_max", turn_max);
        fprintf(stderr, "starting nav; speed_max=%.2f turn_max=%.2f\n",
                speed_max, turn_max);
        atexit(stop_navigation_thread);
        navRunning = true;
        pthread_create(&navThread, NULL, nav_fn, NULL);
        fprintf(stderr, "started nav thread");
    }
}

void stop_navigation_thread() {
    if (navThread) {
        navRunning = false;
        void *ret = NULL;
        pthread_join(navThread, &ret);
        navThread = 0;
        fprintf(stderr, "finished nav thread\n");
    }
}

