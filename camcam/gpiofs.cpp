#include "gpiofs.h"
#include <unistd.h>
#include <sys/types.h>
#include <sys/fcntl.h>
#include <assert.h>
#include <string.h>

#include <map>

struct GPIO {
    GPIO() {
        path[0] = 0;
        fd = -1;
        last = -1;
        out = false;
    }
    char path[64];
    int fd;
    int last;
    bool out;
};

static std::map<int, GPIO> sGpio;

void open_gpio(int gpio, bool out) {
    fprintf(stderr, "open_gpio(%d)\n", gpio);
    close_gpio(gpio);
    int fd = open("/sys/class/gpio/export", O_WRONLY);
    if (fd < 0) {
        perror("/sys/class/gpio/export");
        return;
    }
    char id[20];
    sprintf(id, "%d\n", gpio);
    write(fd, id, strlen(id));
    close(fd);
    GPIO gp;
    sprintf(gp.path, "/sys/class/gpio/gpio%d/direction", gpio);
    fd = open(gp.path, O_WRONLY);
    if (fd < 0) {
        perror(gp.path);
        return;
    }
    write(fd, out ? "out\n" : "in\n", out ? 4 : 3);
    close(fd);
    sprintf(gp.path, "/sys/class/gpio/gpio%d/value", gpio);
    fd = open(gp.path, O_RDWR);
    if (fd < 0) {
        perror(gp.path);
        return;
    }
    gp.fd = fd;
    gp.out = out;
    sGpio[gpio] = gp;
    fprintf(stderr, "gpio %d: %s\n", gpio, gp.path);
}

int read_gpio(int gpio) {
    auto gp(sGpio.find(gpio));
    if (gp == sGpio.end()) {
        open_gpio(gpio, false);
        gp = sGpio.find(gpio);
        if (gp == sGpio.end()) {
            fprintf(stderr, "gpio %d: not open\n", gpio);
            return -1;
        }
    }
    if ((*gp).second.out) {
        fprintf(stderr, "%s: not input\n", (*gp).second.path);
        return -1;
    }
    char buf[2];
    read((*gp).second.fd, buf, 2);
    int ret = 1;
    if (buf[0] == '0') {
        ret = 0;
    }
    if ((*gp).second.last != ret) {
        fprintf(stderr, "read_gpio(%d): %d\n", gpio, ret);
        (*gp).second.last = ret;
    }
    return ret;
}

void write_gpio(int gpio, int value) {
    auto gp(sGpio.find(gpio));
    if (gp == sGpio.end()) {
        open_gpio(gpio, true);
        gp = sGpio.find(gpio);
        if (gp == sGpio.end()) {
            fprintf(stderr, "gpio %d: not open\n", gpio);
            return;
        }
    }
    if (!(*gp).second.out) {
        fprintf(stderr, "%s: not output\n", (*gp).second.path);
        return;
    }
    if ((*gp).second.last != value) {
        (*gp).second.last = value;
        fprintf(stderr, "write_gpio(%d, %d)\n", gpio, value);
    }
    assert((*gp).second.fd > 2);
    write((*gp).second.fd, value ? "1\n" : "0\n", 2);
}

void close_gpio(int gpio) {
    fprintf(stderr, "close_gpio(%d)\n", gpio);
    auto gp(sGpio.find(gpio));
    if (gp == sGpio.end()) {
        fprintf(stderr, "gpio %d: not open\n", gpio);
        return;
    }
    close((*gp).second.fd);
    sGpio.erase(gp);
    int fd = open("/sys/class/gpio/unexport", O_WRONLY);
    if (fd < 0) {
        perror("/sys/class/gpio/unexport");
        return;
    }
    char id[20];
    sprintf(id, "%d\n", gpio);
    write(fd, id, strlen(id));
    close(fd);
}


