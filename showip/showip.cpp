#include <FL/Fl.H>
#include <FL/Fl_Double_Window.H>
#include <FL/Fl_Output.H>
#include <FL/Fl_Multiline_Output.H>
#include <FL/Fl_Button.H>
#include <time.h>
#include <list>
#include <map>
#include <string>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <ctype.h>
#include <dirent.h>
#include <pthread.h>



time_t lastTime = 0;
Fl_Double_Window *gWin;
Fl_Multiline_Output *gCycleBox;

std::list<std::string> lines;
std::list<Fl_Output *> boxes;

time_t last_eth0;
time_t last_wlan0;
time_t last_var;
std::string cycleInfo;
Fl_Output *gClock;

void clean(char const *path) {
    fprintf(stderr, "clean(%s)\n", path);
    std::multimap<time_t, std::string> older;
    DIR *d = opendir(path);
    struct dirent *dent;
    while ((dent = readdir(d)) != NULL) {
        std::string p(path);
        p += "/";
        p += dent->d_name;
        struct stat64 st;
        if (!stat64(p.c_str(), &st)) {
            if (S_ISREG(st.st_mode)) {
                older.insert(
                        older.upper_bound(st.st_mtime),
                        std::pair<time_t, std::string>(st.st_mtime, dent->d_name));
            }
        }
        else {
            perror((std::string("stat(")+p+")").c_str());
        }
    }
    closedir(d);
    fprintf(stderr, "There are %u files under consideration\n", older.size());
    while (older.size() > 4) {
        std::string p(path);
        p += "/";
        p += older.begin()->second;
        if (unlink(p.c_str())) {
            perror(p.c_str());
        }
        else {
            fprintf(stderr, "deleted %s\n", p.c_str());
        }
        older.erase(older.begin());
    }
    fprintf(stderr, "cleanup done\n");
}



pthread_t cycleThread = 0;
bool volatile cycling = false;

void *cycle_fn(void *v_ifname) {
    char buf[200];
    sprintf(buf, "sudo ifdown %s", (char *)v_ifname);
    system(buf);
    sleep(1);
    sprintf(buf, "sudo ifup %s", (char *)v_ifname);
    system(buf);
    free(v_ifname);
    cycling = false;
    return NULL;
}

char nowbuf[100];

char const *nowstr() {
    time_t t;
    time(&t);
    strftime(nowbuf, 100, "%Y-%m-%d %H:%M:%S", localtime(&t));
    return nowbuf;
}

void cycle_interface(char const *name) {
    cycling = true;
    if (cycleThread != 0) {
        void *v_res;
        //  clear out thread status
        fprintf(stderr, "%s: reaping old cycle thread %ld\n", nowstr(), (long)cycleThread);
        pthread_join(cycleThread, &v_res);
    }
    char *ifname = strdup(name);
    pthread_create(&cycleThread, NULL, &cycle_fn, ifname);
    fprintf(stderr, "%s: started cycle thread %ld for interface %s\n", nowstr(), (long)cycleThread, ifname);
}



void idle() {
    usleep(20000);
    time_t nowTime;
    time(&nowTime);
    char buf[40];
    strftime(buf, 40, "%H:%M", localtime(&nowTime));
    if (strcmp(buf, gClock->value())) {
        gClock->value(buf);
        gWin->redraw();
    }
    std::string cycle;
    if (nowTime - lastTime > 2) {
        lastTime = nowTime;
        std::list<std::string> ifs;
        FILE *f = popen("ip addr", "r");
        char ifname[128];
        char ifaddr[128];
        while (!feof(f) && !ferror(f)) {
            char line[1024] = { 0 };
            fgets(line, 1024, f);
            if (!line[0]) {
                break;
            }
            int ix = -1;
            if (sscanf(line, " %d : %s", &ix, ifname) == 2) {
            }
            else if (sscanf(line, "    inet %s", ifaddr) == 1) {
                std::string ifns(ifname);
                std::string s(ifname);
                (s += " ") += ifaddr;
                ifs.push_back(s);
                if (ifns == "eth0:") {
                    last_eth0 = nowTime;
                }
                else if (ifns == "wlan0:") {
                    last_wlan0 = nowTime;
                }
                else if (ifns == "lo:") {
                }
                else {
                    fprintf(stderr, "unknown interface is up: '%s'\n", ifname);
                }
            }
        }
        pclose(f);
        if ((nowTime - last_eth0 > 120) && !cycling) {
            char buf[100];
            strftime(buf, 100, "cycle eth0 at %H:%M:%S\n", localtime(&nowTime));
            cycle += buf;
            cycle_interface("eth0");
            last_eth0 = nowTime;
        }
        if ((nowTime - last_wlan0 > 109) && !cycling) {
            char buf[100];
            strftime(buf, 100, "cycle wlan0 at %H:%M:%S\n", localtime(&nowTime));
            cycle += buf;
            cycle_interface("wlan0");
            last_wlan0 = nowTime;
        }
        if (ifs != lines) {
            ifs.swap(lines);
            for (auto const &b : boxes) {
                delete b;
            }
            boxes.clear();
            int n = 0;
            gWin->begin();
            for (auto const &p : lines) {
                Fl_Output *b = new Fl_Output(10, 10+30*n, 300, 25, "");
                b->textsize(18);
                b->box(FL_NO_BOX);
                b->value(p.c_str());
                boxes.push_back(b);
                n++;
            }
            gWin->end();
            gWin->redraw();
        }
    }
    if (nowTime - last_var > 120) {
        last_var = nowTime;
        FILE *f = popen("df", "r");
        if (f) {
            char line[300];
            int percent = 0;
            while (fgets(line, 300, f) && !feof(f) && !ferror(f)) {
                if (strstr(line, "/dev/root")) {
                    char *p = strrchr(line, '%');
                    if (p) {
                        while ((p != line) && isdigit(p[-1])) {
                            --p;
                        }
                        sscanf(p, "%d", &percent);
                        if (percent >= 90) {
                            cycle += "/dev/root usage > 90%\n";
                            cycle += line;
                            clean("/var/tmp/mpq");
                        }
                    }
                }
            }
            pclose(f);
        }
        else {
            perror("popen(df)");
        }
    }
    if (!cycle.empty() && (cycle != cycleInfo)) {
        char buf[20];
        strftime(buf, 20, "%H:%M:%S ", localtime(&nowTime));
        cycleInfo = buf + cycle;
        delete gCycleBox;
        gWin->begin();
        Fl_Multiline_Output *o = new Fl_Multiline_Output(320, 10, 300, 75, "");
        o->box(FL_NO_BOX);
        o->value(cycleInfo.c_str());
        gCycleBox = o;
        gWin->end();
        gWin->redraw();
    }
}


int nbut = 0;

void mkbtn(char const *label, void (*fn)(Fl_Widget *, void *)) {
    Fl_Button *b = new Fl_Button(20 + nbut * 130, 130, 110, 30, label);
    b->callback(fn);
    ++nbut;
}

void btn_arduino(Fl_Widget *, void *) {
    if (fork() == 0) {
        execlp("/home/pi/arduino-1.8.3/arduino", "arduino", NULL);
    }
}

void btn_shutdown(Fl_Widget *, void *) {
    if (fork() == 0) {
        execlp("/usr/bin/sudo", "sudo", "shutdown", "-h", "now", NULL);
    }
}

void btn_camcam(Fl_Widget *, void *) {
    if (fork() == 0) {
        execlp("/usr/local/src/camcam/camcam", "camcam", NULL);
    }
}

void btn_terminal(Fl_Widget *, void *) {
    if (fork() == 0) {
        execlp("/usr/bin/lxterminal", "lxterminal", NULL);
    }
}

int main() {
    time(&last_wlan0);
    last_wlan0 -= 79;
    time(&last_eth0);
    last_eth0 -= 87;
    time(&last_var);
    last_var -= 113;
    Fl_Double_Window win(0, 10, 640, 180, "addresses");
    gWin = &win;
    gCycleBox = new Fl_Multiline_Output(320, 10, 300, 75, "");
    gCycleBox->textsize(18);
    gCycleBox->box(FL_NO_BOX);
    char buf[100];
    strftime(buf, 100, "Starting at %H:%M:%S", localtime(&last_eth0));
    gCycleBox->value(buf);
    gClock = new Fl_Output(560, 80, 80, 40, "");
    gClock->textsize(24);
    gClock->box(FL_NO_BOX);
    mkbtn("Arduino", btn_arduino);
    mkbtn("Shutdown", btn_shutdown);
    mkbtn("CamCam", btn_camcam);
    mkbtn("Terminal", btn_terminal);
    win.end();
    win.show();
    Fl::set_idle(idle);
    Fl::run();
    return 0;
}

