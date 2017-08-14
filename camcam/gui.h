#if !defined(gui_h)
#define gui_h


void create_main_window();
void run_main_loop();
void add_incoming_text(char const *line);

extern int snapshotState;
extern bool volatile running;
extern unsigned char snapshot_yuv[];
extern unsigned long long get_microseconds();


#endif  //  gui_h

