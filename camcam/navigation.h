#if !defined(navigation_h)
#define navigation_h

void start_navigation_thread();
void stop_navigation_thread();
void navigation_set_image(float ispeed, float iturn);
void navigation_get_image(float &ispeed, float &iturn);
bool navigation_get_enable();
void navigation_enable(bool nav);

#endif  //  navigation_h

