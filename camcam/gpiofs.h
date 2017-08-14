#if !defined(gpiofs_h)
#define gpiofs_h

#define LIGHT_GPIO 21

void open_gpio(int gpio, bool out);
int read_gpio(int gpio);
void write_gpio(int gpio, int value);
void close_gpio(int pgio);

#endif  //  gpiofs_h

