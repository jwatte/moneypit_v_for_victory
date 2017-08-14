#if !defined(serport_h)
#define serport_h

#include "Packets.h"

int open_ser(char const *path, void (*incoming_text)(char const *));
void poll_ser();
void close_ser();

bool has_tstate();
T2H_State const &tstate();
void ser_set_hstate(uint8_t mode, float drive, float turn);

#endif  //  serport_h
