#include "hardware.h"

#ifndef __cplusplus
extern "C" {
#endif

extern volatile bool newStats;

extern void printMenu();
extern unsigned char getStatus();
extern void printStatus();
extern void homeMachine();
extern void kill();
extern void performAPPT();

#ifndef __cplusplus
}
#endif