#pragma once

#include "config.h"

#ifndef __cplusplus
extern "C" {
#endif

extern bool startFlag;
extern bool homeFlag;
extern bool APPTFlag;
extern int feedrate;
extern int dryY;
extern int cyclesY;
extern int cyclesN;

extern volatile uint8_t menuPos;
extern volatile int8_t encPos;

extern void hwInit();
extern void btnISR();
extern void encISR();
extern void print(char i);
extern void print(const char i[]);
extern void print(const char i[], int a);
extern void print(const char i[], int a, int b);
extern void lcdprint(int x, int y, const char i[]);
extern void lcdprint(int x, int y, const char i[], int a);
extern void lcdprint(int x, int y, const char i[], int a, int b);
extern void lcdprint(int x, int y, const char i[], int a, int b, char *c);

extern void zeroEncoder();
extern void lcdSetCursor(int x, int y);
extern void lcdBlink();
extern void lcdNoBlink();

#ifndef __cplusplus
}
#endif