#include <Arduino.h>
#include <Encoder.h>
#include <Wire.h>
#include <LiquidCrystal.h>
#include "utility.h"
#include "functions.h"

// 0 > TXD
// 1 > RXD
// 2 > ENC_A
// 3 > ENC_B
// 4 > LCD_RS
// 5 > LCD_EN
// 6 > RTS
// 7 > ENC_BTN, PD7
// 8 > LCD_D4
// 9 > LCD_D5
// 10 > LCD_D6
// 11 > LCD_D7
// 12 > RESET, PD6

// uint8_t axes[2];					// x, y position
// uint8_t feeds[2];				// live feedrates
// uint8_t bf[2];						// buffer state
// char *state;

Encoder enc(2, 3);
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);

bool startFlag = true;
bool homeFlag = false;
bool APPTFlag = false;
int feedrate = 20;		// feedrate in cm/s (min: 1)
int dryY = 0;					// columns to move w/o APPT
int cyclesY = 0;			// columns to move w/ APPT
int cyclesN = 1;			// how many times to perform treatment (min: 1)
//                         ________ menu item APPT
//                        | _______ menu item feedrate
//                        || ______ menu item move Y
//                        ||| _____ menu item home machine
//                        |||| ____ submenu item APPT cycles/repeats
//                        ||||| ___ submenu item APPT columns to cover
//                        |||||| __ submenu item feedrate
//                        ||||||| _ submenu item move Y
volatile uint8_t menuPos = 0b00010000;
volatile int8_t encPos = 0;
int8_t shift = 0;
char writeBuffer[WRITEBUFFER_SIZE];

void hwInit() {
	Serial1.setTimeout(200);
	Serial1.begin(115200);
	pinMode(6, OUTPUT);
	pinMode(7, INPUT_PULLUP);
	pinMode(12, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(7), btnISR, RISING);
	lcd.begin(20, 4);
	lcd.clear();
	lcd.setCursor(1, 0);
	lcd.print("Home machine X:???");
	lcd.setCursor(1, 1);
	lcd.print("             Y:???");
	lcd.setCursor(0, 0);
	lcd.cursor();
	lcd.blink();
}

// screw it, menu functionality handled inside ISRs
void btnISR() {
  switch (menuPos << 4) {
		case 0b00010000:
			homeFlag = true;
			break;
    case 0:   					// enter submenu
      menuPos |= menuPos >> 5;
      break;
    case 0b01000000:  	// change from columns to cycles
      menuPos ^= 0b00001100;
      break;
		case 0b10000000:
			APPTFlag = true;
			break;
    default:  					// exit submenu
      menuPos &= 0b11110000;
  }
} 

void encISR() {
	if (enc.read() != 0) {
		switch (menuPos) {
			case 0b10000000:  // menu navigation
			case 0b01000000:
			case 0b00100000:
			case 0b00010000:
				shift = (byte)enc.read();				// enc.read() returns int32_t/long, must be crammed into 8 bits for optimisation ig			
				while (shift < 0) shift += 4;		// convert shift to positive, circular rotation of 4 bits by -n is the same as rotation by (4-n) for n < 4
				menuPos = menuPos >> 4;					// shift menuPos right by 4 for the circular rotation, rotate, mask lower half of byte and shift it back up
				menuPos = ((menuPos << shift) | (menuPos >> (4 - shift))) & 0b00001111;
				menuPos = menuPos << 4;					// there's definitely a more efficient way to do this
				break;													// this might be redundant anyways if there isn't more than 1 pulse per interrupt
				// if (enc.read() < 0) menuPos = menuPos >> 1;
				// else menuPos = menuPos << 1;		// this is only good for single bit shifts
				// if (menuPos == 8) menuPos = 128;
				// if (menuPos == 0) menuPos = 16;// manually handled wraparound
				// break;
			case 0b00100001:  // move Y
				dryY += enc.read();
				dryY = clamp(dryY, -Y_COLUMNS_MAX, Y_COLUMNS_MAX);
				break;
			case 0b01000010:  // feedrate
				feedrate += enc.read();
				feedrate = clamp(feedrate, 1, FEEDRATE_MAX);
				break; 
			case 0b10000100:	// columns to cover
				cyclesY += enc.read();
				cyclesY = clamp(cyclesY, -Y_COLUMNS_MAX, Y_COLUMNS_MAX);
				break;
			case 0b10001000:	// repeating instances
				cyclesN += enc.read();
				cyclesN = clamp(cyclesN, 1, CYCLES_MAX);
				break;
			default:          // just in case, there shouldn't be any more cases
				menuPos = 0b00010000; // set to known case (home machine)
				break;
		}
		enc.write(0);
		newStats = true;
	}
}

void print(char i) {
	PORTD |= 0b10000000;
	Serial1.flush();
	Serial1.print(i);
	PORTD &= 0b01111111;
}

void print(const char i[]) {
	PORTD |= 0b10000000;
	Serial1.flush();
	Serial1.println(i);
	PORTD &= 0b01111111;
}

void print(const char i[], int a) {
	snprintf(writeBuffer, WRITEBUFFER_SIZE, i, a);
	const char j[WRITEBUFFER_SIZE] = {0};
	strcpy(writeBuffer, j);
	print(j);
}

void print(const char i[], int a, int b) {
	snprintf(writeBuffer, WRITEBUFFER_SIZE, i, a, b);
	const char j[WRITEBUFFER_SIZE] = {0};
	strcpy(writeBuffer, j);
	print(j);
}

void lcdprint(int x, int y, const char i[]) {
	lcd.setCursor(x, y);
	lcd.print(i);
}

void lcdprint(int x, int y, const char i[], int a) {
	snprintf(writeBuffer, WRITEBUFFER_SIZE, i, a);
	lcdprint(x, y, writeBuffer);
}

void lcdprint(int x, int y, const char i[], int a, int b) {
	snprintf(writeBuffer, WRITEBUFFER_SIZE, i, a, b);
	lcdprint(x, y, writeBuffer);
}

void lcdprint(int x, int y, const char i[], int a, int b, char *c) {
	snprintf(writeBuffer, WRITEBUFFER_SIZE, i, a, b, c);
	lcdprint(x, y, writeBuffer);
}

void zeroEncoder() {
	enc.write(0);
}

void lcdSetCursor(int x, int y) {
	lcd.setCursor(x, y);
}

void lcdBlink() {
	lcd.blink();
}

void lcdNoBlink() {
	lcd.noBlink();
}