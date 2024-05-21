#include <Arduino.h>
#include <avr/wdt.h>
#include <TimerOne.h>
#include "functions.h"
#include "utility.h"

volatile bool newStats = false;
int axes[2];
int feeds[2];
int bf[2];
char *state;
char receiveBuffer[128];

void printMenu() {
	lcdprint(17, 0, "  0");
	lcdprint(1, 1, "Move Y        Y:  0");
	lcdprint(1, 2, "FR:%3d FS:  0", feedrate);
	lcdprint(1, 3, "APPT C:%3d R:%+3d", cyclesY, cyclesN);
}

uint8_t getStatus() {				// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
	char *startC = NULL, *value, *mainNext;
	uint8_t endC = 0;
	while (!endC || startC == NULL) {
		print("$X");
		endC = Serial1.readBytesUntil('>', receiveBuffer, 128);
		startC = strchr(receiveBuffer, '<') + 1;
	}
	receiveBuffer[endC] = '\0';
	split(startC, &mainNext, '|');
	if (*mainNext == '\0') {
		return 0;  // Malformed report
	}
	state = startC;
	while (*mainNext) {
		startC = mainNext;
		split(startC, &mainNext, '|');
		split(startC, &value, ':');
		if (strcmp(startC, "MPos") == 0 || strcmp(startC, "WPos") == 0) {
				// x,y,z,...
				parse_integers(value, axes, 2);
				continue;
		}
		if (strcmp(startC, "Bf") == 0) {
				parse_integers(value, bf, 2);
				continue;
		}
		if (strcmp(startC, "Ln") == 0) continue;
		if (strcmp(startC, "FS") == 0) {
				// feedrate,spindle_speed
				parse_integers(value, feeds, 1);  // feed in [0], spindle in [1]
				continue;
		}
	}
	newStats = true;
	return 1;
}

void printStatus() {
	lcdprint(17, 0, "%3d", (int)(axes[0]/10));
	lcdprint(17, 1, "%3d", (int)(axes[1]/10));
	lcdprint(1, 2, "FR:%3d FS:%3d %5c", feedrate, (int)(feeds[0]/600), state);
	lcdprint(1, 3, "APPT C:%+3dR:%4d", cyclesY, cyclesN);
	switch (menuPos) {
		case 0b00010000:
			lcdSetCursor(0, 0);
			break;
		case 0b00100000:
		case 0b00100001:
			lcdSetCursor(0, 1);
			break;
		case 0b01000000:
		case 0b01000010:
			lcdSetCursor(0, 2);
			break;
		case 0b10000000:
			lcdSetCursor(0, 3);
			break;
		case 0b10000100:
			lcdSetCursor(6, 3);
			break;
		case 0b10001000:
			lcdSetCursor(11, 3);
	}
	if ((menuPos & 0b00001111) > 0 || APPTFlag) lcdBlink();
	else lcdNoBlink();
}

void homeMachine() {
	Timer1.detachInterrupt();
	lcdBlink();
	//while (!getStatus());			// wait until machine is online
	//Serial.print("homeing");
	print("$H");
	//while (axes[0] != 0 || axes[1] != 0) getStatus();
	if (startFlag) {
		Timer1.attachInterrupt(encISR, 10000);	// update encoder every 10ms
		zeroEncoder();
		printMenu();
	}
	print("G92 X0 Y0");
	lcdSetCursor(0,0);
	lcdNoBlink();
	zeroEncoder();
	Timer1.attachInterrupt(encISR, 10000);
}

void kill() {								// resets the CNC controller, and also resets itself for good measure
	print((char)0x18);
	wdt_disable();							// https://forum.arduino.cc/t/soft-reset-and-arduino/367284/5
	wdt_enable(WDTO_15MS);
	while (1);
}

void performAPPT() {
	int limits[2];						// index 0 stores start position, index 1 stores Y steps
	int i = cyclesN, j;
	limits[0] = axes[1];
	if (cyclesY > 0) {
		// round down to nearest feasible if exceeding bed size
		if (MAX_Y < (limits[0] + cyclesY * STEP_SIZE)) limits[1] = (int)((MAX_Y - limits[0]) / STEP_SIZE);
		else limits[1] = cyclesY;
	} else if (cyclesY < 0) {
		if (0 > (limits[0] + cyclesY * STEP_SIZE)) limits[1] = (int)(limits[0] / STEP_SIZE);
		else limits[1] = cyclesY;
		limits[1] = abs(limits[1]);
	}
	// disable inputs for safety, allow encoder button to function as reset
	Timer1.detachInterrupt();
	detachInterrupt(digitalPinToInterrupt(7));
	attachInterrupt(digitalPinToInterrupt(7), kill, RISING); // the urge to kill is rising
	do {
		lcdprint(17, 3, "%3d", i);
		j = limits[1];
		do {
			print("G1 X%d F%d", (int)MAX_X, (int)(feedrate * 600));
			print("G1 X0");
			while (axes[0] != 0 || axes[1] != 0);
			if (j > 0) {
				if (cyclesY > 0) print("$J=G91 Y%d F%d", (int)(STEP_SIZE), (int)(feedrate * 30));
				else print("$J=G91 Y%d F%d", (int)(-STEP_SIZE), (int)(feedrate * 30));
			}
			j--;
		} while (j > 0);
		if (limits[1] > 0) print("G1 X0 Y%d", limits[0]);
		i--;
	} while (i > 1);
	detachInterrupt(digitalPinToInterrupt(7));
	zeroEncoder();
	Timer1.attachInterrupt(encISR, 10000);
	attachInterrupt(digitalPinToInterrupt(7), btnISR, RISING);
	lcdprint(17, 3, "   ");
}