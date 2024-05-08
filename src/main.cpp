#include <Arduino.h>
#include "functions.h"

unsigned long startTime = 0;

void setup() {
	hwInit();
	startTime = millis();
}

void loop() {
	if (millis() - startTime >= LOOP_MS) {
		getStatus();
		if (newStats) {
			printStatus();
			newStats = false;
		}
		if (dryY != 0) {
			print("$J=G91 Y%d F%d", (int)(STEP_SIZE * dryY), (int)(feedrate * 60));
			dryY = 0;
		}
		startTime = millis();
	}
	if ((PORTD & 0b01000000) == 0) {	// normal non-interrupt killswitch
		kill();
	}
	if (homeFlag) {
		homeFlag = false;
		homeMachine();
	}
	if (APPTFlag) {
		APPTFlag = false;
		performAPPT();
	}
}