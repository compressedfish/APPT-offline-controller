#define Y_COLUMNS_MAX 30
#define CYCLES_MAX 999
#define FEEDRATE_MAX 250 	// cm/s (150000 mm/min)
#define LOOP_MS 200
#define STEP_SIZE 40			// mm
#define MAX_X 1000				// mm
#define MAX_Y 1000				// mm

#include <Arduino.h>
#include <Wire.h>
#include <Encoder.h>
#include <LiquidCrystal.h>
#include <avr/wdt.h>

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

Encoder enc(2, 3);
LiquidCrystal lcd(4, 5, 6, 7, 8, 9);

byte axes[2];					// x, y position
byte feeds[2];				// live feedrates
byte bf[2];						// buffer state
char *state;

boolean startFlag = true;
boolean newStats = false;
int feedrate = 20;		// feedrate in cm/s (min: 1)
int dryY = 0;					// columns to move w/o APPT
int cyclesY = 0;			// columns to move w/ APPT
int cyclesN = 1;			// how many times to perform treatment (min: 1)
int *pointy;					// pointer to reduce redundant code
//                         ________ menu item APPT
//                        | _______ menu item feedrate
//                        || ______ menu item move Y
//                        ||| _____ menu item home machine
//                        |||| ____ submenu item APPT cycles/repeats
//                        ||||| ___ submenu item APPT columns to cover
//                        |||||| __ submenu item feedrate
//                        ||||||| _ submenu item move Y
volatile byte menuPos = 0b00010000;
volatile byte encPos = 0;
int overflowMenuPos = 0;
byte shift = 0;
char receiveBuffer[128];
unsigned long startTime = 0;
char writeBuffer[19];

void btnISR();
void encISR();
void homeMachine();
void printMenu();
byte getStatus();
void printStatus();
void performAPPT();
void print(char *i);
void kill();

void setup() {
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
	Serial1.setTimeout(200);
  Serial1.begin(115200);
	startTime = millis();
}

void loop() {
	if (millis() - startTime >= LOOP_MS) {
		getStatus();
		if (newStats) {
			printStatus();
			newStats = false;
		}
		if (dryY) {
			sprintf(writeBuffer, "$J=G91 Y%d F%d", STEP_SIZE * dryY, feedrate * 600);
			print(writeBuffer);
			dryY = 0;
		}
	}
	if ((PORTD & 0b01000000) == 0) {	// normal non-interrupt killswitch
		kill();
	}
}

int clamp(int n, int minn, int maxn) {							// https://stackoverflow.com/questions/5996881/how-to-limit-a-number-to-be-within-a-specified-range-python
	return max(min(maxn, n), minn);
}

bool split(char* input, char** next, char delim) {	// https://github.com/bdring/PendantsForFluidNC/blob/main/lib/GrblParserC/src/GrblParserC.c
    char* pos = strchr(input, delim);
    if (pos) {
        *pos  = '\0';
        *next = pos + 1;
        return true;
    }
    *next = input + strlen(input);  // End of string
    return false;
}

static void parse_integers(char* s, byte* nums, byte maxnums) {	// modified it so it actually returns bytes lmao
    char*  next;
    size_t i = 0;
    do {
        if (i >= maxnums) {
            return;
        }
        split(s, &next, ',');
        nums[i++] = atoi(s);

        s = next;
    } while (*s);
}

void print(char *i) {		
	PORTD |= 0b10000000;
	Serial1.flush();
	Serial1.println(i);
	PORTD &= 0b01111111;
}

// screw it, menu functionality handled inside ISRs
void btnISR() {
  switch (menuPos << 4) {
		case 0b00010000:
			homeMachine();
			break;
    case 0:   					// enter submenu
      menuPos |= menuPos >> 5;
      break;
    case 0b01000000:  	// change from columns to cycles
      menuPos ^= 0b00001100;
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
				dryY = clamp(dryY, Y_COLUMNS_MAX, -Y_COLUMNS_MAX);
				break;
			case 0b01000010:  // feedrate
				feedrate += enc.read();
				feedrate = clamp(feedrate, 1, FEEDRATE_MAX);
				break; 
			case 0b10000100:	// columns to cover
				cyclesY += enc.read();
				cyclesY = clamp(cyclesY, Y_COLUMNS_MAX, -Y_COLUMNS_MAX);
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

void printMenu() {
  lcd.setCursor(17, 0);
  lcd.print("  0");
  lcd.setCursor(1, 1);
  lcd.print("Move Y       Y:  0");
  lcd.setCursor(1, 2);
  char buf[19];
	sprintf(buf, "FR:%3d FS:  0", feedrate);
  lcd.print(buf);
  lcd.setCursor(1, 3);
	sprintf(buf, "APPT C:%3d R:%+3d", cyclesY, cyclesN);
  lcd.print(buf);
}

byte getStatus() {				// https://forum.arduino.cc/t/serial-input-basics-updated/382007/2
	char *startC = NULL, *value, *mainNext;
	byte endC = 0;
	while (!endC || startC == NULL) {
		char temp[] = {'$', 'X', '\0'};
		print(temp);
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
				// buf_avail,rx_avail
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
	char buf[5], lbuf[19];
	lcd.setCursor(17, 0);
	sprintf(buf, "%3d", (int)(axes[0]/10));
	lcd.print(buf);
	lcd.setCursor(17, 1);
	sprintf(buf, "%3d", (int)(axes[1]/10));
	lcd.print(buf);
	lcd.setCursor(1, 2);
	sprintf(lbuf, "FR:%3dFS:%3d %5s", feedrate, (int)(feeds[0]/600), state);
	lcd.print(lbuf);
	lcd.setCursor(1, 3);
	sprintf(lbuf, "APPT C:%+3dR:%3d", cyclesY, cyclesN);
	lcd.print(lbuf);
	switch (menuPos) {
		case 0b00010000:
			lcd.setCursor(0, 0);
			break;
		case 0b00100000:
		case 0b00100001:
			lcd.setCursor(0, 1);
			break;
		case 0b01000000:
		case 0b01000010:
			lcd.setCursor(0, 2);
			break;
		case 0b10000000:
			lcd.setCursor(0, 3);
			break;
		case 0b10000100:
			lcd.setCursor(5, 3);
			break;
		case 0b10001000:
			lcd.setCursor(11, 3);
	}
	if ((menuPos & 0b00001111) > 0) lcd.blink();
	else lcd.noBlink();
}

void homeMachine() {
	lcd.blink();
	while (!getStatus());			// wait until machine is online
	char temp[] = {'$', 'H', '\0'};
	print(temp);
	while (axes[0] != 0 || axes[1] != 0) getStatus();
	if (startFlag) {
		attachInterrupt(digitalPinToInterrupt(2), encISR, CHANGE);
		attachInterrupt(digitalPinToInterrupt(3), encISR, CHANGE);
		enc.write(0);
		printMenu();
	}
	char temp2[] = {'G', '9', '2', ' ', 'X', '0', ' ', 'Y', '0', '\0'};
	print(temp2);
	lcd.setCursor(0,0);
	lcd.noBlink();
}

void kill() {								// resets the CNC controller, and also resets itself for good measure
	char c[] = {0x18, '\0'};
	print(c);
	wdt_disable();						// https://forum.arduino.cc/t/soft-reset-and-arduino/367284/5
  wdt_enable(WDTO_15MS);
  while (1);
}

void performAPPT() {
	int limits[2];						// index 0 stores start position, index 1 stores Y steps
	int i = cyclesN, j;
	char stat[3];
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
	detachInterrupt(digitalPinToInterrupt(2));	
	detachInterrupt(digitalPinToInterrupt(3));
	detachInterrupt(digitalPinToInterrupt(7));
	attachInterrupt(digitalPinToInterrupt(7), kill, RISING); // the urge to kill is rising
	do {
		lcd.setCursor(17, 3);
		sprintf(stat, "%3d", i);
		lcd.print(stat);
		j = limits[1];
		do {
			sprintf(writeBuffer, "G1 X%d F%d", MAX_X, feedrate * 600);
			print(writeBuffer);
			char temp[] = {'G', '1', ' ', 'X', '0'};
			print(temp);
			while (axes[0] != 0 || axes[1] != 0);
			if (j > 0) {
				if (cyclesY > 0) sprintf(writeBuffer, "$J=G91 Y%d F%d", STEP_SIZE, feedrate * 30);
				else sprintf(writeBuffer, "$J=G91 Y%d F%d", -STEP_SIZE, feedrate * 30);
				print(writeBuffer);
			}
			j--;
		} while (j > 0);
		if (limits[1] > 0) {
			sprintf(writeBuffer, "G1 X0 Y%d", limits[0]);
			print(writeBuffer);
		}
		i--;
	} while (i > 1);
	detachInterrupt(digitalPinToInterrupt(7));
	attachInterrupt(digitalPinToInterrupt(2), encISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(3), encISR, CHANGE);
	attachInterrupt(digitalPinToInterrupt(7), btnISR, RISING);
	lcd.setCursor(17, 3);
	lcd.print("   ");
}