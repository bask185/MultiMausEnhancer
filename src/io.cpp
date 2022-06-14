#include <Arduino.h>
#include "io.h"
extern void initIO(void) {
	pinMode(RS485DIR, OUTPUT);
	pinMode(detector1pin, INPUT_PULLUP);
	pinMode(detector2pin, INPUT_PULLUP);
	pinMode(servoPin1, OUTPUT);
	pinMode(playPin, INPUT_PULLUP);
	pinMode(stopPin, INPUT_PULLUP);
	pinMode(recordPin, INPUT_PULLUP);
	pinMode(greenLedPin, OUTPUT);
	pinMode(yellowLedPin, OUTPUT);
	pinMode(redLedPin, OUTPUT);
	pinMode(lowPosPin, INPUT);
	pinMode(highPosPin, INPUT);
	pinMode(speedPin, INPUT);
	pinMode(waitPin, INPUT);
}