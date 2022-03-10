#include <Arduino.h>
#include "io.h"
extern void initIO(void) {
	pinMode(RS485DIR, OUTPUT);
	pinMode(greenLed, OUTPUT);
	pinMode(yellowLed, OUTPUT);
	pinMode(redLed, OUTPUT);
}