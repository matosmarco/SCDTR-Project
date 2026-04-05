#include "led.h"

LED::LED(){}

void LED::initPWM(int freq, int range){
	analogWriteFreq(freq);
	analogWriteRange(range);
	pinMode(LED_PIN, OUTPUT);
}

// Duty Cycle (0.0 to 1.0)
void LED::setDuty(float duty, int range){
	duty = constrain(duty, 0.0, 1.0);// just to guarantee that the program is robust, just values between 0 and 1 are used
	analogWrite(LED_PIN, duty*range);
}

// Percentage (0 to 100%)
void LED::setPercentage(float percentage, int range){
	percentage = constrain(percentage, 0.0, 100.0);
	float duty = percentage/100.0;
	analogWrite(LED_PIN, duty*range);
}

// Raw data (from 0 to range; e.g: 255 or 4095)
void LED::setRaw(int value, int range){
	value = constrain(value, 0.0, range);
	analogWrite(LED_PIN, value);
}

void LED::off(){
	 analogWrite(LED_PIN, 0);
}
