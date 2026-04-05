#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include "config.h"

class LED{
public:
	LED();

	// 1 kHz lowers the noise in the circuit and 4095 (ADC_MAX - 1) corresponds to the 12-bits ADC Resolution
	void initPWM(int freq = 1000, int range = (ADC_MAX-1));
	
	// Different formats for the dimming specification
	void setDuty(float duty, int range = (ADC_MAX - 1)); // 0.0 to 1.0 
	void setPercentage(float percent2age, int range = (ADC_MAX-1));// 0.0% to 100.0%
	void setRaw(int value, int range = (ADC_MAX-1)); // 0 to 4095
	void off();
};

#endif
