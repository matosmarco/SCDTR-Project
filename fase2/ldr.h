#ifndef LDR_H
#define LDR_H

#include <Arduino.h>
#include "config.h"
#include "led.h"

class LDR{
private:
	float m;
	float b;
	int adc_buffer[MEDIAN_BUFFER_SIZE];
	int bufferIndex;
	bool bufferFilled;

	// Transient variables protection
	int last_median_adc;
	const int TRANSIENT_THRESHOLD = 400;

	// Auxiliary function to order vector
	void sortArray(int* array, int size);
	void quickSort(int* array, int low, int high);
	int partition(int* array, int low, int high);
	void swap(int* a, int* b);

public:
	LDR(); // constructor
	
	// Function to read Voltage using the median filter
	float readVoltage();
	float readVoltage_avg(int samples= 50);
	float R_LDR_compute(float v);
	float luxCompute(float r);


	float readLux();
	
	void calibrate_b(float knownLux=500.0);

	void calibrate_m(LED& led);
	float get_m();
	float get_b();
};

#endif
