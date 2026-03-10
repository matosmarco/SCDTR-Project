#ifndef METRICS_H
#define METRICS_H
#include <Arduino.h>

#define BUFFER_SIZE 6000 // 60 seconds at 100Hz

class Metrics{
private:
	unsigned long n_samples;
	float prev_u1; // d(k-1) - duty cycle (for energy and flicker)
	float prev_u2; // d(k-2) - for the flicker computation
	float p_max; // maximum power of the luminaire
	float buffer_y[BUFFER_SIZE];
	float buffer_u[BUFFER_SIZE];
	int head = 0; // Index of the most recent sample
public:
	float energy = 0.0; //[J]
	float visibility_error= 0.0; //[LUX]
	float flicker = 0.0; // [s^-1]
	Metrics(float pmax = 1.0); // Constructor with max power
	void reset();
	void update(float u_k, float lux_k, float ref_k, float delta_t);
	float getAverageVisibility();
	float getInstantaneousPower(float u_k);

	// Access to the buffer
	float getBufferValue(char var, int pos);
    	int getHead() { return head; } 
};

#endif
