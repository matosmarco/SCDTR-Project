#ifndef BOX_H
#define BOX_H

#include <Arduino.h>
#include "ldr.h"
#include "led.h"


class Box{
private:
	float backgroundLux;
	float gain;
public:
	Box();

	void calibrate_background(LED& led, LDR& ldr);
	void identify_static_gain(LED& led, LDR& ldr);
	
	// System identification methods
	void do_step_response(LED& led, LDR& ldr, float u_init, float u_final);
	void do_staircase_response(LED& led, LDR& ldr);

	// Get values
	float get_gain();
	float get_background();
};


#endif
