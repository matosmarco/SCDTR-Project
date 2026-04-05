#ifndef CONFIG_H
#define CONFIG_H

// Raspberry Pi Pico Pins (on Pinout document)
const int LED_PIN = 13; // GP13 (physical pin: PIN17)
const int LDR_PIN = 26; // GP26 (also ADC0 and physical pin: PIN31)

// Electrical variables
const float vcc = 3.3;
const float r_read_circuit = 10e3;
const int ADC_MAX = 4096;

// Median Filter, with circular buffer
static const int MEDIAN_BUFFER_SIZE = 11; // odd number makes it easier to compute the median

#endif
