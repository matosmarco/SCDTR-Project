#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h> 

// Raspberry Pi Pico Pins (on Pinout document)
const int LED_PIN = 13; // GP13 (physical pin: PIN17)
const int LDR_PIN = 26; // GP26 (also ADC0 and physical pin: PIN31)

// Electrical variables
const float vcc = 3.3;
const float r_read_circuit = 10e3;
const int ADC_MAX = 4096;

// Median Filter, with circular buffer
static const int MEDIAN_BUFFER_SIZE = 11; // odd number makes it easier to compute the median

// MAX_SIZE of the message queue
#define QUEUE_SIZE 300
#define MAX_NODES 10
// CAN PROTOCOL (Exactly 8 Bytes)
// The __attribute__((packed)) ensures the compiler does not add invisible padding.
// This guarantees the struct takes exactly 8 bytes to fit inside a CAN bus frame.
typedef struct __attribute__((packed)) {
  uint8_t src_id;   // 1 byte: Source ID (Sender)
  uint8_t dest_id;  // 1 byte: Destination ID (255 = Broadcast)
  char cmd;         // 1 byte: Main command (e.g., 'u', 'r', 'g', ...)
  char subcmd;      // 1 byte: Subcommand (used with 'g', e.g., 'y')
  float value;      // 4 bytes: Numeric value (duty cycle, reference, lux, etc.)
} ProtocolMsg_t;

#endif
