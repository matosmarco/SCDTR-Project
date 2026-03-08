#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>
#include "led.h"
#include "ldr.h"
#include "box.h"

// Global function to process commands
void processCommand(String cmd, LED& led, LDR& ldr, Box& box);

#endif
