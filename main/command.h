#ifndef COMMAND_H
#define COMMAND_H

#include <Arduino.h>
#include "led.h"
#include "ldr.h"
#include "box.h"
#include "metrics.h"
#include "luminaire.h"
#include "pid.h"
#include "can_network.h" // Added CAN
#include "hub_router.h"  // Added Router
// Global function to process commands
void processCommand(String cmd, LED& led, LDR& ldr, Box& box, Metrics& metrics, Luminaire& luminaire, pid& pid);

#endif
