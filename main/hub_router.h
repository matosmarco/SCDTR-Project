#ifndef HUB_ROUTER_H
#define HUB_ROUTER_H

#include <Arduino.h>
#include "can_network.h"

// Routes a command that was not intended for the local node to the CAN network
void routeCommandToCAN(String cmd, int sender_id, CanNetwork& can_net);

#endif