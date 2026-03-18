#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include <SPI.h>
#include <mcp2515.h>
#include "can_protocol.h"

class CanNetwork {
private:
    MCP2515 mcp2515;
public:
    CanNetwork(const uint8_t cs_pin = 17); // GP17 é o default para CS no SPI0
    void begin();
    void sendMessage(const CanMessage& msg);
    bool receiveMessage(CanMessage& msg);
};

#endif