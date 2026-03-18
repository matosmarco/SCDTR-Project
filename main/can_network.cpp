#include "can_network.h"

CanNetwork::CanNetwork(const uint8_t cs_pin) : mcp2515(cs_pin) {}

void CanNetwork::begin() {
    SPI.begin();
    mcp2515.reset();
    // ATENÇÃO: Dependendo do cristal do vosso módulo, pode ser MCP_8MHZ ou MCP_16MHZ!
    mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ); 
    mcp2515.setNormalMode();
}

void CanNetwork::sendMessage(const CanMessage& msg) {
    struct can_frame frame;
    frame.can_id = msg.sender_id; // O ID da mensagem CAN é o ID do nó
    frame.can_dlc = 8;            // 8 bytes estritos
    memcpy(frame.data, &msg, 8);
    mcp2515.sendMessage(&frame);
}

bool CanNetwork::receiveMessage(CanMessage& msg) {
    struct can_frame frame;
    if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
        if (frame.can_dlc == 8) {
            memcpy(&msg, frame.data, 8);
            return true;
        }
    }
    return false;
}