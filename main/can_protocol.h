#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H
#include <Arduino.h>

// A magia dos 8 bytes (sem padding do compilador)
#pragma pack(push, 1)
struct CanMessage {
    char type;          // Byte 0: 'C' (Comando do PC), 'T' (Telemetria do Nó)
    uint8_t sender_id;  // Byte 1: ID de quem envia
    uint8_t target_id;  // Byte 2: ID do destino (255 = Broadcast para todos)
    char variable;      // Byte 3: Variável ('r'=ref, 'u'=duty, 'y'=lux, 'p'=pwm)
    float value;        // Byte 4 a 7: O valor (4 bytes)
};
#pragma pack(pop)

#endif