#ifndef NETWORK_H
#define NETWORK_H

#include <Arduino.h>
#include "dualdecomposition.h"
// Declaração das funções de rede e consola
void checkSerial();
void processNetworkMessages();
void sendMessage();
void readMessage();
void handleCANErrors();

#endif