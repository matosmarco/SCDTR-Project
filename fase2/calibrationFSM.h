#ifndef CALIBRATION_FSM_H
#define CALIBRATION_FSM_H

#include <Arduino.h>
#include <FreeRTOS.h>
#include <task.h>
#include "calibrationMatrix.h"
#include "led.h"
#include "ldr.h"
#include "box.h"

// Task Notification Events
#define EVENT_NETWORK_CHANGE  (1 << 0)
#define EVENT_TOKEN_RCVD      (1 << 1)
#define EVENT_TOKEN_DONE      (1 << 2)

enum class CalibState {
    IDLE,
    WAIT_TOPOLOGY,
    SYNC_TOKEN,
    MEASURE_OWN,
    DONE
};

class CalibrationFSM {
private:
    CalibState currentState;
    CalibrationMatrix& calibData;
    LED& led;
    LDR& ldr;
    Box& box;
    uint8_t my_address;
    int current_token_index;
    bool has_measured_kii;
public:
    bool isCalibrated; // Exposes state to the main optimization loop

    CalibrationFSM(CalibrationMatrix& data, LED& l, LDR& r, Box& b, uint8_t addr);

    void process(uint32_t eventBits);
    void triggerRecalibration();
};

#endif