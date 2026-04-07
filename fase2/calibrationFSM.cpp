#include "calibrationFSM.h"
#include "command.h" 
#include <queue.h>

extern QueueHandle_t canTxQueue;

CalibrationFSM::CalibrationFSM(CalibrationMatrix& data, LED& l, LDR& r, Box& b, uint8_t addr) 
    : calibData(data), led(l), ldr(r), box(b), my_address(addr), 
      currentState(CalibState::WAIT_TOPOLOGY), isCalibrated(false), current_token_index(0) {}

void CalibrationFSM::triggerRecalibration() {
    currentState = CalibState::WAIT_TOPOLOGY;
    isCalibrated = false;
    Serial.println("[FSM] Network change detected. Distributed optimization suspended. Recalibrating...");
}

void CalibrationFSM::process(uint32_t eventBits) {
    
    // Always intercept a network change event, regardless of current state
    if (eventBits & EVENT_NETWORK_CHANGE) {
        triggerRecalibration();
    }

    switch (currentState) {
        case CalibState::IDLE:
            // Do nothing, waiting for network changes
            break;

        case CalibState::WAIT_TOPOLOGY:
            // Non-blocking wait for 5 seconds to let topology stabilize (Simultaneous startups)
            vTaskDelay(pdMS_TO_TICKS(5000));
            
            calibData.sortNodes();
            current_token_index = 0;
            currentState = CalibState::SYNC_TOKEN;
            break;

        case CalibState::SYNC_TOKEN: {
            const uint8_t* nodes = calibData.getNodesArray();
            int numNodes = calibData.getNumNodes();

            if (numNodes == 0) return; 

            // If it's our turn in the Token Ring
            if (nodes[current_token_index] == my_address) {
                currentState = CalibState::MEASURE_OWN;
                // Trigger ourselves immediately without waiting
                xTaskNotify(xTaskGetCurrentTaskHandle(), EVENT_TOKEN_RCVD, eSetBits); // Dummy state to avoid unecessary wait time on fase2.ino
            } else {
                // Wait passively. The CAN RX handler will trigger EVENT_TOKEN_DONE
                if (eventBits & EVENT_TOKEN_DONE) {
                    current_token_index++;
                    if (current_token_index >= numNodes) {
                        currentState = CalibState::DONE;
                    }
                }
            }
            break;
        }

        case CalibState::MEASURE_OWN: {
            Serial.println("[FSM] My turn! Measuring self-coupling (K_ii).");
            led.setDuty(1.0f); // Turn on 100% [cite: 29]
            vTaskDelay(pdMS_TO_TICKS(2000));
            // Broadcast 'c' '1' to tell others to measure my influence (K_ij)
            ProtocolMsg_t msgMeasure = {my_address, 255, 'c', '1', 0.0f};
            xQueueSend(canTxQueue, &msgMeasure, 0);

            // Yield CPU for 2 seconds while physics stabilize
            vTaskDelay(pdMS_TO_TICKS(2000)); 

            float y_total = ldr.readLux();
            float d_k = box.fixed_background(ldr); // Retrieve background
            float k_ii = y_total - d_k;
            if (k_ii < 0) k_ii = 0;

            calibData.updateGain(my_address, k_ii);
            Serial.printf("Own gain (K_ii): %.4f\n", k_ii);

            // Turn off and broadcast 'c' '0' to pass the token [cite: 41, 42]
            led.setDuty(0.0f);
            vTaskDelay(pdMS_TO_TICKS(2000));

            ProtocolMsg_t msgEnd = {my_address, 255, 'c', '0', 0.0f};
            xQueueSend(canTxQueue, &msgEnd, 0);

            current_token_index++;
            if (current_token_index >= calibData.getNumNodes()) {
                currentState = CalibState::DONE;
            } else {
                currentState = CalibState::SYNC_TOKEN;
            }
            break;
        }

        case CalibState::DONE:
            isCalibrated = true;
            Serial.println("[FSM] Global Calibration Finished! Ready for ADMM Optimization.");
            currentState = CalibState::IDLE;
            break;
    }
}
