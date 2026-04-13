#ifndef CALIBRATION_MATRIX_H
#define CALIBRATION_MATRIX_H

#include <Arduino.h>
#include "config.h"

class CalibrationMatrix {
private:
    uint8_t my_id;
    uint8_t active_nodes[MAX_NODES];
    float local_gains[MAX_NODES]; // How other nodes affect me (My row in the K matrix)
    int num_active_nodes;


public:
    CalibrationMatrix(uint8_t node_id);

    void setMyId(uint8_t id);
    void addNode(uint8_t id);
    void removeNode(uint8_t id);
    void sortNodes(); // Keeps the token ring order consistent across the network
    
    // Gain updates and retrieval
    void updateGain(uint8_t target_id, float gain);
    float getGain(uint8_t target_id) const;

    // Getters for network iteration
    const uint8_t* getNodesArray() const;
    int getNumNodes() const;
    int findNodeIndex(uint8_t id) const;
    void clear();
};

#endif