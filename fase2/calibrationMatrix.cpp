#include "calibrationMatrix.h"

CalibrationMatrix::CalibrationMatrix(uint8_t node_id) : my_id(node_id), num_active_nodes(0) {
    clear();
}

void CalibrationMatrix::clear() {
    num_active_nodes = 0;
    for (int i = 0; i < MAX_NODES; i++) {
        active_nodes[i] = 0;
        local_gains[i] = 0.0f;
    }
}

void CalibrationMatrix::setMyId(uint8_t id) {
    my_id = id;
}

int CalibrationMatrix::findNodeIndex(uint8_t id) const {
    for (int i = 0; i < num_active_nodes; i++) {
        if (active_nodes[i] == id) return i;
    }
    return -1; 
}

void CalibrationMatrix::addNode(uint8_t id) {
    if (findNodeIndex(id) != -1) return; // Node already exists
    if (num_active_nodes < MAX_NODES) {
        active_nodes[num_active_nodes] = id;
        local_gains[num_active_nodes] = 0.0f;
        num_active_nodes++;
        sortNodes(); // Ensure deterministic order for the Token Ring
    }
}

void CalibrationMatrix::removeNode(uint8_t id) {
    int idx = findNodeIndex(id);
    if (idx == -1) return;

    // Shift arrays left to maintain contiguity
    for (int i = idx; i < num_active_nodes - 1; i++) {
        active_nodes[i] = active_nodes[i + 1];
        local_gains[i] = local_gains[i + 1];
    }
    num_active_nodes--;
}

void CalibrationMatrix::sortNodes() {
    // Insertion sort to ensure all nodes follow the exact same token order
    for (int i = 1; i < num_active_nodes; i++) {
        uint8_t key_node = active_nodes[i];
        float key_gain = local_gains[i];
        int j = i - 1;

        while (j >= 0 && active_nodes[j] > key_node) {
            active_nodes[j + 1] = active_nodes[j];
            local_gains[j + 1] = local_gains[j];
            j = j - 1;
        }
        active_nodes[j + 1] = key_node;
        local_gains[j + 1] = key_gain;
    }
}

void CalibrationMatrix::updateGain(uint8_t target_id, float gain) {
    int idx = findNodeIndex(target_id);
    if (idx != -1) {
        local_gains[idx] = gain;
    }
}

float CalibrationMatrix::getGain(uint8_t target_id) const {
    int idx = findNodeIndex(target_id);
    if (idx != -1) return local_gains[idx];
    return 0.0f;
}

const uint8_t* CalibrationMatrix::getNodesArray() const { return active_nodes; }
int CalibrationMatrix::getNumNodes() const { return num_active_nodes; }