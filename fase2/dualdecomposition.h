#ifndef DUAL_DECOMPOSITION_H
#define DUAL_DECOMPOSITION_H

#include <Arduino.h>
#include "calibrationMatrix.h"
#include "config.h"

class DualDecomposition {
private:
    uint8_t my_id;
    float c;        // Linear cost coefficient
    float q;        // Quadratic cost coefficient (regularization)
    float alpha;    // Gradient ascent step size for dual variable

    float u_own;    // Local primal variable (0.0 to 100.0 scale, i.e. % duty)
    float u_ant;    // Previous primal variable (for low-pass filter)
    float l_own;    // Local dual variable (Lagrange multiplier)

    // Arrays indexed by CalibrationMatrix index (0-based, up to MAX_NODES)
    float u_other[MAX_NODES];
    float price_other[MAX_NODES];

public:
    DualDecomposition(uint8_t node_id, float c_val, float q_val, float alpha_val);

    void reset();

    // Store incoming values from the CAN bus
    void updateOtherU(uint8_t other_id, float u_val, const CalibrationMatrix& matrix);
    void updateOtherPrice(uint8_t other_id, float price_val, const CalibrationMatrix& matrix);

    // Core iterations — return updated value
    // u result is in 0-100 scale (percentage duty cycle)
    float compute_u(float k_ii, const CalibrationMatrix& matrix);

    // d = background illuminance (lux), L = illuminance lower bound (lux)
    //float compute_l(float k_ii, float d, float L, const CalibrationMatrix& matrix);
    float compute_l(float y_meas, float L);
    // Price this node sends to another node
    float compute_price_for(uint8_t other_id, const CalibrationMatrix& matrix);

    // Getters
    float get_u() const { return u_own; }
    float get_l() const { return l_own; }

    // Setters
    void set_c(float new_c) { c = new_c; }
    void set_alpha(float new_alpha) { alpha = new_alpha; }
};

#endif
