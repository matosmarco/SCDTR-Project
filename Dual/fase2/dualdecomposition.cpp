#include "dualdecomposition.h"

extern float global_K_matrix[MAX_NODES][MAX_NODES];

DualDecomposition::DualDecomposition(uint8_t node_id, float c_val, float q_val, float alpha_val) {
    my_id = node_id;
    c = c_val;
    q = q_val;
    alpha = alpha_val;
    reset();
}

void DualDecomposition::reset() {
    u_own = 0.0f;
    u_ant = 0.0f;
    l_own = 0.0f;
    for (int i = 0; i < MAX_NODES; i++) {
        u_other[i] = 0.0f;
        price_other[i] = 0.0f;
    }
}

void DualDecomposition::updateOtherU(uint8_t other_id, float u_val, const CalibrationMatrix& matrix) {
    // Guard: never overwrite our own value
    if (other_id == my_id) return;

    int idx = matrix.findNodeIndex(other_id);
    if (idx != -1 && idx < MAX_NODES) {
        u_other[idx] = u_val;
    }
}

void DualDecomposition::updateOtherPrice(uint8_t other_id, float price_val, const CalibrationMatrix& matrix) {
    if (other_id == my_id) return;

    int idx = matrix.findNodeIndex(other_id);
    if (idx != -1 && idx < MAX_NODES) {
        price_other[idx] = price_val;
    }
}

float DualDecomposition::compute_u(float k_ii, const CalibrationMatrix& matrix) {
    // Sum prices received from all other nodes
    float total_price = 0.0f;
    int n = matrix.getNumNodes();
    const uint8_t* nodes = matrix.getNodesArray();

    for (int i = 0; i < n; i++) {
        uint8_t other_id = nodes[i];
        if (other_id != my_id) {
            int idx = matrix.findNodeIndex(other_id);
            if (idx != -1) {
                total_price += price_other[idx];
            }
        }
    }

    // Primal update: u = (-c + lambda_own * k_ii + sum_prices_from_others) / (2*q)
    // This matches the Matlab: u = (-c + l_own*k_own + price_other) / (2*q)
    float u_raw = (-c + (l_own * k_ii) + total_price) / (2.0f * q);

    // Low-pass filter for convergence (matches Matlab: u = 0.5*u + 0.5*u_ant)
    float u_filtered = 0.5f * u_raw + 0.5f * u_ant;

    // Clamp to [0, 100] (duty cycle percentage)
    if (u_filtered < 0.0f)   u_filtered = 0.0f;
    if (u_filtered > 100.0f) u_filtered = 100.0f;

    u_own = u_filtered;
    u_ant = u_filtered;
    return u_own;
}
/*
float DualDecomposition::compute_l(float k_ii, float d, float L, const CalibrationMatrix& matrix) {
    // Sum physical illuminance contribution: k_ii * u_own + sum_j(k_ij * u_j)
    // u values are in 0-100 scale (percentage). k gains are lux per 100% duty.
    // The product k_ij * u_j must therefore be scaled: k_ij * (u_j / 100) gives lux.
    // But since the Matlab keeps u in 0-100 and K in lux/unit, we match that convention:
    // L and d must also be consistent. If L is in lux and k*u should produce lux,
    // then u should be in [0,1]. We handle this by scaling: the constraint is
    //   k_ii*(u_own/100) + sum_j(k_ij*(u_j/100)) + d >= L
    // which is equivalent to:
    //   (1/100)*[k_ii*u_own + sum_j(k_ij*u_j)] + d >= L
    // Gradient w.r.t. lambda: -(1/100)*sum_k(k*u) - d + L
    //
    // NOTE: To keep this consistent with the Matlab code (which keeps u in 0-100
    // and L in lux units), we normalize the k-matrix gain to work with the 0-100 scale:
    //   effective_k = k / 100  (so that k_effective * u_100 = k * u_01 in lux)
    // This is applied here so the rest of the code does not need to change.

    const float SCALE = 1.0f / 100.0f; // converts 0-100 duty to 0-1 duty

    float sum_k_u = k_ii * u_own * SCALE;

    int n = matrix.getNumNodes();
    int my_idx = matrix.findNodeIndex(my_id);

    for (int i = 0; i < n; i++) {
        uint8_t other_id = matrix.getNodesArray()[i];
        if (other_id != my_id) {
            int other_idx = matrix.findNodeIndex(other_id);
            if (my_idx != -1 && other_idx != -1) {
                float k_ij = global_K_matrix[my_idx][other_idx];
                sum_k_u += k_ij * u_other[other_idx] * SCALE;
            }
        }
    }

    // Dual update: lambda = lambda + alpha * (L - d - sum_k_u)
    // Constraint: sum_k_u + d >= L  =>  violation = L - d - sum_k_u
    float violation = L - d - sum_k_u;
    float new_l = l_own + alpha * violation;

    // Dual variable must be >= 0 (Lagrange multiplier for inequality constraint)
    if (new_l < 0.0f) new_l = 0.0f;

    l_own = new_l;
    return l_own;
}

*/
float DualDecomposition::compute_l(float y_meas, float L) {
    // Calculamos o erro fisicamente medido: Referência - Luz Real
    float violation = L - y_meas;
    
    // Atualiza a Variável Dual (Preço base)
    float new_l = l_own + alpha * violation;

    // O multiplicador de Lagrange não pode ser negativo
    if (new_l < 0.0f) new_l = 0.0f;
    if (new_l > 50.0f) new_l = 50.0f; // Ajusta este valor conforme os teus ganhos K
    l_own = new_l;
    return l_own;
}

float DualDecomposition::compute_price_for(uint8_t other_id, const CalibrationMatrix& matrix) {
    // Price this node sends to other_id = lambda_own * k_ij
    // where k_ij is the gain that THIS node's LED has on OTHER node's sensor.
    // Must use the same SCALE as compute_l so units are consistent.
    const float SCALE = 1.0f / 100.0f;

    int my_idx = matrix.findNodeIndex(my_id);
    int other_idx = matrix.findNodeIndex(other_id);

    float k_ij = 0.0f;
    if (my_idx != -1 && other_idx != -1) {
        // k_ij: effect of MY led on OTHER node's sensor
        // In global_K_matrix[row][col]: row = sensor node, col = LED node
        // So we want global_K_matrix[other_idx][my_idx]
        k_ij = global_K_matrix[other_idx][my_idx];
    }

    return l_own * k_ij * SCALE;
}
