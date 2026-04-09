#ifndef ADMM_H
#define ADMM_H

#include <Arduino.h>
#include "calibrationMatrix.h" // Para usar o MAX_NODES e conversão de IDs

class ADMM {
private:
    uint8_t my_id;
    int my_idx;
    int num_nodes;
    float rho;

    // Mathematics vectors (maximum size MAX_NODES)
    float u_own[MAX_NODES];     // Estimate of u's
    float u_av[MAX_NODES];      // Network u's average
    float lambda[MAX_NODES];    // Dual variables
    float c[MAX_NODES];         // Cost vector (only my index has value)
    float k_row[MAX_NODES];     // My list in the global K matri

    // Matrix to store u received from the neighbours
    float network_u[MAX_NODES][MAX_NODES];

    // Auxiliary mathematical functions to run the algorithm 
    float dot_product(const float* a, const float* b, int n);
    float norm_sq(const float* a, int n);
    bool check_feasibility(const float* u_test, float d, float L);
    float evaluate_cost(const float* u_test);

public:
    // Constructor
    ADMM(uint8_t node_id, float rho_val);

    // Controller reset
    void reset();

    // Update the parameters before iterate
    void update_params(float local_cost, const CalibrationMatrix& matrix, const float global_K[][MAX_NODES]);

    // Receives and stores the 'u' sent by other nodes
    void update_network_u(uint8_t source_id, int source_idx, const float* u_received);

    // Motor principal (tradução do consensus_iterate.m)
    // Retorna o u que este nó deve aplicar no seu próprio LED (escala 0-100)
    float consensus_iterate(float y_meas, float L);
    // Retorna o vetor u_own inteiro (para enviar pela rede CAN)
    const float* get_u() const { return u_own; }
    // Recebe um único elemento do vetor de um vizinho
    void update_network_u_element(int source_idx, int target_idx, float val);
};

#endif
