#include "admm.h"
#include <math.h>

ADMM::ADMM(uint8_t node_id, float rho_val) {
    my_id = node_id;
    rho = rho_val;
    num_nodes = 1;
    my_idx = 0;
    reset();
}

void ADMM::reset() {
    for (int i = 0; i < MAX_NODES; i++) {
        u_own[i] = 0.0f;
        u_av[i] = 0.0f;
        lambda[i] = 0.0f;
        c[i] = 0.0f;
        k_row[i] = 0.0f;
        for (int j = 0; j < MAX_NODES; j++) {
            network_u[i][j] = 0.0f;
        }
    }
}

// === Funções Auxiliares de Álgebra Linear ===
float ADMM::dot_product(const float* a, const float* b, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += a[i] * b[i];
    return sum;
}

float ADMM::norm_sq(const float* a, int n) {
    float sum = 0.0f;
    for (int i = 0; i < n; i++) sum += a[i] * a[i];
    return sum;
}

// Tradução de check_feasibility.m
bool ADMM::check_feasibility(const float* u_test, float d, float L) {
    float tol = 0.001f;
    if (u_test[my_idx] < 0.0f - tol) return false;
    if (u_test[my_idx] > 100.0f + tol) return false;
    
    // Convertemos a restrição física: K * (u/100) + d >= L. Como o u_test está em %, ajustamos.
    // Usamos o ganho puro se a matriz K já estiver normalizada para %.
    if (dot_product(u_test, k_row, num_nodes) < (L - d) - tol) return false; 
    
    return true;
}

// Tradução de evaluate_cost.m
float ADMM::evaluate_cost(const float* u_test) {
    float diff[MAX_NODES];
    for (int i = 0; i < num_nodes; i++) diff[i] = u_test[i] - u_av[i];
    
    return dot_product(c, u_test, num_nodes) + 
           dot_product(lambda, diff, num_nodes) + 
           (rho / 2.0f) * norm_sq(diff, num_nodes);
}

void ADMM::update_network_u(uint8_t source_id, int source_idx, const float* u_received) {
    if (source_idx != -1 && source_idx < MAX_NODES) {
        for (int i = 0; i < num_nodes; i++) {
            network_u[source_idx][i] = u_received[i];
        }
    }
}

// === Atualização de Parâmetros e Rede ===
void ADMM::update_params(float local_cost, const CalibrationMatrix& matrix, const float global_K[][MAX_NODES]) {
    num_nodes = matrix.getNumNodes();
    my_idx = matrix.findNodeIndex(my_id);
    if (my_idx == -1) return;

    for (int i = 0; i < num_nodes; i++) {
        // A GRANDE CORREÇÃO: Os vizinhos não têm energia grátis! 
        // Todos os LEDs têm o mesmo custo no algoritmo.
        c[i] = local_cost*1.5; 
        
        // Mantém a conversão da matriz
        k_row[i] = global_K[my_idx][i] / 100.0f; 
    }
}

// === O Motor do ADMM (Tradução de consensus_iterate.m) ===
// === O Motor do ADMM Numérico (Substitui as fórmulas analíticas) ===
float ADMM::consensus_iterate(float y_meas, float L) {
    if (my_idx == -1) return 0.0f;

    // 1. Calcular o 'd' adaptativo em tempo real (Fim do Efeito Ping-Pong)
    float physical_u[MAX_NODES];
    for (int i = 0; i < num_nodes; i++) {
        physical_u[i] = (i == my_idx) ? u_own[i] : network_u[i][i];
    }
    float led_lux = dot_product(k_row, physical_u, num_nodes);
    float d = y_meas - led_lux;
    if (d < 0.0f) d = 0.0f; // A sala não suga luz

    // 2. Atualizar Média da Rede (u_av)
    for (int i = 0; i < num_nodes; i++) network_u[my_idx][i] = u_own[i];
    for (int j = 0; j < num_nodes; j++) {
        float sum = 0.0f;
        for (int i = 0; i < num_nodes; i++) sum += network_u[i][j];
        u_av[j] = sum / (float)num_nodes;
    }

    // 3. Atualizar a Variável Dual (Lambda) com ANTI-WINDUP!
    for (int i = 0; i < num_nodes; i++) {
        lambda[i] = lambda[i] + rho * (u_own[i] - u_av[i]);
        // Limita a memória de erro para a placa obedecer rapidamente quando mandas para 0!
        if (lambda[i] > 5.0f) lambda[i] = 5.0f;
        if (lambda[i] < -5.0f) lambda[i] = -5.0f;
    }

    // =================================================================
    // 4. SOLVER PGD (Projected Gradient Descent)
    // Percorre infinitas possibilidades automaticamente para qualquer N!
    // =================================================================
    float u_test[MAX_NODES];
    for (int i = 0; i < num_nodes; i++) u_test[i] = u_av[i]; // Arranca do consenso

    // O rho afeta a curvatura. O passo matemático ideal é o inverso do rho.
    float step_size = 0.5f / rho; 
    
    // O microcontrolador faz este ciclo em fracções de milissegundo
    for (int iter = 0; iter < 50; iter++) {
        
        // Passo A: Descer a montanha do Custo (Gradient Step)
        for (int i = 0; i < num_nodes; i++) {
            float gradiente = c[i] + lambda[i] + rho * (u_test[i] - u_av[i]);
            u_test[i] = u_test[i] - step_size * gradiente;
        }

        // Passo B: Projeção Linear (A Restrição da Luz da Sala)
        float luz_estimada = dot_product(u_test, k_row, num_nodes);
        if (luz_estimada < (L - d)) {
            // Se falta luz, empurramos todos os LEDs na direção do seu rendimento (K)
            float k_norm = norm_sq(k_row, num_nodes);
            if (k_norm > 0.001f) {
                float correcao_necessaria = (L - d) - luz_estimada;
                for (int i = 0; i < num_nodes; i++) {
                    u_test[i] += k_row[i] * (correcao_necessaria / k_norm);
                }
            }
        }

        // Passo C: Projeção Física (As Paredes dos 0% e 100%)
        // É aqui que o código "calcula" todos os boundaries sem precisar de IFs!
        for (int i = 0; i < num_nodes; i++) {
            if (u_test[i] < 0.0f) u_test[i] = 0.0f;
            if (u_test[i] > 100.0f) u_test[i] = 100.0f;
        }
    }
    // =================================================================

    // 5. Guardar a Solução
    for (int i = 0; i < num_nodes; i++) u_own[i] = u_test[i];

    // 6. Failsafe de Saturação Extrema de Hardware
    // Se, mesmo com tudo no limite, a luz não chegar à referência, obriga a dar 100%
    if (dot_product(u_own, k_row, num_nodes) < (L - d) - 2.0f) {
        for (int i = 0; i < num_nodes; i++) u_own[i] = 100.0f;
    }

    return u_own[my_idx];
}

void ADMM::update_network_u_element(int source_idx, int target_idx, float val) {
    if (source_idx != -1 && target_idx != -1 && source_idx < MAX_NODES && target_idx < MAX_NODES) {
        network_u[source_idx][target_idx] = val;
    }
}
