#include "metrics.h"

Metrics::Metrics(float pmax) : p_max(pmax) {
    reset();
}

void Metrics::reset() {
    energy = 0; 
    visibility_error = 0; 
    flicker = 0;
    n_samples = 0;
    prev_u1 = 0; prev_u2 = 0;
}

void Metrics::update(float u_k, float lux_k, float ref_k, float delta_t) {
    // Save in the buffer before updating head	
    buffer_y[head] = lux_k;
    buffer_u[head] = u_k;
    head = (head + 1) % BUFFER_SIZE;
    n_samples++;

    // Energy: E = Pmax * sum( d(k-1) * delta_t )
    energy += p_max * prev_u1 * delta_t;

    // Erro de Visibilidade: V = (1/N) * sum( max(0, L-l) )
    // Nota: O guia sugere que a visibilidade é melhor estimada pelo comando LED,
    // mas o erro médio usa a iluminância medida (l).
    visibility_error += max(0.0f, ref_k - lux_k);

    // Flicker: f_k = |d_k - d_k-1| + |d_k-1 - d_k-2| se houver mudança de sinal 
    if (n_samples > 2) {
        if ((u_k - prev_u1) * (prev_u1 - prev_u2) < 0) {
            flicker += (abs(u_k - prev_u1) + abs(prev_u1 - prev_u2));
        }
    }

    // Update the variables for the next cicle of 10 ms
    prev_u2 = prev_u1;
    prev_u1 = u_k;
}

float Metrics::getAverageVisibility() {
    if (n_samples == 0) return 0.0f;
    return visibility_error / n_samples;
}

float Metrics::getInstantaneousPower(float u_k) {
    // A potência instantânea é o duty cycle atual multiplicado pela potência máxima
    return u_k * p_max;
}

// Retorna o valor na posição 'pos' (0 é a mais antiga, 5999 a mais recente)
float Metrics::getBufferValue(char var, int pos) {
    // Calcula o índice real no buffer circular partindo da amostra mais antiga
    int index = (head + pos) % BUFFER_SIZE;
    return (var == 'y') ? buffer_y[index] : buffer_u[index];
}
