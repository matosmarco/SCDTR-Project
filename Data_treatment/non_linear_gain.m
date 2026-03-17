%% SCDTR - Prova de Não-Linearidade do Ganho K
clear; clc; close all;

% Dados de slope_calibration.m
data = [
    0.10, 0.5061, 55207.01, 4.7420;
    0.20, 0.8252, 29990.23, 4.4770;
    0.30, 1.0525, 21355.29, 4.3295;
    0.40, 1.2370, 16677.52, 4.2221;
    0.50, 1.3772, 13961.38, 4.1449;
    0.60, 1.5037, 11945.34, 4.0772;
    0.70, 1.6093, 10505.76, 4.0214;
    0.80, 1.7060,  9343.41, 3.9705;
    0.90, 1.7866,  8470.91, 3.9279;
];

u = data(:,1); v = data(:,2); log10_R = data(:,4);
m = -0.8492; b = 5.22; % Parâmetros identificados

% Conversão para Lux para ver o ganho final
lux = 10.^((log10_R - b) / m);

% Cálculo dos Ganhos Locais (Derivada Numérica)
du = diff(u);
K_raw = diff(v) ./ du;   % Ganho em V/u
K_lux = diff(lux) ./ du; % Ganho em Lux/u
u_mid = (u(1:end-1) + u(2:end)) / 2; % Pontos médios para o plot

% --- PLOTS ---
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 8]);

% Gráfico 1: Ganho Não Linear do Hardware
subplot(2,1,1);
plot(u_mid, K_raw, 'r-s', 'LineWidth', 2, 'MarkerFaceColor', 'r');
grid on; ylabel('Raw Gain [V / u]');
title('Non-Linearity Proof: Hardware Gain K changes with u');
legend('K_{raw} = \Delta V / \Delta u');

% Gráfico 2: Ganho do Sistema Linearizado
subplot(2,1,2);
plot(u_mid, K_lux, 'b-o', 'LineWidth', 2, 'MarkerFaceColor', 'b');
yline(mean(K_lux), 'k--', ['Mean K: ' num2str(mean(K_lux), '%.1f')]);
grid on; xlabel('Operating Point (Duty Cycle u)'); ylabel('System Gain [Lux / u]');
title('Linearized Gain K (System Response after Log-Log Model)');
legend('K_{lux} = \Delta Lux / \Delta u');

fprintf('Variação do Ganho Bruto: de %.2f a %.2f V/u\n', max(K_raw), min(K_raw));