% Dados fornecidos
duty_cycle = linspace(0.1, 1, 10);
lux_measured = [6.06 11.96 17.26 22.58 27.66 32.62 37.24 42.00 46.40 50.22];
zero_lux = 0.2;

% Cálculo do Ganho Instantâneo K
K_instantaneous = (lux_measured - zero_lux) ./ duty_cycle;

% Modelo Linear de Calibração (baseado no ponto u=1.0)
K_static = (lux_measured(end) - zero_lux) / 1.0;
lux_linear_model = K_static * duty_cycle + zero_lux;

% Gráfico 1: Resposta do Sistema vs Modelo Linear
figure('Color', 'w');
subplot(2,1,1);
plot(duty_cycle, lux_measured, 'bo-', 'LineWidth', 1.5, 'DisplayName', 'Measured Data');
hold on;
plot(duty_cycle, lux_linear_model, 'r--', 'DisplayName', ['Linear Model (K=' num2str(K_static) ')']);
grid on; ylabel('Illuminance [Lux]');
title('System Non-linearity Demonstration');
legend('Location', 'best');

% Gráfico 2: Variação do Ganho K
subplot(2,1,2);
plot(duty_cycle, K_instantaneous, 'mo-', 'LineWidth', 1.5);
hold on;
grid on; xlabel('Duty Cycle (u)'); ylabel('Gain K [Lux/u]');
title('Instantaneous Gain Variation');