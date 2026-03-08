% Calibration of the slope in the LDR formula acquisition
clear;
clc;

b_calibrated = 5.22; % Ensure you update this with your actual calibrated value

% Insert your data array here
% Format: [Duty, log10_Duty, V_LDR, R_LDR, log10_R]
% data_1 = [
% 0.10,-1.0000,0.4974,56343.72,4.7508;
% 0.20,-0.6990,0.8126,30608.08,4.4858;
% 0.30,-0.5229,1.0381,21790.02,4.3383;
% 0.40,-0.3979,1.2235,16972.02,4.2297;
% 0.50,-0.3010,1.3634,14204.13,4.1524;
% 0.60,-0.2218,1.4922,12115.53,4.0833;
% 0.70,-0.1549,1.5984,10645.74,4.0272;
% 0.80,-0.0969,1.6957,9461.26,3.9759;
% 0.90,-0.0458,1.7751,8590.15,3.9340;
% ];

data = [
0.10,-1.0000,0.5061,55207.01,4.7420;
0.20,-0.6990,0.8252,29990.23,4.4770;
0.30,-0.5229,1.0525,21355.29,4.3295;
0.40,-0.3979,1.2370,16677.52,4.2221;
0.50,-0.3010,1.3772,13961.38,4.1449;
0.60,-0.2218,1.5037,11945.34,4.0772;
0.70,-0.1549,1.6093,10505.76,4.0214;
0.80,-0.0969,1.7060,9343.41,3.9705;
0.90,-0.0458,1.7866,8470.91,3.9279;
];


log10_duty = data(:,2); % x value
log10_RLDR = data(:,5); % y value

% Linear Regression
p = polyfit(log10_duty, log10_RLDR, 1);
m_calc = p(1);
fprintf("b da regressão: %.4f", p(2))
log10_R_fit = polyval(p, log10_duty);

% R^2 Calculation
SQ_res = sum((log10_RLDR - log10_R_fit).^2);
SQ_tot = sum((log10_RLDR - mean(log10_RLDR)).^2);
R2 = 1 - (SQ_res / SQ_tot);

% Print results to the console
fprintf('LDR Parameters\n');
fprintf('Computed m: %.4f\n', m_calc);
fprintf('Calibrated b: %.4f\n', b_calibrated); % Fixed the print label here
fprintf('R^2 = %.4f\n', R2);

% Data plot
figure('Name', 'm Calibration', 'Color', 'w');
% Fixed variable name from log10_R to log10_RLDR
plot(log10_duty, log10_RLDR, 'ko', 'MarkerFaceColor', 'b', 'MarkerSize', 8, 'DisplayName', 'Data points'); 
hold on;
plot(log10_duty, log10_R_fit, 'r-', 'LineWidth', 2, 'DisplayName', 'Regression Line');

% Plot formatting
title('LDR Calibration: Sensitivity (m) Determination');
xlabel('log_{10}(Duty Cycle)');
ylabel('log_{10}(LDR Resistance)');
grid on;
legend('Location', 'best');

% Write the final equation on the plot
% Fixed variable name from b_fixo to b_calibrated
equation = sprintf('LDR Sensor Equation:\nlog_{10}(R) = %.4f * log_{10}(LUX) + %.4f\nR^2 = %.4f', m_calc, b_calibrated, R2);
annotation('textbox', [0.15, 0.15, 0.4, 0.1], 'String', equation, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white', 'EdgeColor', 'black');