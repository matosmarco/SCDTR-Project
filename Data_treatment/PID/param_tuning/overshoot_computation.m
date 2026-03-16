%% SCDTR Project - PID Analysis with Visual Overshoot Detection
clear all; close all; clc;

% --- PID PARAMETERS ---
Kp = 0.05; Ki = 0.01; b = 1.0; % Ajusta conforme o teu teste

% --- FILE LOADING ---
filename = 'results_kp005_ki01_b05_ff_on.txt';
if ~exist(filename, 'file'), error('File not found.'); end

fid = fopen(filename, 'r');
data = [];
while ~feof(fid)
    line = fgetl(fid);
    if startsWith(line, 's b')
        parts = strsplit(line);
        if length(parts) == 7
            data = [data; str2double(parts(4:7))]; 
        end
    end
end
fclose(fid);

y_lux = data(:,1); u_duty = data(:,2); r_ref = data(:,3); t_ms = data(:,4);
time_s = (t_ms - t_ms(1)) / 1000;

% --- PRECISE OVERSHOOT CALCULATION ---
% 1. Find the first step in reference
step_idx = find(diff(r_ref) > 0.5, 1, 'first');

if ~isempty(step_idx)
    % 2. Define search window (from step until reference changes again or 5s pass)
    next_change = find(abs(diff(r_ref(step_idx+1:end))) > 0.5, 1, 'first');
    if isempty(next_change)
        search_limit = length(y_lux);
    else
        search_limit = step_idx + next_change;
    end
    
    % 3. Find peak and target value
    target_val = r_ref(step_idx + 10); % Value right after the step
    [peak_val, peak_loc_rel] = max(y_lux(step_idx:search_limit));
    peak_idx = step_idx + peak_loc_rel - 1;
    
    overshoot_lux = max(0, peak_val - target_val);
    overshoot_pct = (overshoot_lux / target_val) * 100;
else
    overshoot_lux = 0; overshoot_pct = 0; peak_idx = 1;
end

% --- PLOTTING ---
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 8]);

subplot(2,1,1);
hold on; grid on; box on;
plot(time_s, y_lux, 'b-', 'LineWidth', 2, 'DisplayName', 'Measured (y)');
plot(time_s, r_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Setpoint (r)');

% Marcar o Overshoot no gráfico com um círculo vermelho
if overshoot_lux > 0
    plot(time_s(peak_idx), y_lux(peak_idx), 'ro', 'MarkerSize', 10, 'LineWidth', 2, 'HandleVisibility', 'off');
    text(time_s(peak_idx), y_lux(peak_idx) + 1, ' Overshoot!', 'Color', 'r', 'FontWeight', 'bold');
end

ylabel('Illuminance [Lux]', 'FontSize', 12, 'FontWeight', 'bold');
title(['Overshoot Analysis (Kp=', num2str(Kp), ', Ki=', num2str(Ki), ')'], 'FontSize', 14);
legend('Location', 'best');

% Metrics Box
stats_str = {['Target: ', num2str(target_val, '%.1f'), ' Lux'], ...
             ['Peak: ', num2str(peak_val, '%.2f'), ' Lux'], ...
             ['Overshoot: ', num2str(overshoot_lux, '%.2f'), ' Lux (', num2str(overshoot_pct, '%.1f'), '%)']};
annotation('textbox', [0.15, 0.75, 0.25, 0.12], 'String', stats_str, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

subplot(2,1,2);
plot(time_s, u_duty, 'r-', 'LineWidth', 1.5);
ylabel('Control Effort [u]', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylim([-0.05, 1.05]);
grid on;

fprintf('Overshoot detected: %.2f Lux (%.1f%%)\n', overshoot_lux, overshoot_pct);