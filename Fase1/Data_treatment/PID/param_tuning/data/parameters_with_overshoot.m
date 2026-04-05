%% SCDTR Project - PID Performance Analysis (Noise-Robust Metrics)
clear all; close all; clc;

% --- PID PARAMETERS (Update manually) ---
Kp = 0.05; Ki = 0.06; Kd = 0.00; b = 0.5;

% --- FILE LOADING ---
filename = 'results_kp005_ki01_b07_ff_on.txt';
if ~exist(filename, 'file'), error('Error: File %s not found.', filename); end
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

% --- DATA EXTRACTION ---
y_lux  = data(:,1); 
u_duty = data(:,2); 
r_ref  = data(:,3); 
t_ms   = data(:,4); 
time_s = (t_ms - t_ms(1)) / 1000; 

% --- ROBUST TRANSIENT METRICS ---
% 1. Identify the last step change
step_indices = find(diff(r_ref) ~= 0);
if isempty(step_indices)
    idx_start = 1;
    initial_r = r_ref(1);
else
    idx_start = step_indices(end) + 1;
    initial_r = r_ref(step_indices(end));
end

t_step = time_s(idx_start);
target_r = r_ref(end);
y_segment = y_lux(idx_start:end);
t_segment = time_s(idx_start:end);

% 2. Noise reduction for metrics calculation
% We use a moving average to prevent noise spikes from being counted as overshoot
y_smooth = smoothdata(y_segment, 'movmean', 5); 

% 3. Overshoot Calculation (Relative to step direction)
overshoot_pct = 0;
noise_threshold = 0.01 * target_r; % 1% margin to ignore noise

if target_r > initial_r % Step Up
    peak_y = max(y_smooth);
    if peak_y > (target_r + noise_threshold)
        overshoot_pct = 100 * (peak_y - target_r) / abs(target_r - initial_r);
    end
elseif target_r < initial_r % Step Down
    peak_y = min(y_smooth);
    if peak_y < (target_r - noise_threshold)
        overshoot_pct = 100 * (target_r - peak_y) / abs(target_r - initial_r);
    end
end

% 4. Settling Time Calculation (2% error band)
settling_band = 0.02 * target_r;
% Find the last index where the smoothed signal is outside the 2% band
idx_outside = find(abs(y_smooth - target_r) > settling_band, 1, 'last');
if isempty(idx_outside) || idx_outside == length(y_smooth)
    settling_time = 0; % Already settled or never left
else
    settling_time = t_segment(idx_outside) - t_step;
end

% --- PLOTTING ---
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 8]);
subplot(2,1,1);
hold on; grid on; box on;
plot(time_s, y_lux, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Raw Data'); % Raw in gray
plot(time_s(idx_start:end), y_smooth, 'b-', 'LineWidth', 1.8, 'DisplayName', 'Filtered Signal');
plot(time_s, r_ref, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Reference');
title(['Robust Metrics Analysis (Kp=', num2str(Kp), ', Ki=', num2str(Ki), ')']);
ylabel('Illuminance [LUX]');
legend('Location', 'best');

% Metrics annotation
txt = {['Overshoot: ', num2str(overshoot_pct, '%.2f'), '%'], ...
       ['Settling Time: ', num2str(settling_time, '%.3f'), ' s']};
annotation('textbox', [0.15 0.75 0.2 0.1], 'String', txt, 'FitBoxToText', 'on', 'BackgroundColor', 'w');

subplot(2,1,2);
plot(time_s, u_duty, 'r-', 'LineWidth', 1.5);
ylabel('Control Signal [u]');
xlabel('Time [s]');
ylim([-0.05 1.05]);

fprintf('--- Noise-Robust Metrics ---\n');
fprintf('Overshoot: %.5f%%\n', overshoot_pct);
fprintf('Settling Time: %.5f s\n', settling_time);


overshoot_pct = 100 * (peak_y - target_r) / abs(target_r - initial_r);
% Real overshoot
disp(overshoot_pct)
