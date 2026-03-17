%% SCDTR Project - High-Resolution Control Loop Jitter Analysis
clear all; close all; clc;

% --- 1. DATA LOADING ---
filename = 'jitter.txt'; 
if ~exist(filename, 'file'), error('File %s not found.', filename); end

fid = fopen(filename, 'r');
data = [];
while ~feof(fid)
    line = fgetl(fid);
    if startsWith(line, 's b')
        parts = strsplit(strtrim(line));
        if length(parts) == 7
            % Extracting: Lux(4), Duty(5), Ref(6), Time_us(7)
            data = [data; str2double(parts(4:7))]; 
        end
    end
end
fclose(fid);

% --- 2. CALCULATE JITTER ---
t_us  = data(:,4); 
time_s = (t_us - t_us(1)) / 1000000; 
dt_ms = diff(t_us) / 1000; % Convert us difference to ms
t_plot = time_s(2:end); 

% Statistics
avg_dt = mean(dt_ms);
std_dt = std(dt_ms);
max_dev = max(abs(dt_ms - 10)); % Maximum deviation from 10ms

% --- 3. PROFESSIONAL PLOTTING ---
figure('Color', 'w', 'Units', 'inches', 'Position', [2, 2, 10, 5]);
hold on; grid on; box on;

% The Jitter Line (Blue)
plot(t_plot, dt_ms, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 0.8, 'DisplayName', 'Measured \DeltaT');

% Nominal Target Line (Red Dashed)
%yline(10, 'r--', 'Target: 10ms', 'LineWidth', 1.5, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');
yline(10, 'r--','LineWidth', 1.5, 'FontWeight', 'bold', 'LabelVerticalAlignment', 'bottom');

% --- 4. IMPROVED SCALE (Dynamic Zoom) ---
% We set the limits to show the noise clearly. 
% If your jitter is very small, this will zoom in. If there are spikes, it shows them.
y_padding = max(0.05, 3 * std_dt); % At least 0.05ms padding
ylim([10 - y_padding, 10 + y_padding]); 

% Labels and Title
xlabel('Time [s]', 'FontSize', 11, 'FontWeight', 'bold');
ylabel('Sampling Period \DeltaT [ms]', 'FontSize', 11, 'FontWeight', 'bold');
title('Control Loop Temporal Determinism (Microseconds Resolution)', 'FontSize', 13);

% --- 5. STATISTICS BOX (Moved to Bottom-Left Blank Zone) ---
stats_str = {['Mean \DeltaT: ', num2str(avg_dt, '%.4f'), ' ms'], ...
             ['Std Dev: ', num2str(std_dt, '%.4f'), ' ms'], ...
             ['Max Dev: ', num2str(max_dev, '%.3f'), ' ms'], ...
             ['Frequency: ', num2str(1000/avg_dt, '%.1f'), ' Hz']};

% Position: [Left, Bottom, Width, Height] - Adjusted to avoid data
annotation('textbox', [0.15, 0.15, 0.22, 0.18], 'String', stats_str, ...
    'FitBoxToText', 'on', 'BackgroundColor', [1 1 1 0.8], 'FontSize', 10, ...
    'EdgeColor', [0.7 0.7 0.7], 'LineWidth', 1);

set(gca, 'FontSize', 10, 'GridAlpha', 0.4);
hold off;

fprintf('Jitter Analysis Complete. Std Dev: %.4f ms\n', std_dt);