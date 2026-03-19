%% SCDTR Project - PID Stacked Performance Analysis
% This script reads the log file and plots Illuminance and Control Signal separately.
% Data format: s b <id> <y_k> <u_k> <ref_r> <dist_d> <time_ms>

clear all; close all; clc;

% --- PID PARAMETERS (Update manually for documentation) ---
Kp = 0.05; 
Ki = 0.00;
Kd = 0.00;
b  = 1.0;

% --- FILE LOADING ---
%filename = 'results_kp001.txt';
filename = 'kp005.txt';
%filename = 'results_kp01.txt';
%filename = 'results_kp005_ki001.txt';
%filename = 'results_kp005_ki005.txt';
%filename = 'results_kp005_ki01.txt';
%filename = 'results_kp005_ki01_ff_on.txt';
%filename = 'results_kp005_ki01_b05_ff_on.txt';
%filename = 'results_kp005_ki01_b09_ff_on.txt';
%filename = 'results_kp005_ki01_b07_ff_on.txt';
%filename = 'results_kp005_ki01_b07_ff_on_aoff.txt';
%filename = 'results_with_everything.txt';
%filename = 'disturbance_0.txt';
%filename = 'states.txt';
if ~exist(filename, 'file')
    error('Error: File %s not found.', filename);
end

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

if isempty(data)
    error('No valid data found.');
end

% --- DATA EXTRACTION ---
y_lux  = data(:,1); % Measured Illuminance 
u_duty = data(:,2); % Control Signal (Duty Cycle)
r_ref  = data(:,3); % Reference 
t_ms   = data(:,4); % Timestamp

% --- DATA NORMALIZATION ---
time_s = (t_ms - t_ms(1)) / 1000; % ms to seconds starting at 0s

% --- PLOTTING ---
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 10]);

% TOP PLOT: Illuminance Analysis (Lux)
subplot(2,1,1);
hold on; grid on; box on;
plot(time_s, y_lux, 'b-', 'LineWidth', 1.8);    % Measured Light (y)
plot(time_s, r_ref, 'k--', 'LineWidth', 1.5);   % Reference (r)
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Illuminance [LUX]', 'FontSize', 12, 'FontWeight', 'bold');
title(['PID Performance: Illuminance (Kp=', num2str(Kp), ', Ki=', num2str(Ki), ')'], 'FontSize', 14);
legend('Measured (y)', 'Reference (r)', 'Disturbance (d)', 'Location', 'best');
max_light = max([max(y_lux), max(r_ref)]);
ylim([0, max_light * 1.2]);
set(gca, 'FontSize', 10);
hold off;

% BOTTOM PLOT: Control Signal Analysis (u)
subplot(2,1,2);
hold on; grid on; box on;
plot(time_s, u_duty, 'r-', 'LineWidth', 1.5);   % Duty Cycle (u)
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Control Effort [Duty Cycle]', 'FontSize', 12, 'FontWeight', 'bold');
title(['Control Effort (b=', num2str(b), ')'], 'FontSize', 14);
ylim([-0.05, 1.05]); % Actuator limits
legend('Duty Cycle (u)', 'Location', 'best');
set(gca, 'FontSize', 10);
hold off;

% --- PERFORMANCE METRICS CALCULATION ---
% Instantaneous Visibility Error: V_k = max(0, r - y) [cite: 220]
v_error = max(0, r_ref - y_lux); 

% Cumulative Average Visibility Error [cite: 220]
avg_V = mean(v_error); 

% --- FIGURE 2: Visibility Error (Comfort Metric) ---
figure('Color', 'w', 'Name', 'Visibility Error Analysis');
hold on; grid on; box on;

% Plotting the instantaneous error
plot(time_s, v_error, 'm-', 'LineWidth', 1.5); 

% Plotting the average error line
yline(avg_V, 'r--', 'LineWidth', 1.5); 

% Defines a little offset (e.g.: 2% of the graph scale)
offset = max(ylim) * 0.03; 

text(time_s(end)*0.7, avg_V + offset, ...
    ['Average V = ', num2str(avg_V, '%.2f'), ' LUX'], ...
    'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10);
% Formatting
xlabel('Time [s]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Visibility Error [LUX]', 'FontSize', 12, 'FontWeight', 'bold');
title('Visibility Error Analysis', 'FontSize', 14);
legend('Instantaneous Error (V_k)', 'Mean Visibility Error (V)', 'Location', 'best');

% Dynamic scaling with safety margin
ylim([0, max(v_error) * 1.3 + 1]); 

hold off;

% Save metric figure
% print('visibility_error_analysis', '-dpng', '-r300');
fprintf('Analysis complete. Average Visibility Error (V): %.2f Lux\n', avg_V);