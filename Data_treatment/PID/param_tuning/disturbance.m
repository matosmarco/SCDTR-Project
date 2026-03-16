%% SCDTR Project - Disturbance Rejection Analysis (ID 0 vs ID 1)
% ID 0 (pid_data.txt): Active PID Controller (The "Victim")
% ID 1 (peturb.txt): Manual Input Disturber (The "Aggressor")
% Format: s b <id> <y> <u> <ref> <time_ms>

clear all; close all; clc;

% --- PID PARAMETERS (Update manually for your report) ---
Kp = 0.05; Ki = 0.06; b = 0.8; 

% --- DATA LOADING ---
fprintf('Loading log files...\n');

% Load Lamp 0 (The controlled system)
%data0_raw = read_lamp_log('pid_data.txt', 0);
data0_raw = read_lamp_log('pid_video.txt', 0);
% Load Lamp 1 (The external disturbance)
%data1_raw = read_lamp_log('peturb.txt', 1);
data1_raw = read_lamp_log('peturb_video.txt', 0);
% --- TIME SYNCHRONIZATION ---
% Synchronize T=0 to the first timestamp of the main PID file
t_start = data0_raw(1,4); 

t0 = (data0_raw(:,4) - t_start) / 1000;
y0 = data0_raw(:,1);
u0 = data0_raw(:,2);
r0 = data0_raw(:,3);

t1 = (data1_raw(:,4) - t_start) / 1000;
y1 = data1_raw(:,1);
u1 = data1_raw(:,2);

% --- PROFESSIONAL PLOTTING ---
fig = figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 8]);

% TOP PLOT: Illuminance Analysis
subplot(2,1,1);
hold on; grid on; box on;
p1 = plot(t0, y0, 'Color', [0, 0.4470, 0.7410], 'LineWidth', 2, 'DisplayName', 'Measured Lux (y_0)');
p2 = plot(t0, r0, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Setpoint (r_0)');
p3 = plot(t1, y1, 'Color','r', 'LineStyle', '-.', 'LineWidth', 1.3, 'DisplayName', 'Disturbance Lux (Lamp 1)');

ylabel('Illuminance [Lux]', 'FontSize', 12, 'FontWeight', 'bold');
title(['Disturbance Rejection: Lamp 0 Response'], 'FontSize', 14);
legend('Location', 'northeast', 'FontSize', 10);
set(gca, 'FontSize', 11, 'LineWidth', 1.1);
hold off;

% BOTTOM PLOT: Control Effort (u)
subplot(2,1,2);
hold on; grid on; box on;
p4 = plot(t0, u0, 'Color', [0.8500, 0.3250, 0.0980], 'LineWidth', 1.8, 'DisplayName', 'PID Action (u_0)');
p5 = plot(t1, u1, 'Color', [0.4940, 0.1840, 0.5560], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Disturber Input (u_1)');

xlabel('Time [seconds]', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Control Signal [u]', 'FontSize', 12, 'FontWeight', 'bold');
title('Control Effort: Actuator Compensation', 'FontSize', 14);
ylim([-0.05, 1.05]);
legend('Location', 'northeast', 'FontSize', 10);
set(gca, 'FontSize', 11, 'LineWidth', 1.1);
hold off;

% --- PERFORMANCE METRICS CALCULATION ---
v_error = max(0, r0 - y0);
avg_V = mean(v_error);
max_dev = max(abs(y0 - r0));

fprintf('\n--- Performance Summary (English) ---\n');
fprintf('Average Visibility Error (V): %.4f Lux\n', avg_V);
fprintf('Maximum Deviation from Setpoint: %.4f Lux\n', max_dev);
fprintf('Total Experiment Duration: %.2f seconds\n', t0(end));

% --- HELPER FUNCTION: DATA PARSING ---
function data = read_lamp_log(filename, target_id)
    fid = fopen(filename, 'r');
    if fid == -1, error('File %s not found.', filename); end
    data = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, 's b')
            parts = strsplit(strtrim(line));
            % Format: s b <id> <y> <u> <ref> <time_ms> (7 columns)
            if length(parts) == 7
                vals = str2double(parts(3:7)); % id, y, u, r, t
                if vals(1) == target_id
                    data = [data; vals(2:end)]; % y, u, r, t
                end
            end
        end
    end
    fclose(fid);
    if isempty(data), warning('No data found for ID %d in %s', target_id, filename); end
end