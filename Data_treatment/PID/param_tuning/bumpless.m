%% SCDTR Project - Bumpless Transfer Analysis (Enhanced Legend)
% This script visualizes the transition with all labels inside the legend box.
clear all; close all; clc;

% --- 1. DATA LOADING ---
filename = 'bumpless.txt'; 
if ~exist(filename, 'file'), error('File %s not found.', filename); end

fid = fopen(filename, 'r');
data = [];
while ~feof(fid)
    line = fgetl(fid);
    if startsWith(line, 's b')
        parts = strsplit(strtrim(line));
        if length(parts) == 7
            data = [data; str2double(parts(4:7))]; 
        end
    end
end
fclose(fid);

% --- 2. DATA EXTRACTION ---
y_lux  = data(:,1); 
u_duty = data(:,2); 
r_ref  = data(:,3); 
t_ms   = data(:,4); 
time_s = (t_ms - t_ms(1)) / 1000; 

% --- 3. EVENT DETECTION ---
idx_bump = find(u_duty > 0.5 & time_s < 10, 1); 
if isempty(idx_bump), idx_bump = find(diff(u_duty) > 0.8, 1) + 1; end
t_bump = time_s(idx_bump);

idx_bumpless = find(r_ref < 30, 1);
t_bumpless = time_s(idx_bumpless);

% --- 4. PROFESSIONAL PLOTTING ---
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 9, 7]);

% --- TOP PLOT: SYSTEM OUTPUT ---
subplot(2,1,1);
hold on; grid on; box on;

% Plot data and capture handles
h_ref = plot(time_s, r_ref, 'k--', 'LineWidth', 2.5, 'DisplayName', 'Setpoint');
h_lux = plot(time_s, y_lux, 'Color', [0, 0.447, 0.741], 'LineWidth', 1.8, 'DisplayName', 'Measured Lux');

% Plot vertical lines (without internal labels) and capture handles
if ~isempty(t_bump)
    h_v1 = xline(t_bump, 'r:', 'LineWidth', 2.0, 'DisplayName', 'Manual Step');
end
if ~isempty(t_bumpless)
    h_v2 = xline(t_bumpless, 'g:', 'LineWidth', 2.0, 'DisplayName', 'PID Activation');
end

ylabel('Illuminance [Lux]', 'FontWeight', 'bold');
title('System Response: Transition to Automatic Control', 'FontSize', 13);

% Combine handles for the legend box
legend_handles = [h_ref, h_lux];
if exist('h_v1','var'), legend_handles = [legend_handles, h_v1]; end
if exist('h_v2','var'), legend_handles = [legend_handles, h_v2]; end
legend(legend_handles, 'Location', 'best', 'FontSize', 9);

set(gca, 'FontSize', 10);

% --- BOTTOM PLOT: CONTROL EFFORT ---
subplot(2,1,2);
hold on; grid on; box on;

h_u = plot(time_s, u_duty, 'Color', [0.85, 0.325, 0.098], 'LineWidth', 2.2, 'DisplayName', 'Control Signal');

% Visual Continuity Highlight
if ~isempty(t_bumpless)
    plot(t_bumpless, u_duty(idx_bumpless), 'ko', 'MarkerSize', 8, 'LineWidth', 2, 'HandleVisibility', 'off');
    text(t_bumpless + 0.5, 0.85, '\leftarrow Smooth PID Entry', 'Color', [0, 0.5, 0], 'FontWeight', 'bold');
end

% Match vertical lines for timing
if ~isempty(t_bump), xline(t_bump, 'r:', 'LineWidth', 1.5, 'HandleVisibility', 'off'); end
if ~isempty(t_bumpless), xline(t_bumpless, 'g:', 'LineWidth', 1.5, 'HandleVisibility', 'off'); end

xlabel('Time [seconds]', 'FontWeight', 'bold');
ylabel('Duty Cycle [0-1]', 'FontWeight', 'bold');
title('Control Signal Analysis (Bumpless Validation)', 'FontSize', 13);
ylim([-0.1, 1.1]);
legend('Location', 'best');

set(gca, 'FontSize', 10);

fprintf('Plot with integrated legend box generated.\n');