clear all; close all; clc;

% PARAMETERS
%filename = 'results_kp005_ki01_b09_ff_on.txt';
filename = 'metrics.txt';
P_max = 0.1963;               % Max power
rho = 1.0;                 % Performance index weight

% LOAD DATA
data = read_lamp_log(filename, 0); % Loading ID 0
y = data(:,1); u = data(:,2); r = data(:,3); t_ms = data(:,4);
time_s = (t_ms - t_ms(1)) / 1000;
dt = [0; diff(time_s)]; % Delta time between samples

% ENERGY CONSUMPTION (E)
% Formula: E = P_max * sum( d_k * (t_k - t_k-1) )
E_instant = P_max * (u .* dt);
E_total = sum(E_instant);

% AVERAGE VISIBILITY ERROR (V) 
% Formula: V = (1/N) * sum( max(0, L_k - l_k) )
V_instant = max(0, r - y);
V_avg = mean(V_instant);

% AVERAGE FLICKER (F)
% Logic: Only counts when the control signal CHANGES DIRECTION
% Exclusion: Exclude transients (e.g., 2 seconds after reference change)
f = zeros(size(u));
ref_changed = [0; abs(diff(r)) > 0.5];
time_since_change = 0;

for k = 3:length(u)
    % Exclusion logic (skip if reference changed recently)
    if ref_changed(k), time_since_change = 0; end
    time_since_change = time_since_change + dt(k);
    
    if time_since_change > 2.0 % Skip 2 seconds of transient
        % Flicker condition: (d_k - d_k-1) * (d_k-1 - d_k-2) < 0
        diff1 = u(k) - u(k-1);
        diff2 = u(k-1) - u(k-2);
        
        if (diff1 * diff2) < 0
            % Formula: f_k = (|d_k - d_k-1| + |d_k-1 - d_k-2|) / (t_k - t_k-2)
            f(k) = (abs(diff1) + abs(diff2)) / (time_s(k) - time_s(k-2));
        end
    end
end
F_avg = mean(f);

% CUMULATIVE METRICS FOR PLOTTING
cum_V = cumsum(V_instant) ./ (1:length(V_instant))';
cum_E = cumsum(E_instant); % Energy is total accumulation
cum_F = cumsum(f) ./ (1:length(f))';

% PLOTTING
figure('Color', 'w', 'Units', 'inches', 'Position', [1, 1, 10, 10]);

subplot(3,1,1);
plot(time_s, cum_V, 'b', 'LineWidth', 2);
title('Average Visibility Error (V)'); ylabel('Lux'); grid on;
xlabel('Time [seconds]'); grid on;

subplot(3,1,2);
plot(time_s, cum_E, 'r', 'LineWidth', 2);
title('Total Energy Consumption (E)'); ylabel('Joules [J]'); grid on;
xlabel('Time [seconds]'); grid on;

subplot(3,1,3);
plot(time_s, cum_F, 'm', 'LineWidth', 2);
title('Average Flicker Error (F)'); ylabel('s^{-1}'); 
xlabel('Time [seconds]'); grid on;

% FINAL REPORT SUMMARY
fprintf('\n--- OFFICIAL PERFORMANCE METRICS ---\n');
fprintf('1. Energy Consumed (E):    %.2f Joules\n', E_total);
fprintf('2. Avg Visibility Error (V): %.3f Lux\n', V_avg);
fprintf('3. Avg Flicker Error (F):   %.4f s^-1\n', F_avg);
fprintf('------------------------------------\n');

function data = read_lamp_log(filename, target_id)
    fid = fopen(filename, 'r'); data = [];
    while ~feof(fid)
        line = fgetl(fid);
        if startsWith(line, 's b')
            p = strsplit(strtrim(line));
            if length(p) == 7
                vals = str2double(p(3:7));
                if vals(1) == target_id, data = [data; vals(2:end)]; end
            end
        end
    end
    fclose(fid);
end