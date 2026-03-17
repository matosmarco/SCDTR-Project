% Clear workspace and close all figures
clear; clc; close all;

% SYSTEM PARAMETERS
K_gain = 40.11;     % Experimental gain computed in previous steps
T = 0.01;           % Sampling period (10ms)

% Typical Datasheet values for reference (from class slides)
tau_rise_ds = 0.020; 
tau_decay_ds = 0.030;

% LOAD EXPERIMENTAL DATA
% Loading the .mat file containing 'data_up' and 'data_down'
load("time_constants.mat")

datasets = {data_up, data_down};
labels = {'Rise (Step Up)', 'Decay (Step Down)'};
taus = zeros(1,2);

% Figure setup for the report
figure('Name', 'System Identification: LDR Dynamics', 'Units', 'normalized', 'Position', [0.1 0.1 0.8 0.7], 'Color', 'w');

% PROCESSING AND SIMULATION LOOP
for k = 1:2
    t_exp = datasets{k}(:,1);
    lux_exp = datasets{k}(:,2);
    
    % Analytical Calculation of Tau (63.2%, 5*tau)
    y_init = mean(lux_exp(1:3));                % Initial steady state
    y_final = mean(lux_exp(end-2:end));         % Final steady state
    delta_y = y_final - y_init;
    target_63 = y_init + (0.632 * delta_y);      % 63.2% threshold
    
    if delta_y > 0 % Rise case
        idx_tau = find(lux_exp >= target_63, 1, 'first');
        u_step = [zeros(1,1); ones(length(t_exp)-1, 1)]; % Input transition 0 to 1
    else % Decay case
        idx_tau = find(lux_exp <= target_63, 1, 'first');
        u_step = [ones(1,1); zeros(length(t_exp)-1, 1)]; % Input transition 1 to 0
    end
    
    % Extract experimental tau
    tau_calc = t_exp(idx_tau) - t_exp(1);
    taus(k) = tau_calc;
    
    % Theoretical Simulation (Euler Method)
    lux_sim = zeros(size(t_exp));
    lux_sim(1) = y_init;
    for i = 2:length(t_exp)
        % First-order model: dy/dt = (y_steady_state - y) / tau
        y_ss = y_init + (y_final - y_init) * u_step(i);
        if delta_y < 0
            y_ss = y_init + (y_final - y_init) * (1 - u_step(i));
        end
        
        % Numerical integration using Euler
        lux_sim(i) = lux_sim(i-1) + T * ((y_ss - lux_sim(i-1)) / tau_calc);
    end

    % PLOT

    % Top Row: Input Signal (Step)
    subplot(2, 2, k);
    stairs(t_exp, u_step, 'Color', [0 0.447 0.741], 'LineWidth', 1.5);
    title(['Input: ' labels{k}]);
    ylabel('Step Input u(t)');
    xlabel('Time [s]')
    grid on; ylim([-0.1 1.1]);
    
    % Bottom Row: Output Comparison (Experimental vs. Theoretical)
    subplot(2, 2, k + 2);

    plot(t_exp, lux_exp, 'k.', 'MarkerSize', 10, 'DisplayName', 'Experimental Data');
    hold on
    
    plot(t_exp, lux_sim, 'r-', 'LineWidth', 2, 'DisplayName', 'Theoretical Model');
    
    xline(t_exp(idx_tau), '--b', ...
        'DisplayName', ['\tau = ' num2str(tau_calc*1000,'%.1f') ' ms'], ...
        'LineWidth', 1.2);
    
    yline(target_63, ':g', ...
        'DisplayName', '63.2% level', ...
        'Alpha', 0.5, 'LineWidth', 2.5);
    
    title(['Output: ' labels{k}]);
    xlabel('Time [s]');
    ylabel('Illuminance [LUX]');
    legend('Location','southeast');
    grid on
end

% 5. CONSOLE OUTPUT
fprintf('\n========================================\n');
fprintf('         IDENTIFICATION SUMMARY            ');
fprintf('\n========================================\n');
fprintf('Rise Tau  | Exp: %.1f ms | Datasheet Ref: %.1f ms\n', taus(1)*1000, tau_rise_ds*1000);
fprintf('Decay Tau | Exp: %.1f ms | Datasheet Ref: %.1f ms\n', taus(2)*1000, tau_decay_ds*1000);

% 6. EMPIRICAL MODEL IDENTIFICATION (Corrected)
% Formula: tau(x) = tau_max * exp(-k*x)

% 1. Identify coordinates for the two points
% Point 1 (Rise): tau1 at x_high
% Point 2 (Decay): tau2 at x_low
x1 = mean(datasets{1}(end-2:end, 2)); % Approx. 40 LUX
tau1 = taus(1); 
x2 = mean(datasets{2}(end-2:end, 2)); % Approx. 0 LUX
tau2 = taus(2);

% 2. Calculate k using linearization
% k = ln(tau1 / tau2) / (x2 - x1)
k_exp = log(tau1 / tau2) / (x2 - x1);

% 3. Calculate tau_max (The intercept at x = 0)
% From the formula: tau_max = tau / exp(-k * x)
tau_max_exp = tau1 / exp(-k_exp * x1);

% --- Final Results Output ---
fprintf('\n========================================\n');
fprintf('      NON-LINEAR MODEL PARAMETERS\n');
fprintf('========================================\n');
fprintf('Point 1 (Rise):  %.1f ms at %.2f LUX\n', tau1*1000, x1);
fprintf('Point 2 (Decay): %.1f ms at %.2f LUX\n', tau2*1000, x2);
fprintf('----------------------------------------\n');
fprintf('Identified k:       %.6f\n', k_exp);
fprintf('Identified tau_max: %.1f ms\n', tau_max_exp*1000); % At x = 0 (0 lux)
% This value is bigger that the theoretical, since the RC filter dominates
% the delay
fprintf('Empirical Formula:  tau(x) = %.3f * exp(-%.4f * x)\n', tau_max_exp, k_exp);
fprintf('========================================\n');