% Simulator system identification

% Initialization of the theoretical values
tau_u = 0.02; % theoretical value of tau rise
tau_d = 0.03; % theoretical value of tau delay
R = 10000; % resistance value (10 kOhm)
C = 0.00001; % 10 uF
Vcc = 3.3;
T = 0.001; 
step = 1000;
% Experimental values
b_calib = 5.22;
m_calib = -0.8492; 
g = -m_calib; % real gamma of the sensor (according to the calibration)
K_gain = 39.43; % in this experiment, it has some insignificant flutuations
bg_lux = 0.02; % real background lux (registered by the system)

R0 = 10^b_calib; % real value of resistance at 1 Lux


% Experimental data

% Load data from experiment
load("adc_voltage_exp.mat")

t_exp = data_exp(:,1); % time
v_exp = data_exp(:,2); % voltage

% Simulation
% Initial conditions
x = bg_lux; 
r(1) = R0*x^(-g); % R_LDR
v(1) = Vcc*R/(R+r(1)); % Init voltage


%Cycle with increments of 0.1 in the duty cycle (0.1 to 1.0)
for (loop = 0:9)
    u = (loop + 1) * 0.1;
    x = K_gain * u + bg_lux; % Lux alvo real do teu sistema
    
    for i = (loop*step + 2) : ((loop+1)*step + 1)
        r(i) = r(i-1) + T*(-r(i-1)/tau_u + R0*x^(-g)/tau_u);
        v(i) = v(i-1) + T*( (Vcc-v(i-1))/C/r(i-1) - v(i-1)/R/C );
    end
end


%cycle with decrements f 0.1 in the duty cycle (0.9 to 0.0)
for (loop = 10:19)
    u = 1.0 - ((loop - 9) * 0.1);
    x = K_gain * max(u, 0) + bg_lux;
    
    for i = (loop*step + 2) : ((loop+1)*step + 1)
        r(i) = r(i-1) + T*(-r(i-1)/tau_d + R0*x^(-g)/tau_d);
        v(i) = v(i-1) + T*( (Vcc-v(i-1))/C/r(i-1) - v(i-1)/R/C );
    end
end



%Plot comparison
t_sim = (0:length(v)-1) * T;

figure('Name', 'System Identification - Corrected', 'Color', 'w');
plot(t_exp, v_exp, 'k.', 'MarkerSize', 6); hold on;
plot(t_sim, v, 'r-', 'LineWidth', 2);

title('ADC Voltage - Simulator vs Real System');
xlabel('t [sec]'); ylabel('v [volt]');
legend('Experimental (Real Data)', 'Theoretical Model (Calibrated)', 'Location', 'best');
grid on;