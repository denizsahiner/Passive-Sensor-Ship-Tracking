clear all; clc; close all;
rng_value=50;
rng(rng_value); 
%% 1. Model Derivation

dt = 1; 
T = 50; 

% Groundtruth Initial State Of Ship
v = [5; 3]; 
x0 = [100; 200]; 
x0_full = [x0; v]; 

% State Transition Matrix F
F = [1 0 dt 0;
     0 1 0 dt;
     0 0 1 0;
     0 0 0 1];

% Process Noise Covarianca Matrix
q_scalar = 1; 
Q = q_scalar * [dt^4/4, 0, dt^3/2, 0;
                0, dt^4/4, 0, dt^3/2;
                dt^3/2, 0, dt^2, 0;
                0, dt^3/2, 0, dt^2]; 

% Sensor Positions
sensor_pos = [0, 0; 500, 0; 250, 400]; 

% Radar
radar_fov = 60; 


% Noise
sigma_rad = deg2rad(0.5);



%% 2. Initial Nonlinear Least Squares Estimation
% First three Bearing measurements for NLS
z_init_nls = zeros(3,1);
times_nls = [0; 1; 2]; 
for k_nls = 0:2
    pos_true = x0 + k_nls * v; 
    s_nls = sensor_pos(k_nls+1,:); 
    dx_nls = pos_true(1) - s_nls(1);
    dy_nls = pos_true(2) - s_nls(2);
    z_init_nls(k_nls+1) = atan2(dy_nls, dx_nls) + randn * sigma_rad; % Gürültülü ölçüm
end

x0_est = nls_estimate(z_init_nls, sensor_pos(1:3,:), v, times_nls, 10); % NLS çağrısı

fprintf('Real inital position: [%.2f; %.2f]\n', x0(1), x0(2));
fprintf('Estimated inital position with NLS: [%.2f; %.2f]\n', x0_est(1), x0_est(2));
%% 4. EKF Implementation and Plots

rng(rng_value);
[total_rmse, X_est_path, sensors_used_log] = ekf_simulation( ...
    x0_est, v, T, dt, x0_full, F, Q, sensor_pos, sigma_rad, radar_fov, true);

fprintf('Total RMSE For EKF : %f m\n', total_rmse);


%% 5. Sensitivity Analysis

sigma_list = deg2rad([0.2 0.5 1 2 3]); % Standart deviations for testing
rmse_list = zeros(size(sigma_list));

fprintf("Sigma        RMSE\n");

for s_idx = 1:length(sigma_list)
    rng(rng_value);
    [rmse_list(s_idx), ~, ~] = ekf_simulation( ...
        x0_est, v, T, dt, x0_full, F, Q, sensor_pos, sigma_list(s_idx), radar_fov, false);
    
    fprintf("%f     %f\n",rad2deg(sigma_list(s_idx)), rmse_list(s_idx));
end


figure;
plot(rad2deg(sigma_list), rmse_list, 'ro-', 'LineWidth', 2);
xlabel('Standart Deviation of Noise');
ylabel('Total RMSE');
title('Sensitivity Analysis');
grid on;


