function [total_rmse, X_est_path, sensors_used_log] = run_ekf_simulation( ...
    x0_est, v, T, dt, x0_full, F, Q, sensor_pos, sigma_rad, radar_fov, plot_results_bool)

% Inputs:
%   x0_est: NLS ile tahmin edilen ilk konum
%   v: Geminin hızı
%   T: Simülasyonun toplam zaman adımı sayısı.
%   dt
%   x0_full: Grountruth initial state
%   F: State Transition Matrix
%   Q: Process Noise Covariance Matrix
%   sensor_pos: Sensor coordinates
%   sigma_rad: Gürültünün standart sapması
%   radar_fov: Radar derecesi
%   plot_results_bool: Grafik çizme işlemi
%
% Çıkışlar:
%   total_rmse: RMSE
%   X_est_path: Zamana göre EKF ile tahmin edilen durum vektörleri
%   sensors_used_log: Her i zamanda kullanılan sensör

    % Sabitler
    
    R = sigma_rad^2;
    num_sensors = size(sensor_pos, 1);

    % Gerçek Yol Hesaplama
    X_true = zeros(4, T + 1);
    X_true(:,1) = x0_full;
    for t = 2:T+1
        X_true(:,t) = F * X_true(:,t-1);
    end

    % EKF Takip
    X_est_path = zeros(4, T+1);
    x_est = [x0_est; v];
    P_est = eye(4);
    X_est_path(:,1) = x_est;

    sensors_used_log = zeros(T,1);

    for t = 1:T
        % Predict
        x_pred = F * x_est;
        P_pred = F * P_est * F' + Q;

        % Radar angle
        radar_angle = mod(radar_fov * (t-1), 360);
        ship_pos_true = X_true(1:2, t);
        radar_start = wrapTo180(radar_angle - radar_fov/2);
        radar_end = wrapTo180(radar_angle + radar_fov/2);

        
        current_sensor_idx = 0;
        for i = 1:num_sensors
            dx_s = sensor_pos(i,1) - ship_pos_true(1);
            dy_s = sensor_pos(i,2) - ship_pos_true(2);
            sensor_angle = wrapTo180(rad2deg(atan2(dy_s, dx_s)));           
            
            if (sensor_angle >= radar_start) && (sensor_angle <= radar_end)
                current_sensor_idx = i;                 
                break;
            end
        end
        sensors_used_log(t) = current_sensor_idx;

        idx = current_sensor_idx;
        if idx ~= 0 && rand() < 0.9
            dx_meas = X_true(1,t) - sensor_pos(idx,1);
            dy_meas = X_true(2,t) - sensor_pos(idx,2);
            z = atan2(dy_meas, dx_meas) + randn * sigma_rad;

            dx_pred = x_pred(1) - sensor_pos(idx,1);
            dy_pred = x_pred(2) - sensor_pos(idx,2);
            dist2 = dx_pred^2 + dy_pred^2;
            z_hat = atan2(dy_pred, dx_pred);

            H = [-dy_pred/dist2, dx_pred/dist2, 0, 0];
            S = H * P_pred * H' + R;
            K = P_pred * H' / S;
            residual = wrapToPi(z - z_hat);

            x_pred = x_pred + K * residual;
            P_pred = (eye(4) - K * H) * P_pred;
        end

        x_est = x_pred;
        P_est = P_pred;
        X_est_path(:,t+1) = x_est;
    end

    % RMSE 
    err = X_est_path(1:2,1:T) - X_true(1:2,1:T);
    total_rmse = sqrt(mean(sum(err.^2,1)));
    
   
    if plot_results_bool
        figure; hold on;
        plot(X_true(1,:), X_true(2,:), 'g-', 'LineWidth', 2);
        plot(X_est_path(1,:), X_est_path(2,:), 'b--o', 'MarkerSize', 4);
        plot(sensor_pos(:,1), sensor_pos(:,2), 'r o', 'MarkerSize', 8);
        legend('Real Path', 'EKF Estimation', 'Sensors');
        xlabel('x(m)'); ylabel('y(m)');
        axis equal; grid on;
        title(['Ship Movement Estimation With EKF (Sigma = ', num2str(rad2deg(sigma_rad)), ')']);
        hold off;

        % RMSE per time
        rmse_per_t_plot = sqrt(err(1,:).^2 + err(2,:).^2);
        figure;
        plot(1:T, rmse_per_t_plot, 'k-', 'LineWidth', 1.5);
        xlabel('Time(t)');
        ylabel('RMSE of position');
        title(['RMSE Position - Time for (Sigma = ', num2str(rad2deg(sigma_rad)),')']);
        grid on;

        figure;
        bar(1:T,sensors_used_log);
        xlabel('Time(t)');
        ylabel("Sensor number");
        title("Sensors used per time");
        grid on;
    end
end
