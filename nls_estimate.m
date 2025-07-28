function x0_est = nls_estimate(z, sensor_pos, v, times, max_iter)
    x0_est = [120; 10]; % rastgele başlangıç tahmini

    for iter = 1:max_iter
        r = [];
        J = [];

        for i = 1:length(z)
            t = times(i);
            s = sensor_pos(i, :); % sensör konumu

            xk = x0_est + t * v; % tahmin edilen konum
            dx = xk(1) - s(1);
            dy = xk(2) - s(2);
            h = atan2(dy, dx);   % ölçüm modeli

            r(end+1, 1) = z(i) - h;

            denom = dx^2 + dy^2; % H_i
            dh_dx0 = -dy / denom;
            dh_dy0 = dx / denom;

            J(end+1, :) = [dh_dx0, dh_dy0]; 

        end

        delta = (J' * J) \ (J' * r);
        x0_est = x0_est + delta;

        if norm(delta) < 1e-6
            
            break;
        end
    end
end