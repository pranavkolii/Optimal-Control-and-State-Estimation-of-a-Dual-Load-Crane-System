function dzdt = lqg_dynamics_wrapper(t, z, M, m1, m2, l1, l2, g, A, B, C, K_x, K_i, L, ref, dist)
    % LQG_DYNAMICS_WRAPPER
    % Simulates the system: Nonlinear Plant + Linear Observer + Integrator
    
    % Unpack State
    x_plant = z(1:6);    % True nonlinear plant state
    x_hat = z(7:12);     % Observer state estimate
    xi_err = z(13);      % Integral error state
    
    % Measurements
    y_meas = C * x_plant;  % Sensor measurement
    y_hat = C * x_hat;     % Observer prediction
    
    % Control Law (Output Feedback with Integral Action)
    u_ctrl = -K_x * x_hat + K_i * xi_err; 
    
    % Plant Dynamics (Nonlinear Physics + Disturbance)
    dx_plant = nonlinear_crane_dynamics(t, x_plant, M, m1, m2, l1, l2, g, u_ctrl + dist);
    
    % Observer Dynamics (Linear Estimator)
    dx_hat = A * x_hat + B * u_ctrl + L * (y_meas - y_hat);
    
    % Integrator Dynamics (Error accumulation)
    d_xi = ref - y_meas(1);
    
    % Return full derivative vector
    dzdt = [dx_plant; dx_hat; d_xi];
end