%% Part 3: Luenberger Observer Design (Linear & Nonlinear Simulation)
% Simulates Observer performance for C1, C3, and C4 output cases.

clear; clc; close all;
run('setup_parameters.m'); 

% Controller Setup (LQR)
Q_lqr = diag([100, 10, 1000, 1, 1000, 1]); 
R_lqr = 0.01;
K = lqr(A_sys, B_sys, Q_lqr, R_lqr);

% Define Cases
% Case 1: y = x
C1 = [1 0 0 0 0 0];
% Case 3: y = [x, theta2]
C3 = [1 0 0 0 0 0; 0 0 0 0 1 0]; 
% Case 4: y = [x, theta1, theta2]
C4 = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0];

cases = {C1, 'C1 (y=x)'; C3, 'C3 (y=x, th2)'; C4, 'C4 (y=x, th1, th2)'};

% Simulation Settings
% High Initial Conditions for Observer test
x0_sys = [0; 0; deg2rad(30); 0; deg2rad(60); 0]; 
x0_est = zeros(6,1); 
x0_aug = [x0_sys; x0_est]; 
t_span = [0 450]; 

% Main Loop Through Cases
for i = 1:size(cases, 1)
    C_curr = cases{i, 1};
    label = cases{i, 2};
    
    fprintf('\nSimulating Case: %s\n', label);
    
    % Design Observer Gain L (Poles 3x faster than system)
    sys_poles = eig(A_sys - B_sys*K);
    obs_poles = real(sys_poles)*3 - 1i*imag(sys_poles);
    L = place(A_sys', C_curr', obs_poles)';
    
    % LINEAR SIMULATION
    A_lin = [A_sys - B_sys*K, B_sys*K; zeros(6,6), A_sys - L*C_curr];
    B_lin = [B_sys; zeros(6,1)];
    C_lin = [C_curr, zeros(size(C_curr))]; 
    sys_closed = ss(A_lin, B_lin, C_lin, 0);
    
    % Linear Initial Condition
    figure('Name', ['Linear ' label ' - IC']);
    initial(sys_closed, x0_aug, t_span(end));
    title(['Linear ' label ' - Response to Initial Conditions']); 
    
    % Linear Step Response
    figure('Name', ['Linear ' label ' - Step']);
    step(sys_closed, t_span(end));
    title(['Linear ' label ' - Step Response']); 

    % NONLINEAR SIMULATION
    % Nonlinear Initial Condition
    [t_nl, z_nl] = ode45(@(t, z) obs_dynamics_wrapper(t, z, M_cart, m_load1, m_load2, L_cable1, L_cable2, g, A_sys, B_sys, C_curr, K, L, 0), t_span, x0_aug);
    
    figure('Name', ['Nonlinear ' label ' - IC']);
    subplot(2,1,1); 
    plot(t_nl, z_nl(:,1), 'b', 'LineWidth', 1.5); hold on; 
    ylabel('Position (m)'); title(['Nonlinear ' label ' – Response to Initial Conditions']); grid on;
    subplot(2,1,2);
    plot(t_nl, rad2deg(z_nl(:,3)), 'r'); ylabel('\theta_1 (deg)'); grid on;

    % Nonlinear Step Response (Reference = 1m)
    [t_st, z_st] = ode45(@(t, z) obs_dynamics_wrapper(t, z, M_cart, m_load1, m_load2, L_cable1, L_cable2, g, A_sys, B_sys, C_curr, K, L, 1), t_span, zeros(12,1));
    
    figure('Name', ['Nonlinear ' label ' - Step']);
    subplot(2,1,1);
    plot(t_st, z_st(:,1), 'b', 'LineWidth', 1.5); hold on;
    yline(1, 'r--');
    ylabel('Position (m)'); title(['Nonlinear ' label ' – Step Response']); grid on;
    subplot(2,1,2);
    plot(t_st, rad2deg(z_st(:,3)), 'r'); ylabel('\theta_1 (deg)'); grid on;
end

% Helper
function dz = obs_dynamics_wrapper(t, z, M, m1, m2, l1, l2, g, A, B, C, K, L, ref_val)
    x_true = z(1:6);
    x_est = z(7:12);
    y = C * x_true;
    y_hat = C * x_est;
    u = -K * x_est + ref_val; % Control Law
    
    % Nonlinear Plant + Linear Observer
    dx_true = nonlinear_crane_dynamics(t, x_true, M, m1, m2, l1, l2, g, u);
    dx_est = A * x_est + B * u + L * (y - y_hat);
    
    dz = [dx_true; dx_est];
end