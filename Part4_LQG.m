% Part 4: LQG Controller (Nonlinear Plant + Linear Observer + Integrator)

clear; clc; close all;
run('setup_parameters.m');

% LQR Setup
Q_lqr = diag([100, 10, 1000, 1, 1000, 1]); 
R_lqr = 0.01;
[~, ~, E_poles] = lqr(A_sys, B_sys, Q_lqr, R_lqr);

% Observer Setup
observer_poles = E_poles * 4; 
C_best = [1 0 0 0 0 0]; % Measuring x only
L_obs = place(A_sys', C_best', observer_poles)';

% LQR
% New State Vector Z = [x_sys; xi] where xi_dot = Ref - x
A_aug = [A_sys, zeros(6,1); -C_best, 0]; 
B_aug = [B_sys; 0];
Q_aug = diag([100, 10, 1000, 1, 1000, 1, 5000]); % High penalty on integral error
K_aug = lqr(A_aug, B_aug, Q_aug, R_lqr);

K_state = K_aug(1:6); % Feedback gains
K_int = K_aug(7);     % Integral gain

% Simulation
t_sim = 0:0.01:30;
ref_val = 5;          % Target: 5 meters
dist_force = 20;      % Disturbance: 20 N

% Initial Conditions [6 plant; 6 observer; 1 integrator]
init_cond = zeros(13, 1); 

[t_lqg, z_hist] = ode45(@(t, z) lqg_dynamics_wrapper(t, z, M_cart, m_load1, m_load2, L_cable1, L_cable2, g, A_sys, B_sys, C_best, K_state, K_int, L_obs, ref_val, dist_force), t_sim, init_cond);

% Plotting
figure('Name', 'LQG Performance');

subplot(2,1,1);
plot(t_lqg, z_hist(:,1), 'b', 'LineWidth', 2); hold on; % True Position
plot(t_lqg, z_hist(:,7), 'g--', 'LineWidth', 1);        % Est Position
yline(ref_val, 'r--', 'Reference');
ylabel('Position (m)'); title(['LQG Tracking (Ref=5m) with Disturbance (F=20N)']);
legend('True Pos', 'Est Pos', 'Ref'); grid on;

subplot(2,1,2);
plot(t_lqg, rad2deg(z_hist(:,3)), 'r', 'LineWidth', 1.5); hold on;
plot(t_lqg, rad2deg(z_hist(:,5)), 'b', 'LineWidth', 1.5);
ylabel('Swing Angles (deg)'); legend('\theta_1', '\theta_2');
xlabel('Time (s)'); grid on;