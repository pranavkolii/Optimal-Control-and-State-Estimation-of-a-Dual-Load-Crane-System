% Part 1: LQR Controller Design and Simulation

% Load Parameters
run('setup_parameters.m');

% LQR Design
% Weighting Matrices
% High penalty on angles (1000) and position (100)
Q_lqr = diag([100, 10, 1000, 1, 1000, 1]); 
R_lqr = 0.01; % Low cost on control effort

% Compute Optimal Gain K
[K_gain, S_sol, E_poles] = lqr(A_sys, B_sys, Q_lqr, R_lqr);

disp('LQR Gain Matrix K:');
disp(K_gain);

% Stability Analysis
cl_eigenvalues = eig(A_sys - B_sys * K_gain);
disp('Closed-Loop Eigenvalues:');
disp(cl_eigenvalues);

if all(real(cl_eigenvalues) < -1e-6)
    disp('Stability Check: PASSED (Asymptotically Stable)');
else
    disp('Stability Check: FAILED');
end

% Simulation: Linear vs Nonlinear
% Initial Conditions: Small angles for linear comparison
% x=0, theta1=5 deg, theta2=10 deg
X0 = [0; 0; deg2rad(5); 0; deg2rad(10); 0];
t_span = [0 40];

% Linear Simulation
sys_closed_loop = ss(A_sys - B_sys*K_gain, [], eye(6), []);
[y_lin, t_lin, x_lin] = initial(sys_closed_loop, X0, t_span(end));

% Nonlinear Simulation
% Define control law u = -K*x inside the dynamics function handle
ode_func = @(t, x) nonlinear_crane_dynamics(t, x, M_cart, m_load1, m_load2, L_cable1, L_cable2, g, -K_gain * x);
[t_nonlin, x_nonlin] = ode45(ode_func, t_span, X0);

% Plotting Results
figure('Name', 'LQR Comparison');

subplot(2,1,1);
plot(t_lin, x_lin(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t_nonlin, x_nonlin(:,1), 'r', 'LineWidth', 1);
ylabel('Position (m)'); legend('Linear Model', 'Nonlinear Plant');
title('Linear vs Nonlinear Comparison'); grid on;

subplot(2,1,2);
plot(t_lin, rad2deg(x_lin(:,3)), 'b--', 'LineWidth', 1.5); hold on;
plot(t_lin, rad2deg(x_lin(:,5)), 'k--', 'LineWidth', 1.5);
plot(t_nonlin, rad2deg(x_nonlin(:,3)), 'r', 'LineWidth', 1);
plot(t_nonlin, rad2deg(x_nonlin(:,5)), 'g', 'LineWidth', 1);
ylabel('Angles (deg)'); legend('Lin \theta_1', 'Lin \theta_2', 'Nonlin \theta_1', 'Nonlin \theta_2');
xlabel('Time (s)'); grid on;