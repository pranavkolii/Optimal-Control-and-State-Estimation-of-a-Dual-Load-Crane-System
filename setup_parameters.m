clear; clc; close all;

% System Physical Parameters
M_cart   = 1000;    % Mass of the cart (kg)
m_load1  = 100;     % Mass of load 1 (kg)
m_load2  = 100;     % Mass of load 2 (kg)
L_cable1 = 20;      % Length of cable 1 (m)
L_cable2 = 10;      % Length of cable 2 (m)
g        = 9.81;    % Gravity (m/s^2)

% State Space Matrices (Linearized)
% State Vector X = [x; x_dot; theta1; theta1_dot; theta2; theta2_dot]

% Pre-calculate denominators for readability
den_M  = M_cart;
den_L1 = M_cart * L_cable1;
den_L2 = M_cart * L_cable2;

% Matrix A (System Dynamics)
A_sys = [
    0, 1, 0, 0, 0, 0;
    0, 0, -(m_load1*g)/den_M, 0, -(m_load2*g)/den_M, 0;
    0, 0, 0, 1, 0, 0;
    0, 0, -((M_cart+m_load1)*g)/den_L1, 0, -(m_load2*g)/den_L1, 0;
    0, 0, 0, 0, 0, 1;
    0, 0, -(m_load1*g)/den_L2, 0, -((M_cart+m_load2)*g)/den_L2, 0
];

% Matrix B (Input Matrix)
B_sys = [
    0;
    1/den_M;
    0;
    1/den_L1;
    0;
    1/den_L2
];

% Matrix C (Default Output - Cart Position)
C_sys = [1, 0, 0, 0, 0, 0]; 

disp('System parameters loaded and matrices A, B initialized.');