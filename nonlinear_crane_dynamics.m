function dxdt = nonlinear_crane_dynamics(t, state, M, m1, m2, l1, l2, g, F_input)
    % NONLINEAR_CRANE_DYNAMICS
    % Computes the derivatives of the state vector based on Lagrangian dynamics.
    % State = [x; dx; theta1; dtheta1; theta2; dtheta2]

    % Unpack state variables
    x = state(1);      dx = state(2);
    th1 = state(3);    dth1 = state(4);
    th2 = state(5);    dth2 = state(6);
    
    % Equations of Motion
    % Calculate Acceleration of the Cart (x_ddot)
    % Numerator terms derived from Lagrangian
    num = F_input ...
        + m1 * l1 * (dth1^2) * sin(th1) ...
        + m2 * l2 * (dth2^2) * sin(th2) ...
        - m1 * g * cos(th1) * sin(th1) ...
        - m2 * g * cos(th2) * sin(th2);
    
    % Denominator
    den = M + m1 * (sin(th1)^2) + m2 * (sin(th2)^2);
    
    x_ddot = num / den;
    
    % Calculate Angular Accelerations
    th1_ddot = (1/l1) * (x_ddot * cos(th1) - g * sin(th1));
    th2_ddot = (x_ddot * cos(th2) - g * sin(th2)) / l2;
    
    % Pack derivatives into output vector
    dxdt = [dx; x_ddot; dth1; th1_ddot; dth2; th2_ddot];
end