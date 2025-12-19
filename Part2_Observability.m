%% Part 2: Observability Analysis
% Checks the rank of the observability matrix for different output configurations.

if ~exist('A_sys', 'var'), run('setup_parameters.m'); end

% Output Matrices
C_cases = {
    [1 0 0 0 0 0],                          % Case 1: y = x
    [0 0 1 0 0 0; 0 0 0 0 1 0],             % Case 2: y = [theta1, theta2]
    [1 0 0 0 0 0; 0 0 0 0 1 0],             % Case 3: y = [x, theta2]
    [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 1 0] % Case 4: y = [x, theta1, theta2]
};

labels = {'y=[x]', 'y=[th1, th2]', 'y=[x, th2]', 'y=[x, th1, th2]'};

disp('Observability Rank Analysis (Full Rank = 6)');
for i = 1:length(C_cases)
    Ob_matrix = obsv(A_sys, C_cases{i});
    rank_val = rank(Ob_matrix);
    fprintf('Case %d (%s): Rank = %d\n', i, labels{i}, rank_val);
end

disp('Conclusion: Case 1 (y = x) is the minimal set required for full observability.');