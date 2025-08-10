close all;
clear;
clc;

% sampling time
tf = 1;
T = 0.1;
time_step = floor(tf/T) + 1; 

% system
A = [1 T T^2/2 ; 0 1 T ; 0 0 1];
B = [T^3/6 ; T^2/2 ; T];
C = [1 0 0];
Q = [0.2 0 0 ; 0 0.9 0; 0 0 0.1] * 0.0000001;
R = 0.05 * 0;
n = size(A, 1);

% Initialize variables
u = 0;
x = [2.5 -1 3]';
estimatedState = 0 * x; % a vector with the same dimension as x
estimatedCov =  eye(n); % n * n matrix (change to zeros)

xx = [x,zeros(n, time_step)]; % real value system
collectedEstimatedStates = [estimatedState, zeros(n, time_step)]; 
% collection of estimated states
collectedCovariances = zeros(n, n, time_step); 

for i = 2:(time_step + 1)
    % update state
    [x, y_meas] = update_sys(A, B, C, Q, R, x, u);
    % update estimate using y
    collectedCovariances(:, :, i) = estimatedCov;
    [innovation, e_cov, estimatedState, estimatedCov] = ...
    KalmanFilter(A, B, C, Q, R,  y_meas, u, estimatedState, estimatedCov);
    % save x
    xx(: , i) = x;
    collectedEstimatedStates(:, i) = estimatedState;
    % generate new input for the state
    u = rand(1, 1);
end

figure(1);
for i = 1:n
    subplot(n,1,i);
    hold on
    plot(xx(i, :), 'r-'); % 1st of real
    plot(collectedEstimatedStates(i, :), 'bo'); % plot estimates
    hold on
    for j = 1:time_step
        estimatedState_i = collectedEstimatedStates(i, j);
        estimatedCov_i = collectedCovariances(i, i, j);
        plot([j, j], [estimatedState_i - estimatedCov_i, ...
            estimatedState_i + estimatedCov_i], 'b-'); 
        % uncertainty region
    end
end