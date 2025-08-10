close all
clearvars
clc

tf = 2;
T = 0.1;
time_step = floor(tf/T) + 1;

x = [2.5 -1 3 4]';
mu_hat = [0.2 0.5 0.1 0.1 0.1]';

states = ["x","v_x","y","v_y"];
F = [1 T 0 0; 0 1 0 0; 0 0 1 T; 0 0 0 1];
G = [T^2/2 0; T 0; 0 T^2/2; 0 T];
H = [1 0 0 0; 0 0 1 0];
R_noise = [0.005 0; 0 0.005];

W = eye(size(G,2)) * 1e-5;
Q_LQR = diag([0, 1, 10, 1]);
R_LQR = diag([0.2, 0.5]);

K = get_LQR_gain(F, G, Q_LQR, R_LQR);

u = {
    [10; 10; 0; 0]; % Going straight
    [10; 10; 3; 0];  % Changing lanes
    [10; 10; -3; 0];   
    [10; 10; 5; 0];
    [10; 10; -5; 0];
}; % x_reference
ni = size(u,1);
A = cell(ni, 1);
B = cell(ni, 1);
Q = cell(ni, 1); % Q is for covariance
C = cell(ni, 1);
for i = 1:ni
    A{i,1} = F + G * K;
    B{i,1} = - G * K;
    Q{i,1} = G * W * G'; 
    C{i,1} = H;
end

% define a real system
A_real = F + G * K;
B_real = - G * K;
Q_real = G * W * G'; 
C_real = H;
u_real = [10; 10; -8; 0];

%% run IMM
% variables for IMM
xx = [x, zeros(size(x,1), time_step)]; % real value system
zz = [0 * x, zeros(size(x,1), time_step)]; % estimate
mumu = [mu_hat, zeros(ni, time_step)];
collectedCovariances = zeros(size(x,1), size(x,1), time_step);

% initialize values that are subjected to update
z_i = cell(1, ni);
p_i = cell(1, ni);
for j=1:ni
    z_i{1, j} = x*rand(1,1);
    p_i{1, j} = diag([0.1, 0.08, 0.05, 0.09]);
end
PI = [0.96, 0.01, 0.01, 0.01, 0.01; 
    0.01, 0.96, 0.01, 0.01, 0.01; 
    0.01, 0.01, 0.96, 0.01, 0.01;
    0.01, 0.01, 0.01, 0.96, 0.01
    0.01, 0.01, 0.01, 0.01, 0.96];

for i = 2:(time_step + 1)
    % update state
    [x, gamma] = update_sys(A_real, B_real, C_real, ...
        Q_real, R_noise, x, u_real);
    % update estimate using y
    [z_i, p_i, p_hat, mu_hat, z_hat] = ...
    IMM(A, B, C, Q, u, R_noise, z_i, p_i, gamma, mu_hat, PI);
    % save values needed
    xx(: , i) = x;
    zz(:, i) = z_hat;
    mumu(:, i) = mu_hat;
    collectedCovariances(:, :, i) = p_hat;
end
%% Plot
figure(1);
for i = 1:4
    subplot(4,1,i);
    hold on
    plot(zz(i, :), 'r*'); % estimated
    plot(xx(i, :), 'b-'); % real
    hold on
    for k = 1:time_step+1
        zz_i_k = zz(i, k);
        pp_i_k = collectedCovariances(i, i, k);
        plot([k, k], [zz_i_k - pp_i_k, ...
            zz_i_k + pp_i_k], 'r-'); 
    end
    xlabel('time step');
    ylabel(states(i));
    legend('estimated', 'real')
end
figure(2);
for i = 1:ni
    hold on
    plot(mumu(i, :));
end
legend('Model 1', 'Model 2', 'Model 3','Model 4','Model 5')
xlabel('time step');
ylabel('probability');

function K = get_LQR_gain(A, B, Q_LQR, R_LQR)
    P = 0;
    K = zeros(size(B'));
    for h = 1:100
        P = Q_LQR + K' * R_LQR * K + A' * P * A + 2 * A' * P * B * K ...
        + K' * B' * P * B * K;
        K = -inv(R_LQR + B' * P * B) * B' * P * A;
    end
end