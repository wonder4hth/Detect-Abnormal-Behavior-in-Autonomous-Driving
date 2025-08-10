clear;
close all;
clc;

tf = 3;
T = 0.1;
time_step = floor(tf/T) + 1;
A = {
    [1 T T^2/2 ; 0 1 T ; 0 0 1];
    [1 T T^2/2 ; 0 1 T ; 0 0 1];
    [1 T T^2/2 ; 0 1 T ; 0 0 1]
    }; 
B = {
    [T^3/6 ; T^2/2 ; T];
    [T^3/6 ; T^2/2 ; T];
    [T^3/6 ; T^2/2 ; T]
    };
C = {
    [1 0 0];
    [0 1 0];
    [0 0 1]
    };
Q = {
    [0.1 0 0 ; 0 0.01 0; 0 0 0.1] * 1e0;
    [0.1 0 0 ; 0 0.01 0; 0 0 0.1] * 1e0;
    [0.1 0 0 ; 0 0.01 0; 0 0 0.1] * 1e0
    };

R = 0.005;
u = {0; 4; 2}; 
x = [2.5 -1 3]';
mu_hat = [0.2 0.5 0.3]';

ni = size(A,1); % size of collection

% variables for IMM
xx = [x, zeros(ni, time_step)]; % real value system
zz = [0 * x, zeros(ni, time_step)]; % estimate
mumu = [mu_hat, zeros(ni, time_step)];
collectedCovariances = zeros(size(x,1), size(x,1), time_step);

% initialize values that are subjected to update
mu_hat = [0.3 0.5 0.2]; % probability vector
z_i = cell(1, ni);
p_i = cell(1, ni);
for j=1:ni
    z_i{1, j} = x*0; %*rand(1,1);
    p_i{1, j} = diag([0.1, 0.08, 0.05]);
end
PI = [0.9, 0.05, 0.5; 0.05, 0.9, 0.05; 0.05, 0.05, 0.9];

for i = 2:(time_step + 1)
    % update state
    [x, gamma] = update_sys(A{1,1}, B{1,1}, C{1,1}, Q{1,1}, R, ...
        x, u{2,1});
    % update estimate using y
    [z_i, p_i, p_hat, mu_hat, z_hat] = ...
    IMM(A, B, C, Q, u, R, z_i, p_i, gamma, mu_hat, PI);
    % save values needed
    xx(: , i) = x;
    zz(:, i) = z_hat;
    mumu(:, i) = mu_hat;
    collectedCovariances(:, :, i) = p_hat;
    % generate new input u for the state
    for j = 1:ni
        u{j , 1} = rand(1, 1);
    end
end
%% Plot
figure(1);
for i = 1:ni
    subplot(ni,1,i);
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
    ylabel('result');
    legend('estimated', 'real')
end
figure(2);
for i = 1:ni
    hold on
    plot(mumu(i, :));
end
legend('Model 1', 'Model 2', 'Model 3')
xlabel('time step');
ylabel('probability');