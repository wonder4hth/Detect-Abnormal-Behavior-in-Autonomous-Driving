function [z_hat_i_out, p_hat_i_out, p_hat, mu_hat, z_hat] = IMM(A, B, C, Q, u, ...
    R, z_hat_i, p_hat_i, gamma, mu_hat, PI)
ni = size(A,1);
%% State interaction
c_square_matrix = zeros(ni);
u_tilde = zeros(ni);
z_hat_j = cell(1,ni);
P_hat_j = cell(1,ni);
z_hat_0j = cell(1,ni);
p_hat_0j = cell(1,ni);

% normalization factor
for j = 1:ni
    for i = 1:ni
        c_square_matrix(j,i) = PI(i,j) * mu_hat(i);
    end
end
cj = sum(c_square_matrix,2);

for j = 1:ni
% conditional model probability
    z_hat_0j{j} = 0 * z_hat_i{1, j};
    for i = 1:ni
        u_tilde(j,i) = PI(i,j) * mu_hat(i) * inv(cj(j));
        % u_tilde{j,i} means u_tilde_j|i
        % mix state estimate
        z_hat_j{i, j} = u_tilde(j,i) * z_hat_i{1, i};
        z_hat_0j{j} = z_hat_0j{j} + u_tilde(j,i) * z_hat_i{1, i};
    end

    % covariance
    p_hat_0j{j} = 0 * p_hat_i{1,j};
    for i = 1:ni
        P_hat_j{i} = u_tilde(j,i) * ...
        (p_hat_i{i} + (z_hat_i{1, i} - cell2mat(z_hat_0j)) * ...
        (z_hat_i{1, i} - cell2mat(z_hat_0j))');
        p_hat_0j{j} = p_hat_0j{j} + P_hat_j{i};
    end
end

%% Kalman Filtering
z_hat_i_out = cell(1,ni);
p_hat_i_out = cell(1,ni);
z_kf = z_hat_0j{1,j};
p_kf = p_hat_0j{1,j};
lambda_j = zeros(1, ni); % probability
for j = 1:ni
    [innovation, e_cov, z_kf, p_kf] = ...
    KalmanFilter(A{j,1}, B{j,1}, C{j,1}, Q{j,1}, R, gamma, u{j,1}, z_kf, p_kf);
    z_hat_i_out{j} = z_kf;
    p_hat_i_out{j} = p_kf;
    % get lambda stored here
    lambda_j(j) = exp((-0.5) * innovation' * inv(e_cov) * innovation)/(sqrt(det(2*pi*e_cov)));
end
%% Model Probability Update
% normalization factor -> sum(normalization factor * lambda)
c_collection = zeros(1,ni);
mu_hat = zeros(1,ni);
c = 0;
for i = 1:ni
    c_collection(i) = lambda_j(i) *  cj(i);
    c = c + c_collection(i);
end
for j = 1:ni
    mu_hat(j) =  lambda_j(j) *  cj(j) * inv(c);
end
%% State estimate Combination
z_hat = zeros(size(z_hat_i_out{1}));
for i = 1:ni
    z_hat = z_hat + mu_hat(i) * z_hat_i_out{i};
end
P_hat_collection = cell(1,ni);
p_hat = 0 * p_hat_i{1,j};
for i = 1:ni 
    P_hat_collection{i} = mu_hat(i) *... 
    (P_hat_j{i} + (z_hat_j{i} - z_hat) * (z_hat_j{i} - z_hat)');
    p_hat = p_hat + P_hat_collection{i};
end
end