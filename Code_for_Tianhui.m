
%{

The code is written in MATLAB and simulates a kinematic bicycle model and a double integrator model.

1. Define reference values for the "real" system and assumed system.
2. Specify initial conditions for the "real" system and assumed system.
4. Define the parameters for the bicycle model.
5. Implement the bicycle model using differential equations.
6. Define the parameters for the double integrator model.
7. Apply the Value Iteration Algorithm to find optimal control gain (LQR).
8. Simulate the systems and update the state variables. The double
integrator uses as initial state the last measured value (not the
previously predicted state)
9. Calculate and evaluate the residuals between the models.
10. Plot the results for comparison.

%}


close all
clearvars
clc

% reference value "real" system --> intention
x_ref = [0, 4.6, 0, 40]';   % (xpos [m], ypos [m], yaw [rad], speed [m/s])

% reference value assumed system --> assumed intention
x_refdi = [0, 20, 3.6, 0]'; % (xpos [m], vxpos [m/s], ypos [m], vypos [m/s])

% initial condition "real" system
x0 = [0, 0, 0, 10]';    % (xpos [m], ypos [m], yaw [rad], speed [m/s])

% initial condition assumed system
x0dic = [0, 10, 0, 0]';  % (xpos [m], vxpos [m/s], ypos [m], vypos [m/s])

tf = 5;    % end sim time [s]
T = 0.5;   % sampling time [s]
Nsteps = floor(tf / T);

%% Bicycle Model

delta_sat = pi / 3;

K_a = 1;
K_xyphi = [0, 0.1, 1]; % Try to modify the 2nd value to see where it converges

lf = 2;   % distance front axle [m]
lr = 2;   % distance rear axle [m]

% Define the acceleration control input
acc = @(x) K_a * (x_ref(4) - x(4, :));

% Define the steering control input
delt = @(x) min(delta_sat, max(-delta_sat, K_xyphi * (x_ref(1:3, :) - x(1:3, :))));

% Calculate the slip angle
alph = @(d) atan(lr / (lr + lf) * tan(d));

% Define the kinematic bicycle model equations
f = @(t, x) [x(4) * cos(alph(delt(x)) + x(3));
             x(4) * sin(alph(delt(x)) + x(3));
             x(4) * sin(alph(delt(x))) / lr;
             acc(x)];

%% Double Integrator

A = eye(4);
A(1, 2) = T;
A(3, 4) = T;
B = [0.5 * T^2, 0; T, 0; 0, 0.5 * T^2; 0, T];

Q = diag([0, 1, 10, 1]);
R = diag([0.2, 0.5]);

K = get_LQR_gain(A, B, Q, R);

%% Simulation

xx = [x0, zeros(length(x0), Nsteps)];
xxdic = [x0dic, zeros(length(x0dic), Nsteps)];

for k = 1:floor(tf / T)
    % Update the state of the assumed system (with correction)
    x0dic = project_state(x0);
    x0dic = A * x0dic + B * K * (x0dic - x_refdi);
    xxdic(:, k + 1) = x0dic;

    % Update the state of the kinematic bicycle model
    [~, x_] = ode45(f, [0, T], x0);
    x0 = x_(end, :)';
    xx(:, k + 1) = x0;
end

%% Evaluate residuals

rxc = xx(1, :) - xxdic(1, :);
fprintf('Mean residual x: \t \t %.3f, \t St.Dev: %.3f\n', mean(rxc), sqrt(var(rxc)))

ryc = xx(2, :) - xxdic(3, :); % Save ryc for each simulation (5-7)
fprintf('Mean residual y: \t \t %.3f, \t St.Dev: %.3f\n', mean(ryc), sqrt(var(ryc)))
fprintf('Max value for the residual y: \t \t %.3f\n', max(ryc))
fprintf('Steady state value for the residual y: \t %.3f\n', ryc(5))

%% Plots

t_vec = 0:T:tf;
loc = 'best';

% subplot(2, 1, 1)
% hold on
% plot(t_vec, xx(2, :))
% plot(t_vec, xxdic(3, :))
% grid on
% xlabel('t [s]', 'Interpreter', 'Latex')
% ylabel('y [m]', 'Interpreter', 'Latex')
% title('y position', 'Interpreter', 'Latex')
% legend('Kinematic Bicycle ("real" model)', ...
%     'Double Integrator (assumed model)', ...
%     'Interpreter', 'Latex', 'Location', loc)
% 
% subplot(2, 1, 2)
% hold on
figure(1);
plot(t_vec, ryc)
hold on
grid on
xlabel('t [s]', 'Interpreter', 'Latex')
ylabel('r [m]', 'Interpreter', 'Latex')
title('Residual y position', 'Interpreter', 'Latex')
%% recording standard deviations
y = 0.1 : 0.1 : 0.9;
sdx = [0.247 0.383 0.555 0.697 0.810 0.900 0.975 1.034 1.083];
sdy = [0.192 0.559 0.818 0.985 1.101 1.182 1.238 1.277 1.304];
%% Matrix for SD
ryc05 = [0   -0.2085   -0.2572   -0.2443   -0.1723   -0.1073   -0.0635   -0.0367   -0.0210   -0.0119   -0.0068];
ryc1 = [0    0.5688    0.0331   -0.1841   -0.1195   -0.0437   -0.0114   -0.0022   -0.0003    0.0000    0.0000];
ryc2 = [0    1.7384   -0.1959   -0.4055   -0.0611    0.0143    0.0041   -0.0002   -0.0002   -0.0000    0.0000];
ryc3 = [0    2.4911   -0.7198   -0.3837    0.0386    0.0114   -0.0017   -0.0001    0.0000   -0.0000   -0.0000];
ryc4 = [0    2.9123   -1.1730   -0.2496    0.0595   -0.0013   -0.0008    0.0001   -0.0000   -0.0000    0.0000];
ryc5 = [0    3.1667   -1.5169   -0.1224    0.0438   -0.0040    0.0001    0.0000   -0.0000    0.0000   -0.0000];
ryc6 = [0    3.3261   -1.7656   -0.0355    0.0247   -0.0027    0.0002   -0.0000    0.0000    0.0000   -0.0000];
ryc7 = [0    3.4293   -1.9446    0.0144    0.0117   -0.0013    0.0001   -0.0000    0.0000   -0.0000    0.0000];
ryc8 = [0    3.4957   -2.0710    0.0383    0.0046   -0.0005    0.0000   -0.0000    0.0000   -0.0000    0.0000];
ryc9 = [0    3.5388   -2.1621    0.0466    0.0013   -0.0001    0.0000   -0.0000    0.0000   -0.0000    0.0000];

collection = zeros(10,11);
collection(1, :) = ryc05;
collection(2, :) = ryc1;
collection(3, :) = ryc2;
collection(4, :) = ryc3;
collection(5, :) = ryc4;
collection(6, :) = ryc5;
collection(7, :) = ryc6;
collection(8, :) = ryc7;
collection(9, :) = ryc8;
collection(10, :) = ryc9;

sd_over_time = zeros(1, 11);
sd_over_time(1) = sqrt(var(collection(:, 1)));
sd_over_time(2) = sqrt(var(collection(:, 2)));
sd_over_time(3) = sqrt(var(collection(:, 3)));
sd_over_time(4) = sqrt(var(collection(:, 4)));
sd_over_time(5) = sqrt(var(collection(:, 5)));
sd_over_time(6) = sqrt(var(collection(:, 6)));
sd_over_time(7) = sqrt(var(collection(:, 7)));
sd_over_time(8) = sqrt(var(collection(:, 8)));
sd_over_time(9) = sqrt(var(collection(:, 9)));
sd_over_time(10) = sqrt(var(collection(:, 10)));
sd_over_time(11) = sqrt(var(collection(:, 11)));
threshold = sd_over_time.*3

plot(t_vec,sd_over_time);
grid on
plot(t_vec, threshold);
xlabel("Time");
ylabel("Residual");
legend("sd over time","threshold");
figure(1);
plot(t_vec, threshold,'r--');
neg_threshold = threshold .* (-1);
plot(t_vec, neg_threshold,'r--');
%% Helper function

% Project the state variables to match the assumed system
function x0di = project_state(x0)
    x0di = x0;
    x0di(3) = x0(2);                  % Match the third state variable
    x0di(2) = x0(4) * cos(x0(3));     % Calculate the second state variable based on the fourth
    x0di(4) = x0(4) * sin(x0(3));     % Calculate the fourth state variable based on the fourth
end

function K = get_LQR_gain(A, B, Q, R)
    % Value Iteration Algorithm to find optimal control gains
    P = 0;
    K = zeros(size(B'));
    for h = 1:100
        P = Q + K' * R * K + A' * P * A + 2 * A' * P * B * K + K' * B' * P * B * K;
        K = -inv(R + B' * P * B) * B' * P * A;
    end
end