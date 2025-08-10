%% set up
tf = 5;    % end sim time [s]
T = 0.5;   % sampling time [s]
Nsteps = floor(tf / T);
t_vec = 0:T:tf;
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
%% Plot residuals
figure(1);
hold on
plot(t_vec, ryc05);
plot(t_vec, ryc1);
plot(t_vec, ryc2);
plot(t_vec, ryc3);
plot(t_vec, ryc4);
plot(t_vec, ryc5);
plot(t_vec, ryc6);
plot(t_vec, ryc7);
plot(t_vec, ryc8);
plot(t_vec, ryc9);
grid on
xlabel('t [s]', 'Interpreter', 'Latex')
ylabel('r [m]', 'Interpreter', 'Latex')
title('Residual y position', 'Interpreter', 'Latex')
legend("ryc05","ryc1","ryc2","ryc3","ryc4","ryc5","ryc6","ryc7","ryc8","ryc9");
%% recording standard deviations
y = 0.1 : 0.1 : 0.9;
sdx = [0.247 0.383 0.555 0.697 0.810 0.900 0.975 1.034 1.083];
sdy = [0.192 0.559 0.818 0.985 1.101 1.182 1.238 1.277 1.304];
figure(2);
plot(y,sdx);
hold on
plot(y,sdy);
grid on
legend('sdx','sdy');
xlabel("y");
ylabel("Standard Deviation");

%% Matrix for SD
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
sd_over_time

threshold = sd_over_time.*3

% plot SD and threshold
figure(1);
plot(t_vec,sd_over_time);
hold on
grid on
plot(t_vec, threshold);
xlabel("Time");
ylabel("Residual");
legend("sd over time","threshold");
figure(1);
plot(t_vec, threshold,'r--');
neg_threshold = threshold .* (-1);
plot(t_vec, neg_threshold,'r--');