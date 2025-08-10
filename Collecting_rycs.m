%% setup
tf = 5;    % end sim time [s]
T = 0.5;   % sampling time [s]
Nsteps = floor(tf / T);
t_vec = 0:T:tf;
ryc05 = [0   -0.1355   -0.1669   -0.1593   -0.1123   -0.0699   -0.0413   -0.0239   -0.0137   -0.0078   -0.0044];
ryc1 = [0    0.3731    0.0217   -0.1212   -0.0777   -0.0283   -0.0074   -0.0014   -0.0002    0.0000    0.0000];
ryc2 = [0    1.1392   -0.1366   -0.2637   -0.0391    0.0094    0.0027   -0.0001   -0.0001   -0.0000    0.0000];
ryc3 = [0    1.6592   -0.4895   -0.2399    0.0260    0.0071   -0.0011   -0.0001    0.0000   -0.0000   -0.0000];
ryc4 = [0    2.0006   -0.7878   -0.1282    0.0367   -0.0013   -0.0004    0.0001   -0.0000   -0.0000    0.0000];
ryc5 = [0    2.2018   -0.9813   -0.0331    0.0233   -0.0025    0.0001    0.0000   -0.0000    0.0000   -0.0000];
ryc7 = [0    2.3881   -1.1482    0.0517    0.0027   -0.0005    0.0000   -0.0000    0.0000   -0.0000    0.0000];

figure(1);
hold on
plot(t_vec, ryc05);
plot(t_vec, ryc1);
plot(t_vec, ryc2);
plot(t_vec, ryc3);
plot(t_vec, ryc4);
plot(t_vec, ryc5);
plot(t_vec, ryc7);
grid on
xlabel('t [s]', 'Interpreter', 'Latex')
ylabel('r [m]', 'Interpreter', 'Latex')
title('Residual y position', 'Interpreter', 'Latex')
legend();

%% Matrix for SD
collection = zeros(7,11);
collection(1, :) = ryc05;
collection(2, :) = ryc1;
collection(3, :) = ryc2;
collection(4, :) = ryc3;
collection(5, :) = ryc4;
collection(6, :) = ryc5;
collection(7, :) = ryc7;