tf = 5;    % end sim time [s]
T = 0.5;   % sampling time [s]
Nsteps = floor(tf / T);
t_vec = 0:T:tf;
ryc1 = [0    0.5688    0.0331   -0.1841   -0.1195   -0.0437   -0.0114   -0.0022   -0.0003    0.0000    0.0000];
ryc15 = [0    0.2682   -0.2141   -0.2065   -0.1257   -0.0689   -0.0362   -0.0185   -0.0094   -0.0047   -0.0023];
ryc2 = [0    0.0500   -0.3596   -0.2725   -0.1798   -0.1148   -0.0719   -0.0444   -0.0273   -0.0167   -0.0101];
ryc25 = [0   -0.1134   -0.4588   -0.3437   -0.2436   -0.1696   -0.1166   -0.0796   -0.0541   -0.0367   -0.0248];
ryc3 = [0   -0.2391   -0.5331   -0.4097   -0.3059   -0.2256   -0.1651   -0.1202   -0.0872   -0.0631   -0.0457];
figure(1);
hold on
plot(t_vec, ryc1);
plot(t_vec, ryc15);
plot(t_vec, ryc2);
plot(t_vec, ryc25);
plot(t_vec, ryc3);
xlabel('t [s]', 'Interpreter', 'Latex')
ylabel('r [m]', 'Interpreter', 'Latex')
title('Residual y position', 'Interpreter', 'Latex')
legend();