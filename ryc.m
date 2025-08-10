function result = ryc(a)
% reference value "real" system --> intention
x_ref = [0, 5.5, 0, 20]';   % (xpos [m], ypos [m], yaw [rad], speed [m/s])

% reference value assumed system --> assumed intention
x_refdi = [0, 20, 5.5, 0]'; % (xpos [m], vxpos [m/s], ypos [m], vypos [m/s])

% initial condition "real" system
x0 = [0, 2.5, 0, 10]';    % (xpos [m], ypos [m], yaw [rad], speed [m/s])

% initial condition assumed system
x0dic = [0, 10, 2.5, 0]';  % (xpos [m], vxpos [m/s], ypos [m], vypos [m/s])

tf = 5;    % end sim time [s]
T = 0.5;   % sampling time [s]
Nsteps = floor(tf / T);

K_xyphi = [0, a, 1]; % Try to modify the 2nd value to see where it converges

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
% Evaluating residuals
rxc = xx(1, :) - xxdic(1, :);
fprintf('Mean residual x: \t \t %.3f, \t St.Dev: %.3f\n', mean(rxc), sqrt(var(rxc)))

result = xx(2, :) - xxdic(3, :); 
end
