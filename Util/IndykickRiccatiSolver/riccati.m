% Solve riccati equation for indykick

% Some basic definition of the cart table model.
z = 200.0 % Height of the center of mass in (mm)
g = 9.81e-3; % gravitational acceleration in (mm)/(ms)^2
zg = z / g;
T = 10; % time of one motion cycle in (ms)
zmppenalty = 1 % penalty for the zmp
comPositionPenalty = 0 % penalty for the com
comVelocityPenalty = 0 % penalty for the movement of the com
comAccelerationPenalty = 0 % penalty for accelerating the com
comJerkPenalty = 1e9 % penalty for changing the acceleration of the com

A = [1 T 0.5 * T**2; 0 1 T; 0 0 1];
B = [1 / 6 * T**3; 1/2 * T**2; T];
C = [1 0 -zg];

% Calculate input matrices for the controller.
It = [1; 0; 0; 0];
Bt = [C * B; B];
Ft = [C * A; A];
At = [It Ft];
Qt = diag([zmppenalty, comPositionPenalty, comVelocityPenalty, comAccelerationPenalty]);
Rt = [comJerkPenalty];

[P, L, G] = dare(At, Bt, Qt, Rt)

format long e
save riccatisolution P comJerkPenalty z T
