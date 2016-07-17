clear all;

import casadi.*;

Ts = 0.05;
k = 0;

% Actual settings
x_hat = [5;0;0;0;0;0];
u_act = 0;
w_state = 0;
cov1 = 1;
cov2 = 1;
cov3 = 1;
cov4 = 1;
cov5 = 1;
cov6 = 1;
P = diag([cov1 cov2 cov3 cov4 cov5 cov6]); % Covariance matrix of state vector

nx = 6;     %State dimension
nu = 1;     %Control dimension

% Parameter
a = 1;
b = 1;
m = 2050;
I = 3344;
g = 9.81;

u = MX.sym('u', nu);
xi = MX.sym('xi', nx);

% System dynamics
delta_f = u * pi / 180;
delta_r = 0;

vx_f = xi(1);
vy_f = xi(2) + a * xi(4);
vx_r = xi(1);
vy_r = xi(2) - b * xi(4);

vl_f = vy_f * sin(delta_f) + vx_f * cos(delta_f);   % u corresponds to front steering angle
vc_f = vy_f * cos(delta_f) - vx_f * sin(delta_f);
vl_r = vx_r; % rear steering angle = 0 --> simplification
vc_r = vy_r;

alpha_f = atan(vc_f / vl_f);
alpha_r = atan(vc_r / vl_r);
s_f = 0; %TODO: Check how to implement if-else-condition
s_r = 0; %TODO: Check how to implement if-else-condition
Fz_f = b * m * g / (2000 * (a + b));
Fz_r = a * m * g / (2000 * (a + b));

% Use friction model to determine F[l/r]_[f/r]
mu = 0;
[Fl_f, Fc_f] = Pacejka(alpha_f, s_f, mu, Fz_f);
[Fl_r, Fc_r] = Pacejka(alpha_r, s_r, mu, Fz_r);

Fx_f = Fl_f * cos(delta_f) - Fc_f * sin(delta_f);   % u corresponds to front steering angle
Fy_f = Fl_f * sin(delta_f) + Fc_f * cos(delta_f);   % u corresponds to front steering angle
Fx_r = Fl_r; % rear steering angle = 0 --> simplification
Fy_r = Fc_r; % rear steering angle = 0 --> simplification

% Define NL system dynamics
xdot = [xi(2) * xi(4) + (2 / m) * (Fx_f + Fx_r);
        -xi(1) * xi(4) + (2 / m) * (Fy_f + Fy_r);
        xi(4);
        (2 / I) * (a * Fy_f - b * Fy_r);
        xi(1) * cos(xi(3)) - xi(2) * sin(xi(3));
        xi(1) * sin(xi(3)) + xi(2) * cos(xi(3))];
% Simulation of noisy output of plant
y = [xdot(3); xdot(6)];
    
% Linearize NL system dynamics
jacobi_xdot = jacobian(xdot, xi);
A_c_F = Function('jacobi_xdot', {xi, u}, {jacobi_xdot});
A_c = A_c_F(x_hat, u_act);
A_d = eye(nx) + Ts * A_c;
jacobi_y = jacobian(y, xi);
C_c_F = Function('jacobi_y', {xi, u}, {jacobi_y});
C_c = C_c_F(x_hat, u_act);
C_d = C_c;

% Noise model for state
W_c = diag(ones(1, nx));
W_c = 0.00000001 * W_c;
W_d = Ts * W_c;

% Noise model for output
V_c = [[0.1 0]; [0 0.1]];
V_d = (1 / Ts) * V_c;

P = W_d;

y = [x_hat(3); x_hat(6)] * 1.1;

% Prediction step
x_hat = A_d * x_hat;
P = A_d * P * A_d' + W_d;

% Update step
P = inv(inv(P) + C_d' * inv(V_d) * C_d);
x_hat = x_hat + P * C_d' * inv(V_d) * (y - C_d * x_hat);