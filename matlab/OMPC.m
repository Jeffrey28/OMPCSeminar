function [ u ] = OMPC( xHat, prev_u, Ts, flag )
%OMPC calculates optimal contols based on a state estimate xHat.
% INPUTS:
%   xHat: estimated state of the plant
%   Ts  : sampling time in seconds
%   flag: 1: NLMPC (non-linear)
%         2: LTVMPC (linear time variant)
%         3: LTIMPC (linear time invariant)
%
% OUTPUTS:
%  u    : control inputs for the plant at step k+1.

import casadi.*;

Ts = 0.05;

Np = 25;    %Prediction horizon
Nc = 25;    %Control horizon
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
% xi:
% xi(1) := x_dot
% xi(2) := y_dot
% xi(3) := psi
% xi(4) := psi_dot
% xi(5) := X
% xi(6) := Y

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

xdot = [xi(2) * xi(4) + (2 / m) * (Fx_f + Fx_r);
        -xi(1) * xi(4) + (2 / m) * (Fy_f + Fy_r);
        xi(4);
        (2 / I) * (a * Fy_f - b * Fy_r);
        xi(1) * cos(xi(3)) - xi(2) * sin(xi(3));
        xi(1) * sin(xi(3)) + xi(2) * cos(xi(3))];

f = Function('f', {xi, u}, {xdot});

% RK4 integrator
Mrk = 20;
U   = MX.sym('U');
X0  = MX.sym('X0', nx);
X = X0;
DT = Ts / Mrk;

for i=1:Mrk
    k1 = f(X, U);
    k2 = f(X + DT * k1 / 2, U);
    k3 = f(X + DT * k2 / 2, U);
    k4 = f(X + DT * k3, U);
    X   = X + (DT / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

RK4 = Function('RK4', {X0, U}, {X});

X_0 = xHat;
U_init = prev_u;

% Linearization

% For state vector
jacobiX_state = jacobian(X, X0);
jacobiXF_state = Function('jacobiXF_state', {X0, U}, {jacobiX_state});
jacobiU_state = jacobian(X, U);
jacobiUF_state = Function('jacobiUF_state', {X0, U}, {jacobiU_state});
A = jacobiXF_state(X_0, U_init);
B = jacobiUF_state(X_0, U_init);

% For spip angle
% Front angle
jacobiX_front = jacobian(alpha_f, xi);
jacobiXF_front = Function('jacobiFX', {xi, u}, {jacobiX_front});
jacobiU_front = jacobian(alpha_f, u);
jacobiUF_front = Function('jacobiUX', {xi, u}, {jacobiU_front});
C_front = jacobiXF_front(X_0, U_init);
D_front = jacobiUF_front(X_0, U_init);
% Rear angle
jacobiX_rear = jacobian(alpha_r, xi);
jacobiXF_rear = Function('jacobiFX', {xi, u}, {jacobiX_rear});
jacobiU_rear = jacobian(alpha_r, u);
jacobiUF_rear = Function('jacobiUX', {xi, u}, {jacobiU_rear});
C_rear = jacobiXF_rear(X_0, U_init);
D_rear = jacobiUF_rear(X_0, U_init);
%offset = RK4(X_0, U_init);


% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

% "Lift" initial conditions
U_prev = MX.sym('U_prev', 1);
X0 = MX.sym('X0', 6);
w = {w{:}, U_prev, X0};
lbw = [lbw; U_init; X_0];
ubw = [ubw; U_init; X_0];
w0 = [w0; U_init; X_0];

epsilon = MX.sym('epsilon', 1);
w = {w{:}, epsilon};
lbw = [lbw; 0];
ubw = [ubw; inf];
w0 = [w0; 0];

% Add penalty for slip angle constraint violation
rho = 1000;
J = J + rho * epsilon;

alpha_min = -2.2 * (pi / 180);
alpha_max = 2.2 * (pi / 180);

% Formulate the NLP
Xk = X0;
for k=0:Np
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    DUk = MX.sym(['DU_' num2str(k)]);
    alphaf_k = MX.sym(['alphaf_' num2str(k)]);
    alphar_k = MX.sym(['alphar_' num2str(k)]);
    w = {w{:}, Uk, DUk, alphaf_k, alphar_k};
    
    if k < Nc
        lbw = [lbw; -10; -0.85; alpha_min; alpha_min];
        ubw = [ubw;  10;  0.85; alpha_max; alpha_max];
        w0 = [w0;  0;  0; 0; 0];
    else
        lbw = [lbw; -10; 0; alpha_min; alpha_min];
        ubw = [ubw;  10; 0; alpha_max; alpha_max];
        w0 = [w0;  0;  0; 0; 0];
    end

    % Add action constraint
    g = {g{:}, U_prev - Uk + DUk};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    U_prev = Uk;
    
    % Front
    g = {g{:}, - alphaf_k + alpha_min - epsilon};
    lbg = [lbg; -inf];
    ubg = [ubg; 0];
    
    g = {g{:}, - alphaf_k + alpha_max + epsilon};
    lbg = [lbg; 0];
    ubg = [ubg; inf];
    
    % Rear
    g = {g{:}, - alphar_k + alpha_min - epsilon};
    lbg = [lbg; -inf];
    ubg = [ubg; 0];
    
    g = {g{:}, - alphar_k + alpha_max + epsilon};
    lbg = [lbg; 0];
    ubg = [ubg; inf];
    
    % State linearization
    Xk_end = A * Xk + B * Uk;
    %Xk_end = offset + A * (Xk-X_0) + B * (Uk-U_init);
    
    % Slip angle linearization
    % Front
    alpha_fk_end = C_front * Xk + D_front * Uk;
    % Rear
    alpha_rk_end = C_rear * Xk + D_rear * Uk;
    
  
    %J = J + 1000 * Xk_end(6) * Xk_end(6);
    %%J = J + 200 * Xk_end(3) * Xk_end(3);
    %%J = J + 10 * Xk_end(4) * Xk_end(4);
    %J = J + 10 * (pi / 180) * DUk * DUk;
    
    %J = J + 1750 * Xk_end(6) * Xk_end(6);
    J = J + 75 * (Xk_end(6) - YTrajectory(Xk_end(5)))^2;
    %J = J + 1750 * (Xk_end(6) - 1)^2;
    J = J + 500 * (Xk_end(3) - PsiTrajectory(Xk_end(5)))^2;
    %J = J + 500 * Xk_end(3) * Xk_end(3);
    J = J + 150 * (pi / 180) * DUk * DUk;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], 6);
    w = {w{:}, Xk};
    lbw = [lbw; -inf; -inf; -inf; -inf; -inf; -inf];
    ubw = [ubw;  inf;  inf;  inf;  inf;  inf;  inf];
    w0 = [w0; 5; 0; 0; 0; 0; 0];

    % Add equality constraint for state
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0; 0];
end

% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);
%solver = qpsol('solver', 'qpoases', prob);

sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

f_opt       = full(sol.f);
u_opt       = full(sol.x);
u_opt       = u_opt(9:10:end);

u = u_opt(1); % TODO: solve some optimization problem to compute this

end

