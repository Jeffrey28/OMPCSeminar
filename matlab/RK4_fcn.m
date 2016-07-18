function [ RK4 ] = RK4_fcn( M_RK, Ts )

import casadi.*;

tic

nx = 6;     %State dimension
nu = 1;     %Control dimension

% Parameter
a = 1;
b = 1;
m = 2050;
I = 3344;
g = 9.81;

u = SX.sym('u', nu);
xi = SX.sym('xi', nx);
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
vl_r = vx_r;                            % rear steering angle = 0 --> simplification
vc_r = vy_r;
% vl_f = vy_f * sin(delta_f) + vx_f * cos(delta_f);
% vc_f = vy_f * cos(delta_f) - vx_f * sin(delta_f);
% vl_r = vy_r * sin(delta_r) + vx_r * cos(delta_r);
% vc_r = vy_r * cos(delta_r) - vx_r * sin(delta_r);

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
Fx_r = Fl_r;                            % rear steering angle = 0 --> simplification
Fy_r = Fc_r;                            % rear steering angle = 0 --> simplification
%Fx_f = Fl_f * cos(delta_f) - Fc_f * sin(delta_f);
%Fy_f = Fl_f * sin(delta_f) + Fc_f * cos(delta_f);
%Fx_r = Fl_r * cos(delta_r) - Fc_r * sin(delta_r);
%Fy_r = Fl_r * sin(delta_r) + Fc_r * cos(delta_r);

xdot = [xi(2) * xi(4) + (2 / m) * (Fx_f + Fx_r);
        -xi(1) * xi(4) + (2 / m) * (Fy_f + Fy_r);
        xi(4);
        (2 / I) * (a * Fy_f - b * Fy_r);
        xi(1) * cos(xi(3)) - xi(2) * sin(xi(3));
        xi(1) * sin(xi(3)) + xi(2) * cos(xi(3))];

%l = ...        % Maybe add a cost function to RK4

f = Function('f', {xi, u}, {xdot});

% RK4 integrator
U   = MX.sym('U');
X0  = MX.sym('X0', nx);
X = X0;
DT = Ts / M_RK;

for i=1:M_RK
    k1 = f(X, U);
    k2 = f(X + DT * k1 / 2, U);
    k3 = f(X + DT * k2 / 2, U);
    k4 = f(X + DT * k3, U);
    X   = X + (DT / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

RK4.fcn = Function('RK4', {X0, U}, {X});


RK4.JacX_s = Function('jacobiXF_state', {X0, U}, {jacobian(X, X0)});
RK4.JacU_s = Function('jacobiUF_state', {X0, U}, {jacobian(X, U)});

% For spip angle
% Front angle
RK4.JacX_df = Function('jacobiFX_df', {xi, u}, {jacobian(alpha_f, xi)});
RK4.JacU_df = Function('jacobiUX_df', {xi, u}, {jacobian(alpha_f, u)});
% Rear angle
RK4.JacX_dr = Function('jacobiFX_dr', {xi, u}, {jacobian(alpha_r, xi)});
RK4.JacU_dr = Function('jacobiUX_dr', {xi, u}, {jacobian(alpha_r, u)});

end

