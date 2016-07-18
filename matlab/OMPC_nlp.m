function [ u ] = OMPC_nlp( xHat, last_U, Ts, RK4, flag )
%OMPC calculates optimal contols based on a state estimate xHat.
% INPUTS:
%   xHat: estimated state of the plant
%   Ts  : sampling time in seconds
%
% OUTPUTS:
%  u    : control inputs for the plant at step k+1.

import casadi.*;

% Np = 7;    %Prediction horizon
% Nc = 3;     %Control horizon
Np = flag.Np;
Nc = flag.Nc;

X_0 = xHat;
U_init = last_U(1);
u_opt_prev = last_U;
u_opt_prev = [u_opt_prev; u_opt_prev(end); u_opt_prev(end)];


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

du_min = -1.5;
du_max = 1.5;
u_min = -10;
u_max = 10;

% Formulate the NLP
Xk = X0;
for k=0:Np
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    DUk = MX.sym(['DU_' num2str(k)]);
    w = {w{:}, Uk, DUk};
    
    if k < Nc
        lbw = [lbw; u_min; du_min];
        ubw = [ubw;  u_max;  du_max];
    else
        lbw = [lbw; u_min; 0];
        ubw = [ubw;  u_max; 0];
    end
    w0 = [w0;  u_opt_prev(k+2);  (u_opt_prev(k+2) - u_opt_prev(k+1))];

    % Add action constraint
    g = {g{:}, Uk - (U_prev + DUk)};
    lbg = [lbg; 0];
    ubg = [ubg; 0];
    U_prev = Uk;

    % Integrate till the end of the interval
    Xk_end = RK4.fcn(Xk, Uk);
    %Xk_end = Fk.xf;
    %J=J+Fk.qf;
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

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; 0; 0; 0; 0; 0; 0];
    ubg = [ubg; 0; 0; 0; 0; 0; 0];
end

% Create an NLP solver
opts = struct();
opts.ipopt.max_iter = 1000;
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob, opts);

sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw, 'lbg', lbg, 'ubg', ubg);

%f_opt       = full(sol.f);
u_opt       = full(sol.x);

u_opt       = u_opt(8:8:end);

u = u_opt;
end

