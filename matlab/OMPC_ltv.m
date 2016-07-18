function [ u ] = OMPC_ltv( xHat, prev_u, Ts, RK4, flag )
%OMPC calculates optimal contols based on a state estimate xHat.
% INPUTS:
%   xHat: estimated state of the plant
%   Ts  : sampling time in seconds
%
% OUTPUTS:
%  u    : control inputs for the plant at step k+1.

import casadi.*;

% Np = 25;    %Prediction horizon
% Nc = 25;    %Control horizon
Np = flag.Np;
Nc = flag.Nc;

X_0 = xHat;
U_init = prev_u(1);
u_opt_prev = prev_u;
u_opt_prev = [u_opt_prev; u_opt_prev(end); u_opt_prev(end)];

% For state vector
A = RK4.JacX_s(X_0, U_init);
B = RK4.JacU_s(X_0, U_init);

% For spip angle
% Front angle
C_front = RK4.JacX_df(X_0, U_init);
D_front = RK4.JacU_df(X_0, U_init);
% Rear angle
C_rear = RK4.JacX_dr(X_0, U_init);
D_rear = RK4.JacU_dr(X_0, U_init);
%offset = RK4.fcn(X_0, U_init);


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

% Allowed ranges
alpha_min = -2.2 * (pi / 180);
alpha_max = 2.2 * (pi / 180);
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
    alphaf_k = MX.sym(['alphaf_' num2str(k)]);
    alphar_k = MX.sym(['alphar_' num2str(k)]);
    w = {w{:}, Uk, DUk, alphaf_k, alphar_k};
    
    if k < Nc
        lbw = [lbw; u_min; du_min; alpha_min; alpha_min];
        ubw = [ubw;  u_max;  du_max; alpha_max; alpha_max];
    else
        lbw = [lbw; u_min; 0; alpha_min; alpha_min];
        ubw = [ubw;  u_max; 0; alpha_max; alpha_max];
    end
    w0 = [w0;  u_opt_prev(k+2);  (u_opt_prev(k+2) - u_opt_prev(k+1)); 0; 0];

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

u = u_opt; % TODO: solve some optimization problem to compute this

end

