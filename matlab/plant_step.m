function [ xNext ] = plant_step( xPrevious, u, Ts, k, pars )
%PLANT_STEP returns the next state of the plant.
%   This is the simulation function of the plant.
% INPUTS:
%   xPrevious  : current state of the plant
%   u   : input
%   Ts  : sampling time in seconds
%   k   : current time step
%   pars: parameters of the plant model
%
% OUTPUTS:
%  xNext: state of the plant at step k+1

% Initialization of working variables
Dx = xPrevious(1);
Dy = xPrevious(2);
p = xPrevious(3);
Dp = xPrevious(4);
X = xPrevious(5);
Y = xPrevious(6);

% Integration step of system plant
tspan = [k*Ts,(k+1)*Ts];
init = [Dx; Dy; p; Dp; X; Y];
[tout, Yout] = ode45(@(t, x) dgl(t, x, pars, u), tspan, init);

% Compute the next state of the plant for time step (k+1)*Ts
Dx_values = Yout(:, 1);
xNext(1) = Dx_values(end);
Dy_values = Yout(:, 2);
xNext(2) = Dy_values(end);
p_values = Yout(:, 3);
xNext(3) = p_values(end);
Dp_values = Yout(:, 4);
xNext(4) = Dp_values(end);
X_values = Yout(:, 5);
xNext(5) = X_values(end);
Y_values = Yout(:, 6);
xNext(6) = Y_values(end);

end