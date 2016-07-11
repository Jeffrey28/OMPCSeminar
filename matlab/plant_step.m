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
x = xPrevious(1);
Dx = xPrevious(2);
y = xPrevious(3);
Dy = xPrevious(4);
p = xPrevious(5);
Dp = xPrevious(6);
X = xPrevious(7);
Y = xPrevious(8);

% Integration step of system plant
tspan = [k*Ts,(k+1)*Ts];
init = [x; Dx; y; Dy; p; Dp; X; Y];
[tout, Yout] = ode45(@(t, x) dgl(t, x, [Dx, Dy, Dp], pars, u), tspan, init);

% Compute the next state of the plant for time step (k+1)*Ts
x_values = Yout(:, 1);
xNext(1) = x_values(end);
Dx_values = Yout(:, 2);
xNext(2) = Dx_values(end);
y_values = Yout(:, 3); 
xNext(3) = y_values(end);
Dy_values = Yout(:, 4);
xNext(4) = Dy_values(end);
p_values = Yout(:, 5);
xNext(5) = p_values(end);
Dp_values = Yout(:, 6);
xNext(6) = Dp_values(end);
X_values = Yout(:, 7);
xNext(7) = X_values(end);
Y_values = Yout(:, 8);
xNext(8) = Y_values(end);

end