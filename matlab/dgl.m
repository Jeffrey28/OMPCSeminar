function [ result ] = dgl( t, x, pars, u )
%DGL returns the solution for the next plant state [x,Dx,y,Dy,p,Dp,X,Y]
% INPUTS:
%   t  : time
%   x  : state vector for calculation within right side of plant ODE
%   xPrevious: old state of the plant system
%   pars: parameters of the plant system
%   u  : input for plant system 
%
% OUTPUTS:
%  result: left side of plant ODE

% Initialization of car's geometrics
m = pars.m;
g = pars.g;
a = pars.a;
b = pars.b;
I = pars.I;
r = pars.r;
omega = pars.omega;
mu = pars.mu;

% Nominal forces F_z on front (_f) and rear (_r) wheels
F_z_f = (b*m*g) / (2000 * (a+b));
F_z_r = (a*m*g) / (2000 * (a+b));

delta_f = u; % Front axis steering angle in degrees
delta_r = 0; % Rear axis steering angle in degrees

% Wheel's equations update for x-direction (_x) and y-direction (_y)
v_y_f = x(2) + a * x(4);
v_y_r = x(2) - b * x(4);
v_x_f = x(1);
v_x_r = x(1);

v_l_f = v_y_f * sind(delta_f) + v_x_f * cosd(delta_f);
v_l_r = v_y_r * sind(delta_r) + v_x_r * cosd(delta_r);
v_c_f = v_y_f * cosd(delta_f) - v_x_f * sind(delta_f);
v_c_r = v_y_r * cosd(delta_r) - v_x_r * sind(delta_r);

% Tire models by Pacejka
alpha_f = atan(v_c_f / v_l_f);
alpha_r = atan(v_c_r / v_l_r);
v_wheel = r * omega;

if v_l_f > v_wheel
    slip_f = (v_wheel / v_l_f) - 1;
else
    slip_f = 1 - (v_l_f / v_wheel);
end

if v_l_r > v_wheel
    slip_r = (v_wheel / v_l_r) - 1;
else
    slip_r = 1 - (v_l_r / v_wheel);
end
%slip_f = 1 - (v_l_f / v_wheel);    % Drive case
%slip_r = 1 - (v_l_r / v_wheel);    % Drive case
%slip_f = 0;    % Zero slip
%slip_r = 0;    % Zero slip

[F_l_f, F_c_f] = Pacejka(alpha_f, slip_f, mu, F_z_f);
[F_l_r, F_c_r] = Pacejka(alpha_r, slip_r, mu, F_z_r);
% [F_l_f, F_c_f] = PacejkaD(atand(v_c_f / v_l_f), slip_f, 0, F_z_f);
% [F_l_r, F_c_r] = PacejkaD(atand(v_c_r / v_l_r), slip_r, 0, F_z_r);

% Complete force formulas
F_x_f = F_l_f * cosd(delta_f) - F_c_f * sind(delta_f);
F_x_r = F_l_r * cosd(delta_r) - F_c_r * sind(delta_r);
F_y_f = F_l_f * sind(delta_f) + F_c_f * cosd(delta_f);
F_y_r = F_l_r * sind(delta_r) + F_c_r * cosd(delta_r);

% Physical system dynamics as 1.st order ODE
result = zeros(6, 1);
result(1) = x(2) * x(4) + 2 * F_x_f * (1 / m) + 2 * F_x_r * (1 / m); % res(1)=x''
result(2) = -x(1) * x(4) + 2 * F_y_f * (1 / m) + 2 * F_y_r * (1 / m); % res(2)=y''
result(3) = x(4); % res(3)=p'
result(4) = 2 * a * F_y_f * (1 / I) - 2 * b * F_y_r * (1 / I); % res(4)=p''
result(5) = x(1) * cos(x(3)) - x(2) * sin(x(3)); % res(5)=X'
result(6) = x(1) * sin(x(3)) + x(2) * cos(x(3)); % res(6)=Y'

end

