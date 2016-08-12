function [ alpha_f, alpha_r ] = calc_slip_angle( x, u, pars )
%CALC_SLIP_ANGLE Summary of this function goes here
%   Detailed explanation goes here

% Initialization of car's geometrics
a = pars.a;
b = pars.b;

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

% Tire slip angles
alpha_f = atan(v_c_f / v_l_f);
alpha_r = atan(v_c_r / v_l_r);

end

