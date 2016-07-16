function [ result ] = trajectory( X )
% Takes a position X and returns a trajectory for Y and psi

d_y1 = 4.05;
d_y2 = 5.7;
d_x1 = 25;
d_x2 = 21.95;
z1 = (2.4 / 25) * (X - 27.19) - 1.2;
z2 = (2.4 / 21.95) * (X - 56.46) - 1.2;

result = zeros(1, 2);
% Y
result(1, 1) = atan(d_y1 * (1 / cosh(z1))^2 * (1.2 / d_x1) - d_y2 * (1 / cosh(z2))^2 * (1.2 / d_x2));
% psi
result(1, 2) = (d_y1 / 2)* (1 + tanh(z1)) - (d_y2 / 2) * (1 + tanh(z2));
end

