function [ Y ] = YTrajectory( X )
% Takes a position X and returns a trajectory for Y

d_y1 = 4.05;
d_y2 = 5.7;
z1 = (2.4 / 25) * (X - 27.19) - 1.2;
z2 = (2.4 / 21.95) * (X - 56.46) - 1.2;

% psi
Y = (d_y1 / 2)* (1 + tanh(z1)) - (d_y2 / 2) * (1 + tanh(z2));

end

