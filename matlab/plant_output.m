function [ y ] = plant_output( x, u, Ts, k, pars)
%PLANT_OUTPUT returns the output of the model at the current time step k.
%   (i.e. a subset of it's states with additive measurment noise)
% INPUTS:
%   x   : current state of the plant
%   u   : input
%   Ts  : sampling time in seconds
%   k   : current time step
%   pars: parameters of the plant model
%
% OUTPUTS:
%  y    : output of the plant at step k.

y = x;

% % Get ideal measurements
% yExact = zeros(2,1);
% yExact(1) = x(3); % p
% yExact(2) = x(6); % Y
% 
% % Add noise to ideal measurement
% mu1 = 0;
% std1 = 0.5;
% mu2 = 0;
% std2 = 0.5;
% measurementNoise = [normrnd(mu1, std1); normrnd(mu2, std2)];
% y = yExact + measurementNoise;

end

