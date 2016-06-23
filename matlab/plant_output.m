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

yExact = 0; % TODO: compute this
measurementNoise = 0; % TODO: choose appropriately
y = yExact + measurementNoise; % add noise to outputs

end

