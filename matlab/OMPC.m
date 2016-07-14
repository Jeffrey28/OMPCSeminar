function [ u ] = OMPC( xHat, Ts, flag )
%OMPC calculates optimal contols based on a state estimate xHat.
% INPUTS:
%   xHat: estimated state of the plant
%   Ts  : sampling time in seconds
%   flag: 1: NLMPC (non-linear)
%         2: LTVMPC (linear time variant)
%         3: LTIMPC (linear time invariant)
%
% OUTPUTS:
%  u    : control inputs for the plant at step k+1.

u = []; % TODO: solve some optimization problem to compute this
end

