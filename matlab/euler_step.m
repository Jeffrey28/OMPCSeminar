function [ xNext ] = euler_step( xPrevious, u, xSet, uSet, h, pars )
%UNTITLED3 Summary of this function goes here
%   This is the LTV Euler integration step to get dicretized system
%   dynamics (The return value can be used in the LTV contoller)
% INPUTS:
%   xPrevious  : current state of the plant
%   u   : input
%   xSet: The wanted state set_point where to linearize
%   uSet: The wanted input set_point where to linearize
%   Ts  : sampling time in seconds
%   h   : Euler integration step width
%   pars: parameters of the plant model
%
% OUTPUTS:
%  xNext: next state of discretized system

%% UNTESTED !! %%

% Initialization of working variables
Dx = xPrevious(1);
Dy = xPrevious(2);
Dp = xPrevious(3);

I = eye(8);
f = @(t, x) dgl(t, x, [Dx, Dy, Dp], pars, u);
A = I + jacobian_x(xSet,uSet,pars);
B = h * jacobian_u(xSet,uSet,pars);
C = xSet + h*f - (I + h*jacobian_x(xSet,uSet,pars))*xSet-h*jacobian_u(xSet, uSet, pars);

xNext = A*xPrevious+B*u+C;

end

