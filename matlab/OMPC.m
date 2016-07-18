function [ u ] = OMPC( xHat, prev_u, Ts, RK4, flag )
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

if flag.opt == 1
    disp('Solving MPC-problem as NLP');
    u = OMPC_nlp(xHat, prev_u, Ts, RK4, flag);
elseif flag.opt == 2
    disp('Solving MPC-problem as LTV');
    u = OMPC_ltv(xHat, prev_u, Ts, RK4, flag);
else
    disp('ERROR: Unknown controller in OMPC!');
end

end

