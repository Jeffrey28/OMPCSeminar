function [ xHat, P ] = estimator( xHat, P, y, u,  Ts )
%ESTIMATOR returns state estimate and covariance for the current time step
%   Example Model assumputions (Probabalistic Robotics, pp. 34):
%     x    = A * x_ + B * u + r
%     y    = C * x + q
%   Example Estimator (Kalman Filter):
%     xBar = A * xHat + B * u
%     PBar = A * P * A' + R
%     K    = PBar * C' * inv( C * PBar * C' + Q)
%     xHat = xBar + K * (y - C * xBar)
%     P    = (eye(N) - K * C) * PBar
%
% INPUTS:
%   xHat: previous plant state estimate
%   P   : covariance of previous plant state estimate
%   y   : current measurments
%   u   : current plant controls
%   Ts  : sampling time in seconds
%
% OUTPUTS:
%   xHat: current plant state estimate
%   P   : covariance of current plant state estimate

% vla

end

