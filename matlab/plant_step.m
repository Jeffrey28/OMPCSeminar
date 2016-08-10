function [ xNext ] = plant_step( xPrevious, u, Ts, k, pars )
%PLANT_STEP returns the next state of the plant.
%   This is the simulation function of the plant.
% INPUTS:
%   xPrevious  : current state of the plant
%   u   : input
%   Ts  : sampling time in seconds
%   k   : current time step
%   pars: parameters of the plant model
%
% OUTPUTS:
%  xNext: state of the plant at step k+1

% if nargin < 6
%     integrator = pars.integrator.type;
% end

if strcmp(pars.integrator.type, 'RK4') == 1
    % RK4 integrator
    Mrk = pars.integrator.RK4.M;
    X = xPrevious;
    DT = Ts / Mrk;

    for i=1:Mrk
        k1 = dgl(0, X, pars, u);
        k2 = dgl(0, X + DT * k1 / 2, pars, u);
        k3 = dgl(0, X + DT * k2 / 2, pars, u);
        k4 = dgl(0, X + DT * k3, pars, u);
        X   = X + (DT / 6) * (k1 + 2*k2 + 2*k3 + k4);
    end

    xNext = X;
else
    % Integration step of system plant
    tspan = [k*Ts, (k+1)*Ts];
    [tout, Yout] = ode45(@(t, x) dgl(t, x, pars, u), tspan, xPrevious);

    % Compute the next state of the plant for time step (k+1)*Ts
    xNext = Yout(end, :);
    xNext = xNext';
end

end