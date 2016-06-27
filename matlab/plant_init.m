function [ pars ] = plant_init()
%PLANT_INIT returns all parameters of the model in the struc pars
%   execute once, at the beginning of the script.

pars.Ts = 0; % time step in seconds TODO: Choose a meaningful value
pars.x0 = []; % some initial state of the system TODO: Choose wisely :-)
pars.m = 1000; % Car's mass [kg]
pars.a = 1; % Distance of front wheels to gravity center [m]
pars.b = 1; % Distance of rear wheels to gravity center [m]
pars.g = 9.81; % Gravitational constant [N/kg]
pars.I = 1000; % Car inertia [m^2*kg]
pars.mu = 0.5; % Friction coefficient [1]

% test commit

end

