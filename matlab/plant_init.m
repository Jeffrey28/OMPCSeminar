function [ pars ] = plant_init()
%PLANT_INIT returns all parameters of the model in the struc pars
%   execute once, at the beginning of the script.

pars.Ts = 0; % time step in seconds TODO: Choose a meaningful value
pars.x0 = []; % some initial state of the system TODO: Choose wisely :-)
pars.m = 2000; % Car's mass
pars.a = 1; % Distance of front wheels to gravity center
pars.b = 1; % Distance of rear wheels to gravity center

% test commit

end

