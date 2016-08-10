function [ pars ] = plant_init()
%PLANT_INIT returns all parameters of the model in the struc pars
%   execute once, at the beginning of the script.

%% Construction parameters
v_init = 10; % initial speed in meter per second
phi_init = 0 * pi / 180; % initial angle

vx = cos(phi_init) * v_init;
vy = sin(phi_init) * v_init;

%% General parameter
pars.Ts = 0.05; % time step in seconds TODO: Choose a meaningful value
pars.x0 = [vx; vy; phi_init; 0; 0; 0]; % Initial state vector Dx,Dy,p,Dp,X,Y

%% Plant parameter
pars.m = 2050; % Car's mass [kg]
pars.a = 1; % Distance of front wheels to gravity center [m]
pars.b = 1; % Distance of rear wheels to gravity center [m]
pars.g = 9.81; % Gravitational constant [N/kg]
pars.I = 3344; % Car inertia [m^2*kg]
pars.r = 0.25; % Wheel diameter
pars.omega = v_init / pars.r; % Wheel rotational speed
pars.mu = 0.9; % Friction coefficient

%% Integrator parameter
pars.integrator.type = 'ode45'; % 'ode45' or 'RK4'
%pars.integrator.type = 'RK4';
pars.integrator.RK4.M = 20;

end

