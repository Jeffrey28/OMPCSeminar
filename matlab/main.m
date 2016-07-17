clear all;

%% initialize
pars = plant_init();
k=0;
Ts = pars.Ts;
x0 = pars.x0;
flag = 1;

xHat = [5;0;0;0;0;0]; % TODO: Initial state guess (don't cheat and use x0 ;-P )
u = 0;
x = [5;0;0;0;0;0];
P = 0;

u_opt = [];
%res = zeros(6, 100);

figure;

while k < 500
    %% compute next controls
    u = OMPC(xHat, u, Ts, flag);
    u_opt(k+1) = u;
    disp(k);
    
    %% update to time step k+1
    xNext = plant_step(x, u, Ts, k, pars); % simulate next timestep
    k = k+1;
    x = xNext;
    res(k, :) = xNext;
    
    %% simulate plant
    y = plant_output(x, u, Ts, k, pars); % current measurement

    %% estimate next plant state
    [xHat, P] = estimator(xHat, P, y, u, Ts); % update state estimate
    
    %% Plot results
    plot_result( u_opt, res );
end