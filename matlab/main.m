clear all;

%% initialize
pars = plant_init();
k=0;
Ts = pars.Ts;
x0 = pars.x0;

xHat = []; % TODO: Initial state guess (don't cheat and use x0 ;-P )

while true
    %% compute next controls
    u = OMPC(xHat, Ts);
    
    %% update to time step k+1
    xNext = plant_step(x, u, Ts, k, pars); % simulate next timestep
    k = k+1;
    x = xNext;

    %% simulate plant
    y = plant_output(x, u, Ts, k, pars); % current measurement

    %% estimate next plant state
    [xHat, P] = estimator(xHat, P, y, u, Ts); % update state estimate
end