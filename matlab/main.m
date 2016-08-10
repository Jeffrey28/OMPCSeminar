clear all;

%% initialize
pars = plant_init();
k = 0;
Ts = pars.Ts;
mu = pars.mu;

%flag.opt = 0;
flag.Np = 7;
flag.Nc = 3;

xHat = pars.x0; % TODO: Initial state guess (don't cheat and use x0 ;-P )
RK4 = RK4_fcn(20, Ts, mu);
u_opt = zeros(flag.Np, 1);
x = pars.x0;
P = 0;


%% NLP Optimization
flag.opt = 1;
figure;

tic

upper = 125 / (xHat(1)*Ts);

while k < upper
    %% compute next controls
    u_opt = OMPC(xHat, u_opt, Ts, RK4, flag);
    u = u_opt(1);
    u_opt_nlp(k+1) = u;
    disp(k);
    
    toc
    
    %% update to time step k+1
    xNext = plant_step(x, u, Ts, k, pars); % simulate next timestep
    k = k+1;
    x = xNext;
    res_nlp(k, :) = xNext;
    
    %% simulate plant
    y = plant_output(x, u, Ts, k, pars); % current measurement

    %% estimate next plant state
    [xHat, P] = estimator(xHat, P, y, u, Ts); % update state estimate
    
    %% Plot results
    plot_result( u_opt_nlp, res_nlp );
end

time_nlp = toc;


%% LTV Optimization
k = 0;
flag.opt = 2;
xHat = pars.x0;
u = 0;
u_opt = zeros(flag.Np, 1);
x = pars.x0;
P = 0;

figure;

tic

while k < upper
    %% compute next controls
    u_opt = OMPC(xHat, u_opt, Ts, RK4, flag);
    u = u_opt(1);
    u_opt_ltv(k+1) = u;
    disp(k);
    
    toc
    
    %% update to time step k+1
    xNext = plant_step(x, u, Ts, k, pars); % simulate next timestep
    k = k+1;
    x = xNext;
    res_ltv(k, :) = xNext;
    
    %% simulate plant
    y = plant_output(x, u, Ts, k, pars); % current measurement

    %% estimate next plant state
    [xHat, P] = estimator(xHat, P, y, u, Ts); % update state estimate
    
    %% Plot results
    plot_result( u_opt_ltv, res_ltv );
end

time_ltv = toc;
