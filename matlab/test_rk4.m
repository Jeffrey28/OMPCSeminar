clear all;

% 
% Physical equations
pars = plant_init();
x0 = pars.x0;
Ts = pars.Ts;

maxRK4Iter = 20;
maxIter = 1000;

resOde45 = zeros(6, 5, maxIter);
resRK4 = zeros(6, 5, maxIter, maxRK4Iter);

xNext = zeros(6, 5);
for k=1:5
    xNext(:, k) = x0;
end

for k=0:maxIter-1
    if mod(k,10) == 0
        disp(k);
    end
    %u=-10;
    u = random('unif', -10, 10);

    for muk=1:5
        pars.mu = -0.1 + 0.2 * muk;
        
        % ODE45 solution
        pars.integrator.type = 'ode45';
        ode45Sol = plant_step(xNext(:, muk), u, Ts, k, pars);
        resOde45(:, muk, k+1) = ode45Sol;

        % RK4 solutions
        pars.integrator.type = 'RK4';
        for i=1:maxRK4Iter
            pars.integrator.RK4.M = i;

            rk4Sol = plant_step(xNext(:, muk), u, Ts, k, pars);
            resRK4(:, muk, k+1, i) = rk4Sol;
        end

        % Take ode45 solution as next state
        xNext(:, muk) = ode45Sol;
    end
end

%% Calculate rms errors over all iterations and mus' for each RK4 maxIter

rms = zeros(maxRK4Iter, 5);

for i=1:maxRK4Iter % for each max-RK4-iter
    for muk=1:5 % run over all mus'
        for k=1:maxIter % run over all iterations
            % Sum up all squared errors
            rms(i, muk) = rms(i, muk) + (resOde45(:, muk, k) - resRK4(:, muk, k, i))' * (resOde45(:, muk, k) - resRK4(:, muk, k, i));
        end
        
        % Calculate RMS from sum of squred errors
        rms(i, muk) = sqrt(rms(i, muk) / maxIter);
    end
end


%% Plot result
startIter = 1;
muks = startIter:maxRK4Iter;

figure;
hold on;
plot(muks, rms(muks, 1));
plot(muks, rms(muks, 2));
plot(muks, rms(muks, 3));
plot(muks, rms(muks, 4));
plot(muks, rms(muks, 5));
hold off;

xlabel('RK4 iterations'); ylabel('RMS Error');
title('RK4 vs. ode45 Evaluation');
legend('mu: 0.1','mu: 0.3','mu: 0.5','mu: 0.7','mu: 0.9')


