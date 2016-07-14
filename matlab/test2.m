clear all;

% 
% Physical equations
pars = plant_init();
x0 = pars.x0;
Ts = pars.Ts;

k = 0;

upper = 5000;
res = zeros(upper, 6);

xNext = x0;
u=-10;

for k=0:upper-1
    xNext = plant_step(xNext, u, Ts, k, pars);
    res(k+1, :) = xNext;
    disp(xNext);
    %u = 0.999 * u;
    %y = plant_output(x0, u, Ts, k, pars);
    if k < upper/2
%         if mod(k, 10) == 0
%             u = u * -1;
%         end
    elseif k == upper/2
       u = -5;
    end
end

res_Dx = res(:, 1);
res_Dy = res(:, 2);
res_p = res(:, 3);
res_Dp = res(:, 4);
res_X = res(:, 5);
res_Y = res(:, 6);

plot(res_X, res_Y);
axis equal;

