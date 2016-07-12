clear all;

% 
% Physical equations
pars = plant_init();
x0 = pars.x0;
Ts = pars.Ts;

k = 0;

upper = 5000;
res_Dx = zeros(upper, 2);
res_Dy = zeros(upper, 2);
res_p = zeros(upper, 2);
res_Dp = zeros(upper, 2);
res_X = zeros(upper, 2);
res_Y = zeros(upper, 2);

u=-10;

for i=1:upper/2
xNext = plant_step(x0,u,Ts,k,pars);
x0 = xNext;
disp([xNext(1),xNext(2),xNext(3),xNext(4),xNext(5),xNext(6)]);
res_Dx(i,1) = xNext(1);
res_Dy(i,1) = xNext(2);
res_p(i,1) = xNext(3);
res_Dp(i,1) = xNext(4);
res_X(i,1) = xNext(5);
res_Y(i,1) = xNext(6);
k=k+1;
%u = 0.999 * u;
%y = plant_output(x0, u, Ts, k, pars);
end

u=-5;

for i=(upper/2+1):upper
xNext = plant_step(x0,u,Ts,k,pars);
x0 = xNext;
disp([xNext(1),xNext(2),xNext(3),xNext(4),xNext(5),xNext(6)]);
res_Dx(i,1) = xNext(1);
res_Dy(i,1) = xNext(2);
res_p(i,1) = xNext(3);
res_Dp(i,1) = xNext(4);
res_X(i,1) = xNext(5);
res_Y(i,1) = xNext(6);
k=k+1;
%y = plant_output(x0, u, Ts, k, pars);
end

