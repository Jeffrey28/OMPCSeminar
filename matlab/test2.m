clear all;

pars = plant_init();
x0 = pars.x0;
Ts = pars.Ts;
u = -0.17;
k = 0;

res_x = zeros(10, 1);
res_Dx = zeros(10, 1);
res_y = zeros(10, 1);
res_Dy = zeros(10, 1);

for i=1:2
xNext = plant_step(x0,u,Ts,k,pars);
x0 = xNext;
% %disp([xNext(1),xNext(2),xNext(3),xNext(4),xNext(5),xNext(6),xNext(7),xNext(8)]);
% res_x(i) = xNext(1);
% res_y(i) = xNext(3);
% res_Dx(i) = xNext(2);
% res_Dy(i) = xNext(4);
k=k+1;
y = plant_output(x0, u, Ts, k, pars);
end
