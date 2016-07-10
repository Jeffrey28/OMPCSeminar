pars = plant_init();
x0 = [10;0;0;0];
u = 200;

res = jacobian_x(x0,u,pars);