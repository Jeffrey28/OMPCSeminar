addpath('C:\Program Files\MATLAB\R2016a\casadi-matlabR2014b-v3.0.0')
import casadi.*

syms x(t) y(t) X(t) Y(t) p(t);

m = 1000;
I = 300;
a = 1;
b = 1;
F_x_f = 0;
F_x_r = 0;
F_y_f = 0;
F_y_r = 0;

eq1 = m*diff(x, 2) == m*diff(y)*diff(p)+2*F_x_f+2*F_x_r;
eq2 = m*diff(y, 2) == -m*diff(x)*diff(p)+2*F_y_f+2*F_y_r;
eq3 = I*diff(p, 2) == -m*diff(x)*diff(p)+2*F_y_f+2*F_y_r;
eq4 = diff(X) == diff(x)*cos(p)-diff(y)*sin(p);
eq5 = diff(Y) == diff(x)*sin(p)+diff(y)*cos(p);

[rightSideOfODE, substitutions] = odeToVectorField(eq1, eq2, eq3, eq4, eq5);
func = matlabFunction(rightSideOfODE, 'vars', {'t','Y'});

t_horizon = [0, 100];
init_values = [0;0;1;0;0;0;0;0]; % p,Dp,x,Dx,y,Dy,X,Y (just drives in x-dir.)

[tout, Yout] = ode45(func, t_horizon, init_values);

p = Yout(:, 1)';
Dp = Yout(:, 2)';
x = Yout(:, 3)';
Dx = Yout(:, 4)';
y = Yout(:, 5)';
Dy = Yout(:, 6)';
X = Yout(:, 7)';
Y = Yout(:, 8)';

plot(tout, p);
%plot(tout, Dp);

% [tout, Yout] = ode45(func, [0, 30], [init1, init2]);
% 
% plot(tout, Yout);                