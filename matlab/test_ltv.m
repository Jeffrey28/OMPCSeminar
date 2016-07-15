clear all;

import casadi.*;

Ts = 0.05;

Np = 12;    %Prediction horizon
Nc = 10;     %Control horizon
nx = 6;     %State dimension
nu = 1;     %Control dimension

% Parameter
a = 1;
b = 1;
m = 2050;
I = 3344;
g = 9.81;

ui = SX.sym('u', nu);
xi = SX.sym('xi', nx);

% System dynamics
delta_f = ui * pi / 180;
delta_r = 0;

vx_f = xi(1);
vy_f = xi(2) + a * xi(4);
vx_r = xi(1);
vy_r = xi(2) - b * xi(4);

vl_f = vy_f * sin(delta_f) + vx_f * cos(delta_f);   % u corresponds to front steering angle
vc_f = vy_f * cos(delta_f) - vx_f * sin(delta_f);
vl_r = vx_r;                            % rear steering angle = 0 --> simplification
vc_r = vy_r;

alpha_f = atan(vc_f / vl_f);
alpha_r = atan(vc_r / vl_r);
s_f = 0; %TODO: Check how to implement if-else-condition
s_r = 0; %TODO: Check how to implement if-else-condition
Fz_f = b * m * g / (2000 * (a + b));
Fz_r = a * m * g / (2000 * (a + b));

% Use friction model to determine F[l/r]_[f/r]
mu = 0;
[Fl_f, Fc_f] = Pacejka(alpha_f, s_f, mu, Fz_f);
[Fl_r, Fc_r] = Pacejka(alpha_r, s_r, mu, Fz_r);

Fx_f = Fl_f * cos(delta_f) - Fc_f * sin(delta_f);   % u corresponds to front steering angle
Fy_f = Fl_f * sin(delta_f) + Fc_f * cos(delta_f);   % u corresponds to front steering angle
Fx_r = Fl_r;                            % rear steering angle = 0 --> simplification
Fy_r = Fc_r;                            % rear steering angle = 0 --> simplification

xdot = [xi(2) * xi(4) + (2 / m) * (Fx_f + Fx_r);
        -xi(1) * xi(4) + (2 / m) * (Fy_f + Fy_r);
        xi(4);
        (2 / I) * (a * Fy_f - b * Fy_r);
        xi(1) * cos(xi(3)) - xi(2) * sin(xi(3));
        xi(1) * sin(xi(3)) + xi(2) * cos(xi(3))];
    
%l = ...        % Maybe add a cost function to RK4

f = Function('f', {xi, ui}, {xdot});

% RK4 integrator
Mrk = 20;
U   = MX.sym('U');
X0  = MX.sym('X0', nx);
X = X0;
DT = Ts / Mrk;

for i=1:Mrk
    k1 = f(X, U);
    k2 = f(X + DT * k1 / 2, U);
    k3 = f(X + DT * k2 / 2, U);
    k4 = f(X + DT * k3, U);
    X   = X + (DT / 6) * (k1 + 2*k2 + 2*k3 + k4);
end

RK4 = Function('RK4', {X0, U}, {X});
% Formulate NLP
u = SX.sym('ud',Np);

x_0 = [5;0;0;0;0;0.1];
X = x_0;

J = 0;
g=[];

% Linearization
xSet = X;
uSet = 0;
offset = RK4(xSet, uSet);
JacX = jacobian(xdot, xi);
JacXF = Function('XF',{xi, ui}, {JacX});
JacU = jacobian(xdot, ui);
JacUF = Function('UF',{xi, ui}, {JacU});
for k = 1 : Np
   X = offset + JacXF(xSet, uSet) * (X - xSet) + JacUF(xSet, uSet) * (u(k) - uSet);
  % disp(JacUF(xSet, uSet));
   J = J + 175 * X(6)^2;
   %J = J + 10 * X(4)^2;
   %J = J + 20 * X(3)^2;
   if k == 1
       %J = J + (50000 * pi / 180) * (u(k)-uSet)^2;
   else
      % J = J + (50000 * pi / 180) * (u(k)-u(k-1))^2;
   end
end

uVar = MX.sym('uVar', nu);
xVar = MX.sym('xVar', nx);

%JacXi = jacobian(xdot, xi);
% JacXiF = Function('j',{xi,u},{JacXi});
% res = JacXiF([5;0;0;0;0;0],0);

% % Linearization
% xSet = [5;0;0;0;0;0]; % state where to linearize
% uSet = 0; % input where to linearize
% offset = RK4(xSet, uSet);

%jacobi_x = eye(nx) + (DT / 6) * (jacobian(k1,xVar) + 2 * jacobian(k2,xVar) + 2 * jacobian(k3,xVar) + jacobian(k4,xVar));
%jacobi_y = (DT / 6) * (jacobian(k1,uVar) + 2 * jacobian(k2,uVar) + 2 * jacobian(k3,uVar) + jacobian(k4,uVar));

%xNext = offset + jacobi_x(xSet, uSet) * (X - xSet) + jacobi_u(xSet, uSet) * (u - uSet);

% % Objective function
% J=0;
% g=[];
% X = RK4(X, u(1));
% for i = 2:Np
%     % build the cost of the NLP
%     % 1 - get x_next using RK4
%     % 2 - J = J + ...
%     X = RK4(X, u(i));
%     %J = J + 500 * X(3)^2;
%     %J = J + 75 * (X(6) - 0.5)^2;
%     J = J + 750 * X(6)^2;
%     J = J + (150 * pi / 180) * (u(i-1) - u(i))^2;
% end


% % Objective (by Dominique)
% J=0;
% g=[];
% 
% %% TODO: Linearize around x(j) and u(j-1) (Actually: x(j) and u(j))
% %xSet = RK4(X, u(1)); % The state where we linearize
% uOld = -0;
% xSet = X; % The state where we linearize
% uSet = uOld; % The input where we linearize
% fSet = RK4(xSet, uSet);
% jacobi_x = jacobian(fSet, xi);
% jacobi_u = jacobian(fSet, ui);
% 
% tmp = jacobian(RK4(xi, ui));
% 
% for j=1:Np
% % Update the actual point where we are
% xActual = X;
% if j == 1
%     uActual = uOld + ud(1);
% else
%     uActual = uActual + ud(j);
% end
% 
% % Compute the jacobians around xSet, uSet
% X = fSet + jacobi_x * (xActual - xSet) + jacobi_u * (uActual - uSet);
% 
% %J = J + 500 * X(3)^2;
% %     %J = J + 75 * (X(6) - 0.5)^2;
% %     J = J + 750 * X(6)^2;
% %     J = J + (150 * pi / 180) * (u(i-1) - u(i))^2;
% J = J + 7500 * X(6)^2;
% if j == 1
%     J = J + (150 * pi / 180) * (ud(j))^2;
% else
%     J = J + (150 * pi / 180) * (ud(j))^2;
% end
% J = J + 500 * X(3)^2;
% J = J + 10 * X(4)^2;
% end

% % Objective function
% J=0;
% g=[];
% X = RK4(X, u(1));
% for i = 2:Np
%     % build the cost of the NLP
%     % 1 - get x_next using RK4
%     % 2 - J = J + ...
%     X = RK4(X, u(i));
%     %J = J + 500 * X(3)^2;
%     %J = J + 75 * (X(6) - 0.5)^2;
%     J = J + 750 * X(6)^2;
%     J = J + (150 * pi / 180) * (u(i-1) - u(i))^2;
% end

% testN = 5000;
% 
% x_test = zeros(testN);
% y_test = zeros(testN);
% 
% Fk = x_0;
% myU = -10;
% for i=0:testN-1
%     Fk = RK4(Fk, myU);
%     Fk = full(Fk);
%     x_test(i+1) = Fk(5);
%     y_test(i+1) = Fk(6);
%     if i==testN/2
%        myU = -5; 
%     end
% end
% disp([x_test(1), y_test(1)]);
% disp([x_test(testN), y_test(testN)]);
% 
% plot(x_test, y_test); hold on;


% Terminal constraints: x_0(T)=x_1(T)=0
%g = [g; X];

% Allocate an NLP solver
nlp = struct('x', u, 'f', J, 'g', g);

% Create IPOPT solver object
options = struct;
%options.hessian_approximation = 'limited-memory';
solver = nlpsol('solver', 'ipopt', nlp);

% arg = struct;
% % YOUR CODE HERE: upper and lower bounds on x and g(x)
% arg.x0  =  zeros(1, Np);        % solution guess
% arg.lbx =  -inf;       % lower bound on x
% arg.ubx =  inf;       % upper bound on x
% % arg.lbg =  [];       % lower bound on g
% % arg.ubg =  [];       % upper bound on g

w0 = [];
lbw = [];
ubw = [];

for i=1:Np
    w0 = [w0, 5];
    lbw = [lbw, -10];
    ubw = [ubw, 10];
end

%% Solve the NLP
res = solver('x0', w0, 'lbx', lbw, 'ubx', ubw);

f_opt       = full(res.f);
u_opt       = full(res.x);

xNext = x_0;
pars = plant_init();

for k=0:Np-1
    xNext = plant_step(xNext, u_opt(k+1), Ts, k, pars);
    res1(k+1, :) = xNext;
    %disp(xNext);
end

res_Dx = res1(:, 1);
res_Dy = res1(:, 2);
res_p = res1(:, 3);
res_Dp = res1(:, 4);
res_X = res1(:, 5);
res_Y = res1(:, 6);


% Plot results
plot(u_opt); hold on;
plot(res_X, res_Y); hold on;
axis equal;
