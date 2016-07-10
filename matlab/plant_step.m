function [ xNext ] = plant_step( xOld, u, Ts, k, pars )
%PLANT_STEP returns the next state of the plant.
%   This is the simulation function of the plant.
% INPUTS:
%   x   : current state of the plant
%   u   : input
%   Ts  : sampling time in seconds
%   k   : current time step
%   pars: parameters of the plant model
%
% OUTPUTS:
%  xNext: state of the plant at step k+1

% Initialization of working variables
x_value = xOld(1);
Dx_value = xOld(2);
y_value = xOld(3);
Dy_value = xOld(4);
p_value = xOld(5);
Dp_value = xOld(6);
X_value = xOld(7);
Y_value = xOld(8);

% Initialization of car's geometrics
m = pars.m; % 1000?
g = pars.g; % 9.81?
a = pars.a; % 1?
b = pars.b; % 1?

% Nominal forces on front and rear wheels
F_z_f = (b*m*g) / (2*(a+b));
F_z_r = (a*m*g) / (2*(a+b));

delta_f = u; % Front axis steering angle in degrees
delta_r = 0; % Rear axis steering angle in degrees

% Wheel's equations update
v_y_f = Dy_value+a*Dp_value;
v_y_r = Dy_value-b*Dp_value;
v_x_f = Dx_value;
v_x_r = Dx_value;

v_l_f = v_y_f*sind(delta_f)+v_x_f*cosd(delta_f);
v_l_r = v_y_r*sind(delta_r)+v_x_r*cosd(delta_r);
v_c_f = v_y_f*cosd(delta_f)-v_x_f*sind(delta_f);
v_c_r = v_y_r*cosd(delta_r)-v_x_r*sind(delta_r);

% Tire models by Pacejca
% Web page: http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf

% Lateral / front
a1_c = -22.1;
a2_c = 1011;
a3_c = 1078;
a4_c = 1.82;
a5_c = 0.208;
a6_c = 0.000;
a7_c = -0.354;
a8_c = 0.707;
a9_c = 0.028;
a10_c = 0.000;
a11_c = 14.8;
C_c_f = 1.3;
D_c_f = a1_c*F_z_f^2+a2_c*F_z_f;
B_c_f = (a3_c*sind(a4_c*atand(a5_c*F_z_f)))/(C_c_f*D_c_f);
E_c_f = a6_c*F_z_f^2+a7_c*F_z_f+a8_c;
Sh_c_f = a9_c*delta_f;
Sv_c_f = (a10_c*F_z_f^2+a11_c*F_z_f)*delta_f;
slip_c_f = atand(v_c_f/v_l_f);
phi_c_f = (1-E_c_f)*(slip_c_f+Sh_c_f)+atand(B_c_f*(slip_c_f+Sh_c_f))*(E_c_f/B_c_f);
F_c_f = D_c_f*sind(C_c_f*atand(B_c_f*phi_c_f))+Sv_c_f;

% Lateral / rear
C_c_r = 1.3;
D_c_r = a1_c*F_z_r^2+a2_c*F_z_r;
B_c_r = (a3_c*sind(a4_c*atand(a5_c*F_z_r)))/(C_c_r*D_c_r);
E_c_r = a6_c*F_z_r^2+a7_c*F_z_r+a8_c;
Sh_c_r = a9_c*delta_r;
Sv_c_r = (a10_c*F_z_r^2+a11_c*F_z_r)*delta_r;
slip_c_r = atand(v_c_r/v_l_r);
phi_c_r = (1-E_c_r)*(slip_c_r+Sh_c_r)+atand(B_c_r*(slip_c_r+Sh_c_r))*(E_c_r/B_c_r);
F_c_r = D_c_r*sind(C_c_r*atand(B_c_r*phi_c_r))+Sv_c_r;

% Longitudinal / front
a1_l = 21.3;
a2_l = 1144;
a3_l = 49.6;
a4_l = 226;
a5_l = 0.069;
a6_l = -0.006;
a7_l = 0.056;
a8_l = 0.486;
C_l_f = 1.65;
D_l_f = a1_l*F_z_f^2+a2_l*F_z_f;
B_l_f = (a3_l*F_z_f^2+a4_l*F_z_f)/(exp(a5_l*F_z_f)*C_l_f*D_l_f);
E_l_f = a6_l*F_z_f^2+a7_l*F_z_f+a8_l;
slip_l_f = atand(v_c_f/v_l_f);
phi_l_f = (1-E_l_f)*slip_l_f+atand(B_l_f*slip_l_f*(E_l_f/B_l_f));
F_l_f = D_l_f*sind(C_l_f*atand(B_l_f*phi_l_f));

% Longitudinal / rear
C_l_r = 1.65;
D_l_r = a1_l*F_z_r^2+a2_l*F_z_r;
B_l_r = (a3_l*F_z_r^2+a4_l*F_z_r)/(exp(a5_l*F_z_r)*C_c_r*D_l_r);
E_l_r = a6_l*F_z_r^2+a7_l*F_z_r+a8_l;
slip_l_r = atand(v_c_r/v_l_r);
phi_l_r = (1-E_l_r)*slip_l_r+atand(B_l_r*slip_l_r*(E_l_r/B_l_r));
F_l_r = D_l_r*sind(C_l_r*atand(B_l_r*phi_l_r));

% Complete force formulas
F_x_f = F_l_f*cosd(delta_f)-F_c_f*sind(delta_f);
F_x_r = F_l_r*cosd(delta_r)-F_c_r*sind(delta_r);
F_y_f = F_l_f*sind(delta_f)+F_c_f*cosd(delta_f);
F_y_r = F_l_r*sind(delta_r)+F_c_r*cosd(delta_r);

tspan = [k*Ts,(k+1)*Ts];
init = [x_value; Dx_value; y_value; Dy_value; p_value; Dp_value; X_value; Y_value]; % x,Dx,y,Dy,p,Dp,X,Y
[tout, Yout]=ode45(@(t, x) dgl(t,x,pars,F_x_f,F_x_r,F_y_f,F_y_r),tspan,init);

% Update the working variables
x_values = Yout(:, 1);
xNext(1) = x_values(end);
Dx_values = Yout(:, 2);
xNext(2) = Dx_values(end);
y_values = Yout(:, 3); 
xNext(3) = y_values(end);
Dy_values = Yout(:, 4);
xNext(4) = Dy_values(end);
p_values = Yout(:, 5);
xNext(5) = p_values(end);
Dp_values = Yout(:, 6);
xNext(6) = Dp_values(end);
X_values = Yout(:, 7);
xNext(7) = X_values(end);
Y_values = Yout(:, 8);
xNext(8) = Y_values(end);

end