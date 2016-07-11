function [ result ] = dgl( t, x, xPrevious, pars, u )
%DGL returns the solution for the next plant state [x,Dx,y,Dy,p,Dp,X,Y]
% INPUTS:
%   t  : time
%   x  : state vector for calculation within right side of plant ODE
%   xPrevious: old state of the plant system
%   pars: parameters of the plant system
%   u  : input for plant system 
%
% OUTPUTS:
%  result: left side of plant ODE

Dx_value = xPrevious(1);
Dy_value = xPrevious(2);
Dp_value = xPrevious(3);

% Initialization of car's geometrics
m = pars.m;
g = pars.g;
a = pars.a;
b = pars.b;
I = pars.I;

% Nominal forces F_z on front (_f) and rear (_r) wheels
F_z_f = (b*m*g) / (2*(a+b));
F_z_r = (a*m*g) / (2*(a+b));

delta_f = u; % Front axis steering angle in degrees
delta_r = 0; % Rear axis steering angle in degrees

% Wheel's equations update for x-direction (_x) and y-direction (_y)
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

% Lateral force influencing the front axis of the car
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
slip_c_f = atand(v_c_f/v_l_f); % Slip angle depending on input steering angle
phi_c_f = (1-E_c_f)*(slip_c_f+Sh_c_f)+atand(B_c_f*(slip_c_f+Sh_c_f))*(E_c_f/B_c_f);
F_c_f = D_c_f*sind(C_c_f*atand(B_c_f*phi_c_f))+Sv_c_f; % Final force

% Lateral force influencing the rear axis of the car
C_c_r = 1.3;
D_c_r = a1_c*F_z_r^2+a2_c*F_z_r;
B_c_r = (a3_c*sind(a4_c*atand(a5_c*F_z_r)))/(C_c_r*D_c_r);
E_c_r = a6_c*F_z_r^2+a7_c*F_z_r+a8_c;
Sh_c_r = a9_c*delta_r;
Sv_c_r = (a10_c*F_z_r^2+a11_c*F_z_r)*delta_r;
slip_c_r = atand(v_c_r/v_l_r); % Slip angle depending on input steering angle
phi_c_r = (1-E_c_r)*(slip_c_r+Sh_c_r)+atand(B_c_r*(slip_c_r+Sh_c_r))*(E_c_r/B_c_r);
F_c_r = D_c_r*sind(C_c_r*atand(B_c_r*phi_c_r))+Sv_c_r; % Final force

% Longitudinal force influencing the front axis of the car
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
slip_l_f = atand(v_c_f/v_l_f); % Slip angle depending on input steering angle
phi_l_f = (1-E_l_f)*slip_l_f+atand(B_l_f*slip_l_f*(E_l_f/B_l_f));
F_l_f = D_l_f*sind(C_l_f*atand(B_l_f*phi_l_f)); % Final force

% Longitudinal force influencing the rear axis of the car
C_l_r = 1.65;
D_l_r = a1_l*F_z_r^2+a2_l*F_z_r;
B_l_r = (a3_l*F_z_r^2+a4_l*F_z_r)/(exp(a5_l*F_z_r)*C_c_r*D_l_r);
E_l_r = a6_l*F_z_r^2+a7_l*F_z_r+a8_l;
slip_l_r = atand(v_c_r/v_l_r); % Slip angle depending on input steering angle
phi_l_r = (1-E_l_r)*slip_l_r+atand(B_l_r*slip_l_r*(E_l_r/B_l_r));
F_l_r = D_l_r*sind(C_l_r*atand(B_l_r*phi_l_r)); % Final force

% Complete force formulas
F_x_f = F_l_f*cosd(delta_f)-F_c_f*sind(delta_f);
F_x_r = F_l_r*cosd(delta_r)-F_c_r*sind(delta_r);
F_y_f = F_l_f*sind(delta_f)+F_c_f*cosd(delta_f);
F_y_r = F_l_r*sind(delta_r)+F_c_r*cosd(delta_r);

% Physical system dynamics as 1.st order ODE
result = zeros(8, 1);
result(1) = x(2); % res(1)=x'
result(2) = x(4)*x(6)+2*F_x_f*(1/m)+2*F_x_r*(1/m); % res(2)=x''
result(3) = x(4); % res(3)=y'
result(4) = -x(2)*x(6)+2*F_y_f*(1/m)+2*F_y_r*(1/m); % res(4)=y''
result(5) = x(6); % res(5)=p
result(6) = 2*a*F_y_f*(1/I)-2*b*F_y_r*(1/I); % res(6)=p'
result(7) = x(2)*cos(x(5))-x(4)*sin(x(5)); % res(7)=X'
result(8) = x(2)*sin(x(5))+x(4)*cos(x(5)); % res(8)=Y'

end

