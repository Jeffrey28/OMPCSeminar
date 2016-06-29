function [ res ] = dgl( t, x, pars, F_x_f, F_x_r, F_y_f, F_y_r )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

m = pars.m;
I = pars.I;
a = pars.a;
b = pars.b;

res = zeros(5, 1);
res(1) = x(2);
res(2) = x(4)*x(6)+2*F_x_f*(1/m)+2*F_x_r*(1/m);
res(3) = x(4);
res(4) = -x(2)*x(6)+2*F_y_f*(1/m)+2*F_y_r*(1/m);
res(5) = x(6);
res(6) = 2*a*F_y_f*(1/I)-2*b*F_y_r*(1/I);
res(7) = x(2)*cosd(x(5))-x(4)*sind(x(5));
res(8) = x(2)*sind(x(5))+x(4)*cosd(x(5));

end

