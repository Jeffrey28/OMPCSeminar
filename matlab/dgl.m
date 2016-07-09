function [ res ] = dgl( t, x, pars, F_x_f, F_x_r, F_y_f, F_y_r )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

m = pars.m;
I = pars.I;
a = pars.a;
b = pars.b;

res = zeros(8, 1);
res(1) = x(2); % res(1)=x'
res(2) = x(4)*x(6)+2*F_x_f*(1/m)+2*F_x_r*(1/m); % res(2)=x''
res(3) = x(4); % res(3)=y'
res(4) = -x(2)*x(6)+2*F_y_f*(1/m)+2*F_y_r*(1/m); % res(4)=y''
res(5) = x(6); % res(5)=p
res(6) = 2*a*F_y_f*(1/I)-2*b*F_y_r*(1/I); % res(6)=p'
res(7) = x(2)*cos(x(5))-x(4)*sin(x(5)); % res(7)=X'
res(8) = x(2)*sin(x(5))+x(4)*cos(x(5)); % res(8)=Y'

end

