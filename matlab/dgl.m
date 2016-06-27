function [ result ] = dgl( var0, var1, var2, pars )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

% m = pars.m;
% I = pars.I;
% a = pars.a;
% b = pars.b;

m = 1000;
I = 1000;
a = 1;
b = 1;
F_x_f = 1000;
F_x_r = 1000;
F_y_f = 1000;
F_y_r = 1000;

result = ones(5,1);
result(1) = -m*var2(1)+m*var1(2)*var1(5)+2*F_x_f+2*F_x_r;
result(2) = -m*var2(2)-m*var1(1)*var1(5)+2*F_y_f+2*F_y_r;
result(3) = -I*var2(5)-2*a*F_y_f-2*b*F_y_r;
result(4) = -var1(3)+var1(1)*cos(var0(5))-var1(2)*sin(var0(5));
result(5) = -var1(2)+var1(1)*sin(var0(5))+var1(2)*cos(var0(5));

end

