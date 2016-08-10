function [ Fl, Fc ] = Pacejka( alpha, s, mu, Fz )
%Pacejka tyre model
% Web page: http://www-cdr.stanford.edu/dynamic/bywire/tires.pdf
% INPUTS:
%   alpha   : current lateral slip angle in radian
%   s       : current longitudial slip ratio
%   mu      : friction coefficient
%   Fz      : the normal force
%
% OUTPUTS:
%  Fl       : longitudal force
%  Fc       : lateral or cornering force

camber = 0;                 % camber angle in degrees
%alpha = alpha * 180 / pi;   % slip angle in degrees

% Lateral/Cornering force
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

Cc = 1.3;
%Cc = Cc * mu;
Dc = a1_c * Fz^2 + a2_c * Fz;
Dc = Dc * mu;
Bc = (a3_c * sin(a4_c * atan(a5_c * Fz))) / (Cc * Dc);
%Bc = Bc * mu;
Ec = a6_c * Fz^2 + a7_c * Fz + a8_c;
%Ec = Ec * mu;

Sh = a9_c * camber;
Sv = (a10_c * Fz^2 + a11_c * Fz) * camber;

phic = (1 - Ec) * (alpha + Sh) + atan(Bc * (alpha + Sh)) * (Ec / Bc) * (180 / pi);
Fc = Dc * sin(Cc * atan(Bc * phic)) + Sv; % Final force


% Longitudial force
a1_l = 21.3;
a2_l = 1144;
a3_l = 49.6;
a4_l = 226;
a5_l = 0.069;
a6_l = -0.006;
a7_l = 0.056;
a8_l = 0.486;

Cl = 1.65;
%Cl = Cl * mu;
Dl = a1_l * Fz^2 + a2_l * Fz;
Dl = Dl * mu;
Bl = (a3_l * Fz^2 + a4_l * Fz) / (exp(a5_l * Fz) * Cl * Dl);
%Bl = Bl * mu;
El = a6_l * Fz^2 + a7_l * Fz + a8_l;
%El = El * mu;

phil = (1 - El) * s + atan(Bl * s) * (El / Bl) * (180 / pi);
Fl = Dl * sin(Cl * atan(Bl * phil)); % Final force

end

