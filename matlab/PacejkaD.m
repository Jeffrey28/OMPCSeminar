function [ Fl, Fc ] = PacejkaD( alpha, s, mu, Fz )
%Pacejka tyre model
% INPUTS:
%   alpha   : current lateral slip angle
%   s       : current longitudial slip ratio
%   mu      : friction coefficient
%   Fz      : the normal force
%
% OUTPUTS:
%  Fl       : longitudal force
%  Fc       : lateral or cornering force

camber = 0;
%piBy180 = pi / 180;

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
Dc = a1_c * Fz^2 + a2_c * Fz;
Bc = (a3_c * sind(a4_c * atand(a5_c * Fz))) / (Cc * Dc);
Ec = a6_c * Fz^2 + a7_c * Fz + a8_c;

Sh = a9_c * camber;
Sv = (a10_c * Fz^2 + a11_c * Fz) * camber;

phic = (1 - Ec) * (alpha + Sh) + atand(Bc * (alpha + Sh)) * (Ec / Bc);
Fc = Dc * sind(Cc * atand(Bc * phic)) + Sv; % Final force


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
Dl = a1_l * Fz^2 + a2_l * Fz;
Bl = (a3_l * Fz^2 + a4_l * Fz) / (exp(a5_l * Fz) * Cl * Dl);
El = a6_l * Fz^2 + a7_l * Fz + a8_l;

phil = (1 - El) * s + atand(Bl * s) * (El / Bl);
Fl = Dl * sind(Cl * atand(Bl * phil)); % Final force

end

