clear all;

% Parameter
a = 1;
b = 1;
m = 2050;
I = 3344;
g = 9.81;

Fz = a * m * g / (2000 * (a + b));

x0 = -10;
x1 = 10;
spaces = 100;
X = linspace(x0, x1, spaces);

Fc = zeros(spaces, 1);
Fl = zeros(spaces, 1);

for i=1:spaces
    [fl, fc] = Pacejka(X(i) * pi / 180, 0, 0, Fz);
    Fl(i) = fl;
    Fc(i) = fc;
end

hold on;
plot(X, Fl);
plot(X, Fc);
hold off;
xlabel('Slip angle alpha (deg)'); ylabel('Force (N)');
title('Tire forces as a function of slip angle with zero slip ratio (Pacejka)');
legend('F_l - longitudinal force','F_c - lateral force')

