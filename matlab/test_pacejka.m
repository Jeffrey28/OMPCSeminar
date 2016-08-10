clear all;

% Parameter
a = 1;
b = 1;
m = 2050;
I = 3344;
g = 9.81;

Fz = a * m * g / (2000 * (a + b));

%% Lateral Force
x0 = -45;
x1 = 45;
spaces = 400;
X = linspace(x0, x1, spaces);

Fc = zeros(spaces, 5);
Fl = zeros(spaces, 5);

alpha = 0;
slip = 0;
mu = 0.1;

for i=1:spaces
    alpha = X(i) * pi / 180;
    for j=1:5
        mu = -0.1 + (j * 0.2);
        [Fl(i, j), Fc(i, j)] = Pacejka(alpha, slip, mu, Fz);
    end
end

figure;

hold on;
plot(X, Fl(:, 1));
plot(X, Fc(:, 1));
plot(X, Fc(:, 2));
plot(X, Fc(:, 3));
plot(X, Fc(:, 4));
plot(X, Fc(:, 5));
hold off;
xlabel('Slip angle alpha (deg)'); ylabel('Force (N)');
title('Tire forces as a function of slip angle with zero slip ratio (Pacejka)');
legend('F_l - longitudinal force','F_c - lateral force (mu 0.1)','F_c - lateral force (mu 0.3)','F_c - lateral force (mu 0.5)','F_c - lateral force (mu 0.7)','F_c - lateral force (mu 0.9)')



%% Lonitudial Force
x0 = -30;
x1 = 30;
spaces = 400;
X = linspace(x0, x1, spaces);


Fc = zeros(spaces, 5);
Fl = zeros(spaces, 5);

alpha = 0;
slip = 0;
mu = 0.1;

for i=1:spaces
    slip = X(i) / 100;
    for j=1:5
        mu = -0.1 + (j * 0.2);
        [Fl(i, j), Fc(i, j)] = Pacejka(alpha, slip, mu, Fz);
    end
end

figure;
hold on;
plot(X, Fl(:, 1));
plot(X, Fl(:, 2));
plot(X, Fl(:, 3));
plot(X, Fl(:, 4));
plot(X, Fl(:, 5));
plot(X, Fc(:, 1));
hold off;
xlabel('Slip ratio s (percent)'); ylabel('Force (N)');
title('Tire forces as a function of slip ratio with zero slip angle (Pacejka)');
legend('F_l - longitudinal force (mu 0.1)','F_l - longitudinal force (mu 0.3)','F_l - longitudinal force (mu 0.5)','F_l - longitudinal force (mu 0.7)','F_l - longitudinal force (mu 0.9)', 'F_c - lateral force')

