clear all;

x0 = 0;
x1 = 100;
spaces = 100;
X = linspace(x0, x1, spaces);

res = zeros(spaces, 2);
resY = zeros(spaces, 1);
resPsi = zeros(spaces, 1);

for i = 1 : spaces
    res(i, :) = trajectory(X(i));
    resY(i) = YTrajectory(X(i));
    resPsi(i) = PsiTrajectory(X(i));
end

figure;
subplot(2,1,1);
hold on;
plot(res(:, 1));
%plot(resY, '--g');
hold off;

grid on;
xlabel('X (m)'); ylabel('Y (m)');
%axis([0 2 -1 1]);
title('Y reference trajectory');

subplot(2,1,2);
hold on;
plot(res(:, 2));
%plot(resPsi, '--g');
hold off;

grid on;
xlabel('X (m)'); ylabel('psi (rad)');
title('Psi reference trajectory');

% plot(res(:, 1));
% hold on;
% plot(res(:, 2));