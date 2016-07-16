clear all;

x0 = 0;
x1 = 200;
spaces = 100;
X = linspace(x0, x1, spaces);

res = zeros(spaces, 2);

for i = 1 : spaces
    res(i, :) = trajectory(X(i));
end

figure;
subplot(2,1,1);
plot(res(:, 2));

grid on;
axis equal;
xlabel('X (m)'); ylabel('Y (m)');
%axis([0 2 -1 1]);
title('Y reference trajectory');

subplot(2,1,2);
plot(res(:, 1));

grid on;
xlabel('X (m)'); ylabel('psi (°)');
title('psi reference trajectory');

% plot(res(:, 1));
% hold on;
% plot(res(:, 2));