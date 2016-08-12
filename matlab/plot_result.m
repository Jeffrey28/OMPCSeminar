function [ ] = plot_result( u_opt, res )
%PLOT_RESULT Plot the result

size = length(res(:, 1));
pars = plant_init();

%figure;
subplot(4,1,1);

% Plot car XY-trajectory
hold on;
plot(res(:, 5), res(:, 6), 'r');
y_trajectory = zeros(size, 1);
for i=1:size
    y_trajectory(i) = YTrajectory(res(i, 5));
end
plot(res(:, 5), y_trajectory, '--g');


grid on;
axis equal;
xlabel('x (m)'); ylabel('y (m)');
%axis([0 2 -1 1]);
title('Car trajectory');


% Car angle trajectory
subplot(4,1,2);
hold on;
plot(res(:, 5), res(:, 3).*180/pi, 'r');
psi_trajectory = zeros(size, 1);
for i=1:size
    psi_trajectory(i) = PsiTrajectory(res(i, 5)) * 180 / pi;
end
plot(res(:, 5), psi_trajectory, '--g');

grid on;
xlabel('x (m)'); ylabel('psi (deg)');
title('Car angle trajectory');


% Plot steering actions
subplot(4,1,3);
plot(u_opt);

grid on;
xlabel('step'); ylabel('Steering angle (deg)');
title('Steering actions');


% Plot slip angle actions
subplot(4,1,4);
alphas = zeros(size, 1);
for i=1:size
    [alpha_f, alpha_r] = calc_slip_angle(res(i, :), u_opt(i), pars);
    alphas(i) = alpha_f * 180 / pi;
end

plot(alphas);

grid on;
xlabel('step'); ylabel('Slip angle (deg)');
title('Slip angle alpha');

end

