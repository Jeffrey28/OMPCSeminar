function [ ] = plot_result( u_opt, res )
%PLOT_RESULT Plot the result

size = length(res(:, 1));

%figure;
subplot(3,1,1);

% Plot car XY-trajectory
hold on;
plot(res(:, 5), res(:, 6));
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
subplot(3,1,2);
hold on;
plot(res(:, 5), res(:, 3));
psi_trajectory = zeros(size, 1);
for i=1:size
    psi_trajectory(i) = PsiTrajectory(res(i, 5));
end
plot(res(:, 5), psi_trajectory, '--g');

grid on;
xlabel('x (m)'); ylabel('psi (deg)');
title('Car angle trajectory');


% Plot steering actions
subplot(3,1,3);
plot(u_opt);

grid on;
xlabel('step'); ylabel('Steering angle (deg)');
title('Steering actions');

end

