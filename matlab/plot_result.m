function [ ] = plot_result( u_opt, res )
%PLOT_RESULT Plot the result

figure;
subplot(2,1,1);

% Plot car trajectory
plot(res(:, 5), res(:, 6));

grid on;
axis equal;
xlabel('x (m)'); ylabel('y (m)');
%axis([0 2 -1 1]);
title('Car trajectory');

subplot(2,1,2);
plot(u_opt);

grid on;
xlabel('step'); ylabel('Steering angle (deg)');
title('Steering actions');

end

