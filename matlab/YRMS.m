function [ YRMS, YMax ] = YRMS( res )

X_sim = res(:, 5);
Y_sim = res(:, 6);

size = length(res);

Y_ref = zeros(size, 1);

for i = 1 : size
    Y_ref(i) = YTrajectory(X_sim(i));
end

Y_diff = Y_sim - Y_ref;
YMax = max(abs(Y_diff));
sqDiff = Y_diff.^2;
YRMS = sum(sqDiff) / size;

end