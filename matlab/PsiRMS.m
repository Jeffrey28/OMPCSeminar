function [ PsiRMS, PsiMax ] = PsiRMS( res )

X_sim = res(:, 5);
Psi_sim = res(:, 3) * (180 / pi);

size = length(res);

Psi_ref = zeros(size, 1);

for i = 1 : size
    Psi_ref(i) = (180 /pi) * PsiTrajectory(X_sim(i));
end

Psi_diff = Psi_sim - Psi_ref;
PsiMax = max(abs(Psi_diff));
sqDiff = Psi_diff.^2;
PsiRMS = sum(sqDiff) / size;

end

