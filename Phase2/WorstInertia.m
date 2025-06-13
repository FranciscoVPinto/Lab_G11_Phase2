syms q1 q2 q3 q4 q5 real
joint_syms = [q1, q2, q3, q4, q5];
B_sym = B;

joint_limits = repmat([-pi, pi], 5, 1);
lb = joint_limits(:,1)';
ub = joint_limits(:,2)';

opts = optimoptions('fmincon', 'Display','off', 'Algorithm','sqp');
ms   = MultiStart('UseParallel', false, 'StartPointsToRun','bounds');

worst_inertia  = zeros(5,1);
max_positions  = zeros(5,5);

for i = 1:5
    fprintf('Maximizing B(%d,%d)...\n', i, i);
    Bii_fun = matlabFunction(B_sym(i,i), 'Vars', joint_syms);
    obj      = @(q) -Bii_fun(q(1),q(2),q(3),q(4),q(5));
    problem  = createOptimProblem('fmincon', ...
                  'objective', obj, ...
                  'x0',       zeros(1,5), ...
                  'lb',       lb, ...
                  'ub',       ub, ...
                  'options',  opts);
    [q_opt, fval] = run(ms, problem, 10);

    worst_inertia(i)  = -fval;
    max_positions(i,:) = q_opt;

    fprintf('  → B_%d,max = %.6g at q = [%s]\n\n', ...
            i, worst_inertia(i), num2str(q_opt,' %.2f'));
end

omega_n = 5;
xi      = 0.8;

Kp = worst_inertia * omega_n^2;
Kd = 2 * xi * omega_n * worst_inertia;

fprintf('PD gains (ω_n=%.2f, ξ=%.2f):\n', omega_n, xi);
for i = 1:5
    fprintf('  Joint %d: Kp = %.4g,   Kd = %.4g\n', i, Kp(i), Kd(i));
end

save('PDgainResults.mat', 'worst_inertia', 'max_positions', 'Kp', 'Kd');
fprintf('\nResults saved to PDgainResults.mat\n');
