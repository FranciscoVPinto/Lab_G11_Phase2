%% compute_worst_case_inertia_and_gains_5dof.m

syms q1 q2 q3 q4 q5 real
joint_syms = [q1 q2 q3 q4 q5];
joint_limits = repmat([-pi, pi], 5, 1);  % same limits for all joints

% Load symbolic B(q) matrix or ensure B is in workspace
% Example:
% load('MakoDynamicsSymbols.mat', 'B');

% Pre-allocate
worst_inertia = zeros(5,1);
max_positions = zeros(5,5);

options = optimoptions('fmincon', 'Display','none');

for i = 1:5
    fprintf('Maximizing B(%d,%d)...\n', i, i);
    Bii_fun = matlabFunction(B(i,i), 'Vars', joint_syms);

    lb = joint_limits(:,1)';
    ub = joint_limits(:,2)';
    x0 = zeros(1,5);
    obj = @(q) -Bii_fun(q(1),q(2),q(3),q(4),q(5));

    [q_opt, fval] = fmincon(obj, x0, [],[],[],[], lb, ub, [], options);
    worst_inertia(i) = -fval;
    max_positions(i,:) = q_opt;
    fprintf('  B_%d,max = %.3f at [%s]\n', i, -fval, num2str(q_opt, ' %.2f'));
end

% Save results
save('worst_case_inertia_5dof.mat','worst_inertia','max_positions');
T = array2table([worst_inertia, max_positions], ...
    'VariableNames', {'Bii', 'q1','q2','q3','q4','q5'});
writetable(T, 'worst_case_inertia_5dof.csv');

%% Gain Calculation

% User-defined desired dynamics
omega_n = 5;   % rad/s (example)
xi = 0.8;      % damping ratio

Kp = worst_inertia .* (omega_n^2);
Kd = 2 * worst_inertia .* xi * omega_n;

fprintf('\nCalculated gains (using ωₙ = %.2f, ξ = %.2f):\n', omega_n, xi);
for i = 1:5
    fprintf('  Joint %d: Kp = %.2f, Kd = %.2f\n', i, Kp(i), Kd(i));
end

%% Plot the gains

figure('Name','Joint Gains vs. Joint Index','NumberTitle','off');
subplot(2,1,1);
bar(1:5, Kp);
ylabel('K_p');
title('Proportional Gains');

subplot(2,1,2);
bar(1:5, Kd);
xlabel('Joint #');
ylabel('K_d');
title('Derivative Gains');
