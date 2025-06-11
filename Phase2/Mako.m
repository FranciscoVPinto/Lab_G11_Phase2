function [Robot, pars] = Mako()
    syms q1 q2 q3 q4 q5 q6 real

    % Link offsets
    d3 = 0.4; a4 = 0.1; d5 = 0.5; d6 = 0.1; d6p = 0.15; gamma = pi/8;

    % D-H table: [d v a alpha offset jointType]
    Robot = [0     q1   0          pi/2        pi/2     0;
             0     q2   0         -pi/2        pi/2     0;
            -d3    q3   0          pi/2        pi/2     0;
             0     q4   a4         pi/2       -pi/2     0;
             d5    q5   0         -pi/2          0      0;
            -d6    q6   0     pi/2 - gamma     -pi/2     0;
             d6p    0   0           0             0      1];

    % Center of mass positions
    pars.cm{1} = [0; 0; 0];
    pars.cm{2} = [0; 0; 0];
    pars.cm{3} = [0; 0.25; 0];
    pars.cm{4} = [0; 0; 0];
    pars.cm{5} = [0; 0.5; 0];
    pars.cm{6} = [0; 0; 0.2];
    pars.cm{7} = [0; 0; 0];

    % Masses
    pars.m{1} = 9;
    pars.m{2} = 7;
    pars.m{3} = 5;
    pars.m{4} = 4;
    pars.m{5} = 4;
    pars.m{6} = 1;
    pars.m{7} = 0;

    % Explicit inertia tensors
    pars.I{1} = diag([0.28125, 0.28125, 0.28125]);
    pars.I{2} = diag([0.21875, 0.21875, 0.21875]);
    pars.I{3} = diag([0.1151041666666667, 0.1151041666666667, 0.078125]);
    pars.I{4} = diag([0.0833333333333333, 0.0833333333333333, 0.0833333333333333]);
    pars.I{5} = diag([0.101333333333333, 0.101333333333333, 0.08]);
    pars.I{6} = diag([0.017333333333333, 0.017333333333333, 0.02]);
    pars.I{7} = zeros(3);
end
