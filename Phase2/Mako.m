function [DH_table, pars] = Mako()
    syms q1 q2 q3 q4 q5 q6 real

    d3   = 0.4;
    a4   = 0.1;
    d5   = 0.5;
    d6   = 0.1;
    d6p  = 0.15;
    gamma = pi/8;

    DH_table = [ ...
         0     q1   0        pi/2      pi/2   0;
         0     q2   0       -pi/2      pi/2   0;
        -d3    q3   0        pi/2      pi/2   0;
         0     q4   a4       pi/2     -pi/2   0;
         d5    q5   0       -pi/2        0    0;
        -d6    q6   0  pi/2 - gamma     -pi/2  0;
        d6p    0    0          0          0    1 ...
    ];

    pars = MakoParameters();

    pars.cm{1} = [0; 0;   0];
    pars.cm{2} = [0; 0;   0];
    pars.cm{3} = [0; 0.25;0];
    pars.cm{4} = [0; 0;   0];
    pars.cm{5} = [0; 0.5; 0];
    pars.cm{6} = [0; 0; 0.2];
    pars.cm{7} = [0; 0;   0];

end
