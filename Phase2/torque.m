[DH_full, params_full] = Mako();
DH = DH_full(1:5,:);
m  = params_full.m(1:5);
cm = params_full.cm(1:5);
I  = params_full.I(1:5);

syms q1 q2 q3 q4 q5 real
q = [q1; q2; q3; q4; q5];
g = 9.81;

w0   = zeros(3,1);
dw0  = zeros(3,1);
ddp0 = zeros(3,1);
g0   = [0;0;g];

g = simplify( NewtonEuler(DH, m, cm, I, ...
    zeros(5,1), zeros(5,1), w0, dw0, ddp0, g0) );

libName = 'g_block';
blkName = 'g_block';

new_system(libName,'Library');
open_system(libName);

matlabFunctionBlock([libName '/' blkName], g, ...
    'Vars',{q}, ...
    'Outputs',{'g'});

save_system(libName);
close_system(libName);
