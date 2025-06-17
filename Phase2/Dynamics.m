[DH_table, params] = Mako();
DH_table      = DH_table(1:5,:);
params.m       = params.m(1:5);
params.cm      = params.cm(1:5);
params.I       = params.I(1:5);

syms dq1 dq2 dq3 dq4 dq5 ddq1 ddq2 ddq3 ddq4 ddq5 real
g = 9.81;

dq   = [dq1;  dq2;  dq3;  dq4;  dq5];
ddq  = [ddq1; ddq2; ddq3; ddq4; ddq5];
w0   = [0;0;0];
dw0  = [0;0;0];
ddp0 = [0;0;0];    
g0   = [0;0;g];   

tau_all = NewtonEuler( DH_table, params.m, params.cm, params.I, dq, ddq, ...
    w0, dw0, ddp0, g0);
tau = tau_all(1:5);

B = collect( jacobian(tau, [ddq1 ddq2 ddq3 ddq4 ddq5]) );

G = subs( tau, ...
    {dq1,dq2,dq3,dq4,dq5, ddq1,ddq2,ddq3,ddq4,ddq5}, ...
    {0,0,0,0,0,     0,    0,    0,    0,    0} ...
);

phi = subs( tau, {ddq1,ddq2,ddq3,ddq4,ddq5, g}, ...
                {0,    0,    0,    0,    0, 0} );

libName = 'Mako_Dynamics';
blkName = 'Mako_Dynamics';

new_system(libName, 'Library');
open_system(libName);
matlabFunctionBlock([libName '/' blkName], B, G, phi);
save_system(libName);
close_system(libName);
