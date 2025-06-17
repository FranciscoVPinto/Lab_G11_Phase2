% VALIDATION AND GRAVITY COMPENSATION BLOCK GENERATION

% Load MAKO robot parameters (first 5 joints)
[DH_full, params_full] = Mako();
DH  = DH_full(1:5, :);
m   = params_full.m(1:5);
cm  = params_full.cm(1:5);
I   = params_full.I(1:5);

% Declare symbolic joint positions (5-DOF)
syms q1 q2 q3 q4 q5 real;
q = [q1; q2; q3; q4; q5];

% Base at rest and gravity
w0   = zeros(3,1);
dw0  = zeros(3,1);
ddp0 = zeros(3,1);
g_val = 9.81;
g0   = [0;0;g_val];

% Compute gravity compensation torque g(q)
g = NewtonEuler(DH, m, cm, I, zeros(5,1), zeros(5,1), w0, dw0, ddp0, g0);
g = simplify(g);

% Create Simulink library for g(q)
libName = 'gravity_comp_lib';
blkName = 'g_compensation';

if bdIsLoaded(libName)
    close_system(libName, 0);
end
new_system(libName, 'Library');
open_system(libName);

% Add MATLAB Function block
blockPath = [libName '/' blkName];
add_block('simulink/User-Defined Functions/MATLAB Function', blockPath, ...
    'Position', [100 100 300 200]);

% Generate block with input q and output g
matlabFunctionBlock(blockPath, g, ...
    'Vars',    {q}, ...
    'Outputs', {'g'});

% Save and close
save_system(libName);
close_system(libName);

