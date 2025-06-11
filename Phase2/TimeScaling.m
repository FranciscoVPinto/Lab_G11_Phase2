syms t R tf

sf = (3*pi/2 + 2) * R;

a2 = 3/tf^2 * sf;
a3 = -2/tf^3 * sf;

Ts = a2*t^2 + a3*t^3;

libName = 'Mako_TimeScaling';
blkName = 'Mako_TimeScaling';

if ~bdIsLoaded(libName)
    new_system(libName, 'Library');
end
open_system(libName);

matlabFunctionBlock([libName '/' blkName], Ts);

save_system(libName);
close_system(libName);