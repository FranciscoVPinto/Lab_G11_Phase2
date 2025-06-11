syms tf Ts R real

% Total arc-length for the circular path
sf = (3*pi/2 + 2)*R;

% Time-scaling function: s(t) = a2*t^2 + a3*t^3
a2 = 3*sf / tf^2;
a3 = -2*sf / tf^3;
s = a2*Ts^2 + a3*Ts^3;

% Now define trajectory position and rotation as functions of 's'
ch1 = R;
ch2 = (3*pi/2 + 1)*R;
ch3 = (3*pi/2 + 2)*R;
theta = (s - R)/R - 3*pi/4;

% Piecewise position
Pad_x = piecewise(s <= ch1, cos(pi/4)*s, ...
                  s <= ch2, R*cos(theta) + sqrt(2)*R, ...
                  s <= ch3, cos(pi/4)*((3*pi/2 + 2)*R - s));
Pad_y = piecewise(s <= ch1, -sin(pi/4)*s, ...
                  s <= ch2, R*sin(theta), ...
                  s <= ch3, sin(pi/4)*((3*pi/2 + 2)*R - s));
Pad_z = 0.2;  % Constant height

TrajPosition = [Pad_x; Pad_y; Pad_z];

% Piecewise orientation
Rad = [
    0, piecewise(s <= ch1, cos(pi/4), s <= ch2, -cos(theta), s <= ch3, cos(pi/4)), ...
       piecewise(s <= ch1, cos(pi/4), s <= ch2, -sin(theta), s <= ch3, -cos(pi/4));
    0, piecewise(s <= ch1, cos(pi/4), s <= ch2, -sin(theta), s <= ch3, -cos(pi/4)), ...
       piecewise(s <= ch1, -cos(pi/4), s <= ch2, cos(theta), s <= ch3, -cos(pi/4));
   -1, 0, 0
];

% Create Simulink block
libName = 'Mako_Trajplanning';
blkName = 'Mako_Trajplanning';

if ~bdIsLoaded(libName)
    new_system(libName, 'Library');
end
open_system(libName);

matlabFunctionBlock([libName '/' blkName], TrajPosition, Rad,'vars', {tf, Ts, R},'outputs', {'TrajPosition', 'Rad'});

save_system(libName);
close_system(libName);

disp('Time-scaled trajectory planning block "Mako_Trajplanning" created in "Mako_Lib".');
