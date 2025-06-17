function tau = NewtonEuler(Robot, m, CoM, I_CoM, dq, ddq, w0, dw0, ddp0, g0)

n = size(Robot, 1); % Number of joints.
z0 = [0; 0; 1]; % Unit vector along the z-axis.

% Pre-allocate symbolic containers.
[R, p, w, dw, ddp, ddp_C] = deal(cell(n+1, 1));
[f, mu] = deal(cell(n, 1));

w_prev = w0(:);  % Base angular velocity as 3x1 column
   dw_prev = dw0(:);  % Base angular accel as 3x1 column
   tmp_ddp = ddp0 + g0;             % Sum base accel and gravity
   ddp_prev = tmp_ddp(:);           % Base linear accel + gravity as 3x1 column


R_prev = eye(3);
p_prev = sym(zeros(3, 1));

tau = sym(zeros(n, 1));
f_next = sym(zeros(3, 1));
mu_next = sym(zeros(3, 1));

for i = 1 : n
    % Compute the homogeneous D-H transformation matrix of the i-th joint.
    A{i} = DHTransf(Robot(i, :)); % 4×4 homogeneous D-H transformation matrix for joint i.
    R_rel{i} = A{i}(1 : 3, 1 : 3); % Rotation matrix from frame i-1 to i.
    p_rel{i} = A{i}(1 : 3, 4); % Position vector from frame i-1 to i.

    if ~isempty(symvar(Robot(i, 2))) % If the joint is rotational.
        w{i+1} = R_rel{i}.'* (w_prev + dq(i)*z0);
        dw{i+1} = R_rel{i}.'* (dw_prev + ddq(i)*z0 + cross(w_prev, dq(i)*z0));
        ddp{i+1} = R_rel{i}.'* ddp_prev+ ...
                 cross(dw{i+1}, R_rel{i}.'*p_rel{i})+...
                 cross(w{i+1}, ...
                 cross(w{i+1}, R_rel{i}.'*p_rel{i}));

    else % If the joint is prismatic.
        w{i+1} = R_rel{i}.'* w_prev;
        dw{i+1} = R_rel{i}.'* dw_prev;
        ddp{i+1} = R_rel{i}.'*(ddp_prev+ddq(i)*z0) + 2*dq(i)*cross(w{i+1}, R_rel{i}.'*z0)...
            +cross(dw{i+1}, R_rel{i}.'*p_rel{i}) + cross(w{i+1}, cross(w{i+1}, R_rel{i}.'*p_rel{i}));
        
    end

    % Compute the linear acceleration of the center of mass.
    ddp_C{i+1} = ddp{i+1} + cross(dw{i+1}, CoM{i}) + ...
               cross(w{i+1}, cross(w{i+1}, CoM{i}));
    
    % Update the previous values for the next iteration.
    w_prev = w{i+1};
    dw_prev = dw{i+1};
    ddp_prev = ddp{i+1};
    R_prev = R{i+1};
    p_prev = p{i+1};
end

for i = n : -1 : 1
    if i == n
        f{i} = m(i)*ddp_C{i+1};
    else
        f{i} = R_rel{i+1}*f{i+1} + m{i}*ddp_C{i+1};
    end
end

for i = n : -1 : 1
    if i == n
        mu{i} = I_CoM{i}*dw{i+1} + cross(w{i+1}, I_CoM{i}*w{i+1}) ...
            + cross((R_rel{i}.'*p_rel{i} + CoM{i}), f{i});
    else
        mu{i} = I_CoM{i}*dw{i+1} + ...
            cross(w{i+1}, I_CoM{i}*w{i+1}) + ...
            R_rel{i+1}*mu{i+1} - ...
            cross(CoM{i}, R_rel{i+1}*f{i+1}) + ...
            cross((R_rel{i}.'*p_rel{i} + CoM{i}), f{i});
    end
end

for i = 1:n
    if ~isempty(symvar(Robot(i, 2)))
        tau(i) = z0.'*R_rel{i}*mu{i};
    else
        tau(i) = z0.'*R_rel{i}*f{i};
    end
end

end