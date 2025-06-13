function [torque] = NewtonEuler(DH_table, params, q_vel, q_acc, omega0, alpha0, base_lin_acc)

    robot_size = size(DH_table);
    num_links  = robot_size(1);

    joint_axis = [0; 0; 1];
  
    torque = sym(zeros(num_links,1));

    omega    = sym(zeros(3, num_links+1));  omega(:,1)    = omega0;
    alpha    = sym(zeros(3, num_links+1));  alpha(:,1)    = alpha0;
    lin_acc  = sym(zeros(3, num_links+1));  lin_acc(:,1)  = base_lin_acc;

    for i = 1:num_links
        transforms{i} = DHTransf(DH_table(i,:));
    end
   
    for i = 1:num_links
        R   = transforms{i}(1:3,1:3)';
        r_i = transforms{i}(1:3,4);


        omega(:,i+1)   = R*(omega(:,i)   + q_vel(i)*joint_axis);

        alpha(:,i+1)   = R*(alpha(:,i)   + q_acc(i)*joint_axis ...
                                 + q_vel(i)*cross(omega(:,i), joint_axis));

        lin_acc(:,i+1) = R*lin_acc(:,i) ...
                       + cross(alpha(:,i+1), R*r_i) ...
                       + cross(omega(:,i+1), cross(omega(:,i+1), R*r_i));
    end
   
    com_lin_acc = sym(zeros(3, num_links));
    for i = 1:num_links
        com_lin_acc(:,i) = lin_acc(:,i+1) ...
                         + cross(alpha(:,i+1), params.cm{i}) ...
                         + cross(omega(:,i+1), cross(omega(:,i+1), params.cm{i}));
    end
    
    force = sym(zeros(3, num_links));
    force(:,num_links) = params.m{num_links} * com_lin_acc(:,num_links);

    for i = num_links-1:-1:1
        force(:,i) = transforms{i+1}(1:3,1:3)*force(:,i+1) ...
                   + params.m{i} * com_lin_acc(:,i);
    end
    
    moment = sym(zeros(3, num_links));

    R_end = transforms{num_links}(1:3,1:3);
    r_end = R_end' * transforms{num_links}(1:3,4);
    moment(:,num_links) = params.I{num_links}*alpha(:,end) ...
                        + cross(omega(:,end), params.I{num_links}*omega(:,end)) ...
                        + cross(r_end + params.cm{num_links}, force(:,num_links));

    for i = num_links-1:-1:1
        R_next = transforms{i+1}(1:3,1:3);
        r_i     = transforms{i}(1:3,1:3)' * transforms{i}(1:3,4);

        moment(:,i) = params.I{i}*alpha(:,i+1) ...
                    + cross(omega(:,i+1), params.I{i}*omega(:,i+1)) ...
                    + R_next*moment(:,i+1) ...
                    - cross(params.cm{i}, R_next*force(:,i+1)) ...
                    + cross(r_i + params.cm{i}, force(:,i));
    end

    for i = 1:num_links
        R   = transforms{i}(1:3,1:3);
        torque(i) = joint_axis.' * R * moment(:,i);
    end
end
