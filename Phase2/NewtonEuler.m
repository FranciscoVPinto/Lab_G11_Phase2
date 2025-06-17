function tau = NewtonEuler(Robot, m, CoM, I_CoM, dq, ddq, w0, dw0, ddp0, g0)
    n = size(Robot,1);
    z = [0;0;1];

    w     = cell(n+1,1);  
    dw    = cell(n+1,1);  
    ddp   = cell(n+1,1);  
    ddpC  = cell(n+1,1);
    f     = cell(n,1);    
    mu    = cell(n,1);

    w{1}   = w0(:);
    dw{1}  = dw0(:);
    ddp{1} = ddp0(:) + g0(:);

% Forward
    for i = 1:n
        T     = DHTransf(Robot(i,:));
        R{i}  = T(1:3,1:3);
        p{i}  = T(1:3,4);

        w{i+1}   = R{i}'*(w{i}   + dq(i)*z);
        dw{i+1}  = R{i}'*(dw{i}  + ddq(i)*z + cross(w{i},dq(i)*z));
        ddp{i+1} = R{i}'*ddp{i} + ...
                   cross(dw{i+1},R{i}'*p{i}) + ...
                   cross(w{i+1},cross(w{i+1},R{i}'*p{i}));

        ddpC{i+1} = ddp{i+1} + ...
                    cross(dw{i+1},CoM{i}) + ...
                    cross(w{i+1},cross(w{i+1},CoM{i}));
    end


% Backward    
    for i = n:-1:1         
        if i==n
            f{i} = m(i)*ddpC{i+1};
        else
            f{i} = R{i+1}*f{i+1} + m(i)*ddpC{i+1};
        end
    end

% Torque backward
    for i = n:-1:1         
        if i==n
            mu{i} = I_CoM{i}*dw{i+1} + ...
                    cross(w{i+1},I_CoM{i}*w{i+1}) + ...
                    cross(R{i}'*p{i}+CoM{i},f{i});
        else
            mu{i} = I_CoM{i}*dw{i+1} + ...
                    cross(w{i+1},I_CoM{i}*w{i+1}) + ...
                    R{i+1}*mu{i+1} - ...
                    cross(CoM{i},R{i+1}*f{i+1}) + ...
                    cross(R{i}'*p{i}+CoM{i},f{i});
        end
    end

    tau = sym(zeros(n,1));
   
% Joint torque
    for i = 1:n            
        if ~isempty(symvar(Robot(i,2)))
            tau(i) = z'*R{i}*mu{i};
        else
            tau(i) = z'*R{i}*f{i};
        end
    end
end
