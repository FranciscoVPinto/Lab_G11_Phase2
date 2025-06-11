function [tau] = NewtonEuler(Robot, pars, dq, ddq, w0, dw0, ddp0)

    rsize = size(Robot);
    linkno = rsize(1);
    z0 = [0; 0; 1];

    tau = sym(zeros(linkno, 1));

    w = sym(zeros(3, linkno+1));
    w(:, 1) = w0;

    dw = sym(zeros(3, linkno+1));
    dw(:, 1) = dw0;

    ddp = sym(zeros(3, linkno+1));
    ddp(:, 1) = ddp0;

    for i = 1:linkno
        A{i} = DHTransf(Robot(i, :));
    end

    for i = 1:linkno
        if Robot(i, 6) == sym(1) % Prismatic
            w(:, i+1) = A{i}(1:3, 1:3).' * w(:, i);
        else
            w(:, i+1) = A{i}(1:3, 1:3).' * (w(:, i) + dq(i)*z0);
        end
    end

    for i = 1:linkno
        if Robot(i, 6) == sym(1)
            dw(:, i+1) = A{i}(1:3, 1:3).' * dw(:, i);
        else
            dw(:, i+1) = A{i}(1:3, 1:3).' * (dw(:, i) + ddq(i)*z0 + dq(i)*cross(w(:, i), z0));
        end
    end

    for i = 1:linkno
        R = A{i}(1:3, 1:3);
        p = A{i}(1:3, 4);
        if Robot(i, 6) == sym(1)
            ddp(:, i+1) = R.' * (ddp(:, i) + ddq(i)*z0) + 2*dq(i)*cross(w(:, i+1), R.'*z0) + ...
                          cross(dw(:, i+1), R.'*p) + cross(w(:, i+1), cross(w(:, i+1), R.'*p));
        else
            ddp(:, i+1) = R.' * ddp(:, i) + cross(dw(:, i+1), R.'*p) + ...
                          cross(w(:, i+1), cross(w(:, i+1), R.'*p));
        end
    end

    ddpc = sym(zeros(3, linkno));
    for i = 1:linkno
        ddpc(:, i) = ddp(:, i+1) + cross(dw(:, i+1), pars.cm{i}) + cross(w(:, i+1), cross(w(:, i+1), pars.cm{i}));
    end

    f = sym(zeros(3, linkno));
    f(:, linkno) = pars.m{linkno} * ddpc(:, linkno);

    for i = 1:linkno-1
        f(:, linkno-i) = A{linkno-i+1}(1:3, 1:3) * f(:, linkno-i+1) + pars.m{linkno-i} * ddpc(:, linkno-i);
    end

    mu = sym(zeros(3, linkno));
    mu(:, linkno) = pars.I{linkno} * dw(:, linkno+1) + cross(w(:, linkno+1), pars.I{linkno} * w(:, linkno+1)) + ...
                    cross(A{linkno}(1:3, 1:3).' * A{linkno}(1:3, 4) + pars.cm{linkno}, f(:, linkno));

    for i = 1:linkno-1
        mu(:, linkno-i) = pars.I{linkno-i} * dw(:, linkno-i+1) + ...
            cross(w(:, linkno-i+1), pars.I{linkno-i} * w(:, linkno-i+1)) + ...
            A{linkno+1-i}(1:3, 1:3) * mu(:, linkno+1-i) - ...
            cross(pars.cm{linkno-i}, A{linkno+1-i}(1:3, 1:3) * f(:, linkno+1-i)) + ...
            cross(A{linkno-i}(1:3, 1:3).' * A{linkno-i}(1:3, 4) + pars.cm{linkno-i}, f(:, linkno-i));
    end

    for i = 1:linkno
        if Robot(i, 6) == sym(1)
            tau(i) = z0.' * A{i}(1:3, 1:3) * f(:, i);
        else
            tau(i) = z0.' * A{i}(1:3, 1:3) * mu(:, i);
        end
    end
end
