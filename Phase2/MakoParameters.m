function pars = MakoParameters()

    M_total = 30;


    types = {'hollow_cylinder', 'cylinder', 'cylinder', 'cube', 'cylinder', 'cylinder', 'none'};


    dims = {
        [1, 0.5, 0.5], 
        [0.5, 0.5],
        [0.4, 0.5],
        [0.5, 0.5, 0.5],
        [0.4, 1],
        [0.4, 0.4],
        []
    };

    n = numel(types);
    V = zeros(1,n);

    for i = 1:n
        tipo = types{i};
        d = dims{i};

        switch tipo
            case 'cylinder'
                r = d(1); h = d(2);
                V(i) = pi * r^2 * h;
            case 'hollow_cylinder'
                R = d(1); r = d(2); h = d(3);
                V(i) = pi * (R^2 - r^2) * h;
            case 'cube'
                a = d(1); b = d(2); c = d(3);
                V(i) = a * b * c;
            otherwise
                V(i) = 0;
        end
    end

    mass = M_total * V / sum(V);
    pars.m = cell(1,n);
    pars.I = cell(1,n);

    for i = 1:n
        mi = mass(i);
        tipo = types{i};
        d = dims{i};
        pars.m{i} = mi;

        switch tipo
            case 'cylinder'
                r = d(1); h = d(2);
                Ixx = (1/12)*mi*(3*r^2 + h^2);
                Izz = (1/2)*mi*r^2;
            case 'hollow_cylinder'
                R = d(1); r = d(2); h = d(3);
                Ixx = (1/12)*mi*(3*(R^2 + r^2) + h^2);
                Izz = (1/2)*mi*(R^2 + r^2);
            case 'cube'
                a = d(1); b = d(2); c = d(3);
                Ixx = (1/12)*mi*(b^2 + c^2);
                Izz = (1/12)*mi*(a^2 + b^2);
            otherwise
                Ixx = 0; Izz = 0;
        end
        pars.I{i} = diag([Ixx, Ixx, Izz]);
    end
end
