function pars = MakoParameters()

    M_total = 30;

    types = repmat({'cylinder'}, 1, 7);
    dims  = {[0.25, 0.5], ...  
        [0.2, 0.2], ...  
        [0.125, 0.4], ...  
        [0.1, 0.2], ...  
        [0.09, 0.5], ...  
        [0.08, 0.15], ...  
        [0.0, 0.0]};

    n = numel(types);
    V = zeros(1, n);

    for i = 1:n
        r = dims{i}(1);
        h = dims{i}(2);
        V(i) = pi * r^2 * h;
    end

    mass = M_total * V / sum(V);

    pars.m = cell(1, n);
    pars.I = cell(1, n);

    for i = 1:n
        mi = mass(i);
        r  = dims{i}(1);
        h  = dims{i}(2);

        pars.m{i} = mi;

        Ixx = (1/12) * mi * (3*r^2 + h^2);
        Izz = (1/2)  * mi * r^2;

        pars.I{i} = diag([Ixx, Ixx, Izz]);
    end

    fprintf('Link masses [kg]:\n');
    fprintf(' %g', mass);
    fprintf('\n');
end
