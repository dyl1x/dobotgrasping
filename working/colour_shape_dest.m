function [dest, ncol] = colour_shape_dest(objects, i, ncol)

    if strcmp(objects{i}.shape, 'cube')
        z_coord = 0.02;
    elseif strcmp(objects{i}.shape, 'pyramid')
        z_coord = 0.03;
    elseif strcmp(objects{i}.shape, 'sphere')
        z_coord = 0.02;
    else
        z_coord = 0.04;
    end

    if strcmp(objects{i}.colour, 'red')
        xr = 0.1;
        dest = [eye(3), [xr, -0.4 + (ncol(1) * 0.1), z_coord]'; zeros(1, 3), 1];
        ncol(1) = ncol(1) + 1;
    
    elseif strcmp(objects{i}.colour, 'green')
        xg = 0;
        dest = [eye(3), [xg, -0.4 + (ncol(2) * 0.1), z_coord]'; zeros(1, 3), 1];
        ncol(2) = ncol(2) + 1;
    
    elseif strcmp(objects{i}.colour, 'blue')
        xb = -0.1;
        dest = [eye(3), [xb, -0.4 + (ncol(3) * 0.1), z_coord]'; zeros(1, 3), 1];
        ncol(3) = ncol(3) + 1;
    end