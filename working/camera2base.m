function [n_objects, world_coords, world_transforms] = camera2base(shape_array, shape_labels, b2c, b2c_rot, n)

world_coords = zeros(1, 3, n);
world_transforms = zeros(4, 4, n);
colours = strings(1, n);

for i=1:n
    pt_raw = shape_array(i, :);
    pt = pt_raw / 1000;
    
    c2p = [b2c_rot(1:3, 1:3), pt(1:3)'; zeros(1,3), 1]; % camera to point transform
    ptWorld = b2c * c2p; % point in camera to base transformation

    world_coords(:, :, i) = ptWorld(1:3, 4);
    world_transforms(:, :, i) = ptWorld;

    disp([char(shape_labels(i)), ': ', num2str([ptWorld(1:3, 4)]')])
    line2_h = plot3([b2c(1, 4), ptWorld(1, 4)], [b2c(2, 4), ptWorld(2, 4)], [b2c(3, 4), ptWorld(3, 4)], 'm');

    objects{i} = Objects(strcat('../res/shapes/', shape_labels(i), '.ply'), [ptWorld(1:3, 4)]', [0 0 0]);
    split_string = shape_labels(i).split(' ');

    objects{i}.shape = split_string(1);
    objects{i}.colour = split_string(2);
    colours(i) = split_string(2);

end

[~, idx] = sort(colours', 'ascend');
B = idx(1:size(colours')); % indexes

for i=1:size(B)
    n_objects{i} = objects{B(i)};
end