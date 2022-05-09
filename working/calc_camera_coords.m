function [shape_array, annot_color_img] = calc_camera_coords(bbox, bbox_idx, aligned_img, annot_color_img, intrinsic_matrix, n)
    shape_array = zeros(n, 3);

    % Intrinsic camera matrix for the raw (distorted) images.
    %     [fx  0 cx]
    % K = [ 0 fy cy]
    %     [ 0  0  1]
    
    % Projects 3D points in the camera coordinate frame to 2D pixel
    % coordinates using the focal lengths (fx, fy) and principal point
    % (cx, cy).

    fx = intrinsic_matrix(1); % focal length
    fy = intrinsic_matrix(5);
    
    cx = intrinsic_matrix(3); % principle point
    cy = intrinsic_matrix(6);

    for i=1:n
        ix = bbox_idx(i);
        row = bbox(ix, :);
        [x_, y_, w_, h_] = deal(row(1), row(2), row(3), row(4));
        [u, v] = calc_centroid(x_, y_, w_, h_); % bounding box centre pixel
    
        d = aligned_img(v, u); % get depth at img centroid
    
        X = ((u - cx) * d) / fx; % image point u - principal point cx * depth / focal length
        Y = ((v - cy) * d) / fy;
        Z = d;
    
        disp(['idx: ', num2str(ix)]);
        disp([num2str(X), ', ', num2str(Y), ', ', num2str(Z)])
    
        annot_color_img = insertMarker(annot_color_img, [u, v],'x', 'color', 'green', 'size', 10);
        shape_array(i, :) = [X, Y, Z];
    end