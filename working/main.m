%% SNC Group Assignment
% Dobot control and grasping

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');

% 1. Capturing images from camera
% 2. Robot Control with objects
% 3. ROS BAGS : Load/Explore
% 4. ROS BAGS : Visualise as point cloud
% 5. ROS BAGS : Extract Image
% 6. ROS BAGS : Get Image, detect shape, simulate in scene with camera

%%
idx = 22;
%% 1. Capture Images from camera with ros subscription 

% rosinit()
sub = rossubscriber("/camera/color/image_raw");
figure(1)
i = readImage(sub.receive);
imshow(i)

% imwrite(i, ['working/images/scene_dataset/IMG_', num2str(idx), '.jpeg']);
% idx = idx + 1;

%% 2. ROBOT CONTROL : Dobot - SNC Project

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','dobot')


% Set Variables

view(2);
ws = [-0.1 0.5 -0.3 0.3 0 0.4];

[so, co, po, ro] = deal(0.017, 0.015, 0.023, 0.028);
[ry, gy, by] = deal(-0.05, 0, 0.05);

red_sphere = Objects('res/shapes/red_sphere.ply', [0.4 ry so], [0 0 0]);
% green_sphere = Objects('res/shapes/green_sphere.ply', [0.4 gy so], [0 0 0]);
% blue_sphere = Objects('res/shapes/blue_sphere.ply', [0.4 by so], [0 0 0]);

% red_cube = Objects('res/shapes/red_cube.ply', [0.45 ry co], [0 0 0]);
% green_cube = Objects('res/shapes/green_cube.ply', [0.45 gy co], [0 0 0]);
% blue_cube = Objects('res/shapes/blue_cube.ply', [0.45 by co], [0 0 0]);

% red_pyramid = Objects('res/shapes/red_pyramid.ply', [0.5 ry po], [0 0 0]);
% green_pyramid = Objects('res/shapes/green_pyramid.ply', [0.5 gy po], [0 0 0]);
% blue_pyramid = Objects('res/shapes/blue_pyramid.ply', [0.5 by po], [0 0 0]);

% red_rectangle = Objects('res/shapes/red_rectangle.ply', [0.55 ry ro], [0 0 0]);
% green_rectangle = Objects('res/shapes/green_rectangle.ply', [0.55 gy ro], [0 0 0]);
% blue_rectangle = Objects('res/shapes/blue_rectangle.ply', [0.55 by ro], [0 0 0]);

% blue_rectangle.tran([0.1 0 0]);

dobot = Dobot(ws, '1');
dobot.model.teach

% axis equal
camlight

% dobot.calc_volume(10);
% axis equal
%% 3. ROS BAGS : Load/Explore

clc
clear all
close all

% rosinit('192.168.1.0');
% depth_sub = rossubscriber('/camera/depth/image_raw'); % Stream data from camera


% bag1 = rosbag('bag/3obj3colours.bag');
% bag1 = rosbag('bag/box2.bag');

% Depth Images
depthImages = select(bag1, 'Topic', '/camera/depth/rgb/image_raw');
firstDepthImage = readMessages(depthImages, 1);
d_img = readImage(firstDepthImage{1,1});
figure(1);
imshow(uint8(double(d_img/5600*256)));

% RGB Images
colourImages = select(bag1, 'Topic', '/camera/rgb/image_raw');
firstColourImage = readMessages(colourImages, 1);
c_img = readImage(firstColourImage{1,1});
figure(2);
imshow(uint8(double(c_img/5600*256)));

img_gray = rgb2gray(d_img);
figure(3)
imshow(img_gray)


%% 4. ROS BAGS : Visualise as point cloud
close all
clearvars
clc


bag1 = rosbag('bag/6obj3colours.bag');

pc_data = select(bag1, 'Topic', '/camera/depth/color/points');

msg = readMessages(pc_data, 1);

xyz = readXYZ(msg{1});
rgb = readRGB(msg{1});

% figure(1);
% scatter3(pc_data1{1});

% figure(2);
% pcshow(xyz);

% trim_data = pc_data1{1}.Data;
% rowsToDelete = trim_data > 180;
% trim_data(rowsToDelete) = [];
% pc_data1{1}.Data = trim_data;

% pc_data1{1}.Data(pc_data1{1}.Data > 200) = 0;

% figure;
% scatter3(msg{1})

figure; % Trim far away background data
new_xyz = [];
for i = 1:size(xyz)
    row = xyz(i, :);
    if row(1) < -1 || row(1) > 1
        continue
    elseif row(2) < -1 || row(2) > 1
        continue
    elseif row(3) < -1 || row(3) > 1
        continue
    else
        new_xyz(i, :) = row;
    end
end

% pcshow(new_xyz)


pcobj = pointCloud(readXYZ(msg{1}),'Color',uint8(255*readRGB(msg{1})));

    
% pcshow(readXYZ(pc_data1{1}));

% [x, y] = pol2cart(linspace(-90, 90, 180), pc_data1{1}.Data);
% pc1_cart = [x', y', zeros(180, 1)];
% pointcloud = pointCloud(pc1_cart, 'Color', [ones(180, 1), zeros(180, 1), zeros(180, 1)] );

%% 5. ROS BAG : Extract Image

filename = '5ObjPCloudColorizer';
bag = rosbag(strcat(['bag/', filename, '.bag']));
img_data = select(bag, 'Topic', '/camera/color/image_raw');

firstColourImage = readMessages(img_data, 1);
img = readImage(firstColourImage{1,1});
imshow(img)
imwrite(img, strcat(['working/images/scene_dataset/', filename, '.jpeg']));


%% 6. ROS BAGS : extract img, detect shape, place in sim


close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');


n = 3; % num features in img

load('scene_detector.mat')
load('shape_detector.mat')

filename = 'RealRobotTest1';
bag = rosbag(strcat(['bag/', filename, '.bag']));

% Show RGB Image
RGB_data = select(bag, 'Topic', '/camera/color/image_raw');
firstColourImage = readMessages(RGB_data, 1);
color_img = readImage(firstColourImage{1,1});
figure;
imshow(color_img)

% Show Depth Image
D_data = select(bag, 'Topic', '/camera/depth/image_rect_raw');
firstDepthImage = readMessages(D_data, 1);
depth_img = readImage(firstDepthImage{1,1});

% Show Aligned RGB-D Image
AD_data = select(bag, 'Topic', '/camera/aligned_depth_to_color/image_raw');
firstADepthImage = readMessages(AD_data, 1);
aligned_img = readImage(firstADepthImage{1,1});


% Show Camera Intrinsics
info = select(bag, 'Topic', '/camera/aligned_depth_to_color/camera_info');
infoMsg = readMessages(info);
intrinsic_matrix = infoMsg{1}.K;

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

% [bbox, scores, labels, annot_color_img] = test_scene_net(scene_detector, color_img, 2);
[bbox, score_idx, bbox_idx, scores, labels, annot_color_img, img_cuts, n] = return_boxes(scene_detector, color_img, aligned_img, n);

shape_array = zeros(n, 3);
for i=1:n
    ix = bbox_idx(i);
    row = bbox(ix, :);
    [x_, y_, w_, h_] = deal(row(1), row(2), row(3), row(4));
    [u, v] = calc_centroid(x_, y_, w_, h_); % bounding box centre pixel

    d = aligned_img(v, u); % get depth at img centroid

    X = ((u - cx) * d) / fx;
    Y = ((v - cy) * d) / fy;
    Z = d;

    disp(['idx: ', num2str(ix)]);
    disp([num2str(X), ', ', num2str(Y), ', ', num2str(Z)])

    annot_color_img = insertMarker(annot_color_img, [u, v],'x', 'color', 'green', 'size', 10);
    shape_array(i, :) = [X, Y, Z];
end
imshow(annot_color_img)


shape_labels = strings(n, 1);
figure;
for i=1:n
    subplot(1, n, i)
    [x, y, w, h] = deal(img_cuts(i, 1), img_cuts(i, 2), img_cuts(i, 3), img_cuts(i, 4));
    img_cut = color_img(y:y+h-1, x:x+w-1, :);
    [R, map] = imresize(img_cut, [224, 224]); % image size
    [shape_label, probability] = classify(net, R);
    imshow(img_cut), title(strcat([char(shape_label), ': ', num2str(max(probability)*100, 5)]));
    shape_labels(i) = char(shape_label);
end


figure;
% Set Variables

view(2);
ws = [-0.1 0.9 -0.4 0.4 0 0.4];

[so, co, po, ro] = deal(0.017, 0.015, 0.023, 0.028);
[ry, gy, by] = deal(-0.05, 0, 0.05);

dobot = Dobot(ws, '1');
dobot.model.teach
hold on

cam = Objects('res/obj/intel_d435.ply', [0.8, 0, 0.2], [0, 0, 0]);
cam.rot([0, 0, pi/2])


for i=1:n
    objects{i} = Objects(strcat('res/shapes/', shape_labels(i), '.ply'), [0.4 ry so], [0 0 0]);
    ry = ry + 0.05;
end


% axis equal
camlight


%% 7. Subscribe to camera, output world coords


close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');

n = 5; % num features in img

load('scene_detector.mat')
load('shape_detector.mat')

% Show RGB Image
RGB_data = rossubscriber("/camera/color/image_raw");
color_img = readImage(RGB_data.receive);
figure;
imshow(color_img)

% Show Depth Image
D_data = rossubscriber('/camera/depth/image_rect_raw');
depth_img = readImage(D_data.receive);

% Show Aligned RGB-D Image
AD_data = rossubscriber('/camera/aligned_depth_to_color/image_raw');
aligned_img = readImage(AD_data.receive);


% Show Camera Intrinsics
info = rossubscriber('/camera/aligned_depth_to_color/camera_info');
intrinsic_matrix = info.receive.K;

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

% [bbox, scores, labels, annot_color_img] = test_scene_net(scene_detector, color_img, 2);
[bbox, score_idx, bbox_idx, scores, labels, annot_color_img, img_cuts, n] = return_boxes(scene_detector, color_img, aligned_img, n);

shape_array = zeros(n, 3);
for i=1:n
    ix = bbox_idx(i);
    row = bbox(ix, :);
    [x_, y_, w_, h_] = deal(row(1), row(2), row(3), row(4));
    [u, v] = calc_centroid(x_, y_, w_, h_); % bounding box centre pixel

    d = aligned_img(v, u); % get depth at img centroid

    X = ((u - cx) * d) / fx;
    Y = ((v - cy) * d) / fy;
    Z = d;

    disp(['idx: ', num2str(ix)]);
    disp([num2str(X), ', ', num2str(Y), ', ', num2str(Z)])

    annot_color_img = insertMarker(annot_color_img, [u, v],'x', 'color', 'green', 'size', 10);
    shape_array(i, :) = [X, Y, Z];
end
imshow(annot_color_img)


shape_labels = strings(n, 1);
figure;
for i=1:n
    subplot(1, n, i)
    [x, y, w, h] = deal(img_cuts(i, 1), img_cuts(i, 2), img_cuts(i, 3), img_cuts(i, 4));
    img_cut = color_img(y:y+h-1, x:x+w-1, :);
    [R, map] = imresize(img_cut, [224, 224]); % image size
    [shape_label, probability] = classify(net, R);
    imshow(img_cut), title(strcat([char(shape_label), ': ', num2str(max(probability)*100, 5)]));
    shape_labels(i) = char(shape_label);
end


figure;
% Set Variables

view(2);
ws = [-0.1 0.9 -0.4 0.4 0 0.4];

[so, co, po, ro] = deal(0.017, 0.015, 0.023, 0.028);
[ry, gy, by] = deal(-0.05, 0, 0.05);

dobot = Dobot(ws, '1');
dobot.model.teach
hold on

cam = Objects('res/obj/intel_d435.ply', [0.8, 0, 0.2], [0, 0, 0]);
cam.rot([0, 0, pi/2])


for i=1:n
    objects{i} = Objects(strcat('res/shapes/', shape_labels(i), '.ply'), [0.4 ry so], [0 0 0]);
    ry = ry + 0.05;
end


% axis equal
camlight

