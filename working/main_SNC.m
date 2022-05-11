%% SNC Group Assignment
% Dobot control and grasping

% Add toolboxes to readme, other load up conditions GoogLeNet


close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');

% 1. Capturing images from camera
% 2. ROS BAGS : Get Image, detect shape, simulate in scene with camera
% 3. ROS SUBS : Subscribe to camera, detect objects, calc c2b [x, y, z]

% 4. ROS BAGS : Load/Explore
% 5. ROS BAGS : Robot Control
% 6. ROS BAGS : Visualnetise as point cloud
% 7. ROS BAGS : Extract Image


%%
idx = 51;
%% 1. Capture Images from camera with ros subscription 

% rosinit()
sub = rossubscriber("/camera/color/image_raw");
figure(1)
i = readImage(sub.receive);
imshow(i)

% imwrite(i, ['working/images/scene_dataset/IMG_', num2str(idx), '.jpeg']);
% idx = idx + 1;

%% 2. ROS BAGS : extract img, detect shape, place in sim

load('scene_detector.mat')
load('shape_detector.mat')

%% 2.1 Calc camera coordinates from known calibration image


filename = 'local3'; % n = 3
bag = rosbag(strcat(['bag/', filename, '.bag']));


%% 2. 
% close all
% clc

color_limit = 1.3;
steps = 50; % trajectories
n = 3; % num features in img

filename = 'RealRobotTest1'; % n = 3
% filename = 'RealRobotTest2'; % n = 5
% filename = 'AllShapesTest'; % n = 12
% filename = '3ObjAlignDepth'; % n = 3
% filename = 'CubesPyramidTest1'; % n = 6
% filename = 'CubesPyramidTest2'; % n = 6

bag = rosbag(strcat(['bag/', filename, '.bag']));

% camera transform
camera_offset =  [0.6, 0, 0.1];
cam_rot = deg2rad(-8); % x rot if cam facing down toward surface

% Get RGB Image
RGB_data = select(bag, 'Topic', '/camera/color/image_raw');
firstColourImage = readMessages(RGB_data, 1);
color_img = readImage(firstColourImage{1,1});
% figure;
% imshow(color_img)

% Get Depth Image
D_data = select(bag, 'Topic', '/camera/depth/image_rect_raw');
firstDepthImage = readMessages(D_data, 1);
depth_img = readImage(firstDepthImage{1,1});

% Get Aligned RGB-D Image
AD_data = select(bag, 'Topic', '/camera/aligned_depth_to_color/image_raw');
firstADepthImage = readMessages(AD_data, 1);
aligned_img = readImage(firstADepthImage{1,1});

% Get Camera Intrinsics
info = select(bag, 'Topic', '/camera/aligned_depth_to_color/camera_info');
infoMsg = readMessages(info);
intrinsic_matrix = infoMsg{1}.K;

% Detect boxes
[bbox, score_idx, bbox_idx, scores, labels, annot_color_img, img_cuts, n] = return_boxes(scene_detector, color_img, aligned_img, n, color_limit);

[shape_array, annot_color_img] = calc_camera_coords(bbox, bbox_idx, aligned_img, annot_color_img, intrinsic_matrix, n);

imshow(annot_color_img) % annotated rgb image with bounding boxes and centre x's

% classify shapes in boxes
shape_labels = strings(n, 1);
figure;
for i=1:n
    subplot(1, n, i)
    [x, y, w, h] = deal(img_cuts(i, 1), img_cuts(i, 2), img_cuts(i, 3), img_cuts(i, 4));
    img_cut = color_img(y:y+h-1, x:x+w-1, :);
    [R, map] = imresize(img_cut, [224, 224]); % image size
    [shape_label, probability] = classify(shape_detector, R);
    imshow(img_cut), title(strcat([char(shape_label), ': ', num2str(max(probability)*100, 5)]));
    shape_labels(i) = char(shape_label);
end


% plot 3d model
figure;
view(2);

ws = [-0.1 0.8 -0.4 0.4 0 0.4];
dobot = Dobot(ws, '1', 2);
hold on

cam = Objects('res/obj/intel_d435.ply', camera_offset, [0, 0, 0]);
cam.rot([0, 0, pi/2])
cam.rot([cam_rot 0 0]) % 10 deg tilt down

axis equal
camlight

% base to camera frame
% trplot(eye(4), 'length', 0.3, 'color', 'r')
trplot(eye(4) * transl(camera_offset), 'length', 0.1, 'color', 'g')
b2c_rot = troty(-pi/2) * trotz(pi/2) * trotx(cam_rot); % rotation component
b2c = eye(4) * transl(camera_offset) * b2c_rot; % base to camera
trplot(b2c, 'length', 0.1, 'color', 'm')

% calculate world points
[objects, world_coords, world_transforms] = camera2base(shape_array, shape_labels, b2c, b2c_rot, n);

% animate pick and place control
% q0 = [0 pi/4 pi/2 pi/4 0];
q0 = [0 pi/4 pi/2 pi/4 0];
dobot.model.animate(q0);
ee = dobot.model.fkine(q0);

ncol = [1 1 1];
for i=1:n
    obj = [eye(3), world_transforms(1:3, 4, i); zeros(1, 3), 1];
    
    [dest, ncol] = colour_shape_dest(objects, i, ncol, false);
    
    q1 = dobot.model.ikcon(obj, q0);
    
    qmatrix = calc_trap_qmatrix(q0, q1, steps); % trapezoidal
    % qmatrix = jtraj(q0, q1, steps); % quintic
    
    for j=1:size(qmatrix)
        dobot.model.animate(qmatrix(j,:));
        pause(0.02);
    end
    
    q2 = dobot.model.getpos;
    q3 = dobot.model.ikcon(dest, q2);
    qmatrix = calc_trap_qmatrix(q2, q3, steps);
    
    for j=1:size(qmatrix)
        dobot.model.animate(qmatrix(j,:));
        ee_pos = dobot.model.fkine(dobot.model.getpos);
        objects{i}.tran(ee_pos(1:3, 4)' - objects{i}.pose(1:3, 4)');
        pause(0.02);
    end
    
    q4 = dobot.model.getpos;
    qmatrix = calc_trap_qmatrix(q4, q0, steps);
    
    for j=1:size(qmatrix)
        dobot.model.animate(qmatrix(j,:));
        pause(0.02);
    end
    pause(1);
end



%% 3. Subscribe to camera, output world coords
close all
clearvars
clc

load('scene_detector.mat')
load('shape_detector.mat')

color_limit = 1.2;
n = 6; % num features in img

% camera translation
camera_offset =  [0.7, 0, 0.06];
cam_rot = deg2rad(0); % x rot if cam facing down toward surface


% Show RGB Image
RGB_data = rossubscriber("/camera/color/image_raw");
color_img = readImage(RGB_data.receive);

% Show Depth Image
D_data = rossubscriber('/camera/depth/image_rect_raw');
depth_img = readImage(D_data.receive);

% Show Aligned RGB-D Image
AD_data = rossubscriber('/camera/aligned_depth_to_color/image_raw');
aligned_img = readImage(AD_data.receive);


% Show Camera Intrinsics
info = rossubscriber('/camera/aligned_depth_to_color/camera_info');
intrinsic_matrix = info.receive.K;

% Detect Boxes
[bbox, score_idx, bbox_idx, scores, labels, annot_color_img, img_cuts, n] = return_boxes(scene_detector, color_img, aligned_img, n, color_limit);

% calc 3d camera frame coordinates
[shape_array, annot_color_img] = calc_camera_coords(bbox, bbox_idx, aligned_img, annot_color_img, intrinsic_matrix, n);

imshow(annot_color_img)

shape_labels = strings(n, 1);
figure;
for i=1:n
    subplot(1, n, i)
    [x, y, w, h] = deal(img_cuts(i, 1), img_cuts(i, 2), img_cuts(i, 3), img_cuts(i, 4));
    img_cut = color_img(y:y+h-1, x:x+w-1, :);
    [R, map] = imresize(img_cut, [224, 224]); % image size
    [shape_label, probability] = classify(shape_detector, R);
    imshow(img_cut), title(strcat([char(shape_label), ': ', num2str(max(probability)*100, 5)]));
    shape_labels(i) = char(shape_label);
end

figure;
view(2);
ws = [-0.1 0.9 -0.4 0.4 0 0.4];

dobot = Dobot(ws, '1', 2);
hold on

cam = Objects('../res/obj/intel_d435.ply', camera_offset, [0, 0, 0]);
cam.rot([0, 0, pi/2])
cam.rot([cam_rot 0 0]) % 10 deg tilt down

axis equal
camlight

% base to camera frame
% trplot(eye(4), 'length', 0.3, 'color', 'r')
trplot(eye(4) * transl(camera_offset), 'length', 0.1, 'color', 'g')
b2c_rot = troty(-pi/2) * trotz(pi/2) * trotx(cam_rot); % rotation component
b2c = eye(4) * transl(camera_offset) * b2c_rot; % base to camera
trplot(b2c, 'length', 0.1, 'color', 'm')

% calculate world points
[objects, world_coords, world_transforms] = camera2base(shape_array, shape_labels, b2c, b2c_rot, n);


axis equal
camlight


