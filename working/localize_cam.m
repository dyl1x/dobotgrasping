set(0, 'DefaultFigureWindowStyle', 'docked')
load('../scene_detector.mat');
load('../shape_detector.mat');
rosinit;

%% manual setup for calibration
%set end effector to position
endEffectorPosition = [0.2,0,0]; % home
endEffectorRotation = [0,0,0]; % home

[targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');

targetEndEffectorMsg.Position.X = endEffectorPosition(1);
targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
targetEndEffectorMsg.Position.Z = endEffectorPosition(3);

qua = eul2quat(endEffectorRotation);
targetEndEffectorMsg.Orientation.W = qua(1);
targetEndEffectorMsg.Orientation.X = qua(2);
targetEndEffectorMsg.Orientation.Y = qua(3);
targetEndEffectorMsg.Orientation.Z = qua(4);

send(targetEndEffectorPub,targetEndEffectorMsg);

%keyboard
%%

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

%% ESTOP

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3;
send(safetyStatePublisher,safetyStateMsg);

%% localize camera
close all

camera_offset =  [0,0,0];
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
%intrinsic_matrix = [917.3447265625, 0.0, 636.757568359375, 0.0, 916.9466552734375, 340.2231750488281, 0.0, 0.0, 1.0];

color_limit = 1.1; % 1.1 - 1.5
n = 1; % num features in img

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

%%
b2c_rot = troty(-pi/2) * trotz(pi/2) * trotx(cam_rot); % rotation component
b2c = eye(4) * transl(camera_offset) * b2c_rot; % base to camera
[objects, world_coords, world_transforms] = camera2base(shape_array, shape_labels, b2c, b2c_rot, n);

ncol = ones(1, 3);
dest = zeros(4, 4, n);
for i=1:n
    [dest_, ncol] = colour_shape_dest(objects, i, ncol, true);
    dest(:, :, i) = dest_;
end

%% calculate offset
eePos = [0.2,0,0];
guess = world_coords(:,:,1);
camera_offset = eePos - guess;
disp(camera_offset)