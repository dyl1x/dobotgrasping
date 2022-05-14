rosinit;

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

while (currentSafetyStatus ~= 4) % reinitialize
    disp('reinit...');
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    safetyStateMsg.Data = 2;
    send(safetyStatePublisher,safetyStateMsg);
    pause(5);
    
    disp('checking...');
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
    pause(0.1);
end

disp('proceding...');

%% Get current tool state

toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
pause(2); %Allow some time for MATLAB to start the subscriber
currentToolState = toolStateSubscriber.LatestMessage.Data;

if currentToolState == 1   
    % Turn off the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [0]; % Send 1 for on and 0 for off
    send(toolStatePub,toolStateMsg);   
end
%% ESTOP

%run this section to stop

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3;
send(safetyStatePublisher,safetyStateMsg);
%%
%%
endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
pause(2); %Allow some time for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z];
% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];
% Convert from quaternion to euler
%[roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

eul = quat2eul(currentEndEffectorQuat');     


%%

% dropPoints = [-0.0564,-0.226,-0.039;
%                0.0151,-0.226,-0.039;
%                0.07445,-0.226,-0.039];
%           
% pickPoints = [0.2599, -0.1526,-0.038;
%               0.1716,-0.05384,-0.038;
%               0.29066,0.06964,-0.038];


dropPoints = [0.07445,-0.226,-0.040;
               0.07445,-0.226,-0.01;
               0.07445,-0.226,0.015];
          
pickPoints = [0.2599, -0.1526,-0.043;
              0.1716,-0.05384,-0.043;
              0.29066,0.06964,-0.039];

%%
% move to pick point 1
%% Set joint state - home

% you can publish a JointTrajectory message with a single joint position,
% as the driver currently does not support a joint trajectory

jointTarget = [0,0,0,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

%% pick point
%endEffectorPosition = pickPoints(3,:); % home
endEffectorPosition = [0.2,0,0];
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

%% activate tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);
%% home

endEffectorPosition = [0.153,0,0.00]; % home
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

%% drop point
endEffectorPosition = dropPoints(3,:); % home
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

%% deactivate tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);



%% 1. Loop, Stack, unstack, return shapes (Real Robot Control)

start = [0.2599, -0.1526, -0.043;
         0.1716, -0.05384, -0.043;
         0.29066, 0.06964, -0.039];

% dest = [-0.0564,-0.226,-0.039; % rows
%          0.0151,-0.226,-0.039;
%          0.07445,-0.226,-0.039];

dest = [0.07445, -0.226, -0.040; % stack
        0.07445, -0.226, -0.01;
        0.07445, -0.226, 0.015];

q0 = zeros(1, 4); % Dobot has 4 joints

% subscribers
safety_sub = rossubscriber('/dobot_magician/safety_status');
ee_sub = rossubscriber('/dobot_magician/end_effector_poses');

% publishers
[target_ee_pub, target_ee_msg] = rospublisher('/dobot_magician/target_end_effector_pose');
[tool_pub, tool_msg] = rospublisher('/dobot_magician/target_tool_state');
[safety_pub, safety_msg] = rospublisher('/dobot_magician/target_safety_status');
[target_joint_pub, target_joint_msg] = rospublisher('/dobot_magician/target_joint_states');
pause(2);


% 1. Check safety status

safety_status = safety_sub.LatestMessage.Data;
while (safety_status ~= 4) % reinitialize
    disp('reinit ...');
    safety_msg.Data = 2;
    send(safety_pub, safety_msg);
    pause(3);
    
    disp('checking ...');
    safety_status = safety_sub.LatestMessage.Data;
    pause(0.1);
end
disp('proceeding ...');


% 2. Send zero joint angles

traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
traj_point.Positions = q0;
target_joint_msg.Points = traj_point;
send(target_pub, target_joint_msg);
keyboard % manual continue awaiting completion of joint movement


% 3. Get current pose
[ee_msg, eul] = get_current_pose(ee_sub);
keyboard % display msg


% 4. Hover above shapes to prealign
for i=1:size(start)

    % move from home to start shape
    start_pt = start(i, :);

    target_ee_msg.Position.X = start_pt(1);
    target_ee_msg.Position.Y = start_pt(2);
    target_ee_msg.Position.Z = start_pt(3) + 0.05;

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_pub, target_ee_msg);
    keyboard
end


% 5. Loop through start to finish
for i=1:size(start)

    % move from home to start shape
    start_pt = start(i, :);

    target_ee_msg.Position.X = start_pt(1);
    target_ee_msg.Position.Y = start_pt(2);
    target_ee_msg.Position.Z = start_pt(3);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_pub, target_ee_msg);
    keyboard

    % activate tooltip
    tool_msg.Data = 1; 
    send(tool_pub, tool_msg);
    pause(0.2);

    % move to dest
    % move from home to start shape
    dest_pt = dest(i, :);

    target_ee_msg.Position.X = dest_pt(1);
    target_ee_msg.Position.Y = dest_pt(2);
    target_ee_msg.Position.Z = dest_pt(3);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_pub, target_ee_msg);
    keyboard

    % deactivate tooltip
    tool_msg.Data = 0; 
    send(tool_pub, tool_msg);
    pause(0.2);

end

disp("Completed Path")

disp('Reverse?')
keyboard
for i=1:size(start)

    % move to dest
    % move from home to dest shape
    dest_pt = dest(i, :);

    target_ee_msg.Position.X = dest_pt(1);
    target_ee_msg.Position.Y = dest_pt(2);
    target_ee_msg.Position.Z = dest_pt(3);


    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_pub, target_ee_msg);
    keyboard

    % activate tooltip
    tool_msg.Data = 1; 
    send(tool_pub, tool_msg);
    pause(0.2);

    % move from home to start shape
    start_pt = start(i, :);

    target_ee_msg.Position.X = start_pt(1);
    target_ee_msg.Position.Y = start_pt(2);
    target_ee_msg.Position.Z = start_pt(3);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_pub, target_ee_msg);
    keyboard

    % deactivate tooltip
    tool_msg.Data = 0; 
    send(tool_pub, tool_msg);
    pause(0.2);

end



%% 2. Connect to Camera to feed points for Real Dobot

load('../scene_detector.mat');
load('../shape_detector.mat');
%%
color_limit = 1.2; % 1.1 - 1.5
n = 3; % num features in img

% camera translation
camera_offset =  [0.7340   -0.0270    0.0720];
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
%%
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

ncol = ones(1, 3);
dest = zeros(4, 4, n);
for i=1:n
    [dest_, ncol] = colour_shape_dest(objects, i, ncol, true);
    dest(:, :, i) = dest_;
end

%%

q0 = zeros(1, 4); % Dobot has 4 joints

% subscribers
safety_sub = rossubscriber('/dobot_magician/safety_status');
ee_sub = rossubscriber('/dobot_magician/end_effector_poses');

% publishers
[target_ee_pub, target_ee_msg] = rospublisher('/dobot_magician/target_end_effector_pose');
[tool_pub, tool_msg] = rospublisher('/dobot_magician/target_tool_state');
[safety_pub, safety_msg] = rospublisher('/dobot_magician/target_safety_status');
[target_joint_pub, target_joint_msg] = rospublisher('/dobot_magician/target_joint_states');
pause(2);


% 1. Check safety status

safety_status = safety_sub.LatestMessage.Data;
while (safety_status ~= 4) % reinitialize
    disp('reinit ...');
    safety_msg.Data = 2;
    send(safety_pub, safety_msg);
    pause(3);
    
    disp('checking ...');
    safety_status = safety_sub.LatestMessage.Data;
    pause(0.1);
end
disp('proceeding ...');


% 2. Send zero joint angles

traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
traj_point.Positions = q0;
target_joint_msg.Points = traj_point;
send(target_joint_pub, target_joint_msg);
pause(1); %keyboard % manual continue awaiting completion of joint movement


% 3. Get current pose
[ee_msg, eul] = get_current_pose(ee_sub);
% keyboard % display msg


% 4. Hover above shapes to prealign
wc = size(world_coords);
for i=1:wc(3)

    % move from home to start shape
    start_pt = world_coords(:, :, i);

    target_ee_msg.Position.X = start_pt(1);
    target_ee_msg.Position.Y = start_pt(2);
    target_ee_msg.Position.Z = start_pt(3) + 0.05;

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_ee_pub, target_ee_msg);
    pause(0.3);
%     keyboard
end


% 5. Loop through start to finish
for i=1:wc(3)

    % move from home to start shape
    start_pt = world_coords(:, :, i);

    target_ee_msg.Position.X = start_pt(1);
    target_ee_msg.Position.Y = start_pt(2);
    target_ee_msg.Position.Z = start_pt(3);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_ee_pub, target_ee_msg);
    keyboard

    % activate tooltip
    tool_msg.Data = 1; 
    send(tool_pub, tool_msg);
    pause(0.2);
    
    % phome
    target_ee_msg.Position.X = 0.17;
    target_ee_msg.Position.Y = -0.15;
    target_ee_msg.Position.Z = 0.2;
    
    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);
    
    send(target_ee_pub, target_ee_msg);
    pause(0.3);

    % move to dest
    % move from home to start shape
    dest_pt = dest(:, :, i);

    target_ee_msg.Position.X = dest_pt(1, 4);
    target_ee_msg.Position.Y = dest_pt(2, 4);
    target_ee_msg.Position.Z = dest_pt(3, 4);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_ee_pub, target_ee_msg);
    keyboard

    % deactivate tooltip
    tool_msg.Data = 0; 
    send(tool_pub, tool_msg);
    pause(0.2);

end





function [ee_msg, eul] = get_current_pose(ee_sub)
    ee_msg = ee_sub.LatestMessage;
    ee_pos = [ee_msg.Pose.Position.X, ...
              ee_msg.Pose.Position.Y, ...
              ee_msg.Pose.Position.Z];
    ee_quat = [ee_msg.Pose.Orientation.W, ...
               ee_msg.Pose.Orientation.X, ...
               ee_msg.Pose.Orientation.Y, ...
               ee_msg.Pose.Orientation.Z];
    eul = quat2eul(ee_quat);
    pause(0.2);
end

