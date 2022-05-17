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
toolStateMsg.Data = [1]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);



%% Real Robot Trace Path IR

z = 0.0683;

% cart = [
%     0.2, 0.12, 0.15;
%     0.25, 0.12, 0.15;
%     0.25, 0.12, z;
%     0.25, -0.12, z;
%     0.25, -0.12, 0.08;
%     0.23, -0.12, 0.08;
%     0.23, -0.12, z;
%     0.23, -0.02, z;
%     0.18, 0, z;  
%     0.23, 0.02, z;
%     0.23, 0.12, z;
%     0.23, 0.12, 0.15;
% ];

cart = [
    0.2, 0.12, 0.15;
    0.25, 0.12, 0.15;
    
    0.25, 0.12, z;
    0.25, -0.12, z;
    

];


q0 = [0, 0, 0, 0]; % Dobot has 4 joints

% subscribers
safety_sub = rossubscriber('/dobot_magician/safety_status');
ee_sub = rossubscriber('/dobot_magician/end_effector_poses');

% publishers
[target_ee_pub, target_ee_msg] = rospublisher('/dobot_magician/target_end_effector_pose');
[tool_pub, tool_msg] = rospublisher('/dobot_magician/target_tool_state');
[safety_pub, safety_msg] = rospublisher('/dobot_magician/target_safety_status');
[target_joint_pub, target_joint_msg] = rospublisher('/dobot_magician/target_joint_states');
pause(2);


% 2. Send zero joint angles

traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
traj_point.Positions = q0;
target_joint_msg.Points = traj_point;
send(target_joint_pub, target_joint_msg);
% keyboard % manual continue awaiting completion of joint movement


% 3. Get current pose
[ee_msg, ee_pose] = get_current_pose(ee_sub);



% 5. Loop through start to finish
for i=1:size(cart)

    % move from home to start shape
    pt = cart(i, :);

    target_ee_msg.Position.X = pt(1);
    target_ee_msg.Position.Y = pt(2);
    target_ee_msg.Position.Z = pt(3);

    qua = eul2quat(zeros(1, 3));
    target_ee_msg.Orientation.W = qua(1);
    target_ee_msg.Orientation.X = qua(2);
    target_ee_msg.Orientation.Y = qua(3);
    target_ee_msg.Orientation.Z = qua(4);

    send(target_ee_pub, target_ee_msg);
    pause(0.5);

end

disp("Completed Path")


%% RMRC Controlled Real Robot Path



cart = [
    0.25, 0.13, 0.1;
    0.25, -0.13, 0.1;
];

q0 = zeros(1, 3); % Dobot has 4 joints

% subscribers
safety_sub = rossubscriber('/dobot_magician/safety_status');
ee_sub = rossubscriber('/dobot_magician/end_effector_poses');

% publishers
[target_ee_pub, target_ee_msg] = rospublisher('/dobot_magician/target_end_effector_pose');
[tool_pub, tool_msg] = rospublisher('/dobot_magician/target_tool_state');
[safety_pub, safety_msg] = rospublisher('/dobot_magician/target_safety_status');
[target_joint_pub, target_joint_msg] = rospublisher('/dobot_magician/target_joint_states');
pause(1);


% 2. Send zero joint angles

traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
traj_point.Positions = q0;
target_joint_msg.Points = traj_point;
send(target_joint_pub, target_joint_msg);

        
L1 = Link('d', 0.1, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', deg2rad([-135, 135]));      % Base
L2 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', -pi/2, 'qlim', deg2rad([5, 85]));           % Rear Arm
L3 = Link('d', 0, 'a', 0.147, 'alpha', 0, 'offset', pi/2, 'qlim', deg2rad([15, 170]));         % Forearm

model = SerialLink([L1 L2 L3], 'name', 'sim');
model.plot([0 0 0])

% 4. Loop through start to finish

pt = cart(1, :);
next_pose = ee_pose;
next_pose(1:3, 4) = pt;
[qmatrix, desired] = rmrc(ee_pose, next_pose, q0, model, false, 6, 0.2, model.n);


for i=1:length(qmatrix)
    traj_point = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    traj_point.Positions = [qmatrix(1, :), 0];
    target_joint_msg.Points = traj_point;
    send(target_joint_pub, target_joint_msg);
    pause(2)
    disp(i)
end


disp("Completed Path")


%%


function [ee_msg, ee_pose] = get_current_pose(ee_sub)
    ee_msg = ee_sub.LatestMessage;
    ee_pos = [ee_msg.Pose.Position.X, ...
              ee_msg.Pose.Position.Y, ...
              ee_msg.Pose.Position.Z];
    ee_quat = [ee_msg.Pose.Orientation.W, ...
               ee_msg.Pose.Orientation.X, ...
               ee_msg.Pose.Orientation.Y, ...
               ee_msg.Pose.Orientation.Z];
    eul = quat2eul(ee_quat);
    ee_pose = rpy2tr(eul(1), eul(2), eul(3));
    ee_pose(1:3, 4) = ee_pos';
    pause(0.2);
end

