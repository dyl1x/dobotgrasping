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