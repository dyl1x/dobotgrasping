%% Use this if the driver is on your Linux laptop
% roslaunch dobot_magician_driver dobot_magician.launch

rosinit;

%% Get the current safety status of the robot

safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data

%   Stfety codes
%     INVALID - 0
%     DISCONNECTED - 1
%     INITIALISING - 2
%     ESTOPPED - 3
%     OPERATING - 4
%     PAUSED - 5
%     STOPPED - 6

% If the safety code is not 4 reinitialize robot

%% Initialize or reinitialize robot
[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 2;
send(safetyStatePublisher,safetyStateMsg);

%% ESTOP

[safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
safetyStateMsg.Data = 3;
send(safetyStatePublisher,safetyStateMsg);

%% Gets joint state

jointStateSubscriber = rossubscriber('/dobot_magician/joint_states'); % Create a ROS Subscriber to the topic joint_states
pause(2); % Allow some time for a message to appear
currentJointState = jointStateSubscriber.LatestMessage.Position % Get the latest message

%% Get current end effector pose

endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses'); % Create a ROS Subscriber to the topic end_effector_poses
pause(2); %Allow some time for MATLAB to start the subscriber
currentEndEffectorPoseMsg = endEffectorPoseSubscriber.LatestMessage;
% Extract the position of the end effector from the received message
currentEndEffectorPosition = [currentEndEffectorPoseMsg.Pose.Position.X,
                              currentEndEffectorPoseMsg.Pose.Position.Y,
                              currentEndEffectorPoseMsg.Pose.Position.Z]
% Extract the orientation of the end effector
currentEndEffectorQuat = [currentEndEffectorPoseMsg.Pose.Orientation.W,
                          currentEndEffectorPoseMsg.Pose.Orientation.X,
                          currentEndEffectorPoseMsg.Pose.Orientation.Y,
                          currentEndEffectorPoseMsg.Pose.Orientation.Z];
% Convert from quaternion to euler
%[roll,pitch,yaw] = quat2eul(currentEndEffectorQuat);

eul = quat2eul(currentEndEffectorQuat');

%% Set joint state

% you can publish a JointTrajectory message with a single joint position,
% as the driver currently does not support a joint trajectory

jointTarget = [0,0,0,0]; % Remember that the Dobot has 4 joints by default.

[targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
trajectoryPoint.Positions = jointTarget;
targetJointTrajMsg.Points = trajectoryPoint;

send(targetJointTrajPub,targetJointTrajMsg);

%% Set target end effector state

% note that this is also the original end effector pose of the Dobot without any tool attached
endEffectorPosition = [0,0,0]; % home
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

%%  Tool state


%     ON - 1
%     OFF - 0

%% Get current tool state

toolStateSubscriber = rossubscriber('/dobot_magician/tool_state');
pause(2); %Allow some time for MATLAB to start the subscriber
currentToolState = toolStateSubscriber.LatestMessage.Data;

%% Set tool state - suction cup

% Turn on the tool
[toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
toolStateMsg.Data = [0]; % Send 1 for on and 0 for off 
send(toolStatePub,toolStateMsg);

%% IO ports

%     INVALID - 0
%     DIGITAL OUTPUT - 1
%     PWM OUTPUT - 2
%     DIGITAL INPUT - 3
%     ADC INPUT - 4
%     PULL-UP DIGITAL INPUT - 5
%     PULL-DOWN DIGITAL INPUT - 6
% 
% There are 20 IO ports on the Dobot, to know which one is which and supports
% what types of IO, please refer to page 29 - 32 of the Dobot's Use Manual.

% The default format of the received data is an array with size of 40. The 
% first 20 values are the corresponding IO Multiplexing of 20 ports and the 
% last 20 values represent the value of those respective ports.

% Get current IO data of the robot

ioDataSubscriber = rossubscriber('/dobot_magician/io_data');
pause(2); %Allow some time for MATLAB to start the subscriber
ioMultiplex = ioDataSubscriber.LatestMessage.Data(2:21);
ioData = ioDataSubscriber.LatestMessage.Data(23:42);

%% 

