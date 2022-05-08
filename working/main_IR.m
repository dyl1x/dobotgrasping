%% project main
% things from this will be moved to functions and buttons in the GUI
% we can use this to map out the main functionality of the project

close all;
clearvars;
clc;
figure;
set(0,'DefaultFigureWindowStyle','docked');

%% joint angles from video

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
r = Dobot(ws, 2, 2);
view([0 -1 0])

n = 36;
q = zeros(1, 5, n); 

% copied from video (Dobot moving through Cartesian Space)
r.test_pos(q) % create test_qmatrix

for i=1:n
    r.model.animate(r.test_qmatrix(:, :, i));
    pause(0.5);
end

%% Inputs
%preset the base position of the ur5 and ur3
r1base = transl(0.2,0,0.490) * trotz(pi);
r2base = transl(-0.2,0,0.490) * trotz(-pi/2);

% size of fenced room, we assume length = width for simplicity
% height is a constant value of 3m.
mapSize = 0.1;

clf
% map = Environments(mapSize);

hold on

ws = [-0.5 0.5 -0.5 0.5 0 0.8]; % q2: 0-pi/2 q3: pi/2-pi
[q2_, q3_] = deal(pi/4, 3*pi/4);
q0 = [-3*pi/8 q2_ q3_ constrain_joint4(q2_, q3_) 0];

r1 = Dobot(ws, 1, 2);
r1.model.base = transl(0.2,0,0.49) * trotz(pi);

r2 = Dobot(ws, 2, 2);
r2.model.base = transl(-0.15,0,0.49) * trotz(-3*pi/4);

r1.model.animate(q0);
r2.model.animate(q0);

drawnow();
axis equal
hold off

% add pcbs
hold on
pcb1 = PCB(1,transl(-0.375,-0.1,0.5));
pcb2 = PCB(2,transl(-0.375,0.3,0.5));
pcb3 = PCB(3,transl(-0.375,0.5,0.5));

% set quide positions
qPickup = [-1.5551, 0.0017, -0.1257, 0];
qmid = [0, 0.4171, -0.4869, 0];
qdrop = [1.5708, 0.0017, -0.1257, 0];


% pick and place pcb 1
current_pose = r2.model.fkine(r2.model.getpos);
q1 = r2.model.ikcon(pcb1.pose, q0);
[qmatrix, desired] = rmrc(current_pose(1:3, 4)', pcb1.pose(1:3, 4)', q0, r2.model, false, false);
% qmatrix = calc_trap_qmatrix(q0, q1, 10);

plot3(desired(1, :), desired(2, :), desired(3, :), 'm.', 'LineWidth', 1);

for i=1:size(qmatrix)
   r2.model.animate(qmatrix(i,:)); 
   pause(0.03);
end

%% lift up
r2qCurrent = r2.model.getpos;
qmatrix = jtraj(r2qCurrent,qPickup,50);

for i=1:size(qmatrix)
   r2.model.animate(qmatrix(i,:)); 
   trPCB = r2.model.fkine(qmatrix(i,:));
   pcb1.MoveMesh(trPCB);
   drawnow();
   pause(0.01);
end

%% turn
r2qCurrent = r2.model.getpos;
qmatrix = jtraj(r2qCurrent,qdrop,50);

for i=1:size(qmatrix)
   r2.model.animate(qmatrix(i,:)); 
   trPCB = r2.model.fkine(qmatrix(i,:));
   pcb1.MoveMesh(trPCB);
   drawnow();
   pause(0.01);
end
%% Pick and return pcb 1
