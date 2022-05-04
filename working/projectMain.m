%% project main
% things from this will be moved to functions and buttons in the GUI
% we can use this to map out the main functionality of the project

close all;
clearvars;
clc;
figure;
set(0,'DefaultFigureWindowStyle','docked');

%% Inputs
%preset the base position of the ur5 and ur3
r1base = transl(0.2,0,0.490)*trotz(pi);
r2base = transl(-0.2,0,0.490)*trotz(-pi/2);

% size of fenced room, we assume length = width for simplicity
% height is a constant value of 3m.
mapSize = 0.1;

clf
map = Environments(mapSize);

hold on

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
qHome = [0,0,0,0];

r1 = Dobot(ws,1);
r1.model.base = transl(0.2,0,0.490)*trotz(pi);

r2 = Dobot(ws,2);
r2.model.base = transl(-0.2,0,0.490)*trotz(-pi/2);

r1.model.animate(qHome);
r2.model.animate(qHome);

drawnow();
axis equal
hold off

%% add pcbs
hold on
pcb1 = PCB(1,transl(-0.375,0,0.5));
pcb2 = PCB(2,transl(-0.375,0.3,0.5));
pcb3 = PCB(3,transl(-0.375,0.5,0.5));

%% set quide positions
qPickup = [-1.5551,    0.0017,   -0.1257,         0];
qmid = [0,    0.4171,   -0.4869,         0];
qdrop = [1.5708,    0.0017,   -0.1257,         0];

%% pick and place pcb 1
r2qCurrent = r2.model.getpos;
r2EndPos = r2.model.fkine(r2qCurrent);
goalEndPos = pcb1.pose;
qGoal = r2.model.ikcon(goalEndPos);
qmatrix = jtraj(r2qCurrent,qGoal,50);

for i=1:size(qmatrix)
   r2.model.animate(qmatrix(i,:)); 
   drawnow();
   pause(0.01);
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
