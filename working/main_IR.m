%% project main
% things from this will be moved to functions and buttons in the GUI
% we can use this to map out the main functionality of the project

close all;
clearvars;
clc;
figure;
set(0,'DefaultFigureWindowStyle','docked');

%% joint angles from video
close all

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
r = Dobot(ws, 2, 2);
view([0 -1 0])

n = 36;
q = zeros(1, 5, n); 

% copied from video (Dobot moving through Cartesian Space)
r.test_pos(q) % create test_qmatrix

for i=1:2
    r.model.animate(r.test_qmatrix(:, :, i));
    pause(0.5);
end

% r.model.animate(r.test_qmatrix(:, :, 1));
% view(3)
% r.calc_volume(15, true);

%% Inputs

clearvars
close all
clc

h = 0.5;
pt = 0.03;

ws_origin = transl(-0.1, -0.42, h);
conv_out = transl(-0.4, -0.4, h);


mapSize = 0.1;
map = Environments(mapSize);
hold on

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
[q2_, q3_] = deal(deg2rad(45), deg2rad(115));
q0 = [-3*pi/8 q2_ q3_ constrain_joint4(q2_, q3_) 0];

r1 = Dobot(ws, 1, 2);
r1.model.base = transl(-0.1, -0.15, 0.49) * trotz(pi);
r1.model.animate(q0);

r2 = Dobot(ws, 2, 2);
r2.model.base = transl(0.2, -0.45, 0.49) * trotz(-3*pi/4);
r2.model.animate(q0);
hold on

drawnow();
axis equal

pcb1 = PCB(1, transl(-0.375, -0.1, 0.5));
pcb2 = PCB(2, transl(-0.375, 0.3, 0.5));
pcb3 = PCB(3, transl(-0.375, 0.5, 0.5));

% set quide positions
qPickup = [-1.5551, 0.0017, -0.1257, 0];
qmid = [0, 0.4171, -0.4869, 0];
qdrop = [1.5708, 0.0017, -0.1257, 0];


% pick and place pcb 1
view([0 0 1]);

% 1. Move from zero pos to pcb 1
animate_traj(q0, pcb1.pose, r1.model, false, 1, 0, false, false);

% 2. Move to origin of workspace
animate_traj(r2.model.getpos, ws_origin, r1.model, pcb1, 5, -0.1, false, true)

% % 3. Move to joint pickup pos
% animate_traj(q0, ws_origin, r2.model, false, 5, -0.1, true, false)
% 
% % 4. Move to zero pos
% animate_traj(r1.model.getpos, r1.model.fkine(q0), r1.model, pcb1, 5, -0.1, true, true)


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


function animate_traj(q, dest, model, obj, path, weight, plot, move_ply)
    if ~exist('pt', 'var'), pt = 0.03; end

    current_pose = model.fkine(model.getpos);
    
    [qmatrix, desired] = rmrc(current_pose(1:3, 4)', dest(1:3, 4)', q, model, false, path, weight);
    
    if plot == true, plot3(desired(1, :), desired(2, :), desired(3, :), 'y.', 'LineWidth', 1); end % plot
    
    for i=1:length(qmatrix)
       model.animate(qmatrix(i, :));
       ee = model.fkine(qmatrix(i, :));

       if plot == true, plot3(ee(1, 4), ee(2, 4), ee(3, 4), 'b*'); end
       if move_ply == true, obj.MoveMesh(ee); end

       pause(pt);
    end
end


