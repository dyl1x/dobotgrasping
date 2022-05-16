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

ws = [-0.5 0.5 -0.5 0.5 -0.3 0.8];
r = Dobot(ws, 2, true, 2);
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

h = 0.51;
pt = 0.03;

ws_origin = transl(-0.03, -0.42, h) * trotz(pi/2);

stow_away = [ws_origin(1:3, 1:3), ws_origin(1:3, 4) - [0.1; -0.1; 0]; zeros(1, 3), 1;];
conv_out = transl(-0.3, -0.4, h);

map = Environments(0.1);
hold on

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
[q2_, q3_] = deal(deg2rad(35), deg2rad(105));
q0 = [-pi/2 q2_ q3_ constrain_joint4(q2_, q3_) 0];

r1 = Dobot(ws, 1, 2);
r1.model.base = transl(-0.1, -0.15, 0.49) * trotz(pi);
r1.model.animate(q0);

r2 = Dobot(ws, 2, 2);
r2.model.base = transl(0.2, -0.45, 0.49) * trotz(pi);
r2.model.animate(q0);
hold on

drawnow();
axis equal

pcb1 = PCB(1, transl(-0.33, 0, h) * trotz(pi/2));
pcb2 = PCB(2, transl(-0.31, 0.3, h));
pcb3 = PCB(3, transl(-0.32, 0.6, h));
pcbs = [pcb1, pcb2, pcb3];

view([0 0 1]);

centerPoint = [0, 0, 0];
radii = [0.07, 0.07, 0.04];
[X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
for i = 1:length(q0)
    r1.model.points{i} = [X(:),Y(:),Z(:)];
    r1.model.faces{i} = delaunay(r1.model.points{i});    
end

r1.model.plot3d(q0);

% animate traj arguments
% init joint, pcb, robot, false/obj, path 1-5, weight, plot, attach obj

for i=1:length(pcbs)
    disp(' ')
    disp(strcat(['PCB: ', num2str(i)]));

    disp('a. Move R1 from q0 to pcb1')
    animate_traj(q0, pcbs(i).pose, r1.model, false, 1, 0, false, false);
    
    disp('b. Move R1 to ws origin')
    animate_traj(r1.model.getpos, ws_origin, r1.model, pcbs(i), 5, -0.1, false, true)
    
    disp('c. Move R1 to stow away pose for laser operation')
    animate_traj(r1.model.getpos, stow_away, r1.model, false, 1, false, false, false)
    
    disp('d. Move R2 to ws origin')
    animate_traj(q0, ws_origin, r2.model, false, 1, false, false, false)
    
    % Insert trace paths here
    trace_path(pcbs(i), r2.model, true)
    
    disp('f. Move R2 to q0')
    animate_traj(q0, r2.model.fkine(q0), r2.model, false, 1, false, false, false)
    r2.model.animate(q0);
    
    disp('g. Move R1 from stow away to pcb1')
    animate_traj(r1.model.getpos, pcbs(i).pose, r1.model, pcbs(i), 1, false, false, false)
    
    disp('h. Move R1 from ws origin to conveyor out with pcb1')
    animate_traj(r1.model.getpos, conv_out, r1.model, pcbs(i), 1, false, false, true)
    
    disp('i. Move R1 from conveyor out to q0')
    animate_traj(r1.model.getpos, r1.model.fkine(q0), r1.model, false, 5, -0.2, false, false)

    % 8a. Animate Conveyor
    animate_conveyor(pcbs(i), pcbs(i).pose, pcbs(i).pose - transl(0.7, 0, 0), 30);
    if i < 2 animate_conveyor(pcbs(2), pcbs(2).pose, pcbs(2).pose - transl(0, 0.3, 0), 30); end
    if i < 3 animate_conveyor(pcbs(3), pcbs(3).pose, pcbs(3).pose - transl(0, 0.3, 0), 30); end

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


function animate_traj(q, dest, model, obj, path, weight, plot, move_ply)
    if ~exist('pt', 'var'), pt = 0.02; end

    current_pose = model.fkine(model.getpos);
    
    [qmatrix, desired] = rmrc(current_pose, dest, q, model, false, path, weight);
    
    if plot == true, plot3(desired(1, :), desired(2, :), desired(3, :), 'y.', 'LineWidth', 1); end % plot
    
    for i=1:length(qmatrix)
       model.animate(qmatrix(i, :));
       ee = model.fkine(qmatrix(i, :));

       if plot == true, plot3(ee(1, 4), ee(2, 4), ee(3, 4), 'b*'); end
       if move_ply == true, obj.MoveMesh(ee); end

       pause(pt);
    end
    if move_ply == true, obj.MoveMesh(dest); end

end

function animate_conveyor(obj, start, finish, steps)
    if ~exist('pt', 'var'), pt = 0.02; end
    start_pos = start(1:3, 4)';
    finish_pos = finish(1:3, 4)';

    x = zeros(3, steps);
    theta = zeros(3, 3, steps);

    s = lspb(0,1,steps);
    for i = 1:steps
        x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1);       % Points in x
        x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);       % Points in y
        x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);       % Points in z
    end

    for i=1:steps
        obj.tran(x(:, i)');
        pause(pt);
    end
end

function trace_path(obj, model, plot)
    if ~exist('pt', 'var'), pt = 0.02; end
    % trace devel from centre pose of shape for translations agnostic of the
    % initial coords

    if obj.type == 1
        plt = [];
        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(-0.02, 0.07, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, false, false);
%         if plot == true, plot3(desired(1, :), desired(2, :), desired(3, :), 'y.', 'LineWidth', 1); end % plot

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0.04, 0, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(-0.04, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0.04, 0, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, cp + transl(0, -0.04, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, true, 'c*');

        cp = model.fkine(model.getpos);
        [qmatrix, desired] = rmrc(cp, model.base + transl(-0.3, -0.05, 0), model.getpos, model, false, 1, false);
        plt = loop_qmatrix(qmatrix, model, plt, false, false);

        disp(strcat(['    Completed trace path for PCB ', num2str(obj.type)]));
        pause(3);

        for i=1:length(plt), delete(plt{i}); end

    end
end

function plots = loop_qmatrix(qmatrix, model, plots, plot, linespec)
    if ~exist('linespec', 'var'), linespec = 'b*'; end
    if ~exist('pt', 'var'), pt = 0.02; end

    for i=1:length(qmatrix)
           model.animate(qmatrix(i, :));
           ee = model.fkine(qmatrix(i, :));
           if plot == true
               plt = plot3(ee(1, 4), ee(2, 4), ee(3, 4), linespec, 'MarkerSize', 0.1); 
               plots{length(plots)+1} = plt;
           end
           pause(pt);
    end
end

function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)

algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centerPoint(3))/radii(3)).^2;
end