% robot with camera at the end

clf
clear all

ws = [-0.5 0.5 -0.5 0.5 0 0.8];
r =  Dobot(ws,1, 2);

q0 = [0,0,pi/2,constrain_joint4(0,pi/2),0];

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

r.model.animate(q0);
Tc0= r.model.fkine(q0)*trotx(pi);
drawnow

cam.T = Tc0;

cam.plot_camera('Tcam',Tc0, 'label','scale',0.025);

% goals
% pStar = [662 362 362 662; 362 362 662 662];

% pStar = [362 662 362 662; 362 362 662 662];

pStar = [362 662;512 512];



cent = transl(0.187,0,0.0) * trotz(0.1);

hold on
 target = Target(cent);
P = getP(cent,0.05,2);
%
% P = getP(cent,0.075,4);
% 
% pl1 = plot_sphere(P(:,1), 0.01, 'b');
% pl2 = plot_sphere(P(:,2), 0.01, 'b');
% pl3 = plot_sphere(P(:,3), 0.01, 'b');
% pl4 = plot_sphere(P(:,4), 0.01, 'b');

% P = [ 0.157    0.217;
%       0         0;
%       0         0];
% 
% P = getP(cent,0.05,2);
% 
% pl1 = plot_sphere(P(:,1), 0.01, 'b');
% pl2 = plot_sphere(P(:,2), 0.01, 'b');




% create the camera view
q0 = r.model.getpos;
Tc0 = r.model.fkine(q0) * trotx(pi);

cam.clf()
cam.plot(pStar, '*'); 
cam.hold(true);
cam.plot(P(:,1), 'Tcam', Tc0, 'o')
cam.plot(P(:,2), 'Tcam', Tc0, 'x')
%d = 0.2130
%%
% realDepth = [getDist(cent,Tc0);getDist(cent,Tc0)];
realDepth = getDist(cent,Tc0);
vsloop(cam,realDepth,0.6,25,pStar,P,r,q0');

%%

%while(1)
    % cecks to see if we can see all 4 points
    q0 = r.model.getpos;
    Tc0 = r.model.fkine(q0) * trotx(pi)*troty(pi/2);
    p = cam.plot(P,'Tcam',Tc0);
    
    if ~(length(pStar) > length(p)) 
        %we can see all the points
        % get depth and compare
        realDepth = getDist(cent,Tc0);
        
        if (realDepth < 0.213)
            % perform movement.
            disp('vsloop');
            vsloop(cam,realDepth,0.3,25,pStar,P,r,q0');
        else
            disp('safe');
        end 
    else
        disp('cant see symbol');
    end 
%end