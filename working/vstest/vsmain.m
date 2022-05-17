% robot with camera at the end

close all
clear all

r = UR10();
q0 = [pi/2; -pi/3; -pi/3; -pi/6; 0; 0];

cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

r.model.animate(q0');
Tc0= r.model.fkine(q0);
drawnow

cam.T = Tc0;

cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);

% goals
pStar = [662 362 362 662; 362 362 662 662];

%% generate points

cent = transl(1,0,1.2) * troty(pi/2);
P = getP(cent,0.25);

pl1 = plot_sphere(P(:,1), 0.025, 'b');
pl2 = plot_sphere(P(:,2), 0.025, 'b');
pl3 = plot_sphere(P(:,3), 0.025, 'b');
pl4 = plot_sphere(P(:,4), 0.025, 'b');

ps = [pl1,pl2,pl3,pl4];

%%

cent =  cent * inv(cent);
P = getP(cent,0.25);

pl1 = plot_sphere(P(:,1), 0.025, 'b');
pl2 = plot_sphere(P(:,2), 0.025, 'b');
pl3 = plot_sphere(P(:,3), 0.025, 'b');
pl4 = plot_sphere(P(:,4), 0.025, 'b');

%%
delete(pl1);
delete(pl2);
delete(pl3);
delete(pl4);

%% create the camera view
cam.clf()
cam.plot(pStar, '*'); 
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o')
%%
% long while loop

%while(1)
    % cecks to see if we can see all 4 points
    q0 = r.model.getpos;
    Tc0 = r.model.fkine(q0);
    p = cam.plot(P,'Tcam',Tc0);
    
    if ~(length(pStar) > length(p)) 
        %we can see all the points
        % get depth and compare
        realDepth = getDist(cent,Tc0);
        
        if (realDepth < 0.65)
            % perform movement.
            disp('vsloop');
            vsloop(cam,realDepth,1,50,pStar,P,r,q0');
        else
            disp('safe');
        end 
    else
        disp('cant see symbol');
    end 
%end