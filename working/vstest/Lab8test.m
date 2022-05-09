%% Robotics
% Lab 8 - Visual Servoing with UR10 


close all
clear all


% 1.1 Definitions


% Create image target (points in the image plane) 
pStar = [662 362 362 662; 362 362 662 662];


pcent = [2,0,1];
%Create 3D points
P=[pcent(1,1),pcent(1,1),pcent(1,1),pcent(1,1);
   pcent(1,2)-0.25,pcent(1,2)+0.25,pcent(1,2)+0.25,pcent(1,2)-0.25;
   pcent(1,3)+0.25,pcent(1,3)+0.25,pcent(1,3)-0.25,pcent(1,3)-0.25;];


% Make a UR10
r = UR10();             

%Initial pose
q0 = [pi/2; -pi/3; -pi/3; -pi/6; 0; 0];

% Add the camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

% frame rate
fps = 25;

%Define values
%gain of the controler
lambda = 0.6;
%depth of the IBVS
depth = mean (P(1,:));

% 1.2 Initialise Simulation (Display in 3D)

%Display UR10
Tc0= r.model.fkine(q0);
r.model.animate(q0');
drawnow

% plot camera and points
cam.T = Tc0;

% Display points in 3D and the camera
cam.plot_camera('Tcam',Tc0, 'label','scale',0.15);
plot_sphere(P, 0.05, 'b')
lighting gouraud
light

% 1.3 Initialise Simulation (Display in Image view)

%Project points to the image
p = cam.plot(P, 'Tcam', Tc0);

%camera view and plotting
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P);    % show initial view

%%
vsloop(cam,depth,lambda,fps,pStar,P,r,q0);

%%
%Initialise display arrays
vel_p = [];
uv_p = [];
history = [];
 
