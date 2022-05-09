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
%% loop 2
 
 ksteps = 0;
 while true
        ksteps = ksteps + 1;
        
        % compute the view of the camera
        uv = cam.plot(P);
        
        % compute image plane error as a column
        e = pStar-uv;   % feature error
        e = e(:);
        Zest = [];
        
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P);
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            J = cam.visjac_p(uv, depth );
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = r.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        r.model.animate(q');

        %Get camera location
        Tc = r.model.fkine(q);
        cam.T = Tc;

        drawnow
        
        pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
 end %loop finishes