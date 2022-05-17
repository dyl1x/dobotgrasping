% functionified loop to run in the background
% moves the robot uding visual servoing
% author: Chamath Edirisinhege
% Used lab 8 solutions of 41013 by Gavin/Teresa
% slightly changed
% INPUTS:
% cam - camera object
% depth - depth value - avg dist from camerea to tracking boject
% lambda - gain of controller (like 0.6)
% fps - movemnt speed ( give 1 for default)
% pStar - target points
% P - 3D world coordinated of the points being tracked 
% r - robot object
% q0 - transpose of starting pos of robot

function vsloop(cam,depth,lambda,fps,pStar,P,r,q0)
ksteps = 0;
while true
    ksteps = ksteps + 1;
    
    % compute the view of the camera
    uv = cam.plot(P);
    
    % compute image plane error as a column
    %e = pStar-uv;   % feature error
    e = uv-pStar;
    e = e(:);
    Zest = [];
    
    % compute the Jacobian
%     try
%         Tc = r.model.fkine(r.model.getpos)*trotx(pi);
%         point1 = eye(4);
%         point1(1:3,4) = P(:,1);
%         point2 = eye(4);
%         point2(1:3,4) = P(:,2);
%         depth = [getDist(point1,Tc);getDist(point2,Tc)];
%     catch
%         disp('no depth');
%     end
    
    if isempty(depth)
        % exact depth from simulation (not possible in practice)
        pt = homtrans(inv(Tcam), P);
        J = cam.visjac_p(uv, pt(3,:) );
    elseif ~isempty(Zest)
        J = cam.visjac_p(uv, Zest);
        disp('used zest');
    else
        J = cam.visjac_p(uv, depth );
    end
    
    % compute the velocity of camera in camera frame
    try
        v = lambda * pinv(J) * e;
    catch
        disp('v error');
        return
    end
    
    
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
    qr = q';
    qr(1,4) = constrain_joint4(qr(1,2),qr(1,3));
    r.model.animate(qr);
    
    %Get camera location
    Tc = r.model.fkine(r.model.getpos);
    %cam.T = Tc*transl(0,0,0.115) * trotx(pi)*troty(pi/2);
    cam.T = Tc*trotx(pi);
    drawnow
    
    pause(1/fps)
    
    if ~isempty(100) && (ksteps > 100)
        break;
    end
    
    %update current joint position
    q0 = qr';
end %loop finishes
disp('vs loop is done');

end