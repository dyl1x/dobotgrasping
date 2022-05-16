
function [qMatrix, x] = rmrc(start, finish, q0, model, plot, path, weight, n)
    if ~exist('n', 'var'), n = 5; end
    if ~exist('weight', 'var'), weight = 1; end
    
    % 1.1) Set parameters for the simulation
    t = 1;                           % Total time (s)
    deltaT = 0.02;                   % Control frequency
    steps = t/deltaT;                % No. of steps for simulation
    delta = 2*pi/steps;              % Small angle change
    epsilon = 1E-6;                   % Threshold value for manipulability/Damped Least Squares
    W = diag([1 1 1 0.01 0.01 0.01]);   % Weighting matrix for the velocity vector
    
    % 1.2) Allocate array data
    m = zeros(steps, 1);             % Array for Measure of Manipulability
    qMatrix = zeros(steps, n);       % Array for joint anglesR
    qdot = zeros(steps, n);          % Array for joint velocities
    theta = zeros(3, steps);         % Array for roll-pitch-yaw angles
    x = zeros(3, steps);             % Array for x-y-z trajectory
    positionError = zeros(3, steps); % For plotting trajectory error
    angleError = zeros(3, steps);    % For plotting trajectory error
    
    % 1.3) Set up trajectory, initial pose
    start_pos = start(1:3, 4)';
    finish_pos = finish(1:3, 4)';

    start_rot = start(1:3, 1:3);
    finish_rot = finish(1:3, 1:3);

    s = lspb(0,1,steps);             % Trapezoidal trajectory scalar
    if path == 1 % straight line
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1);       % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);       % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);       % Points in z
            theta(:, i) = rotm2eul(1-s(i) * start_rot + s(i) * finish_rot);% Roll, Pitch, Yaw angle 
        end
    elseif path == 2
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1) + weight*sin((i/2)*delta); % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);                % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);                % Points in z
            theta(1,i) = 0;                        % Roll angle 
            theta(2,i) = 0;                        % Pitch angle
            theta(3,i) = 0;                        % Yaw angle
        end
    elseif path == 3 % y sin change
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1);       % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2) + weight*sin((i/2)*delta); % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);       % Points in z
            theta(:, i) = rotm2eul(1-s(i) * start_rot + s(i) * finish_rot);% Roll, Pitch, Yaw angle 
        end 
    elseif path == 4 % x and y sin change
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1) + weight*sin((i/2)*delta);  % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2) + weight*sin((i/2)*delta);  % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3);                            % Points in z
            theta(:, i) = rotm2eul(1-s(i) * start_rot + s(i) * finish_rot);                     % Roll, Pitch, Yaw angle 
        end
    elseif path == 5 % x and z sin change
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1) + weight*sin((i/2)*delta);      % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);                                % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3) + (-weight/2)*sin((i/2)*delta); % Points in y                
            theta(:, i) = rotm2eul(1-s(i) * start_rot + s(i) * finish_rot);                         % Roll, Pitch, Yaw angle 
        end
    elseif path == 6 % x and z sin change
        for i = 1:steps
            x(1,i) = (1-s(i)) * start_pos(1) + s(i) * finish_pos(1) + weight * sin(i * delta);      % Points in x
            x(2,i) = (1-s(i)) * start_pos(2) + s(i) * finish_pos(2);                              % Points in y
            x(3,i) = (1-s(i)) * start_pos(3) + s(i) * finish_pos(3) + weight * sin(i * delta); % Points in y                
            theta(:, i) = rotm2eul(1-s(i) * start_rot + s(i) * finish_rot);                       % Roll, Pitch, Yaw angle 
        end 
    end
    
    T = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) x(:,1); zeros(1,3) 1];  % Create transformation of first point and angle
%     qMatrix(1,:) = model.ikcon(T, q0 );                                  % Solve joint angles to achieve first waypoint
    qMatrix(1,:) = model.ikine(T, model.getpos, [1 1 1 0 0 0]);                      % Solve joint angles to achieve first waypoint
    
    % 1.4) Track the trajectory with RMRC
    for i=1:steps-1
%         disp(i);
        T = model.fkine(qMatrix(i, :));                                    % Get forward transformation at current joint state
        deltaX = x(:, i+1) - T(1:3, 4);                                    % Get position error from next waypoint
        
        Rd = rpy2r(theta(1, i+1), theta(2, i+1), theta(3, i+1));           % Get next RPY angles, convert to rotation matrix
        Ra = T(1:3, 1:3);                                                  % Current end-effector rotation matrix
        Rdot = (1 / deltaT) * (Rd - Ra);                                   % Calculate rotation matrix error
        S = Rdot * Ra';                                                    % Skew symmetric!
        
        linear_velocity = (1 / deltaT) * deltaX;
        angular_velocity = [S(3, 2); S(1, 3); S(2, 1)];                    % Check the structure of Skew Symmetric matrix!!
        deltaTheta = tr2rpy(Rd * Ra');                                     % Convert rotation matrix to RPY angles
        
        xdot = W * [linear_velocity; angular_velocity];                    % Calculate end-effector velocity to reach next waypoint.
        J = model.jacob0(qMatrix(i, :));                                   % Get Jacobian at current joint state
        m(i) = sqrt(det(J*J'));
        
        if m(i) < epsilon                                                  % If manipulability is less than given threshold
            lambda = (1 - m(i)/epsilon)*5E-2;
            if ~isreal(lambda), lambda = 0; end
            
        else
            lambda = 0;
        end
        
        invJ = inv(J' * J + lambda * eye(n)) * J';                         % DLS Inverse
        qdot(i,:) = (invJ * xdot)';                                        % Solve the RMRC equation (you may need to transpose the vector)
        
        for j = 1:n                                                        % Loop through joints 1 to 5
            if qMatrix(i, j) + deltaT * qdot(i, j ) < model.qlim(j, 1)     % If next joint angle is lower than joint limit...
                qdot(i, j) = 0;                                            % Stop the motor
            elseif qMatrix(i, j) + deltaT * qdot(i, j) > model.qlim(j, 2)  % If next joint angle is greater than joint limit ...
                qdot(i, j) = 0;                                            % Stop the motor
            end
            if j == 4, qMatrix(i, 4) = constrain_joint4(qMatrix(i, 2), qMatrix(i, 3)); end
        end
        
        qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot(i,:);                % Update next joint state based on joint velocities
        positionError(:,i) = x(:,i+1) - T(1:3,4);                          % For plotting
        angleError(:,i) = deltaTheta;                                      % For plotting
%         model.animate(qMatrix(i, :));
%         model.teach;
%         pause(0.5);
    end
    
    if plot == true
        
        % 1.5) Plot the results
        figure(2)        
        for i = 1:6
            figure(3)
            subplot(3,2,i)
            plot(qMatrix(:,i),'k','LineWidth',1)
            title(['Joint ', num2str(i)])
            ylabel('Angle (rad)')
            refline(0, model.qlim(i,1));
            refline(0, model.qlim(i,2));
            
            figure(4)
            subplot(3,2,i)
            plot(qdot(:,i),'k','LineWidth',1)
            title(['Joint ',num2str(i)]);
            ylabel('Velocity (rad/s)')
            refline(0,0)
        end
        
        figure(5)
        subplot(2,1,1)
        plot(positionError'*1000,'LineWidth',1)
        refline(0,0)
        xlabel('Step')
        ylabel('Position Error (mm)')
        legend('X-Axis','Y-Axis','Z-Axis')
        
        subplot(2,1,2)
        plot(angleError','LineWidth',1)
        refline(0,0)
        xlabel('Step')
        ylabel('Angle Error (rad)')
        legend('Roll','Pitch','Yaw')
        figure(5)
        plot(m,'k','LineWidth',1)
        refline(0,epsilon)
        title('Manipulability')
    end
end




