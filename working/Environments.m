% creates the sorrounding and spwan in static props and the dynamic bricks.
% uses the set of code from canvas to plot images on the surfaces using
% surf.
% Imports the static props using code from week 4 lab (moving pen example)
% it didnt work for the bricks so bricks are using code from monkey head
% and r2d2 example. link in brickv2.m


classdef Environments < handle
    properties
        envSize = 2; % default value

        pcbPos = zeros(9,3); % brick positions
        height = 2.5; % room height
        pcbArray; %s tores all the brick obkects so we can call them as brickArray(x)
    end
    
    methods
        function self = Environments(size)
            %set environment size and other variables from the inputs
            self.envSize = size; 
            
            self.GenerateEnv();
            self.LoadProps();
            
        end
        
        % setup the floor, walls and warning sighs on the walls
        % images from google.
        % the estop stand model is mine.
        % other models are not mine, but coloured by me.
        function GenerateEnv(self)
            
            disp('generating environment ...');

            floorSize = self.envSize + 0.5;
            surf([-1.25, -1.25; 0.8, 0.8],[-0.8, 0.8; -0.8, 0.8],[0.01,0.01;0.01,0.01],...
                'CData', imread('images/floor.jpg'), 'FaceColor','texturemap');
            hold on

            axis equal
            
            disp('environment completed');
            
            hold off
        end

        
        %% load props
        function LoadProps(self)
            
            hold on

            % 1st dobot table
            mesh_h = PlaceObject('models/table.PLY');
            tf = trotz(-pi/2) * transl(-0.05, 0, 0.01)';
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            % 2nd dobot table
            mesh_h = PlaceObject('models/table.PLY');
            tf = transl(0.1, -0.42, 0.01)';
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));       

            % conveyor incoming
            mesh_h = PlaceObject('models/conveyor.PLY');
            tf = trotz(-pi/2) * transl(-0.33, 0.26, 0.01)';
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            % conveyor outgoing
            mesh_h = PlaceObject('models/conveyor.PLY');
            tf = transl(-0.7, -0.4, 0.01)';
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            % E-Stop
            mesh_h = PlaceObject('models/estop.ply');
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0.05, 0.2, 0.5)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

            % Fence
            mesh_h = PlaceObject('models/scaled_fence.PLY');
            tf = transl(-0.8, 0, 0.01)' * trotz(pi/2);
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            % Fence
            mesh_h = PlaceObject('models/scaled_fence.PLY');
            tf = transl(-1.2, 0.3, 0.01)';
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tf;
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
        end

    end
end