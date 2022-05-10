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
            
            disp('generating environment...');
            %surf(x,y,z)
            %x = [left top, right top, left b, right b]
            %floor
            floorSize = self.envSize +1;
            surf([-floorSize, -floorSize; floorSize, floorSize],[-floorSize, floorSize; -floorSize, floorSize],[0.01,0.01;0.01,0.01],...
                'CData', imread('images/floor.jpg'), 'FaceColor','texturemap');
            hold on
            %fenced walls
%             surf([-self.envSize, -self.envSize; self.envSize, self.envSize],[self.envSize, self.envSize; self.envSize, self.envSize],[0.01,self.height;0.01,self.height], ...
%                 'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');
% 
%             surf([-self.envSize, -self.envSize; self.envSize, self.envSize],[-self.envSize, -self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
%                 'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');
% 
%             surf([-self.envSize, -self.envSize; -self.envSize, -self.envSize],[self.envSize, self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
%                 'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');
% 
%             %backwall
%             surf([self.envSize, self.envSize; self.envSize, self.envSize],[self.envSize, self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
%                 'CData', imread('images/wall.jpeg'), 'FaceColor','texturemap');
%             %warning
%             surf([self.envSize-0.01, self.envSize-0.01; self.envSize-0.01, self.envSize-0.01],[self.envSize, self.envSize-1; self.envSize, self.envSize-1],[self.height,self.height;self.height-0.5,self.height-0.5],...
%                 'CData', imread('images/robotsselfaware.jpg'), 'FaceColor','texturemap');
%             %door
%             surf([-self.envSize, -self.envSize; -self.envSize, -self.envSize],[-(self.envSize-2), -(self.envSize-0.5); -(self.envSize-2), -(self.envSize-0.5)],[2, 2; 0.01, 0.01],...
%                 'CData', imread('images/door.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');
            
            axis equal
            
            disp('environment gerating comleted,');
            
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
            tf = transl(0.1, -0.45, 0.01)';
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

            mesh_h = PlaceObject('models/estop.ply');
            
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(deg2rad(30))';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(self.envSize-0.5,self.envSize/2,0.99)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));

        end

    end
end