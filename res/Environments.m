% clreates the sorrounding and spwan in static props and the dynamic bricks.
% uses the set of code from canvas to plot images on the surfaces using
% surf.
% Imports the static props using code from week 4 lab (moving pen example)
% it didnt work for the bricks so bricks are using code from monkey head
% and r2d2 example. link in brickv2.m


classdef Environments < handle
    properties
        envSize = 2; % default value
        ur3Pos;
        ur5Pos;
        brickPos = zeros(9,3); % brick positions
        
        height = 3; % room height
        brickArray; %s tores all the brick obkects so we can call them as brickArray(x)
    end
    
    methods
        function self = Environments(size,ur3,ur5)
            %set environment size and other variables from the inputs
            self.envSize = size; 
            self.ur3Pos = ur3;
            self.ur5Pos = ur5;
            
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
            surf([-self.envSize, -self.envSize; self.envSize, self.envSize],[self.envSize, self.envSize; self.envSize, self.envSize],[0.01,self.height;0.01,self.height], ...
                'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');

            surf([-self.envSize, -self.envSize; self.envSize, self.envSize],[-self.envSize, -self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
                'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');

            surf([-self.envSize, -self.envSize; -self.envSize, -self.envSize],[self.envSize, self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
                'CData', imread('images/fence.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');

            %backwall
            surf([self.envSize, self.envSize; self.envSize, self.envSize],[self.envSize, self.envSize; -self.envSize, -self.envSize],[0.01,self.height;0.01,self.height],...
                'CData', imread('images/wall.jpeg'), 'FaceColor','texturemap');
            %warning
            surf([self.envSize-0.01, self.envSize-0.01; self.envSize-0.01, self.envSize-0.01],[self.envSize, self.envSize-1; self.envSize, self.envSize-1],[self.height,self.height;self.height-0.5,self.height-0.5],...
                'CData', imread('images/robotsselfaware.jpg'), 'FaceColor','texturemap');
            %door
            surf([-self.envSize, -self.envSize; -self.envSize, -self.envSize],[-(self.envSize-2), -(self.envSize-0.5); -(self.envSize-2), -(self.envSize-0.5)],[2, 2; 0.01, 0.01],...
                'CData', imread('images/door.jpg'),'FaceAlpha','texturemap', 'FaceColor','texturemap');
            
            axis equal
            
            disp('environment gerating comleted,');
            self.envSize
            hold off
        end

        
        %% load props
        function LoadProps(self)
            hold on
            
            %meshload code from lab4
            
            % back stand and estop
            mesh_h = PlaceObject('models/stand.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(self.envSize-0.5,self.envSize-0.5,0.01)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            mesh_h = PlaceObject('models/estop.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(deg2rad(30))';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(self.envSize-0.5,self.envSize-0.5,0.99)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            % front stand and estop
            mesh_h = PlaceObject('models/stand.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-self.envSize-0.5,-self.envSize,0.01)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            mesh_h = PlaceObject('models/estop.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * troty(deg2rad(30))';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-self.envSize-0.5,-self.envSize,0.99)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            
            mesh_h = PlaceObject('models/fireextinguisher.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(self.envSize-0.5,self.envSize-0.2,0.01)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            mesh_h = PlaceObject('models/fireextinguisher.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(-self.envSize-0.5,-(self.envSize-0.4),0.01)';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            
            %ends here
            
            hold off
        end
%% generate brick positions
        % randomly generate coordinates and pick 9 of the suitable ones for
        % the bricks to spawn on.
        %not the best if the ur3 base is nonzero
        function GenerateBrickPos(self)
            disp('generating brick positions around the UR3 base')
            ur3 = self.ur3Pos(1:2,4)';
            rng(0,'twister');
            upperlim = max(ur3)+2;
            lowerlim = min(ur3)-2;
            
            r = (upperlim - lowerlim).*rand(300,1) + lowerlim;
            
            brickpos = zeros(9,3);
            count = 1;
            for i =1: size(r)-1
                tr = [r(i),r(i+1)];
                
                d = sqrt((tr(1,1)-ur3(1,1))*tr(1,1)-ur3(1,1) + (tr(1,2)-ur3(1,2))*tr(1,2)-ur3(1,2));
                if d > 0.05 && d < 0.7
                    brickpos(count,1:2) = tr;
                    count = count +1;
                    if count > 9
                        break;
                    end
                end                
            end
            
            % addjust z so they are on the same plane as the base of the
            % robot
%             for j=1:9
%                 self.brickPos(i:3) = self.ur3Pos(1,3);
%             end
            
            self.brickPos = brickpos;                                  
        end
%%  spawn in some bricks
        function SpawnBricks(self)
            
            hold on
            brick1 = Brickv2(transl(self.brickPos(1,:)));
            brick2 = Brickv2(transl(self.brickPos(2,:)));
            brick3 = Brickv2(transl(self.brickPos(3,:)));
            brick4 = Brickv2(transl(self.brickPos(4,:)));
            brick5 = Brickv2(transl(self.brickPos(5,:)));
            brick6 = Brickv2(transl(self.brickPos(6,:)));
            brick7 = Brickv2(transl(self.brickPos(7,:)));
            brick8 = Brickv2(transl(self.brickPos(8,:)));
            brick9 = Brickv2(transl(self.brickPos(9,:)));
            hold off
            
            %check em in the array
            self.brickArray = [brick1,brick2,brick3,brick4,brick5,brick6,brick7,brick8,brick9];
            
            disp('Brick positions generated and models have been loaded');
        end
       
    end
end