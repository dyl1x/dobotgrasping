classdef Dobot < handle
    properties
        model;
        
        workspace = [];   
        volume = [];
        computed_volume;
        transversal_reach;
        vertical_reach;
        arc_radius;
        name;

    end
    
    methods
        function self = Dobot(workspace, num, tooltip)    

        self.GetDobotRobot(num);

        self.PlotAndColourRobot(workspace, tooltip);
        end
        
        %% GetDobotRobot
        
        function GetDobotRobot(self, num)
        
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.01);
            self.name = ['Dobot_', num2str(num, '%d')];
        
            L1 = Link('d', 0.138, 'a', 0, 'alpha', pi/2, 'offset', 0, 'qlim', [deg2rad(-135), deg2rad(135)]); % Base
            L2 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-5), deg2rad(80)]); % Rear Arm
            L3 = Link('d', 0, 'a', 0.147, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-45), deg2rad(45)]); % Forearm
            L4 = Link('d', 0, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', [deg2rad(90), deg2rad(-90)]);
            L5 = Link('d', 0.061, 'a', 0, 'alpha', 0, 'offset', 0, 'qlim', [deg2rad(-85), deg2rad(85)]); % End effector
            
            self.model = SerialLink([L1 L2 L3 L4 L5], 'name', self.name);
        
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self, workspace, tooltip)
            % tooltip: 1 - gripper, 2 - suction cup
            for linkIndex = 0:self.model.n
                if (linkIndex == 4)
                    continue
                elseif (linkIndex == 5)
                    if tooltip == 1
                        disp('d5.ply')
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('res/dobot/d5.ply','tri');
                    else
                        disp('d6.ply')
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread('res/dobot/d6_joined.ply','tri');
                    end
                else
                    disp(strcat(['d', num2str(linkIndex), '.ply']))
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['res/dobot/d',num2str(linkIndex),'.ply'],'tri');
                    
                end
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end
        
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
        
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                
                if (linkIndex == 4)
                else
                    handles = findobj('Tag', self.model.name);
                    h = get(handles,'UserData');
                    try
                        h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                            , plyData{linkIndex+1}.vertex.green ...
                            , plyData{linkIndex+1}.vertex.blue]/255;
                        h.link(linkIndex+1).Children.FaceColor = 'interp';
                    catch ME_1
                        disp(ME_1);
                        continue;
                    end
                end                 
            end
        end
        function calc_volume(self, degrees)
            steps = deg2rad(degrees);
            qlim = self.model.qlim;

            % Don't need to worry about joint 4
            size = prod(floor((qlim(1:3, 2)-qlim(1:3, 1)) / steps + 1));
            point_cloud = zeros(size, 3);
            cnt = 1;
            tic % start counter

            for q1 = qlim(1, 1):steps:qlim(1, 2)
                for q2 = qlim(2, 1):steps:qlim(2, 2)
                    for q3 = qlim(3, 1):steps:qlim(3, 2)
                        for q4 = qlim(4, 1):steps:qlim(4, 2)
                            q = [q1, q2, q3, q4];
    
                            tr = self.model.fkine(q);
                            point_cloud(cnt, :) = tr(1:3, 4)'; % ' is to get the transpose
    
                            cnt = cnt + 1;
                            if mod(cnt/size*100, 1) == 0
                                disp(['After ', num2str(toc), ' seconds, completed ', num2str((cnt/size) * 100), '% of poses of Dobot'])
                                self.volume = point_cloud;
                            end
                        end
                    end
                end
            end
            figure;
            plot3(point_cloud(:, 1), point_cloud(:, 2), point_cloud(:, 3), 'r.');

        end

        function get_reach(self)
            maxX = max(self.volume(:, 1)) - self.model.base(1, 4);
            minX = min(self.volume(:, 1)) - self.model.base(1, 4);
            maxY = max(self.volume(:, 2)) - self.model.base(2, 4);
            minY = min(self.volume(:, 2)) - self.model.base(2, 4);
            maxZ = max(self.volume(:, 3)) - self.model.base(3, 4);
            minZ = min(self.volume(:, 3)) - self.model.base(3, 4);

            x_reach = max(maxX, abs(minX));
            y_reach = max(maxY, abs(minY));
            self.transversal_reach = max(x_reach, y_reach);
            self.vertical_reach = max(maxZ, abs(minZ));
            self.arc_radius = (self.vertical_reach / 2) + ((self.transversal_reach)^2 / 8 * self.vertical_reach);
        end

        function plot(self)
            [k, self.computed_volume] = convhull(self.volume(:, 1), self.volume(:, 2), self.volume(:, 3));
            trisurf(k, self.volume(:, 1), self.volume(:, 2), self.volume(:, 3), 'Facecolor', 'cyan');

        end
    end
end