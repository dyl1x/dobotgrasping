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
        test_qmatrix;

    end
    
    methods
        function self = Dobot(workspace, num, c1, c2)    
        if ~exist('c1', 'var'), c1 = false; end
        if ~exist('c2', 'var'), c2 = 1; end

        self.GetDobotRobot(num, c1);

        self.PlotAndColourRobot(workspace, c2);
        end
        
        %% GetDobotRobot
        
        function GetDobotRobot(self, num, c)
        
            % Create a unique name (ms timestamp after 1ms pause)
            pause(0.01);
            self.name = ['Dobot_', num2str(num, '%d')];
        
            L1 = Link('d', 0.1, 'a', 0, 'alpha', -pi/2, 'offset', 0, 'qlim', deg2rad([-135, 135]));      % Base
            L2 = Link('d', 0, 'a', 0.135, 'alpha', 0, 'offset', -pi/2, 'qlim', deg2rad([5, 85]));           % Rear Arm
            L3 = Link('d', 0, 'a', 0.147, 'alpha', 0, 'offset', 0, 'qlim', deg2rad([15, 170]));         % Forearm
            
            if ~c
                L4 = Link('d', 0, 'a', -0.04, 'alpha', -pi/2, 'offset', pi/2, 'qlim', deg2rad([-90, 90]));
                L5 = Link('d', -0.05, 'a', 0, 'alpha', 0, 'offset', pi, 'qlim', deg2rad([-90, 90]));            % End effector
            else
                L4 = Link('d', 0.065, 'a', -0.05, 'alpha', -pi/2, 'offset', pi/2, 'qlim', deg2rad([-90, 90]));
                L5 = Link('d', -0.02, 'a', 0, 'alpha', 0, 'offset', pi, 'qlim', deg2rad([0, 0]));
            end

            self.model = SerialLink([L1 L2 L3 L4 L5], 'name', self.name);
        
        end
        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        function PlotAndColourRobot(self, workspace, c)

            if c == 2
                for linkIndex = 0:self.model.n % d0.ply - d5.ply
                    if linkIndex == 4
                        self.model.faces{linkIndex+1} = [];
                        self.model.points{linkIndex+1} = [];
                    elseif linkIndex == 5
                        disp(strcat(['laser_gripper.ply']))
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['../res/dobot/laser_gripper.ply'], 'tri');
                        self.model.faces{linkIndex+1} = faceData;
                        self.model.points{linkIndex+1} = vertexData;
                    else
                        disp(strcat(['d', num2str(linkIndex), '.ply']))
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['../res/dobot/d',num2str(linkIndex),'.ply'],'tri');
               
                        self.model.faces{linkIndex+1} = faceData;
                        self.model.points{linkIndex+1} = vertexData;
                    end
                end
            else
                for linkIndex = 0:self.model.n % d0.ply - d5.ply
    
                    disp(strcat(['d', num2str(linkIndex), '.ply']))
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread(['../res/dobot/d',num2str(linkIndex),'.ply'],'tri');
           
                    self.model.faces{linkIndex+1} = faceData;
                    self.model.points{linkIndex+1} = vertexData;
                end
            end
        
            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'noarrow','workspace',workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;
        
            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
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

        function calc_volume(self, degrees, silent)
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
                            for q5 = qlim(5, 1):steps:qlim(5, 2)
                                q = [q1, q2, q3, constrain_joint4(q2, q3), q5];
    
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
            end
            if silent
                hold on
                plot3(point_cloud(:, 1), point_cloud(:, 2), point_cloud(:, 3), 'r-');
            end

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

        function test_pos(self, q)

            q(:, :, 1) = [0 1.13 1.84 0.18 0];
            q(:, :, 2) = [0 0.98 1.92 0.24 0];
            q(:, :, 3) = [0 0.82 1.99 0.32 0];
            q(:, :, 4) = [0 0.67 2.04 0.42 0];
            q(:, :, 5) = [0 0.52 2.08 0.54 0];
            q(:, :, 6) = [0 0.38 2.09 0.67 0];
            
            q(:, :, 7) = [0 1.17 1.68 0.29 0];
            q(:, :, 8) = [0 1.02 1.77 0.35 0];
            q(:, :, 9) = [0 0.88 1.84 0.42 0];
            q(:, :, 10) = [0 0.74 1.88 0.51 0];
            q(:, :, 11) = [0 0.61 1.91 0.62 0];
            q(:, :, 12) = [0 0.49 1.92 0.73 0];
            
            q(:, :, 13) = [0 1.22 1.52 0.41 0];
            q(:, :, 14) = [0 1.08 1.6 0.46 0];
            q(:, :, 15) = [0 0.95 1.67 0.52 0];
            q(:, :, 16) = [0 0.82 1.71 0.6 0];
            q(:, :, 17) = [0 0.71 1.74 0.7 0];
            q(:, :, 18) = [0 0.59 1.75 0.8 0];
            
            q(:, :, 19) = [0 1.28 1.33 0.53 0];
            q(:, :, 20) = [0 1.15 1.42 0.57 0];
            q(:, :, 21) = [0 1.03 1.48 0.63 0];
            q(:, :, 22) = [0 0.91 1.53 0.7 0];
            q(:, :, 23) = [0 0.8 1.56 0.78 0];
            q(:, :, 24) = [0 0.7 1.56 0.87 0];
            
            q(:, :, 25) = [0 1.37 1.11 0.66 0];
            q(:, :, 26) = [0 1.25 1.2 0.69 0];
            q(:, :, 27) = [0 1.13 1.27 0.74 0];
            q(:, :, 28) = [0 1.02 1.32 0.8 0];
            q(:, :, 29) = [0 0.91 1.35 0.88 0];
            q(:, :, 30) = [0 0.82 1.36 0.96 0];
            
            q(:, :, 31) = [0 1.4 0.99 0.76 0];
            q(:, :, 32) = [0 1.36 0.95 0.83 0];
            q(:, :, 33) = [0 1.24 1.03 0.87 0];
            q(:, :, 34) = [0 1.14 1.08 0.92 0];
            q(:, :, 35) = [0 1.04 1.11 0.99 0];
            q(:, :, 36) = [0 0.96 1.12 1.06 0];

            self.test_qmatrix = q;
        end
    end
end