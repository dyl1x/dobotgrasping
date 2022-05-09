
%    Objects class for handling data from camera and calculating results on
%    that data

classdef Objects < handle
    properties

        name;
        type;
        volume = [];
        centre;
        base;
        camera_coord;
        robot_coord;

        vertex_count;
        dest;
        pose;
        midpoint;
        verts;

        colour;
        shape;

        point;
        mesh;
        position;
        orientation;

    end

    methods

        function self = Objects(file, pos, rot)

            self.createObject(file, pos, rot);

        end

        function self = createObject(self, file, pos, rot)
%             if ~exist('file','var'), file = 'res/shapes/red_sphere.ply'; end
%             if ~exist('pos','var'), pos = [0, 0, 0]; end
%             if ~exist('rot','var'), rot = [0, 0, 0]; end

            [f, v, data] = plyread(file, 'tri');
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
        
            self.dest = eye(4);
            self.mesh = trisurf(f, v(:, 1), v(:, 2), v(:, 3) ...
                , 'FaceVertexCData', vertexColours, 'EdgeColor', 'interp', 'EdgeLighting', 'flat');
            self.vertex_count = size(v, 1);
            self.midpoint = sum(v) / self.vertex_count;
            self.verts = v - repmat(self.midpoint, self.vertex_count, 1);
            
            self.orientation = makehgtform('xrotate', rot(1, 1), 'yrotate', rot(1, 2), 'zrotate', rot(1, 3));
            self.position = makehgtform('translate', [pos(1, 1), pos(1, 2), pos(1, 3)]);
            self.pose = eye(4) * self.orientation * self.position;

            self.point = [self.position(1,1), self.position(1, 2), self.position(1, 3)];
            updated_points = [self.pose * [self.verts, ones(self.vertex_count, 1)]']';
            self.mesh.Vertices = updated_points(:, 1:3);
            drawnow();
            hold on;
        end
        function tran(self, pos)
            self.position = makehgtform('translate', [pos(1, 1), pos(1, 2), pos(1, 3)]);
            self.pose = self.pose * self.position;
            self.point = [self.pose(1, 4), self.pose(2, 4), self.pose(3, 4)];
            updated_points = [self.pose * [self.verts, ones(self.vertex_count, 1)]']';
            self.mesh.Vertices = updated_points(:, 1:3);
        end

        function update(self)
            self.point = [self.pose(1, 4), self.pose(2, 4), self.pose(3, 4)];
            updated_points = [self.pose * [self.verts, ones(self.vertex_count, 1)]']';
            self.mesh.Vertices = updated_points(:, 1:3);
        end

        function rot(self, rot)
            self.orientation = makehgtform('xrotate', rot(1, 1), 'yrotate', rot(1, 2), 'zrotate', rot(1, 3));
            self.pose = self.pose * self.orientation;
            self.point = [self.pose(1, 4), self.pose(2, 4), self.pose(3, 4)];
            updated_points = [self.pose * [self.verts, ones(self.vertex_count, 1)]']';
            self.mesh.Vertices = updated_points(:, 1:3);
        end
       
        function tranrot(self, pos, rot)
            self.position = makehgtform('translate', [pos(1, 1), pos(1, 2), pos(1, 3)]);
            self.orientation = makehgtform('xrotate', rot(1, 1), 'yrotate', rot(1, 2), 'zrotate', rot(1, 3));
            self.pose = self.pose * self.orientation * self.position;
            self.point = [self.pose(1, 4), self.pose(2, 4), self.pose(3, 4)];
            updated_points = [self.pose * [self.verts, ones(self.vertex_count, 1)]']';
            self.mesh.Vertices = updated_points(:, 1:3);
        end
    end
end