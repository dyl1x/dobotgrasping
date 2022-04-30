% using gavins code from PuttingSimulatedObjectsIntoTheEnvironment()
% link: https://au.mathworks.com/matlabcentral/fileexchange/58774-putting-simulated-objects-into-the-environment
% adjusted to work with my models misbehaving when pose is rotated. Now we
% simply replace the pose every time we want to change it.
% class created by:
% chamath edirisinhege - 12977866
%
%inputs : pose
%
classdef pcb <handle
    properties
        mesh_h;
        pose = eye(4);
        vertices;
        vSize;
    end
    methods
        function self = pcb(pose)
            self.LoadModel();
            self.pose = pose;
            self.MoveMesh(pose);
            
        end
        
        function LoadModel(self)
            
            % After saving in blender then load the triangle mesh
            [f,v,data] = plyread('models/brick.ply','tri');
            
            % Get vertex count
            self.vSize = size(v,1);
            
            % Move center point to origin
            midPoint = sum(v)/self.vSize;
            self.vertices = v - repmat(midPoint,self.vSize,1);
            
            % Scale the colours to be 0-to-1 (they are originally 0-to-255
            vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;
            
            % Then plot the trisurf
            self.mesh_h = trisurf(f,self.vertices(:,1),self.vertices(:,2), self.vertices(:,3) ...
                ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
        end
        
        function MoveMesh(self,brickTransf)
            
            % Move the pose
            self.pose = brickTransf; 
            % we did a cheeky replace here cos, * by trotx wont rotate it about its local x axis
            % it made things very hard for no reason, i have no fix for
            % this.
            updatedPoints = [self.pose * [self.vertices,ones(self.vSize,1)]']';
            
            % Now update the Vertices
            self.mesh_h.Vertices = updatedPoints(:,1:3);
            drawnow();
        end
    end
end