clf

mesh_h = PlaceObject('models/table.PLY');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,0.01)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

hold on


mesh_h = PlaceObject('models/conveyor.PLY');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] *(trotz(-pi/2)*transl(-0.3,-0.375,0.01))';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

mesh_h = PlaceObject('models/conveyor.PLY');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] *transl(-0.8,-0.350,0.01)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

%%
clf
mesh_h = PlaceObject('models/pcb3.ply');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,0.01)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

%%
figure
ws = [-0.5 0.5 -0.5 0.5 0 0.8];
qHome = [0,0,0,0];

rt1 = Dobot(ws,3);