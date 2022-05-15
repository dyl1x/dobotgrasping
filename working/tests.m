
figure
mesh_h = PlaceObject('../resources/dobot/d3.ply');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * trotx(90)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

%%
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
mesh_h = PlaceObject('models/target.ply');
axis equal
vertices = get(mesh_h,'Vertices');
transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(0,0,0.01)';
set(mesh_h,'Vertices',transformedVertices(:,1:3));

%%
% figure
ws = [-0.5 0.5 -0.5 0.5 0 0.8];
qHome = [0,pi/2,-pi/2,0];

rt1 = Dobot(ws,1, 2);

%%
q2 = deg2rad(0);
q3 = deg2rad(90);
rt1.model.animate([0,q2,q3,constrain_joint4(q2,q3),0]);


%%

endeffq = rt1.model.getpos();
endeffPos = rt1.model.fkine(endeffq);
xyz = endeffPos(1:3,4)

%%
x = 0.1;
y = 0;
z = 0;

tr = transl(x,y,z);
endeffq = rt1.model.getpos();
endeffPos = rt1.model.fkine(endeffq);
goalPos = endeffPos * tr;
goalq = rt1.model.ikcon(goalPos);
qmatrix = jtraj(endeffq,goalq,20);

for i=1:size(qmatrix)
    rt1.model.animate(qmatrix(i,:));
end

%%

camtr = transl(0.068,0,0)*trotz(180);
robottr = transl(0,0,0);
objwrtcam = transl(0.030,0,0);

%%
objwrtrob =(camtr - objwrtcam)




