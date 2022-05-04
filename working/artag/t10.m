img1 = imread("image_1.png");
img2 = imread("image_2.png");
img3 = imread("image_3.png");
img4 = imread("crop.png");
%%
f = 400;
p = length(img4)/2;
z = 50;
lambda = 0.1;

%%
cp = detectHarrisFeatures(img4);

cp.Location
figure;
imshow(img4);
hold on
plot(cp.Location(:,1),cp.Location(:,2),'*g');

goal = [250,250;
          250,750;
          750,250;
          750,750];

hold on
plot(goal(:,1),goal(:,2),'*r');

%%
ty = (goal - p)/f;
oy = (cp.Location - p)/f;

%%
Lx = [];
for i=1:(length(ty))
     lx = FuncLx(ty(i,1),ty(i,2),z);
     Lx = [Lx;lx];
end


%%
e2 = oy-ty;
e = reshape(e2',[],1);
de = -e*lambda;

%%
Lx2 = inv(Lx'*Lx)*Lx';
Vc = -lambda*Lx2*e



