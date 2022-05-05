img4 = imread("ss2.png");
img4 = rgb2gray(img4);
cp = detectHarrisFeatures(img4);

leftTop = cp.Location(1,:);
leftBot = cp.Location(2,:);
rightTop = cp.Location(15,:);
rightBot = cp.Location(31,:);

cent = cp.Location(32,:);
cp.Location
figure;
imshow(img4);
hold on
plot(cp.Location(:,1),cp.Location(:,2),'*g');

pause(0.01)

sample = imread("ss.png");
sample = rgb2gray(sample);
cp2 = detectHarrisFeatures(sample);

cp2.Location
figure;
imshow(sample);
hold on
plot(cp2.Location(:,1),cp2.Location(:,2),'*g');

%%
i1gs = img4;
i2gs = sample;
% feature decriptors, corresponding location
[features1, validPoints1] = extractFeatures(i1gs, cp, 'Method' , 'Block', 'BlockSize',7);
[features2, validPoints2] = extractFeatures(i2gs, cp2, 'Method' , 'Block', 'BlockSize',7);

% match features
indexPairs = matchFeatures(features1,features2);
matchedPoints1 = validPoints1(indexPairs(:,1));
matchedPoints2 = validPoints2(indexPairs(:,2));
%%
%visualize
figure;
showMatchedFeatures(i1gs,i2gs,matchedPoints1,matchedPoints2,'montage')

%%

[id,loc] = readAprilTag(img4);

cp2.Location
figure;
imshow(img4);
hold on
plot(loc(:,1),loc(:,2),'*g');
