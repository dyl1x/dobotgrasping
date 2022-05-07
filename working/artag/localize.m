bag = rosbag('../bag/May6ARtag1.bag');

% Show RGB Image
RGB_data = select(bag, 'Topic', '/camera/color/image_raw');
firstColourImage = readMessages(RGB_data, 1);
color_img = readImage(firstColourImage{1,1});
figure;
imshow(color_img)

% Show Depth Image
D_data = select(bag, 'Topic', '/camera/depth/image_rect_raw');
firstDepthImage = readMessages(D_data, 1);
depth_img = readImage(firstDepthImage{1,1});

% Show Aligned RGB-D Image
AD_data = select(bag, 'Topic', '/camera/aligned_depth_to_color/image_raw');
firstADepthImage = readMessages(AD_data, 1);
aligned_img = readImage(firstADepthImage{1,1});


% Show Camera Intrinsics
info = select(bag, 'Topic', '/camera/aligned_depth_to_color/camera_info');
infoMsg = readMessages(info);
intrinsic_matrix = infoMsg{1}.K;

%%
img4 = imread("tag.jpg");
%img4 = rgb2gray(img4);
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


sample = rgb2gray(color_img);
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
[features1, validPoints1] = extractFeatures(i1gs, cp, 'Method' , 'Block', 'BlockSize',3);
[features2, validPoints2] = extractFeatures(i2gs, cp2, 'Method' , 'Block', 'BlockSize',3);

% match features
indexPairs = matchFeatures(features1,features2);
matchedPoints1 = validPoints1(indexPairs(:,1));
matchedPoints2 = validPoints2(indexPairs(:,2));
%%
%visualize
figure;
showMatchedFeatures(i1gs,i2gs,matchedPoints1,matchedPoints2,'montage')


%%

[id,loco] = readAprilTag(sample);
l = loco(:,:,13);
plot(l(:,1),l(:,2),'*g');

