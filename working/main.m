%% SNC Group Assignment
% Dobot control and grasping

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');


%% ROBOT CONTROL : Dobot

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','dobot')


% Set Variables

view(2);
ws = [-0.1 0.5 -0.3 0.3 0 0.4];

[so, co, po, ro] = deal(0.017, 0.015, 0.023, 0.028);
[ry, gy, by] = deal(-0.05, 0, 0.05);

uno = Objects('res/obj/arduino_uno.ply', [0.4 ry so], [0 0 0]);
piz = Objects('res/obj/raspberrypi_zero.ply', [0.4 gy so], [0 0 0]);

% red_sphere = Objects('res/shapes/red_sphere.ply', [0.4 ry so], [0 0 0]);
% green_sphere = Objects('res/shapes/green_sphere.ply', [0.4 gy so], [0 0 0]);
% blue_sphere = Objects('res/shapes/blue_sphere.ply', [0.4 by so], [0 0 0]);
% 
% red_cube = Objects('res/shapes/red_cube.ply', [0.45 ry co], [0 0 0]);
% green_cube = Objects('res/shapes/green_cube.ply', [0.45 gy co], [0 0 0]);
% blue_cube = Objects('res/shapes/blue_cube.ply', [0.45 by co], [0 0 0]);
% 
% red_pyramid = Objects('res/shapes/red_pyramid.ply', [0.5 ry po], [0 0 0]);
% green_pyramid = Objects('res/shapes/green_pyramid.ply', [0.5 gy po], [0 0 0]);
% blue_pyramid = Objects('res/shapes/blue_pyramid.ply', [0.5 by po], [0 0 0]);
% 
% red_rectangle = Objects('res/shapes/red_rectangle.ply', [0.55 ry ro], [0 0 0]);
% green_rectangle = Objects('res/shapes/green_rectangle.ply', [0.55 gy ro], [0 0 0]);
% blue_rectangle = Objects('res/shapes/blue_rectangle.ply', [0.55 by ro], [0 0 0]);

% blue_rectangle.tran([0.1 0 0]);

dobot = Dobot(ws, '1');
dobot.model.teach

% axis equal
camlight

% dobot.calc_volume(10);
% axis equal

%% ROS BAGS : Load/Explore

clc
clear all
close all

% rosinit('192.168.1.0');
% depth_sub = rossubscriber('/camera/depth/image_raw'); % Stream data from camera


% bag1 = rosbag('bag/3obj3colours.bag');
% bag1 = rosbag('bag/box2.bag');

% Depth Images
depthImages = select(bag1, 'Topic', '/camera/depth/rgb/image_raw');
firstDepthImage = readMessages(depthImages, 1);
d_img = readImage(firstDepthImage{1,1});
figure(1);
imshow(uint8(double(d_img/5600*256)));

% RGB Images
colourImages = select(bag1, 'Topic', '/camera/rgb/image_raw');
firstColourImage = readMessages(colourImages, 1);
c_img = readImage(firstColourImage{1,1});
figure(2);
imshow(uint8(double(c_img/5600*256)));

img_gray = rgb2gray(d_img);
figure(3)
imshow(img_gray)


%% ROS BAGS : Visualise as point cloud
close all
clearvars
clc


bag1 = rosbag('bag/3obj3colours.bag');

pc_data = select(bag1, 'Topic', '/camera/depth/color/points');

pc_data1 = readMessages(pc_data, 1);

xyz = readXYZ(pc_data1{1});
rgb = readRGB(pc_data1{1});

% figure(1);
% scatter3(pc_data1{1});

% figure(2);
% pcshow(xyz);

% trim_data = pc_data1{1}.Data;
% rowsToDelete = trim_data > 180;
% trim_data(rowsToDelete) = [];
% pc_data1{1}.Data = trim_data;

% pc_data1{1}.Data(pc_data1{1}.Data > 200) = 0;

figure(3);
scatter3(pc_data1{1})

figure(4);
new_xyz = [];
for i = 1:size(xyz)
    row = xyz(i, :);
    if row(1) < -1 || row(1) > 1
        continue
    elseif row(2) < -1 || row(2) > 1
        continue
    elseif row(3) < -1 || row(3) > 1
        continue
    else
        new_xyz(i, :) = row;
    end
end

pcshow(new_xyz)


pcobj = pointCloud(readXYZ(pc_data1{1}),'Color',uint8(255*readRGB(pc_data1{1})));
    
    
% pcshow(readXYZ(pc_data1{1}));

% [x, y] = pol2cart(linspace(-90, 90, 180), pc_data1{1}.Data);
% pc1_cart = [x', y', zeros(180, 1)];
% pointcloud = pointCloud(pc1_cart, 'Color', [ones(180, 1), zeros(180, 1), zeros(180, 1)] );

%% ROS BAG : Extract Image

filename = '5ObjPCloudColorizer';
bag = rosbag(strcat(['bag/', filename, '.bag']));
img_data = select(bag, 'Topic', '/camera/color/image_raw');

firstColourImage = readMessages(img_data, 1);
img = readImage(firstColourImage{1,1});
imshow(img)
imwrite(img, strcat(['working/images/scene_dataset/', filename, '.jpeg']));

%% SIFT SURF Feature Detection : Object Detection in a Cluttered Scene using point feature matching

clc
clear all
close all

% Step 1: Read Images

% lookup images - green_cube, blue_sphere, blue_rectangle, red_pyramid
lookup_img = rgb2gray(imread('working/images/lookup/green_cube.jpeg'));
lookup_img2 = rgb2gray(imread('working/images/lookup/red_pyramid.jpeg'));

% scene images - separated, cluttered
scene_img = rgb2gray(imread('working/images/scenes/cluttered.jpeg'));

% SIFT or SURF
dtype = 'SURF';

% Step 2: Detect Feature Points
num_feats = 100;
if strcmp(dtype, 'SIFT')
    lookupPoints = detectSURFFeatures(lookup_img);
    scenePoints = detectSURFFeatures(scene_img);
else
    lookupPoints = detectSIFTFeatures(lookup_img);
    scenePoints = detectSIFTFeatures(scene_img);
end

figure(1);
imshow(lookup_img); 
title(strcat([num2str(num_feats),' Strongest feature points from lookup img']));
hold on;
plot(selectStrongest(lookupPoints, num_feats));

% Display first 10 features
figure(2)
subplot(5,5,2); title('First 10 Features');
for i=1:10
    scale = lookupPoints(i).Scale;
    img = imcrop(lookup_img, [lookupPoints(i).Location-10*scale 20*scale 20*scale]);
    subplot(5,5,i);
    imshow(img);
    hold on;
    rectangle('Position', [5*scale 5*scale 10*scale 10*scale], 'Curvature',1, 'EdgeColor', 'g');
end

figure(3);
imshow(scene_img);
title(strcat([num2str(num_feats),' Strongest feature points from scene img']));
hold on;
plot(selectStrongest(scenePoints, num_feats));


% Step 3: Extract feature descriptors
[lookupFeatures, lookupPoints] = extractFeatures(lookup_img, lookupPoints);
[sceneFeatures, scenePoints] = extractFeatures(scene_img, scenePoints);


% Step 4: Find Putative Point Matches
lookupPairs = matchFeatures(lookupFeatures, sceneFeatures);
matchedLookupPoints = lookupPoints(lookupPairs(:, 1), :);
matchedScenePoints = scenePoints(lookupPairs(:, 2), :);
figure(5);
showMatchedFeatures(lookup_img, scene_img, matchedLookupPoints, matchedScenePoints, 'montage');
title('Putatively Matched Points (including outliers)');


% Step 5: Locate the object in the scene using putative matches
[tform, inlierIdx] = estimateGeometricTransform2D(matchedLookupPoints, matchedScenePoints, 'affine');
inlierLookupPoints = matchedLookupPoints(inlierIdx, :);
inlierScenePoints = matchedScenePoints(inlierIdx, :);
figure(6);
showMatchedFeatures(lookup_img, scene_img, inlierLookupPoints, inlierScenePoints, 'montage');
title('Matched Points (Inliers Only)');


% Step 6: Create a bounding polygon of the lookup_img in the scene_img
lookupPolygon = [1, 1; ...                        % top-left
    size(lookup_img, 2), 1; ...                   % top-right
    size(lookup_img, 2), size(lookup_img, 1); ... % bottom-right
    1, size(lookup_img, 1); ...                   % bottom-left
    1, 1];                              % top-left to close out poly

newPolygon = transformPointsForward(tform, lookupPolygon);
figure(7);
imshow(scene_img);
hold on;
line(newPolygon(:, 1), newPolygon(:, 2), 'Color', 'y');
title('Detected Polygon');


% Step 7: Detect another lookup_img in the scene_img
figure(8);
imshow(lookup_img2);
title('Lookup Image 2');
if strcmp(dtype, 'SURF')
    lookupPoints2 = detectSURFeatures(lookup_img2);
else
    lookupPoints2 = detectSIFTFeatures(lookup_img2);
end

figure(9);
imshow(lookup_img2); 
title(strcat([num2str(num_feats),' Strongest feature points from lookup img']));
hold on;
plot(selectStrongest(lookupPoints2, num_feats));

[lookupFeatures2, lookupPoints2] = extractFeatures(lookup_img2, lookupPoints2);
lookupPairs2 = matchFeatures(lookupFeatures2, sceneFeatures, 'MaxRatio', 0.9);

matchedLookupPoints2 = lookupPoints2(lookupPairs2(:, 1), :);
matchedScenePoints2 = scenePoints(lookupPairs2(:, 1), :);
figure(10);
showMatchedFeatures(lookup_img2, scene_img, matchedLookupPoints2, matchedScenePoints2, 'montage');
title('Putatively Matched Points (including outliers)');

[tform2, inlierIdx2] = estimateGeometricTransform2D(matchedLookupPoints2, matchedScenePoints2, 'affine');
inlierLookupPoints2 = matchedLookupPoints2(inlierIdx2, :);
inlierScenePoints2 = matchedScenePoints2(inlierIdx2, :);
figure(11);
showMatchedFeatures(lookup_img, scene_img, inlierLookupPoints2, inlierScenePoints2, 'montage');
title('Matched Points (Inliers Only)');


lookupPolygon2 = [1, 1; ...                        % top-left
    size(lookup_img2, 2), 1; ...                   % top-right
    size(lookup_img2, 2), size(lookup_img2, 1); ... % bottom-right
    1, size(lookup_img2, 1); ...                   % bottom-left
    1, 1];                              % top-left to close out poly

newPolygon2 = transformPointsForward(tform, lookupPolygon2);
figure(12);
imshow(scene_img);
hold on;
line(newPolygon(:, 1), newPolygon(:, 2), 'Color', 'y');
line(newPolygon2(:, 1), newPolygon2(:, 2), 'Color', 'g');
title('Detected both Polygons');


%% LOOKUP IMG : Classifier, Googlenet Model
clear
clc

Dataset = imageDatastore('working/images/shape_dataset','IncludeSubfolders',true,'LabelSource','foldernames');
[train, valid] = splitEachLabel(Dataset, 0.7);

net = googlenet;
% analyzeNetwork(net)

input_layer_size = net.Layers(1).InputSize(1:2);
resized_train_imgs = augmentedImageDatastore(input_layer_size, train);
resized_valid_imgs = augmentedImageDatastore(input_layer_size, valid);

feature_learner = net.Layers(142);
output_classifier = net.Layers(144);

num_classes = numel(categories(train.Labels));

new_feature_learner = fullyConnectedLayer(num_classes, ...
    "Name",  'Shape Features', ...
    "WeightLearnRateFactor", 10, ...
    "BiasLearnRateFactor", 10);

new_classifier_layer = classificationLayer('Name', 'Shape Classifier');

layer_graph = layerGraph(net);
new_layer_graph = replaceLayer(layer_graph, feature_learner.Name, new_feature_learner);
new_layer_graph = replaceLayer(new_layer_graph, output_classifier.Name, new_classifier_layer);


minibatchsize = 5;
valid_freq = floor(numel(resized_train_imgs.Files)/minibatchsize);
train_options = trainingOptions('sgdm', ...
    'MiniBatchSize', minibatchsize, ...
    'MaxEpochs', 6, ...
    'InitialLearnRate', 3e-4, ...
    'Shuffle', 'every-epoch', ...
    'ValidationData', resized_valid_imgs, ...
    'ValidationFrequency', valid_freq, ...
    'Verbose', false, ...
    'Plots', 'training-progress');

net = trainNetwork(resized_train_imgs, new_layer_graph, train_options);
save("trained_network_1.mat");

% Testing trained network

% test_network(net, 'working/images/lookup/green_cube.jpeg')

% Ypred = classify(net, resized_valid_imgs);
% Yvalid = resized_valid_imgs.Labels;
% acc = sum(Ypred == Yvalid)/numel(Yvalid);

% Ytest = predict(net, test_imgs);

% confusionchart(test(1:10));
% figure;
% plotconfusion(test(1:10), pred(1:10));



%% Train network with bounding boxes in Scene images

[imds, blds] = objectDetectorTrainingData(gTruth);
cds = combine(imds, blds);
options = trainingOptions('sgdm', ...
    'InitialLearnRate', 0.001, ...
    'Verbose', true, ...
    'MiniBatchSize', 16, ...
    'MaxEpochs', 30, ...
    'Shuffle', 'every-epoch', ...
    'VerboseFrequency', 10);

% [detector, info] = trainRCNNObjectDetector(cds, )


%% Import JPEG images, convert to .mat for image labeller


image_folder = fullfile('/res/images');
imds = imageDatastore(imagefolder);



%% SIFT image recognition

% Colour detection
% Shape detection
% Return point coord of centroid and width of shape
% calc ik solution to grip the payload


clf
close all
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','image recognition')


% img_path = 'images/';

% img1 = rgb2gray(imread( strcat([img_path, 'img1.jpg']) ));

bw = gray2bw(img_gray, 0.4);


s = regionprops(bw,'centroid');
centroids = cat(1,s.Centroid);

% imshow(bw)
% hold on
% plot(centroids(:,1),centroids(:,2),'b*')
% hold off

imshow(bw)
stats = regionprops('table',bw,'Centroid',...
    'MajorAxisLength','MinorAxisLength')
centers = stats.Centroid;
diameters = mean([stats.MajorAxisLength stats.MinorAxisLength],2);
radii = diameters/2;
hold on
viscircles(centers,radii);
hold off


% img2 = rgb2gray(imread( strcat([img_path, 'img2.jpg']) ));
% img3 = rgb2gray(imread( strcat([img_path, 'img3.jpg']) ));
% 
% points1 = detectSURFFeatures(img1);
% points2 = detectSURFFeatures(img2);
% points3 = detectSURFFeatures(img3);
% 
% % points1 = detectORBFeatures(img1);
% % points2 = detectORBFeatures(img2);
% % points3 = detectORBFeatures(img3);
% 
% [features1, validPoints1] = extractFeatures(img1, points1);
% [features2, validPoints2] = extractFeatures(img2, points2);
% [features3, validPoints3] = extractFeatures(img3, points3);
% 
% 
% 
% % img 1 & img 2
% % indexPairs = matchFeatures(features1, features2);
% % matchedPoints1 = validPoints1(indexPairs(:, 1));
% % matchedPoints2 = validPoints2(indexPairs(:, 2));
% 
% 
% 
% % img 1 & img 3
% indexPairs = matchFeatures(features1, features3);
% matchedPoints1 = validPoints1(indexPairs(:, 1));
% matchedPoints2 = validPoints3(indexPairs(:, 2));
% 
% 
% % img 2 & 3
% % indexPairs = matchFeatures(features2, features3);
% % matchedPoints1 = validPoints2(indexPairs(:, 1));
% % matchedPoints2 = validPoints3(indexPairs(:, 2));
% 
% 
% showMatchedFeatures(img1, img2, matchedPoints1, matchedPoints2, 'montage');

%% Classifier

net = googlenet;


I = imread("peppers.png");
figure(1);
imshow(I)
pause(2);

inputSize = net.Layers(1).InputSize;
I = imresize(I,inputSize(1:2));
imshow(I)
pause(2)
% Classify and Display Image

% Classify and display the image with the predicted label.

label = classify(net,I);
figure
imshow(I)
title(string(label))

%% Skeleton


% Variables
% ---------

% vary order of deposition
% colour order: array [red, green, blue]
% shape order: array [cube, ball, pyramid]

% select deposition location
% coords
% offset


% Process
% -------

% Turn on Dobot
% Turn on Camera
 
% Initial scan of environment to determine payload locations
% Calculate types (cube, ball, triangle)
% move to each payload (clarify type with second scan/calc)


% Calc trajectories to each centroid, grip per shape
% Execute trajectories




%% Functions


function [imgBW] = gray2bw(imgGS, threshold)

% Get the size of the input image
[rows, cols, channels] = size(imgGS);

%create an empty matrix for the binary image
imgBW = zeros(rows,cols);

for i = 1:rows
    for j = 1:cols
        % Your logic goes in here
        if imgGS(i, j) >= threshold*256
            imgBW(i, j) = 1;
        end
    end
end

imgBW = logical(imgBW);

end


