%% SNC Group Assignment
% Dobot control and grasping

%% Dobot Control

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

%% ros bags
bag1 = rosbag('bag/boxShort.bag');

depthImages = select(bag1, 'Topic', '/camera/color/image_raw');
firstDepthImage = readMessages(depthImages, 1);
d_img = readImage(firstDepthImage{1,1});

img_gray = rgb2gray(d_img);
figure
imshow(img_gray)

%% Import JPEG images, convert to .mat for image labeller

clc
close all
set(0,'DefaultFigureWindowStyle','docked')

image_folder = fullfile()
imds = imageDatastore(imagefolder)



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


