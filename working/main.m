%% SNC Group Assignment
% Dobot control and grasping


% Dobot Control


%% Dobot Control

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');
figure('Name','dobot')


% Set Variables

view(2);
ws = [-0.5 0.5 -0.5 0.5 0 0.8];

dobot = Dobot(ws, '1');
dobot.model.teach

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


