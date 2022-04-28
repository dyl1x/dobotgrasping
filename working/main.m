%% SNC Group Assignment
% Dobot control and grasping

close all
clearvars
clc
set(0,'DefaultFigureWindowStyle','docked');


idx = 0;
%% Capture Images from ros subscription

% rosinit()
sub = rossubscriber("/camera/color/image_raw");
figure(1)
i = readImage(sub.receive);
imshow(i)
imwrite(i, ['working/images/scene_dataset/IMG_', num2str(idx), '.jpeg']);
idx = idx + 1;

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

pcb1 = Objects('res/obj/pcb1.ply', [0.4 ry so], [0 0 0]);
pcb2 = Objects('res/obj/pcb2.ply', [0.4 0.25 so], [0 0 0]);

% pcb3 = Objects('res/obj/pcb3.ply', [0.4 0.5 so], [0 0 0]);

% uno = Objects('res/obj/arduino_uno.ply', [0.4 ry so], [0 0 0]);
% piz = Objects('res/obj/raspberrypi_zero.ply', [0.4 gy so], [0 0 0]);

% red_sphere = Objects('res/shapes/red_sphere.ply', [0.4 ry so], [0 0 0]);
% green_sphere = Objects('res/shapes/green_sphere.ply', [0.4 gy so], [0 0 0]);
% blue_sphere = Objects('res/shapes/blue_sphere.ply', [0.4 by so], [0 0 0]);

% red_cube = Objects('res/shapes/red_cube.ply', [0.45 ry co], [0 0 0]);
% green_cube = Objects('res/shapes/green_cube.ply', [0.45 gy co], [0 0 0]);
% blue_cube = Objects('res/shapes/blue_cube.ply', [0.45 by co], [0 0 0]);

% red_pyramid = Objects('res/shapes/red_pyramid.ply', [0.5 ry po], [0 0 0]);
% green_pyramid = Objects('res/shapes/green_pyramid.ply', [0.5 gy po], [0 0 0]);
% blue_pyramid = Objects('res/shapes/blue_pyramid.ply', [0.5 by po], [0 0 0]);

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

figure(4); % Trim far away background data
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


