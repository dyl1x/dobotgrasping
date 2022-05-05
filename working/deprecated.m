%% 4. ROS BAGS : Load/Explore

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


%% 5. ROS BAGS : Visualise as point cloud
close all
clearvars
clc


bag1 = rosbag('bag/6obj3colours.bag');

pc_data = select(bag1, 'Topic', '/camera/depth/color/points');

msg = readMessages(pc_data, 1);

xyz = readXYZ(msg{1});
rgb = readRGB(msg{1});

% figure(1);
% scatter3(pc_data1{1});

% figure(2);
% pcshow(xyz);

% trim_data = pc_data1{1}.Data;
% rowsToDelete = trim_data > 180;
% trim_data(rowsToDelete) = [];
% pc_data1{1}.Data = trim_data;

% pc_data1{1}.Data(pc_data1{1}.Data > 200) = 0;

% figure;
% scatter3(msg{1})

figure; % Trim far away background data
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

% pcshow(new_xyz)


pcobj = pointCloud(readXYZ(msg{1}),'Color',uint8(255*readRGB(msg{1})));

    
% pcshow(readXYZ(pc_data1{1}));

% [x, y] = pol2cart(linspace(-90, 90, 180), pc_data1{1}.Data);
% pc1_cart = [x', y', zeros(180, 1)];
% pointcloud = pointCloud(pc1_cart, 'Color', [ones(180, 1), zeros(180, 1), zeros(180, 1)] );
