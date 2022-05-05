img4 = imread("tag.jpg");

x = 430.3082;
y = 426.1695;
cropimg = img4(x-400:x+400,y-400:y+400);

imshow(cropimg)

cp = detectHarrisFeatures(cropimg);

%1,2,15,31,32

leftTop = cp.Location(1,:);
leftBot = cp.Location(2,:);
rightTop = cp.Location(15,:);
rightBot = cp.Location(31,:);
cent = cp.Location(32,:);