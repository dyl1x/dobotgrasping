img4 = imread("crop.png");
cp = detectHarrisFeatures(cropimg);

leftTop = cp.Location(1,:);
leftBot = cp.Location(2,:);
rightTop = cp.Location(15,:);
rightBot = cp.Location(31,:);
cent = cp.Location(32,:);

%%
sample = imread("img.png");
sample = rgb2gray(sample);
cp2 = detectHarrisFeatures(sample);

cp2.Location
figure;
imshow(sample);
hold on
plot(cp2.Location(:,1),cp2.Location(:,2),'*g');