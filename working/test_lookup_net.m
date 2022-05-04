function [R, label, probability] = test_lookup_net(net, image)

I = imread(image);
R = imresize(I, [224, 224]); % image size

[label, probability] = classify(net, R);
disp(label)
disp(probability)
figure;
imshow(R);
title(strcat([char(label), ': ', num2str(max(probability)*100, 5)]));

end