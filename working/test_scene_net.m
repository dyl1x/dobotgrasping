function [bboxs, scores, labels, img] = test_scene_net(detector, img, n)

[bboxs, scores, labels] = detect(detector, img, 'MiniBatchSize', 32);

% disp num scores == 1

if n > size(scores)
    n = size(scores);
end

[~, idx] = sort(scores,'descend');
B = idx(1:n);

img_size = size(img);
figure
for i=1:n
    bbox = bboxs(B(i), :);
    annotation = sprintf('idx: %d, %s: (Confidence = %0.3f)',B(i), labels(B(i)), scores(B(i)));
    
    subplot(n, 1, i)
    x = bbox(1);
    y = bbox(2);
    w = bbox(3);
    h = bbox(4);

    disp(['idx: ', num2str(B(i))])
    disp(['x: ', num2str(x), ', y: ', num2str(y)])
    disp(['w: ', num2str(w), ', h: ', num2str(h)])
    
    if x > img_size(1)
        disp(['greater than limit']);
        continue
    end
    
    if y > img_size(2)
        disp(['greater than limit']);
        continue
    end

    img_cut = img(y:y+h-1, x:x+w-1, :);
    disp(size(img_cut))
    imshow(img_cut), title(annotation);    
    
end

figure
for i=1:n
    bbox = bboxs(B(i), :);
    annotation = sprintf('idx: %d, %s: (Confidence = %0.3f)',B(i), labels(B(i)), scores(B(i)));
    img = insertObjectAnnotation(img, 'rectangle', bbox, annotation);
    imshow(img)
    x = bbox(1);
    y = bbox(2);
    w = bbox(3);
    h = bbox(4);

    disp(['idx: ', num2str(B(i))])
    disp(['x: ', num2str(x), ', y: ', num2str(y)])
    disp(['w: ', num2str(w), ', h: ', num2str(h)])
    pause(1)

end
imshow(img)


