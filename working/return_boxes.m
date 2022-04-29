function [bboxs, scores, labels, color_img] = return_boxes(detector, color_img, aligned_img, n)

[bboxs, scores, labels] = detect(detector, color_img, 'MiniBatchSize', 32);

% disp num scores == 1

if n > size(scores)
    n = size(scores);
end

[~, idx] = sort(scores,'descend');
B = idx(1:n);

img_size = size(color_img);
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
    
    [cx, cy] = calc_centroid(x, y, w, h);
    disp(['cx: ', num2str(cx), ', cy: ', num2str(cy)])

    depth = aligned_img(cy, cx);
    disp(['Depth at ', num2str(cx), ', ', num2str(cy), ' == ', num2str(depth), 'mm']);

    img_cut = color_img(y:y+h-1, x:x+w-1, :);
    disp(size(img_cut))
    imshow(img_cut), title(annotation);
    
end

figure
for i=1:n
    bbox = bboxs(B(i), :);
    annotation = sprintf('idx: %d, %s: (Confidence = %0.3f)',B(i), labels(B(i)), scores(B(i)));
    color_img = insertObjectAnnotation(color_img, 'rectangle', bbox, annotation);
    imshow(color_img)
    x = bbox(1);
    y = bbox(2);
    w = bbox(3);
    h = bbox(4);

   

    disp(['idx: ', num2str(B(i))])
    disp(['x: ', num2str(x), ', y: ', num2str(y)])
    disp(['w: ', num2str(w), ', h: ', num2str(h)])
    pause(1)

end
imshow(color_img)
