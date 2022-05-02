function [bboxs, B, C,  scores, labels, color_img, img_cuts, n] = return_boxes(detector, color_img, aligned_img, n)

[bboxs, scores, labels] = detect(detector, color_img, 'MiniBatchSize', 32);

% disp num scores == 1

if n > size(scores)
    n = size(scores);
end

[~, idx] = sort(scores,'descend');
B = idx(1:size(scores)); % indexes

img_size = size(color_img);
img_cuts = zeros(n, 4);

i = 1;
l = 0;
C = zeros(1, n);
figure
while l < n
    B_size = size(B);
    if i == B_size(1) disp('Could not find all features'); n = l; break; end
    bbox = bboxs(B(i), :);

    x = bbox(1);
    y = bbox(2);
    w = bbox(3);
    h = bbox(4);

    if x > img_size(2)
        i = i + 1;
    elseif x + w - 1 > img_size(2)
        i = i + 1;
    elseif y > img_size(1)
        i = i + 1;
    elseif y + h - 1 > img_size(1)
        i = i + 1;
    else
        img_cut = color_img(y:y+h-1, x:x+w-1, :);

        sum_r = sum(sum(img_cut(:, :, 1)));
        sum_g = sum(sum(img_cut(:, :, 2)));
        sum_b = sum(sum(img_cut(:, :, 3)));
    
        disp(['r: ', num2str(sum_r) ...
            ', g: ',num2str(sum_g), ...  
            ', b: ', num2str(sum_b)]);
        
        total = sum_r + sum_g + sum_b;
        
        rgb_r = (total/3) / sum_r;
        rgb_g = (total/3) / sum_g;
        rgb_b = (total/3) / sum_b;
    
        if abs(1 - (rgb_r * rgb_g * rgb_b)) < 1e-3
            disp('minimal difference')
        elseif abs(1 - (rgb_r * rgb_g * rgb_b)) < 5e-2
            disp('some diff')
        end
    
        if sum_r / (total / 3) > 1.2
            col = 'likely red';
            disp(col);
            cont = true;
        elseif sum_g / (total / 3) > 1.2
            col = 'likely green';
            disp(col);
            cont = true;
        elseif sum_b / (total / 3) > 1.2
            col = 'likely blue';
            disp(col);
            cont = true;
        else
            disp('some dull color')
            cont = false;
        end

        if l >= n; break; end

        if cont
            subplot(n, 1, l + 1)
            annotation = sprintf('%s -- idx: %d, %s: (Conf = %0.2f)',col,B(i), labels(B(i)), scores(B(i)));
            
            disp(['idx: ', num2str(B(i))])
            disp(['x: ', num2str(x), ', y: ', num2str(y)])
            disp(['w: ', num2str(w), ', h: ', num2str(h)])
            
            [cx, cy] = calc_centroid(x, y, w, h);
            disp(['cx: ', num2str(cx), ', cy: ', num2str(cy)])
        
            depth = aligned_img(cy, cx);
            disp(['Depth at ', num2str(cx), ', ', num2str(cy), ' == ', num2str(depth), 'mm']);
        
        
            disp(size(img_cut))
            imshow(img_cut), title(annotation);
            img_cuts(l + 1, :) = [x, y, w, h];
        
            C(l + 1) = B(i);
            i = i + 1; % loop through ranked scores index B but retain idx for bbox
            l = l + 1;
            
            disp(['i: ', num2str(i), ', l: ', num2str(l), ', n: ', num2str(n)]);
            disp(['' ...
                ''])
        else
            disp('no color, skipping bbox');
            i = i + 1;
        end
    end
end

disp(['Size of C: ', num2str(size(C))]);

figure
for i=1:n
    ix = C(i);
    bbox = bboxs(ix, :);
    annotation = sprintf('idx: %d, %s: (Confidence = %0.3f)',ix, labels(ix), scores(B(ix)));
    color_img = insertObjectAnnotation(color_img, 'rectangle', bbox, annotation);
    imshow(color_img)

    x = bbox(1);
    y = bbox(2);
    w = bbox(3);
    h = bbox(4);

    disp(['idx: ', num2str(ix)])
    disp(['x: ', num2str(x), ', y: ', num2str(y)])
    disp(['w: ', num2str(w), ', h: ', num2str(h)])
    pause(1)

end
imshow(color_img)
