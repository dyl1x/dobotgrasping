function [cx, cy] = calc_centroid(x, y, w, h)
    cx = round(x + w/2, 0);
    cy = round(y + (h/2), 0);
