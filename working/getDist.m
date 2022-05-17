function dist = getDist(tr1,tr2)
p1 = tr1(1:3,4)';
p2 = tr2(1:3,4)';

dist = sqrt(((p2(1)-p1(1))^2) + ((p2(2)-p1(2))^2) + ((p2(3)-p1(3))^2));
end