%planes

p1 = [1,1,1];
p2 = [2,2,2];
p3 = [3,3,3];

p = [p1;p2;p3];
syms a b c d

a*p(:,1) + b*p(:,2) + c*p(:,3) = d

%%
clf

 syms x y
 z = 6*x + 3*y +1;
 
 ezmesh(z)