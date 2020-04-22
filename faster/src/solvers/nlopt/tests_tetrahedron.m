% clear; close all; clc;
v1=[1,1,1]';
v2=[-1,-1,1]';
v3=[-1,1,-1]';
v4=[1,-1,-1]'; 

n1=(1/sqrt(3))*v1;
n2=(1/sqrt(3))*v2;
n3=(1/sqrt(3))*v3;
n4=(1/sqrt(3))*v4;


c=(v1+v2)/2.0;

c=[15 24 78]';

(c-v2)'*n1 + (c-v1)'*n2  + (c-v3)'*n4  + (c-v4)'*n3

det([1 1 1 1; -1 -1 1 1; -1 1 -1 1; 1 -1 -1 1])

syms a real
syms t real

v1=a*[1,1,1]';
v2=a*[-1,-1,1]';
v3=a*[-1,1,-1]';
v4=a*[1,-1,-1]'; 

n1=v1/norm(v1);
n2=v2/norm(v2);
n3=v3/norm(v3);
n4=v4/norm(v4);

cte=norm(v1)/(a);

px=t^2+1;
py=-2*t^3 +t;
pz=t^3 -t;


p=[px;py;pz];

const1=simplify(cte*n1'*(p-v2))
const2=simplify(cte*n2'*(p-v3))
const3=simplify(cte*n3'*(p-v4))
const4=simplify(cte*n4'*(p-v1))

interv=[-1,1];
fplot(simplify(const1-a),interv); hold on
fplot(simplify(const2-a),interv);
fplot(simplify(const3-a),interv);
fplot(simplify(const4-a),interv);

figure;
fplot3(px,py,pz,[-1,1]); hold on
a=4*0.3
v1=a*[1,1,1]';
v2=a*[-1,-1,1]';
v3=a*[-1,1,-1]';
v4=a*[1,-1,-1]'; 

vx=[v1(1) v2(1) v3(1) v4(1)]';
vy=[v1(2) v2(2) v3(2) v4(2)]';
vz=[v1(3) v2(3) v3(3) v4(3)]';

[k1,volume] = convhull(vx,vy,vz);
trisurf(k1,vx,vy,vz,'FaceColor','red')
 alpha 0.1; axis equal
xlabel('x'); ylabel('y'); zlabel('z') 