clc; close all;clear;
set(0,'DefaultFigureWindowStyle','docked');



sol1=load('solutionDeg1.mat');
sol2=load('solutionDeg2.mat');
sol3=load('solutionDeg3.mat');
% sol4=load('solutionDeg4.mat');
sol5=load('solutionDeg5.mat');

%%
figure
% scatter(2*ones(size(sol2.rootsA(:))), sol2.rootsA(:)); hold on

% sol1=transformStructureTo01(sol1);
% sol2=transformStructureTo01(sol2);
% sol3=transformStructureTo01(sol3);
% sol5=transformStructureTo01(sol5);


%Swap the first and third rows of sol2.A
tmp1=sol2.A(1,:);
tmp3=sol2.A(3,:);
sol2.A(1,:)=tmp3;
sol2.A(3,:)=tmp1;

sol2B.A=[]


%Swap the first and third rows of sol2.A
tmp1=sol3.A(1,:);
tmp2=sol3.A(2,:);
tmp3=sol3.A(3,:);
tmp4=sol3.A(4,:);
sol3.A=[tmp2; tmp3; tmp1; tmp4];
% 
% sol3.rootsA= sol3.rootsA(1:end/2, :);
% sol5.rootsA= sol5.rootsA(1:end/2, :);

% sol2.A=sol2.A(1:2,:);
% sol3.A=sol3.A(1:2,:);

scatter(3*ones(size(sol3.rootsA(:))), sol3.rootsA(:)); hold on
% scatter(4*ones(size(sol4.rootsA(:))), sol4.rootsA(:))
scatter(5*ones(size(sol5.rootsA(:))), sol5.rootsA(:))

figure
sol3.rootsTA=transformTo01(sol3.rootsA);
sol5.rootsTA=transformTo01(sol5.rootsA);
scatter(3*ones(size(sol3.rootsTA(:))), sol3.rootsTA(:)); hold on
% scatter(4*ones(size(sol4.rootsA(:))), sol4.rootsA(:))
scatter(5*ones(size(sol5.rootsTA(:))), sol5.rootsTA(:))


S=load('solutionDeg3.mat')
A=S.A;

%odd columns of A
oddcol=A(:,1:2:end);

%even columns 
evencol=A(:,2:2:end);

A_reorganized=[oddcol,evencol];

nrow=size(A,1);
ncol=size(A,2);

AA=A_reorganized(1:nrow/2,1:ncol/2);

BB=A_reorganized(1:nrow/2,ncol/2+1:end);

det(A)

det(2*AA)*det(BB)


det(AA)/det(BB)

disp("=============")
global t
syms t real

T=[t*t t 1]';


syms a b c d real

T1=[t 1]';
T2=[t*t t 1]';
T3=[t*t*t t*t t 1]';
T4=[t*t*t*t t*t*t t*t t 1]';
T5=[t^5 T4']';

polys1=vpa(sol1.A*T1,4);
polys2=vpa(sol2.A*T2,4);
polys3=vpa(sol3.A*T3,4);
%%
interv=[-1,1];
% disp("=======================================")
% comb=(a*t +b)*(sol2.A(1,:)*T2) + (c*t+d)*(sol2.A(2,:)*T2 );
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% sol=solve(coeff_comb==sol3.A(1,:));


syms tn real %t_new
tn=t*0.577;
Tn2=[tn*tn tn 1]';
Tn3=[tn*tn*tn     tn*tn     tn     1     ]';
Tn4=[tn*tn*tn*tn  tn*tn*tn  tn*tn  tn 1  ]';
figure
fplot(2*(sol2.A*Tn2-[0;0.5;0]),interv);

disp("=======================================")
%%
comb=combine2pol(polys1(1),polys2(1),a,b,c,d);
coeff_comb=vpa(coeffs(comb,t,'All'),4);
sol=solve(coeff_comb==sol3.A(1,:));


disp("combining first")
comb=combine2pol(polys1(1),polys2(1),sol.a,sol.b,sol.c,sol.d);
coeff_comb=vpa(coeffs(comb,t,'All'),4)
figure; hold on;
fplot(coeff_comb*T3,interv);

disp("combining more")
comb=combine2pol(polys1(1),polys2(2),sol.a,sol.b,sol.c,sol.d);
coeff_comb=vpa(coeffs(comb,t,'All'),4)
fplot(coeff_comb*T3,interv);

figure
fplot(int(sol2.A()*T2-sol2.A*T2,-1,t), interv);
legend('1','2','3')

%%
% disp("combining more")
% comb=combine2pol(polys1(2),polys1(3),0,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% fplot(coeff_comb*T2,[0,1]);


for i=1:2
    for j=1:2
        for k=1:3
            comb=combine2pol(polys1(i),polys1(j),a,b,c,d);
            coeff_comb=vpa(coeffs(comb,t,'All'),4);
            sol=solve(coeff_comb==sol2.A(k,:));
            disp(vpa([i j k 'ss' sol.b sol.c sol.d],4))
        end
    end
end

disp("=======================================")
% disp("combining first")
% comb=combine2pol(polys2(1),polys2(2),sol.a,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)
% 
% disp("combining more")
% comb=combine2pol(polys2(1),polys2(3),sol.a,sol.b,sol.c,sol.d);
% coeff_comb=vpa(coeffs(comb,t,'All'),4)

% T=[t*t*t t*t t 1]';
vpa(sol2.A,4)
vpa(coeff_comb,4);
vpa(sol3.A,4)

figure; hold on

fplot(sol2.A*T2, interv);
legend('1','2','3')
ylim([0,1]);
figure
fplot(sol3.A*T3, interv);
legend('1','2','3','4')
ylim([0,1]);
figure
fplot(sol5.A*T5, interv);
ylim([0,1]);
legend('1','2','3','4')

%%
figure;
fplot(sqrt(sol2.A*T2), interv);
axis equal

figure;
fplot(sqrt(sol3.A*T3), interv);
axis equal


figure;
fplot(sqrt(sol5.A*T5), interv);
axis equal

%%
%%
clf
figure; hold on;
fplot((sol2.A*T2), interv,'r');
axis equal

fplot((sol3.A*T3), interv,'k');
axis equal


fplot((sol5.A*T5), interv,'b');
axis equal

fplot(-0.5*(t-1),interv,'m');
fplot(-0.5*(-t-1),interv,'m');

%%

% close all

figure; hold on
fplot(diff(sol2.A*T2,t), interv);
legend('1','2','3')

figure
fplot(diff(sol3.A*T3,t), interv);
legend('1','2','3')

figure
fplot(int(sol2.A*T2,-1,t), interv);
legend('1','2','3')

figure
fplot(int(sol3.A*T3,-1,t), interv);
legend('1','2','3')

figure;


figure; hold on
sphere
alpha 0.2
shading interp
fplot3(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2),sqrt(sol2.A(3,:)*T2), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal

figure;
fplot3(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2),sqrt(sol2.A(3,:)*T2), interv)
xlabel('x');ylabel('y');
axis equal;

figure; hold on;
fplot(sqrt(sol2.A(1,:)*T2),sqrt(sol2.A(2,:)*T2), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal;

% figure;
fplot(sqrt(sol3.A(1,:)*T3),sqrt(sol3.A(2,:)*T3), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal;

figure;
fplot((sol3.A(1,:)*T3),(sol3.A(2,:)*T3), interv)
xlabel('x');ylabel('y');zlabel('z');
axis equal;


figure; hold on
fplot(sqrt(sol1.A(1,:)*T1),sqrt(sol1.A(2,:)*T1), interv); axis equal;
figure
fplot(sqrt(sol3.A*T3) + sqrt(sol3.A(3,:)*T3),sqrt(sol3.A(2,:))+sqrt(sol3.A(4,:)*T3), interv)

axis equal


%%
clf
figure;  hold on;
tmp=0:0.01:1;
theta=2*pi*tmp;

polarplot(theta,subs(sol3.A*T3,tmp),'r');

polarplot(theta,subs(sol2.A*T2,tmp),'k'); 

polarplot(theta,subs(sol5.A*T5,tmp),'b'); 
%%

function roots_transformed=transformTo01(roots)
    roots_transformed=zeros(size(roots));
    
    for i=1:size(roots,1)
        for j=1:size(roots,2)
            roots_transformed(i,j)=(roots(i,j)-(-1))/2;
        end
    end
end

function comb=combine2pol(pol1, pol2,a,b,c,d)
    global t
    comb=(a*t+b)*pol1+(c*t +d)*pol2;
end

function result=transformStructureTo01(sol)

syms tt real
syms t real
tt=2*(t-0.5);

deg=size(sol.A,2)-1;

T=[];
for i=0:(deg)
    T=[tt^i T];
end

transformed2=sol.A*T';

A=[];

for i=1:size(sol.A,1)
    A=[A ;double(vpa(coeffs(transformed2(i),'All')))];
end

rootsA=[];
for i=1:size(A,1)
    rootsA=[rootsA ; roots(A(i,:))'];
end
rootsA=double(real(rootsA));


result.A=A;
result.rootsA=rootsA;


end
