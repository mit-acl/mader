clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')


sol3=load('solutionDeg3.mat');
sol2=load('solutionDeg2.mat');

% sol2=transformStructureTo01(sol2);
% sol3=transformStructureTo01(sol3);

%Take good order
tmp1=sol3.A(1,:);
tmp2=sol3.A(2,:);
tmp3=sol3.A(3,:);
tmp4=sol3.A(4,:);
sol3.A=[tmp2; tmp3; tmp1; tmp4];

A=double(vpa(sol3.A,4));

%READ THIS: https://yalmip.github.io/example/nonconvexquadraticprogramming/
% A=[-1 3 -3 1;
%    3 -6 3 0;
%    -3 3 0 0;
%    1 0 0 0];

W_solution=[];
V_solution=[];

for i=1:4

polynomial=A(i,:)
    
a=polynomial(1);
b=polynomial(2);
c=polynomial(3);
d=polynomial(4);

W=sdpvar(2,2);
V=sdpvar(2,2);

constraints=[];
constraints=[constraints W(2,2)-V(2,2)==a];
constraints=[constraints -2*V(1,2)+V(2,2)+2*W(1,2)+W(2,2)==b];
constraints=[constraints -V(1,1)+2*V(1,2)+W(1,1)+2*W(1,2)==c];
constraints=[constraints V(1,1)+W(1,1)==d];

constraints=[constraints W>=0 V>=0];

obj=0;

disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
result=optimize(constraints,obj,sdpsettings('usex0',0,'solver','mosek','showprogress',0,'verbose',0,'debug',0,'fmincon.maxfunevals',300000 ))

W_value=value(W)
V_value=value(V)

polynomial_solution=[W_value(2,2)-V_value(2,2)  ,   -2*V_value(1,2)+V_value(2,2)+2*W_value(1,2)   ,     -V_value(1,1)+2*V_value(1,2)+W_value(1,1)   ,  V_value(1,1)]

W_solution=[W_solution W_value];

V_solution=[V_solution V_value];

end

W1=W_solution(:,1:2); V1=V_solution(:,1:2);
W2=W_solution(:,3:4); V2=V_solution(:,3:4);
W3=W_solution(:,5:6); V3=V_solution(:,5:6);
W4=W_solution(:,7:8); V4=V_solution(:,7:8);

sum_Wi=W1+W2+W3+W4;
sum_Vi=V1+V2+V3+V4;

C=sum_Wi-sum_Vi;
D=sum_Vi;

C(2,2)

C(1,2) + C(2,1) + D(2,2)

C(1,1) + D(1,2) +D(2,1)

D(1,1)

sum_total=zeros(2,2);
t=0.8;
for i=1:4
    Wi=W_solution(:,2*i-1:2*i);
    Vi=V_solution(:,2*i-1:2*i);
    
    sum_total=sum_total+ (t+1)*Wi + (1-t)*Vi;
    
end

[1 t]*sum_total*[1;t]
%%
syms t real
interv=[-1,1]
Tf=[1 t]';
figure; hold on;
lambda1=(t+1)*Tf'*W1*Tf+(1-t)*Tf'*V1*Tf;
lambda2=(t+1)*Tf'*W2*Tf+(1-t)*Tf'*V2*Tf;
lambda3=(t+1)*Tf'*W3*Tf+(1-t)*Tf'*V3*Tf;
lambda4=(t+1)*Tf'*W4*Tf+(1-t)*Tf'*V4*Tf;

fplot(lambda1, interv);
fplot(lambda2, interv);
fplot(lambda3, interv);
fplot(lambda4, interv);

% fplot(t*Tf'*W1, interv);
% fplot((1-t)*Tf'*V1*Tf, interv);
% fplot(lambda1/(1-t), interv); %This is a polynomial that is positive for all 

% fplot(-A(1,1)*(t-0.5154412698929707881)^2,interv,'--')


% A1=A;
% A1(:,2)=0.5*(A1(:,2)-A1(:,1));
% A1(:,3)=0.5*(A1(:,3)-A1(:,4));

A_novale=[-V1(2,2) -V1(1,2)+V1(2,2)  -V1(1,1)+V1(1,2)   V1(1,1);
            W2(2,2)    W2(1,2)           W2(1,2)         W2(1,1);
           -V3(2,2) -V3(1,2)+V3(2,2)  -V3(1,1)+V3(1,2)   V3(1,1);
            W4(2,2)    W4(1,2)           W4(1,2)         W4(1,1);]

C=[A(1,1), A(1,3); A(2,1), A(2,3)];
B=[A(1,2), A(1,4); A(2,2), A(2,4)];


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