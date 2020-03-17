clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')

syms a11 a12 a13 a14
syms a21 a22 a23 a24
syms a31 a32 a33 a34
syms a41 a42 a43 a44

A=[a11 a12 a13 a14;
   a21 a22 a23 a24;
   a31 a32 a33 a34;
   a41 a42 a43 a44];

det(A)


B=[a11 a12 a13 -a11-a12-a13;
   a21 a22 a23 -a21-a22-a23;
   a31 a32 a33 -a31-a32-a33;
   a41 a42 a43 1-a41-a42-a43];

det(B)

det(A(1:3,1:3))
%%
w1 = sym('w1_%d%d', [2 2]);
w2 = sym('w2_%d%d', [2 2]);
w3 = sym('w3_%d%d', [2 2]);
w4 = sym('w4_%d%d', [2 2]);

v1 = sym('v1_%d%d', [2 2]);
v2 = sym('v2_%d%d', [2 2]);
v3 = sym('v3_%d%d', [2 2]);
v4 = sym('v4_%d%d', [2 2]);

w1 = tril(w1,0) + tril(w1,-1).' %Make symmetric
w2 = tril(w2,0) + tril(w2,-1).' %Make symmetric
w3 = tril(w3,0) + tril(w3,-1).' %Make symmetric
w4 = tril(w4,0) + tril(w4,-1).' %Make symmetric

v1 = tril(v1,0) + tril(v1,-1).' %Make symmetric
v2 = tril(v2,0) + tril(v2,-1).' %Make symmetric
v3 = tril(v3,0) + tril(v3,-1).' %Make symmetric
v4 = tril(v4,0) + tril(v4,-1).' %Make symmetric


A_simp=[     w1(2,2)-v1(2,2)                         ,  w2(2,2)-v2(2,2)  ,           w3(2,2)-v3(2,2) ;
   -2*v1(1,2) + v1(2,2) + 2*w1(1,2)  ,  -2*v2(1,2) + v2(2,2) + 2*w2(1,2)  ,-2*v3(1,2) + v3(2,2) + 2*w3(1,2) ;
   -v1(1,1) + 2*v1(1,2) + w1(1,1)    ,    -v2(1,1) + 2*v2(1,2) + w2(1,1),  -v3(1,1) + 2*v3(1,2) + w3(1,1)  ];

det(simplify(A_simp))

%%

w1 = sdpvar(2,2); v1 = sdpvar(2,2);
w2 = sdpvar(2,2); v2 = sdpvar(2,2);
w3 = sdpvar(2,2); v3 = sdpvar(2,2);
w4 = sdpvar(2,2); v4 = sdpvar(2,2);

As=[     w1(2,2)-v1(2,2)                         ,  w2(2,2)-v2(2,2)  ,           w3(2,2)-v3(2,2) ;
   -2*v1(1,2) + v1(2,2) + 2*w1(1,2)  ,  -2*v2(1,2) + v2(2,2) + 2*w2(1,2)  ,-2*v3(1,2) + v3(2,2) + 2*w3(1,2) ;
   -v1(1,1) + 2*v1(1,2) + w1(1,1)    ,    -v2(1,1) + 2*v2(1,2) + w2(1,1),  -v3(1,1) + 2*v3(1,2) + w3(1,1)  ];

det_As=  As(1,1)*As(2,2)*As(3,3)  +  As(1,2)*As(2,3)*As(3,1) +  As(2,1)*As(3,2)*As(1,3) +...
             -As(1,3)*As(2,2)*As(3,1) -  As(1,2)*As(2,1)*As(3,3) -  As(1,1)*As(2,3)*As(3,2);

         
sum_Wi=zeros(2,2);
sum_Vi=zeros(2,2);


constraints=[]
W=[w1 w2 w3 w4];
V=[v1 v2 v3 v4];
for i=1:4
    Wi=W(:,(2*i-1):2*i);
    Vi=V(:,(2*i-1):2*i);
   
    sum_Wi=sum_Wi+Wi;
    sum_Vi=sum_Vi+Vi;
    
    %Wi and Vi are psd matrices <=> All ppal minors are >=0
    constraints=[constraints Wi>=0 Vi>=0];
%     constraints=[constraints, (Wi(1,1)>=0):'Wi11>=0', (Vi(1,1)>=0):'Vi11>=0'];
%     constraints=[constraints, ((Wi(1,1)*Wi(2,2)-Wi(1,2)*Wi(1,2))>=0):'(Wi(1,1)*Wi(2,2)-Wi(1,2)*Wi(1,2)>=0'];
%     constraints=[constraints, ((Vi(1,1)*Vi(2,2)-Vi(1,2)*Vi(1,2))>=0):'Vi(1,1)*Vi(2,2)-Vi(1,2)*Vi(1,2)>=0'];
%     constraints=[constraints, (Wi(2,2)>=0):'Wi22>=0', (Vi(2,2)>=0):'Vi22>=0'];
    %%%%%%%
    
end


C=sum_Wi-sum_Vi;
D=sum_Vi;

constraints=[constraints, C(2,2)==0];
constraints=[constraints, C(1,2)+C(2,1)+D(2,2)==0];
constraints=[constraints, C(1,1)+D(1,2)+D(2,1)==0];
constraints=[constraints, D(1,1)==1];


obj=-(det_As);

W_bezier=[0.0051   -0.0051    3.0000   -3.0003    0.0005   -0.0005    0.0000   -0.0022;
         -0.0051    0.0051   -3.0003    3.0006   -0.0005    0.0005   -0.0022    1.0043];

V_bezier=[1.0000   -1.0025    0.0000   -0.0000    0.0000   -0.0003    0.0000   -0.0000
       -1.0025    1.0051   -0.0000    0.0006   -0.0003    3.0005   -0.0000    0.0043];

   
W_guess=[5   2   2   7   2   -7    7    40
        2    7   7   8   -7    3   40    5];

V_guess=[1   2   2    5    2   -7    7   6
         2   1   5    0   -7    3   6    5];
   
assign(W,10*W_guess);
assign(V,10*V_guess);

check(constraints)
disp('Optimizing')
result=optimize(constraints,obj,sdpsettings('usex0',0,'solver','moment','showprogress',1,'verbose',2,'debug',1,'fmincon.maxfunevals',300000,'fmincon.TolCon',1e-10 ));
%check(constraints)
 
As_value=value(As);
V_value=value(V);
W_value=value(W);
C_value=value(C);
D_value=value(D);

W1=W_value(:,1:2); V1=V_value(:,1:2);
W2=W_value(:,3:4); V2=V_value(:,3:4);
W3=W_value(:,5:6); V3=V_value(:,5:6);
W4=W_value(:,7:8); V4=V_value(:,7:8);

last_column=[0 0 0]'-As_value(:,1)-As_value(:,2)-As_value(:,3);
last_row=[V1(1,1),V2(1,1),V3(1,1) 1-V1(1,1)-V2(1,1)-V3(1,1)];
%% 
A_value=[As_value last_column];
A_value=[A_value ; last_row];

det(A_value)

A_bezier=[-1 3 -3 1;
          3 -6  3 0;
         -3  3  0 0;
          1  0  0 0];
     
A_value
disp("abs(|A_bezier|/|A_mio|)=")
abs(det(A_bezier)/det(A_value))