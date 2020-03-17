%read 
%https://math.stackexchange.com/questions/1392916/smaller-enclosing-shape-for-b%C3%A9zier-curves
%Me acabo de encontrar este paper :(
%Polynomial bases for quadratic and cubic polynomials which yield control points with small convex hulls     

clear; clc; close all; set(0,'DefaultFigureWindowStyle','normal') %'docked')

pol_x=[0 0 0.2 0.3 2 1]';%[a b c d]
pol_y=[0 0 -0.3 +3 -5 6]';%[a b c d]
pol_z=[0 0 1 -0.1 -1 -4]';%[a b c d]


W=[]; V=[];
for i=1:4
   W=[W sdpvar(3,3)];
   V=[V sdpvar(3,3)];
end

A = sdpvar(6,4,'full'); 
B= A(3:end,:);
constraints=[];

U=[];
sum_Wi=zeros(3,3);
sum_Vi=zeros(3,3);

beta1=[];

for i=1:4
    Wi=W(:,(3*i-2):3*i);
    Vi=V(:,(3*i-2):3*i);
    sum_Wi=sum_Wi+Wi;
    sum_Vi=sum_Vi+Vi;
    
    %Wi and Vi are psd matrices <=> All ppal minors are >=0
    %constraints=[constraints Wi>=0 Vi>=0];
    
    %Ppal minors of order 1
    constraints=[constraints, (Wi(1,1)>=0):'Wi11>=0', (Vi(1,1)>=0):'Vi11>=0'];
    constraints=[constraints, (Wi(2,2)>=0):'Wi22>=0', (Vi(2,2)>=0):'Vi22>=0'];
    constraints=[constraints, (Wi(3,3)>=0):'Wi33>=0', (Vi(3,3)>=0):'Vi33>=0'];
 
    %Ppal minors of order 2
    constraints=[constraints, ((Wi(1,1)*Wi(2,2)-Wi(1,2)*Wi(1,2))>=0)];
    constraints=[constraints, ((Wi(1,1)*Wi(3,3)-Wi(1,3)*Wi(1,3))>=0)];
    constraints=[constraints, ((Wi(2,2)*Wi(3,3)-Wi(2,3)*Wi(2,3))>=0)];
    
    constraints=[constraints, ((Vi(1,1)*Vi(2,2)-Vi(1,2)*Vi(1,2))>=0)];
    constraints=[constraints, ((Vi(1,1)*Vi(3,3)-Vi(1,3)*Vi(1,3))>=0)];
    constraints=[constraints, ((Vi(2,2)*Vi(3,3)-Vi(2,3)*Vi(2,3))>=0)];
    
    %Ppal minors of order 3         
    constraints=[constraints, computeDet3(Wi)>=0];
    constraints=[constraints, computeDet3(Vi)>=0];
    %%%%%%%
    
lambdai=[
         -Vi(3,3) + Wi(3,3); ...
        -2*Vi(2,3)+Vi(3,3)+2*Wi(2,3); ...
        -2*Vi(1,3)-Vi(2,2)+2*Vi(2,3)+2*Wi(1,3)+Wi(2,2); ...
        -2*Vi(1,2)+2*Vi(1,3)+Vi(2,2)+2*Wi(1,2); ...
         -Vi(1,1)+2*Vi(1,2)+Wi(1,1); ...
         Vi(1,1) ...
        ];
     U=[U lambdai];
   
end
constraints=[constraints (A==U):'A==U']; %

vx=sdpvar(4,1);
vy=sdpvar(4,1);
vz=sdpvar(4,1);
pol_x
B
vx

constraints=[constraints pol_x==A*vx pol_y==A*vy pol_z==A*vz];

%constraints=[constraints beta1'*vx==0 beta1'*vy==0 labeta1mbdai'*vz==0];

% \sum p_i(t)=1 \forall i
constraints=[constraints (A*ones(4,1)==[0 0 0 0 0 1]'):'A*1==[0 0 0 0 0 1]'];%Sum \lambda_i(t)=1

%I want to maximize the absolute value of the determinant of B
detB=computeDet4(B);
obj=detB

%I want to maximize the absolute value of the determinant of A
%obj=-abs(det(A,'polynomial')); %Should I put abs() here?

%% INITIAL GUESSES
% W_bezier=[0.0051   -0.0051    3.0000   -3.0003    0.0005   -0.0005    0.0000   -0.0022;
%          -0.0051    0.0051   -3.0003    3.0006   -0.0005    0.0005   -0.0022    1.0043];
% 
% V_bezier=[1.0000   -1.0025    0.0000   -0.0000    0.0000   -0.0003    0.0000   -0.0000
%        -1.0025    1.0051   -0.0000    0.0006   -0.0003    3.0005   -0.0000    0.0043];

% V_bezier=[0.9858   -0.9895    3.0000   -2.9994    0.0002   -0.0010    0.0000   -0.0039;
%           -0.9895    0.9932   -2.9994    2.9993   -0.0010    3.0018   -0.0039    1.0192]
% 
% W_bezier=[   1.0000   -1.9929    0.9929    0.0000   -0.0000    0.0000    0.0000   -0.0001    0.0001    0.0000   -0.0000    0.0000;
%             -1.9929    3.9790   -1.9861   -0.0000    2.9989   -2.9991   -0.0001    3.0020   -3.0019   -0.0000    0.0078   -0.0136;
%              0.9929   -1.9861    0.9932    0.0000   -2.9991    2.9993    0.0001   -3.0019    3.0018    0.0000   -0.0136    1.0192];
% 
% % W_bezier=ones(3,12);
% V_bezier=ones(2,8);

A_bezier=[0 0 0 0 ;
          0 0 0 0 ;
          -1 3 -3 1;
          3 -1  3 2;
         -3  3  2 -2;
          1  0 0 0  ];
     
A_guess=A_bezier;
 
assign(A,A_guess);
assign(U,A_guess);
% assign(W,W_bezier);
% assign(V,V_bezier);

%% OPTIMIZE

check(constraints)
disp('Starting optimization') %'solver','bmibnb' 'fmincon' ,'solver','sdpt3' 'ipopt' 'knitro' 'scip'
settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','moment','showprogress',1,'verbose',2,'debug',1,'fmincon.maxfunevals',3e100,'fmincon.MaxIter', 1e100);
%settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','ipopt','showprogress',1,'verbose',2,'ipopt.tol',1e-10,'debug',1);
result=optimize(constraints,obj,settings);
%check(constraints)

%% END OF OPTIMIZATION

B_value=value(B);
A_value=value(A);
U_value=value(U);
W_value=value(W);
V_value=value(V);

B_bezier= A_bezier(2:5,:);
disp("abs(|B_bezier|/|B_mio|)=")
abs(det(B_bezier)/det(B_value))

%%
sum_Vi_value=value(sum_Vi);
sum_Wi_value=value(sum_Wi);
% C_value=value(C);
% D_value=value(D);

W1=W_value(:,1:3); V1=V_value(:,1:2);
W2=W_value(:,4:6); V2=V_value(:,3:4);
W3=W_value(:,7:9); V3=V_value(:,5:6);
W4=W_value(:,10:12); V4=V_value(:,7:8);

% t=0.8;
% 
% [1 t]*(t*C_value + D_value)*[1;t]

figure
syms t real
T=[t*t*t*t t*t*t t*t t 1]';
t2=[1 t]';
lambda1= A_value(:,1)'*T;
lambda2= A_value(:,2)'*T;
lambda3= A_value(:,3)'*T;
lambda4= A_value(:,4)'*T;
fplot(lambda1,[0,1]); hold on;
fplot(lambda2,[0,1]);
fplot(lambda3,[0,1]);
fplot(lambda4,[0,1]);
xlim([0 1])

% temporal=t2'*(t*W3 + (1-t)*V3)*t2; %should be lambda1
% coeff_temporal=vpa(coeffs(temporal,t),4)

disp('wwwwwwwwwwwwwwwwwwwwwwwwww')
coeff_lambda1=vpa(coeffs(lambda1,t),4)
coeff_lambda2=vpa(coeffs(lambda2,t),4)
coeff_lambda3=vpa(coeffs(lambda3,t),4)
coeff_lambda4=vpa(coeffs(lambda4,t),4)



%%


T3=[t*t*t t*t t 1]';


%%
figure; hold on;
fplot3(pol_x'*T,pol_y'*T,pol_z'*T,[0 1],'r','LineWidth',3);
% axis equal
volumen_mio=plot_convex_hull(pol_x(2:end),pol_y(2:end),pol_z(2:end),B_value,'b');
volumen_bezier=plot_convex_hull(pol_x(2:end),pol_y(2:end),pol_z(2:end),B_bezier,'g');
disp("abs(|A_bezier|/|A_mio|)=")
abs(det(A_bezier)/det(A_value))
disp("volumen_mio/volumen_bezier=")
volumen_mio/volumen_bezier



% coeff_w1=[-32 64 -40 8]/9.0';
% coeff_w2=[64 -112 49 0]/9.0';
% coeff_w3=[-64 80 -17 1]/9.0';
% coeff_w4=[32 -32 8 0]/9.0';
% 
% figure
% fplot(coeff_w1*T,[0,1]); hold on;
% fplot(coeff_w2*T,[0,1]);
% fplot(coeff_w3*T,[0,1]);
% fplot(coeff_w4*T,[0,1]);

% figure;
% subplot(2,1,1);
% fplot(pol_x'*T,pol_y'*T,[0 1],'r','LineWidth',3)
% xlabel('x'); ylabel('y');
% subplot(2,1,2);
% fplot(pol_x'*T,pol_z'*T,[0 1],'r','LineWidth',3)
% xlabel('x'); zlabel('z');