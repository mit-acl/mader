clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')

%READ THIS: https://yalmip.github.io/example/nonconvexquadraticprogramming/
A=[  0 0 0 0;
    -1 3 -3 1;
    3 -6 3 0;
    -3 3 0 0;
     1 0 0 0];

W_solution=[];
V_solution=[];

for i=1:4

polynomial=A(:,i);



Wi=sdpvar(3,3);
Vi=sdpvar(2,2);

lambdai=[
         -Vi(2,2) + Wi(3,3); ...
        -2*Vi(1,2)+Vi(2,2)+2*Wi(2,3); ...
        -Vi(1,1)+2*Vi(1,2)+2*Wi(1,3)+Wi(2,2); ...
         Vi(1,1)+2*Wi(1,2); ...
         Wi(1,1) ...
        ];

constraints=[];
constraints=[constraints lambdai==polynomial];
constraints=[constraints Wi>=0 Vi>=0];

obj=0;

disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
result=optimize(constraints,obj,sdpsettings('usex0',0,'solver','mosek','showprogress',1,'verbose',2,'debug',0,'fmincon.maxfunevals',300000 ));

W_value=value(Wi)
V_value=value(Vi)

polynomial_solution=[W_value(2,2)-V_value(2,2)  ,   -2*V_value(1,2)+V_value(2,2)+2*W_value(1,2)   ,     -V_value(1,1)+2*V_value(1,2)+W_value(1,1)   ,  V_value(1,1)]

W_solution=[W_solution W_value];
V_solution=[V_solution V_value];

end

% W1=W_solution(:,1:2); V1=V_solution(:,1:2);
% W2=W_solution(:,3:4); V2=V_solution(:,3:4);
% W3=W_solution(:,5:6); V3=V_solution(:,5:6);
% W4=W_solution(:,7:8); V4=V_solution(:,7:8);
% 
% sum_Wi=W1+W2+W3+W4;
% sum_Vi=V1+V2+V3+V4;
% 
% C=sum_Wi-sum_Vi;
% D=sum_Vi;
% 
% C(2,2)
% 
% C(1,2) + C(2,1) + D(2,2)
% 
% C(1,1) + D(1,2) +D(2,1)
% 
% D(1,1)
% 
% sum_total=zeros(2,2);
% t=0.8;
% for i=1:4
%     Wi=W_solution(:,2*i-1:2*i);
%     Vi=V_solution(:,2*i-1:2*i);
%     
%     sum_total=sum_total+ t*Wi + (1-t)*Vi;
%     
% end
% 
% [1 t]*sum_total*[1;t]

