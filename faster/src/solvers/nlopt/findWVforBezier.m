clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')

%READ THIS: https://yalmip.github.io/example/nonconvexquadraticprogramming/

polynomial=[-1 3 -3 1];

a=polynomial(1);
b=polynomial(2);
c=polynomial(3);
d=polynomial(4);

W=sdpvar(2,2);
V=sdpvar(2,2);

constraints=[];
constraints=[constraints W(2,2)-V(2,2)==a];
constraints=[constraints -2*V(1,2)+V(2,2)+2*W(1,2)==b];
constraints=[constraints -V(1,1)+2*V(1,2)+W(1,1)==c];
constraints=[constraints V(1,1)==d];

constraints=[W>=0 V>=0];

obj=0;

disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
result=optimize(constraints,obj,sdpsettings('usex0',1,'solver','bmibnb','showprogress',1,'verbose',2,'debug',0,'fmincon.maxfunevals',300000 ));

W_value=value(W);
V_value=value(V);