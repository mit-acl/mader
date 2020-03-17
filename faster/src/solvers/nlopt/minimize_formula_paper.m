% Polynomial bases for quadratic and cubic polynomials which yield control points with small convex hulls     
close all; clc; clear all;
t0=sdpvar(1,1);
t1=sdpvar(1,1);
constraints=[];

assign(t0,.5);
assign(t1,0.2)

% density=0.00005;
% [t0,t1] = meshgrid(0:density:0.05,-0.8:density:-0.7);

density=0.01;
%Note that a>0 needs  t0>-0.5  and  t1<-0.5
% (and I suspect the obj function is convex in this region, should check
% with the hessian)
[t0,t1] = meshgrid(-0.5:density:1,-1:density:-0.5);

%maximize this function
obj=abs(4*((t0+0.5).^2).*((t1+0.5).^2).*(t0+t1+2)./(((t0-t1).^2).*((2*t0.*t1+t0+t1).^3)));

surf(t0,t1,obj); hold on;
zlim([-1 1])
colorbar
caxis([0 0.5])
t0_sol=0.030882;
t1_sol=-0.773548;
opt_value=0.331888;
stem3(t0_sol,t1_sol,opt_value,'filled','Color','r','MarkerSize',10)
xlabel('t0')
ylabel('t1')
%plot3(t0_sol,t1_sol,opt_value,'-o','Color','b','MarkerSize',10,'filled')
% t0(obj<1);
% t1(obj<1);
% obj(obj<1);
% contourf(t0,t1,obj)

% colorbar
% 

%%
fun = @(x)(-abs(4*((x(1)+0.5).^2).*((x(2)+0.5).^2).*(x(1)+x(2)+2)./(((x(1)-x(2)).^2).*((2*x(1).*x(2)+x(1)+x(2)).^3))));
A = [];
b = [];
Aeq = [];
beq = [];
lb = [-1,-1];
ub = [1,0.0];

x0 = [0,0];
[x,fval,exitflag,output] = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)


f_bezier=feval(fun,[-1,1]);
disp('ratio=')
f_bezier/fval
% 
% t0=x(1);
% t1=x(2);

t0=0.28;
t1=-0.43;

a=(-t1-0.5)/(2*t0*t1*t1-2*t0*t0*t1+t1*t1-t0*t0);
b=(t0+0.5)/(2*t0*t1*t1 - 2*t0*t0*t1 + t1*t1-t0*t0);

syms t
w0=-a*(t-1)*(t-t0)^2;
w3=a*(t+1)*(t+t0)^2;
w2=-b*(t-1)*(t-t1)^2;
w1=b*(t+1)*(t+t1)^2;


figure
fplot(w0,[-1,1]); hold on;
% fplot(w1,[-1,1]);
% fplot(w2,[-1,1]);
% fplot(w3,[-1,1]);


% 
% disp('Starting optimization') %'solver','bmibnb' 'fmincon' ,'solver','sdpt3' 'ipopt' 'knitro'
% result=optimize(constraints,obj,sdpsettings('usex0',1,'solver','bmibnb','showprogress',1,'verbose',2,'debug',1,'fmincon.maxfunevals',300000,'fmincon.MaxIter', 300000));
