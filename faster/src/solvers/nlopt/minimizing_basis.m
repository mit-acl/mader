clc; clear; close all;
constraints=[]
sdpvar t a b c d t0 t1 t2 t3 real

pol_x=[0 0 0.2 0.3 2 1]';%[a b c d]
pol_y=[0 0 -0.3 +3 -5 6]';%[a b c d]
pol_z=[0 0 1 -0.1 -1 -4]';%[a b c d]

w0=-a*(t-1)*((t-t0)^2)*((c*t-t2)^2);
w3=-a*(-t-1)*((-t-t0)^2)*((-c*t-t2)^2);
w2=-b*(t-1)*((t-t1)^2)*((d*t-t3)^2);
w1=-b*(-t-1)*((-t-t1)^2)*((-d*t-t3)^2);

suma=w0+w1+w2+w3;

coeff_w0=flip(coefficients(w0,t));
coeff_w1=flip(coefficients(w1,t));
coeff_w2=flip(coefficients(w2,t));
coeff_w3=flip(coefficients(w3,t));

coeff_sum=flip(coefficients(suma,t));
pad=zeros(length(coeff_w0)-length(coeff_sum),1);
coeff_sum=[pad ;coeff_sum];
tmp=zeros(size(coeff_sum)); tmp(end)=1;




A=[coeff_w0 coeff_w1 coeff_w2 coeff_w3];
B=A(end-3:end,:);
detB=computeDet4(B)

constraints=[constraints coeff_sum==tmp a>=0 b>=0 detB>=0];
vx=sdpvar(4,1);
vy=sdpvar(4,1);
vz=sdpvar(4,1);
constraints=[constraints pol_x==A*vx pol_y==A*vy pol_z==A*vz];

obj=-detB;

assign(t0,0.05);
assign(t1,-0.77);
assign(t2,0.0);
assign(t3,0.0);
assign(c,0.0);
assign(d,0.0);

disp('Starting optimization') %'solver','bmibnb' 'fmincon' ,'solver','sdpt3' 'ipopt' 'knitro' 'scip'
ops=sdpsettings('savesolveroutput',1,'savesolverinput',1,'showprogress',1,'verbose',2,'debug',1);
settings=sdpsettings(ops,'usex0',0,'solver','fmincon','fmincon.maxfunevals',3e100,'fmincon.MaxIter', 1e100);
%settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','ipopt','showprogress',1,'verbose',2,'ipopt.tol',1e-10,'debug',1);
result=optimize(constraints,obj,settings);


A_value=value(A);
t0_value=value(t0)
t1_value=value(t1)
a_value=value(a)
b_value=value(b)
