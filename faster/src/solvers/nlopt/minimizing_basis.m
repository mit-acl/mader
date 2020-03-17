clc; clear; close all;
constraints=[]
sdpvar t a b t0 t1 real

w0=-a*(t-1)*((t-t0)^2);
w3=-a*(-t-1)*((-t-t0)^2);
w2=-b*(t-1)*((t-t1)^2);
w1=-b*(-t-1)*((-t-t1)^2);

suma=w0+w1+w2+w3;

coeff_w0=flip(coefficients(w0,t));
coeff_w1=flip(coefficients(w1,t));
coeff_w2=flip(coefficients(w2,t));
coeff_w3=flip(coefficients(w3,t));

coeff_sum=flip(coefficients(suma,t));
pad=zeros(length(coeff_w0)-length(coeff_sum),1);
coeff_sum=[pad ;coeff_sum];
tmp=zeros(size(coeff_sum)); tmp(end)=1;
constraints=[constraints coeff_sum==tmp a>=0 b>=0];

A=[coeff_w0 coeff_w1 coeff_w2 coeff_w3];
detA=computeDet4(A)
obj=-detA;

assign(t0,0.05);
assign(t1,-0.77);

disp('Starting optimization') %'solver','bmibnb' 'fmincon' ,'solver','sdpt3' 'ipopt' 'knitro' 'scip'
ops=sdpsettings('savesolveroutput',1,'savesolverinput',1,'showprogress',1,'verbose',2,'debug',1);
settings=sdpsettings(ops,'usex0',1,'solver','fmincon','fmincon.maxfunevals',3e100,'fmincon.MaxIter', 1e100);
%settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','ipopt','showprogress',1,'verbose',2,'ipopt.tol',1e-10,'debug',1);
result=optimize(constraints,obj,settings);

t0_value=value(t0)
t1_value=value(t1)
a_value=value(a)
b_value=value(b)
