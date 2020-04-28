%This files solves the problem without any assumptions on the structure of
%the polynomials.
clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')


deg=3;
W=[]; 

deg_is_even = (rem(deg, 2) == 0);

syms t real
A=[]; 

all_S=[]; %the row-block matrices are each S_i
all_R=[];

if(deg_is_even==0)
    
    d=(deg-1)/2.0; %d from Pablo Parrillo's lecture; 
    deg_s=2*d;
    Tf=[]; %T flipped. I.e. [1 t t^2,...]
    for i=0:d
        Tf=[Tf; t^(i)]; 
    end
    
    dim_S=d+1;
    dim_R=d+1;
    
    for i=1:(deg+1)
%         sqrtS=sym(['sqrtS' num2str(i) '_%d%d'],[d+1,d+1],'real');
%         sqrtR=sym(['sqrtR' num2str(i) '_%d%d'],[d+1,d+1],'real');
% 
%         S=sqrtS*sqrtS'; %This is positive semidefinite
%         R=sqrtR*sqrtR'; %This is positive semidefinite
        
        S=sym(['sqrtS' num2str(i) '_%d%d'],[d+1,d+1],'real');
        R=sym(['sqrtR' num2str(i) '_%d%d'],[d+1,d+1],'real');
        
        
        S = tril(S,0) + tril(S,-1).';
        R = tril(R,0) + tril(R,-1).';
        
        all_S=[all_S; S];
        all_R=[all_R; R];
        
        s=Tf'*S*Tf;
        r=Tf'*R*Tf;

        lambda=(t+1)*s + (1-t)*r;
        coeff_lambda= coeffs(lambda,t,'All');
        A=[A ; coeff_lambda]; %A is [a1  b1 c1 d1; a2 b2 c2 d2; ...]
    end
    
end

x=symvar(A);

%Solve the optimization problem. Note that the only constraint is \sum lambda_i=1;

x0 = rand(size(x));

opts = optimoptions('fmincon','Algorithm','sqp','MaxIterations',10000);
problem = createOptimProblem('fmincon','objective', ...
    @(x_i) getObj(x_i,A,x),'x0',x0,'lb',[],'ub',[],...
    'nonlcon',@(x_i) getConstraints(x_i,A,x, all_S, all_R, dim_S, dim_R),'options',opts);

% Construct a GlobalSearch object
% gs = GlobalSearch('Display','iter');
% Construct a MultiStart object based on our GlobalSearch attributes
ms = MultiStart('Display','iter','UseParallel',true);

[xgs,~,~,~,solsgs] = run(ms,problem,100);

%Kill parallel pool
poolobj = gcp('nocreate');
delete(poolobj);

%%
A_solution=double(vpa(subs(A,x,xgs)));

all_S_solution=double(vpa(subs(all_S,x,xgs)));

%%
%     for i=1:((size(all_S,1)/dim_S))
%         S_i=[all_S((i-1)*dim_S+1:(i)*dim_S,:)]
%         -eig(S_i);
%     end

%%

function result=getObj(x_i,A,x)
    A_i=double(subs(A,x,x_i)); %x_i is the current value;
    result=-abs(det(A_i));  %I want to maximize the determinant of A
end

function [c,ceq] =getConstraints(x_i,A,x, all_S, all_R, dim_S, dim_R)
    A_i=double(subs(A,x,x_i)); %x_i is the current value;
    all_S_i=double(subs(all_S,x,x_i));
    all_R_i=double(subs(all_S,x,x_i));
    c=[];
    
    for i=1:((size(all_S,1)/dim_S))
        S_i=[all_S_i((i-1)*dim_S+1:(i)*dim_S,:)];
        c=[c, -eig(S_i)];
    end

    for i=1:((size(all_R,1)/dim_R))
        R_i=[all_R_i((i-1)*dim_R+1:(i)*dim_R,:)];
        c=[c, -eig(R_i)];
    end

    ceq=[sum(A_i(:,1:end-1),1) , sum(A_i(:,end))-1];  %The sum of the rows of A is [0 0 0 1]
end
