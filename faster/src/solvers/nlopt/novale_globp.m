close all; clc; clear;

n=5; %breaks (not enough memory) if n>3 

mpol('B',n-1,1)
mpol('R',(n+1)/2, (n-1)/2)
mpol('r',1,1) %This is not optimized, but needed to handle symbolic expressions


W=[];
%Insert half of the polynomials
for i=1:((n+1)/2)
    pol=-B(i)*(r-1);
    for j=1:(n-1)/2
        pol=pol*((r-R(i,j))^2);
    end
    W=[W;pol];
end

%Insert the other half (with -t)
for i=1:((n+1)/2)
    pol=-B(i)*(-r-1);
    for j=1:(n-1)/2
        pol=pol*((-r-R(i,j))^2);
    end
    W=[W;pol];
end

% Match the coefficients:
coeffic=getCoeffMlp(sum(W),r);
coeffic=[zeros(1,n+1-length(coeffic)) coeffic];

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=getCoeffMlp(W(i),r);
    A=[A; tmp];
end

% d: relaxation order. SDP will be based on the moments up to order 2d.
% 2d>= max ( n(p),n(g_i) )
d=10;  %6 for n=3
       %10 for n=5
P = msdp(max(det(A)),B>=0,coeffic==[zeros(1,n) 1],d);

% Solve Moment SDP
[status,obj] = msol(P)