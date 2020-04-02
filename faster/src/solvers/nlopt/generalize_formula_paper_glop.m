close all; clc; clear;

% mset('yalmip',true);
% mset(sdpsettings('solver','sdpt3'));

deg=5; %breaks (not enough memory) if deg>3 

mpol('B',deg-1,1)
mpol('R',(deg+1)/2, (deg-1)/2)
mpol('t',1,1) %This is not optimized, but needed to handle symbolic expressions


W=[];
%Insert half of the polynomials
for i=1:((deg+1)/2)
    pol=-B(i)*(t-1);
    for j=1:(deg-1)/2
        pol=pol*((t-R(i,j))^2);
    end
    W=[W;pol];
end

%Insert the other half (with -t)
for i=1:((deg+1)/2)
    pol=-B(i)*(-t-1);
    for j=1:(deg-1)/2
        pol=pol*((-t-R(i,j))^2);
    end
    W=[W;pol];
end

% Match the coefficients:
coeffic=getCoeffMlp(sum(W),t);
coeffic=[zeros(1,deg+1-length(coeffic)) coeffic];

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=getCoeffMlp(W(i),t);
    A=[A; tmp];
end

% d: relaxation order. SDP will be based on the moments up to order 2d.
% 2d>= max ( deg(p),deg(g_i) )
d=10;  %6 for deg=3
       %10 for deg=5
P = msdp(max(det(A)),B>=0,coeffic==[zeros(1,deg) 1],d);

% Solve Moment SDP
[status,obj] = msol(P)



