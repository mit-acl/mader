% MIT 16.S498: Risk Aware and Robust Nonlinear Planning, Fall 2019
% Lecture 3: Sum Of Squares Based SDP For Nonlinear Optimization
%% Sparse Bounded Degree SOS based SDP for Constrained optimization


clear all;clc;
% number of variables
N=20; 
% objective function F(x)= 5+(x(1)-1)^2 + (x(2)-1)^2 .... (x(N)-1)^2
%[term1: pow(x1) pow(x2) pow(x3).... coeff;]
Ter=zeros(1,N);Ter(N+1)=5; %Term 5
for i=1:N
    % ter to reresent (x(i)-1)^2  : x(i)^2-2x(i)+1
    ter=zeros(3,N+1);
    ter(1,i)=2;ter(1,N+1)=1; % term1: x(i)^2
    ter(2,i)=1;ter(2,N+1)=-2; % term2: -2x(i)
    ter(3,i)=0;ter(3,N+1)=1; %  term3: 1
 Ter= [Ter;ter]; 
end
pop.F=Ter;

pop.n = N; % number of variables
pop.I = {1:N}; % interacting variables in obj

% Constraints
for i=1:N
    ter=zeros(2,N);
    ter(1,i)=2;ter(1,N+1)=-1;
    ter(2,i)=0;ter(2,N+1)=1;
pop.G{i} = ter;% 1-x(i)^2>=0
end
% interacting variables in constraints: {1,2,3...,N}
pop.J = {1:N};

pop.k=1; %  parameter of SOS
pop.d=2; %  parameter of LP

sdp = gendata2(pop,'SBSOS');

sol = csol(sdp,'sdpt3');

psol = postproc(pop,sdp,sol);

% clc
% optimal x and obj
x_opt=psol.xsol
f_opt=psol.fx

% rank of moment mat
%psol.rnk

% totla cpu time
t_cpu=psol.ttot