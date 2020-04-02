close all; clc; clear;

deg=5;
sdpvar t %This is not optimized, but needed to handle symbolic expressions
W=[];

B=sdpvar((deg-1),1);
R=sdpvar((deg+1)/2,(deg-1)/2);

%Insert half of the polynomials
for i=1:((deg+1)/2)
    pol=-B(i)*(t-1);
    for j=1:(deg-1)/2
        pol=pol*((t-R(i,j))^2);
    end
    pol
    W=[W;pol];
end
%Insert the other half
W=[W;replace(W,t,-t)];

%%

%Solve for the coefficients:
coeffic=flip(coefficients(sum(W),t)');
coeffic=[zeros(1,deg+1-length(coeffic)) coeffic];

constraints=[];
constraints=[constraints coeffic==[zeros(1,deg) 1] ]; %Sum has to be 1
constraints=[constraints B>=zeros(size(B)) ]; %The elements of B have to be >=0

% constraints=[constraints R>=-ones(size(R)) R<=ones(size(R))]; %The roots have to be in [-1 1]

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=flip(coefficients(W(i),t))';
    A=[A; tmp];
end

%Compute the determinant

% For fmincon, det(A,'polynomial')  is much more robust to init cond, but goes VERY slow

obj=-det(A); %I want to maximize this determinant
% determ=simplify(det(A));
assign(R,[-0.5  0.5; -0.9, 0.5; -0.8, 0.0]); %Initial condition (roots of the polynomials)

assign(R,    [-0.5750    0.5897;
             -17.1312   -0.8211;
              0.1447   -0.8924] );
          
assign(B, [0.9982;
           0.0018;
           1.9487;
           7.3920]);

disp('Starting optimization') %'solver','bmibnb' 'fmincon' ,'solver','sdpt3' 'ipopt' 'knitro' 'scip'

settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','sdplr','showprogress',1,'verbose',2,'debug',1);
% settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','moment','showprogress',1,'verbose',2,'debug',1);
% settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','fmincon','showprogress',1,'verbose',2,'debug',1,'fmincon.maxfunevals',300000,'fmincon.MaxIter', 300000);
% settings=sdpsettings('usex0',1,'savesolveroutput',1,'savesolverinput',1,'solver','fmincon','showprogress',1,'verbose',2,'ipopt.tol',1e-10,'debug',1);
result=optimize(constraints,obj,settings);

B_value=value(B);
R_value=value(R);

A_value=value(A);

det_mine=value(det(A))

%%

syms t
B = bernsteinMatrix(deg, t);
B=subs(B,t,(0.5*t+0.5));% Change to interval [-1,1]
A_bez=[];
for i=1:length(W)
    tmp=(coeffs(B(i),t,'All'))
    A_bez=[A_bez; tmp]
end
det_bezier=det(A_bez);

disp('ratio=')
vpa(abs(det_bezier/det_mine,4))

% figure
% fplot(B, [-1,1])


% constraints=[constraints ]
% result=optimize(constraints,obj,settings);

%Generate vector T
syms t
T=[]
for i=flip(0:(deg)) 
    T=[T; t^i];
end

% fplot(A_value*T, [-1,1])


