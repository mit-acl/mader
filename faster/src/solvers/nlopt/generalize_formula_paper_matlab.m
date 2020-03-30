close all; clc; clear;
syms t

global R B_solved determ

deg=3;

W=[];

B=sym('B',[((deg+1)/2),1],'real');
R=sym('R',[(deg+1)/2,(deg-1)/2],'real');

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
W=[W;subs(W,t,-t)]

%Solve for the coefficients:
coeffic=coeffs(sum(W),t,'All'); %When using All, no flip!! (gives directly [a  b c d]

coeffic=[zeros(1,deg+1-length(coeffic)) coeffic];

solution=solve(coeffic==[zeros(1,deg) 1],B); %This is a linear system

B_solved=simplify(struct2array(solution)');
W=subs(W,B,B_solved);

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=coeffs(W(i),t,'All');
    A=[A; tmp];
end
%Compute the determinant
determ=simplify(det(A));



% fun = @(x)100*(x(2)-x(1)^2)^2 + (1-x(1))^2;
% 'lb',lb,'ub',ub,
lb=-ones(size(R));
ub=ones(size(R));

x0 = rand(size(R));
opts = optimoptions('fmincon','Algorithm','sqp','MaxIterations',10000);
problem = createOptimProblem('fmincon','objective', ...
    @(R_x) getObj(R_x),'x0',x0, ...
    'nonlcon',@(R_x) constraints(R_x),'options',opts);



% Construct a GlobalSearch object
gs = GlobalSearch('NumTrialPoints',5000000);
% Construct a MultiStart object based on our GlobalSearch attributes
ms = MultiStart;

[xgs,~,~,~,solsgs] = run(gs,problem);

% [xms,~,~,~,solsgs] = run(ms,problem,100);

%%%%%%%%%%%%%%%%%%%%%%%%%
% ms = MultiStart(ms,'UseParallel',true);
% 
% try
%     demoOpenedPool = false;
%     % Create a parallel pool if one does not already exist
%     % (requires Parallel Computing Toolbox)
%     if max(size(gcp)) == 0 % if no pool
%         parpool
%         demoOpenedPool = true;
%     end
% catch ME
%     warning(message('globaloptim:globaloptimdemos:opticalInterferenceDemo:noPCT'));
% end
% 
% % Run the solver
% tic;
% [xms,~,~,~,solsgs] = run(ms,problem,100);
% toc
% xms

%%%%%%%%%%%%%%%%%%%%

B_solution=vpa(subs(B_solved,R,xgs));
A_solution=vpa(subs(A,R,xgs));
det(A_solution)


function result=getObj(R_x)
    global R determ
    R_x;
    result=subs(-abs(determ),R,R_x);
    result=double(result)
end

function [c,ceq] =constraints(R_x)
    global R B_solved
    c=-subs(B_solved,R,R_x); %  -B(i) <=0
    c=[c ; subs(R,R,R_x) - 2*ones(size(R))]; %R(i) <= 2
    c=[c ; -subs(R,R,R_x) - 2*ones(size(R))]; % R(i) >=-2 //// -2 - R(i) <=0
    c=double(c);
    ceq=[];
end

% function [c,ceq] = ellipseparabola(x)
% c(1) = (x(1)^2)/9 + (x(2)^2)/4 - 1;
% c(2) = x(1)^2 - x(2) - 1;
% ceq = [];
% end
