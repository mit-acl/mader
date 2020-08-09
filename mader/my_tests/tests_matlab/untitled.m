clear; clc; close all;
set(0,'DefaultFigureWindowStyle','docked')
%Useful to plot the result: http://nurbscalculator.in/

%READ THIS: https://yalmip.github.io/example/nonconvexquadraticprogramming/

deg_polynomials=3;
num_polynomials=5;
v=[5; 0];
R=4;
initial_point=[0;0];
final_point=[10;0];

p=deg_polynomials;
M=num_polynomials+2*p;
N=M-p-1;

%Variables
%Control Points
global Q
global a
variables=[]
Q=[];
for i=0:N
   Q=[Q sdpvar(2,1)];
    variables = [variables Q(1,i+1) Q(2,i+1)];
end

%Normal and d of the plane
a=[];
d=[];
for i=0:(N-3)
    a=[a sdpvar(2,1)];
    d=[d; sdpvar(1)];
    variables = [variables a(1,i+1) a(2,i+1) d(i+1)];
end

%objective
obj=0;
for i=1:(N-1)  %(p-1):(N-p+1) is ok if I'm specifying vel as well (so CP(0),CP(1),CP(N-1),CP(N) are fixed)
    obj=obj  + (   norm(CP(i+1)-2*CP(i)+ CP(i-1))  )^2;
end

%constraints
constraints=[];
constraints_ineq=[]; %inequality constraints f(x)>=0;
for i=0:N-3
    f=gn(i)'*gn(i) - 0.001;
    constraints_ineq=[constraints_ineq f];
    constraints=[constraints f>=0]; %Needed to avoid the trivial solution gn=0, d=0
    f=gn(i)'*v + d(i+1);
    constraints_ineq=[constraints_ineq f];
    constraints=[constraints f>=0];
    f=(gn(i)'*v + d(i+1))^2 - (gn(i)'*gn(i)*R^2);
    constraints_ineq=[constraints_ineq f];
    constraints=[constraints f>=0];
    for j=0:3
        %Ensure that we can always find a plane that separates the polytope and
        %the sphere
        f= -( gn(i)'*CP(i+j)+ d(i+1));
        constraints_ineq=[constraints_ineq f];
        constraints=[constraints f>=0];
    end
end

%Assume we are clamping start and end (see http://nurbscalculator.in/ )
constraints_eq=[]; %Equality constraints f(x)==0;
f=CP(0)-initial_point;
constraints_eq=[constraints_eq f(1) f(2)];
constraints=[constraints f==0];
f=CP(N)-final_point;
constraints_eq=[constraints_eq f(1) f(2)];
constraints=[constraints f==0];


assign(CP(0),initial_point);
assign(CP(1),[1;1]);
assign(CP(2),[2;3]);
assign(CP(3),[3;4]);
assign(CP(4),[5;5]);
assign(CP(5),[6;4]);
assign(CP(6),[7;3]);
assign(CP(7),final_point);

disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
result=optimize(constraints,obj,sdpsettings('usex0',1,'solver','ipopt','showprogress',1,'verbose',2,'debug',0 ));

Q_value=value(Q)
a_value=value(a)
d_value=value(d')

global deltaT;
T_max=3; %t \in [0, T_max]
deltaT=T_max/(M-2*p);

knots_interior=deltaT:deltaT:(T_max-0.5*deltaT);
knots=[zeros(1,p+1) knots_interior  (max(knots_interior)+deltaT)*ones(1,p+1)];
tmp=bspline_deboor(p+1,knots,Q_value);
plot(tmp(1,:),tmp(2,:)); axis equal; hold on;
plot_circle(v(1),v(2),R);
plot(Q_value(1,:),Q_value(2,:),'ro')

for i=1:size(d_value,2)
    fimplicit(@(x,y) a_value(1,i)*x + a_value(2,i)*y + d_value(i)) %plot the planes
end

for i=0:N-3
    P=Q_value(:,(i+1):(i+1)+3)';
    [k,av] = convhull(P);
    fill(P(k,1),P(k,2),'r')
end
alpha(.5)

saveampl(constraints,obj,'mya_model1.mod');

disp("**********************************************************************************")
disp("**********************************************************************************")

%% Now formulate the problem as the standard form (solution should be the
%same)

variables_aug=sdpvar(size(variables,2)+1,1);

[C,c,novale,z,info]=quaddecomp(obj,variables); 
c = 0.5*c;  %0.5 to match the expression (22) of the paper "semidefinite relaxation of quadratic optimization problems"
C_aug=[C c; c' 0];
obj=variables_aug'*C_aug*variables_aug;

constraints_aug=[];
for i=1:size(constraints_ineq, 2)
    constraints_ineq(i);
    [A_i,a_i,b_i,z,info] = quaddecomp(constraints_ineq(i),variables); %x'*A_i*x + a_i'*x + b_i
    a_i = 0.5*a_i; %0.5 to match the expression (22) of the paper "semidefinite relaxation of quadratic optimization problems"
    A_aug_i=[A_i a_i; a_i' 0];
    constraints_aug=[constraints_aug variables_aug'*A_aug_i*variables_aug + b_i>= 0];
end

for i=1:size(constraints_eq, 2)
    sdisplay(constraints_eq(i))
    [A_i,a_i,b_i,z,info] = quaddecomp(constraints_eq(i),variables); %x'*A_i*x + a_i'*x + b_i
    a_i = 0.5*a_i; 
    A_aug_i=[A_i a_i; a_i' 0];
    constraints_aug=[constraints_aug variables_aug'*A_aug_i*variables_aug +  b_i==0];
    sdisplay(variables_aug'*A_aug_i*variables_aug +  b_i)
end

f=variables_aug(end)*variables_aug(end) -1;
constraints_aug=[constraints_aug f==0];

assign(variables_aug(1:2),initial_point);
assign(variables_aug(3:4),[1;1]);
assign(variables_aug(5:6),[2;3]);
assign(variables_aug(7:8),[3;4]);
assign(variables_aug(9:10),[5;5]);
assign(variables_aug(11:12),[6;4]);
assign(variables_aug(13:14),[7;3]);
assign(variables_aug(15:16),final_point);
assign(variables_aug(end),1);

result=optimize(constraints_aug,obj,sdpsettings('usex0',1,'solver','ipopt','showprogress',1,'verbose',2,'debug',0 ));
%saveampl(constraints_aug,obj,'mya_model2.mod');

disp("**********************************************************************************")
disp("**********************************************************************************")

%% Now solve the standard form with the trace (solution should be the
% same)
X=sdpvar(size(variables_aug,1),size(variables_aug,1));  % SYMMETRIC by default
obj=trace(C_aug*X);
constraints_augtr=[];

disp("Going to optimize")
for i=1:size(constraints_ineq, 2)
    constraints_ineq(i);
    [A_i,a_i,b_i,z,info] = quaddecomp(constraints_ineq(i),variables); %x'*A_i*x + a_i'*x + b_i
    a_i = 0.5*a_i;   %0.5 to match the expression (22) of the paper "semidefinite relaxation of quadratic optimization problems"
    A_aug_i=[A_i a_i; a_i' 0];
    constraints_augtr=[constraints_augtr trace(A_aug_i*X) + b_i>= 0];
end

disp("Going to optimize")
for i=1:size(constraints_eq, 2)
    [A_i,a_i,b_i,z,info] = quaddecomp(constraints_eq(i),variables); %x'*A_i*x + a_i'*x + b_i
    a_i = 0.5*a_i; 
    A_aug_i=[A_i a_i; a_i' 0];
    constraints_augtr=[constraints_augtr (trace(A_aug_i*X) + b_i) == 0];
end

f=variables_aug(end)*variables_aug(end) -1;
S=zeros(length(variables_aug),length(variables_aug));
S(end,end)=1;
constraints_augtr=[constraints_augtr trace(S*X)-1==0 ];  %This is the constraint t*t=1

constraints_augtr=[constraints_augtr X>=0]; %X is PSD
%constraints_augtr=[constraints_augtr rank(X)==1]; %X has rank 1

disp("Going to optimize")
result=optimize(constraints_augtr,obj,sdpsettings('usex0',0,'solver','mosek','showprogress',1,'verbose',2,'debug',0 ));

disp("The rank of the matrix found is ")
X_value=value(X);
rank(X_value)
[V,D] = eig(X_value);

%Sort in descending order, see https://www.mathworks.com/help/matlab/ref/eig.html
[d,ind] = sort(diag(D),'descend');
Ds = D(ind,ind);
Vs = V(:,ind);

x_tilda=sqrt(Ds(1,1))*V(:,1); %\lambda_max * v(\lambda_max)

%residuals are positive for a constraint that is satisfied
assign(variables_aug,x_tilda'*x_tilda);
check(constraints_augtr)

%% Function definitions
function result=gn(i)%get normal
    global a
    result=a(:,i+1);
end

function result=CP(i)%control points
    global Q
    result=Q(:,i+1);
end

function result=getPos(t,Q)%time, control points
    global deltaT;
    M4=(1/(3*2))*[1 4 1 0;
                  -3 0 3 0;
                  3 -6 3 0;
                  -1 3 -3 1]; %Does not depend on i for uniform b-splines

    i= round((t-mod(t,deltaT))/deltaT)+4;
    %i=floor(t)+4; %or + 3??
    s=(t-(i-4)*deltaT)/deltaT;
    result_x=[1 s s*s s*s*s]*M4*[Q(1,i-3) ; Q(1,i-2); Q(1,i-1) ;Q(1,i)];
    result_y=[1 s s*s s*s*s]*M4*[Q(2,i-3) ; Q(2,i-2); Q(2,i-1) ;Q(2,i)];
    result=[result_x; result_y];
end

function h = plot_circle(x,y,r)
%hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
plot(xunit, yunit);
h=[xunit; yunit];
% hold off
end




% for i=1:size(constraints_ineq, 2)
%     value(constraints_ineq(i)) %Should all be >=0 if the rank-one approx is feasible
% end
% 


% [U S V] = svd(X_value);
% k=1; %rank k approximation, see https://stackoverflow.com/questions/28571399/matrix-low-rank-approximation-using-matlab
% Ak = U(:,1:k)*S(1:k,1:k)*V(:,1:k)';

%saveampl(constraints,obj,'myamplmodel.mod');

% novale=sdisplay(obj)
% novale=cell2mat(novale)

%sdisplay(hessian(obj))

% See https://mathematica.stackexchange.com/questions/114464/get-the-coefficient-matrix-from-a-quadratic-form
%C=full(0.5*hessian(obj,variables));


% for i=1:size(constraints_ineq,2)
%     A_i=full(0.5*hessian(constraints_ineq(i),variables));
%     %all(eig(A_i)>=-0.000001)
% end

% figure
% getPos(0.0001,Q_value)
% t=0.01:0.01:4.99;
% traj=[];
% for i=1:size(t,2)
%     traj=[traj getPos(t(i),Q_value)];
% end
% plot(traj(1,:),traj(2,:))

% tmp=0;
% for i=(p-1):(N-p+1)
%     value((   norm(CP(i+1)-2*CP(i)+ CP(i-1))  )^2)
% end

% for i=0:N
%     tmp=sqrtm((CP(i)-v)'*(CP(i)-v));
%     constraints=[constraints tmp>=0.1];
% end

%     dc=sqrtm((CP(w+3)-v)'*(CP(w+3)-v));
%     constraints=[constraints dc>=0];
%     constraints=[constraints sqrtm((CP(w+3)-CP(w+2))'*(CP(w+3)-CP(w+2)))<=(dc/3)];
%     constraints=[constraints sqrtm((CP(w+2)-CP(w+1))'*(CP(w+2)-CP(w+1)))<=(dc/3)];
%     constraints=[constraints sqrtm((CP(w+1)-CP(w+0))'*(CP(w+1)-CP(w+0)))<=(dc/3)];

%     Q=sdpvar(6,1);
%     Rd=sdpvar(3,3,'full'); %Rotation Matrix
%     delta=sdpvar(1);
%     
%     
% 
% constraints=[sum(b)==number_of_points]; %Number of points chosen
% objective = -trace(A);
% settings=sdpsettings('solver','bnb','bnb.solver','sdpt3');
% disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
% result=optimize(constraints,objective,sdpsettings(settings,'showprogress',1,'verbose',2,'debug',0 )); 
% result.info
% disp('Finished optimization')
