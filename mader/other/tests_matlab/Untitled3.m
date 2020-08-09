%This program solves the dynaics obstacle avoidance (v keeps changing) 
clear; clc; close all;
%set(0,'DefaultFigureWindowStyle','docked')
set(0,'DefaultFigureWindowStyle','normal')
%Useful to plot the result: http://nurbscalculator.in/

%READ THIS: https://yalmip.github.io/example/nonconvexquadraticprogramming/
%READ THIS about envelopes: https://yalmip.github.io/tutorial/envelopesinbmibnb/
deg_polynomials=3;
num_polynomials=5;
v_initial=[0; 0];
R=1;
initial_point=[0;0; 0];
final_point=[10;10; 10];

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
   Q=[Q sdpvar(3,1)];
end

%Normal and d of the plane
a=[];
d=[];
for i=0:(N-3)
    a=[a sdpvar(3,1)];
    d=[d; sdpvar(1)];
end

%objective
obj=0;
for i=1:(N-1)  %(p-1):(N-p+1) is ok if I'm specifying vel as well (so CP(0),CP(1),CP(N-1),CP(N) are fixed)
    obj=obj  + (   norm(CP(i+1)-2*CP(i)+ CP(i-1))  )^2;
end
%%
tt=[0:num_polynomials];
limits_x_spline=[0 10];
limits_y_spline=[0 10];
limits_z_spline=[0 10];
[x_spline, y_spline, z_spline] = getDynTraj(tt, limits_x_spline, limits_y_spline,  limits_z_spline);

%constraints
constraints=[];
  
constraints_ineq=[]; %inequality constraints f(x)>=0;
for i=0:N-3
    [r0, r1, r2, r3]= getCPOfDynTraj(i, x_spline, y_spline, z_spline);

    constraints=[constraints gn(i)'*gn(i)== 1.0]; %Needed to avoid the trivial solution gn=0, d=0
                                                  % ==1.0 works better than
                                                  % >= 1.0
    
    %constraints=[constraints gn(i)'*v + d(i+1)>=0];
    %constraints=[constraints (gn(i)'*v + d(i+1))^2>=(gn(i)'*gn(i)*R^2)];
    
    %novale_gni= (CP(i+0) + CP(i+1) + CP(i+2) +CP(i+3)-(r0 + r1 + r2 + r3));
    
    constraints=[constraints gn(i)'*r0 + d(i+1)>=0];
    constraints=[constraints gn(i)'*r1 + d(i+1)>=0];
    constraints=[constraints gn(i)'*r2 + d(i+1)>=0];
    constraints=[constraints gn(i)'*r3 + d(i+1)>=0];
    
    for j=0:3
        constraints=[constraints gn(i)'*CP(i+j)+ d(i+1)<=0]; %Non convex constraint
    end
        
%Comment this out if you want to use the sphere constraints
%     constraints=[constraints (v-gn(i))'*(v-gn(i))>=(100*R+R)^2]; %outside the sphere
%     for j=0:3       
%         constraints=[constraints norm(CP(i+j)-gn(i))^2<=(100*R)^2]; %inside the sphere
%     end
%     
end


%This constraints are needeed if you use bmibnb
% constraints=[constraints -10<=Q(:)<=10];
% constraints=[constraints -10<=a(:)<=10];
% constraints=[constraints -10<=d(:)<=10];

%Assume we are clamping start and end (see http://nurbscalculator.in/ )
constraints_eq=[]; %Equality constraints f(x)==0;

constraints=[constraints CP(0)==initial_point];
constraints=[constraints CP(N)==final_point];


use_initial_guess=1;
if (use_initial_guess==1)
    assign(CP(0),initial_point);
    assign(CP(1),[1;1;1]);
    assign(CP(2),[2;3;2]);
    assign(CP(3),[3;4;3]);
    assign(CP(4),[5;5;3]);
%     assign(CP(5),[6;4;3]);
%     assign(CP(6),[7;3;3]);
%     assign(CP(7),final_point);
end

disp('Starting optimization') %'solver','bmibnb'  ,'solver','sdpt3'
result=optimize(constraints,obj,sdpsettings('usex0',use_initial_guess,'solver','ipopt','showprogress',1,'verbose',2,'debug',0 ));

Q_value=value(Q)
a_value=value(a)
d_value=value(d')

global deltaT;
T_max=3; %t \in [0, T_max]
deltaT=T_max/(M-2*p);

%plot trajectory found
knots_interior=deltaT:deltaT:(T_max-0.5*deltaT);
knots=[zeros(1,p+1) knots_interior  (max(knots_interior)+deltaT)*ones(1,p+1)];
tmp=bspline_deboor(p+1,knots,Q_value);
h=colormapline(tmp(1,:),tmp(2,:),tmp(3,:),jet(length(tmp(2,:))));axis equal; hold on;
set(h,'linewidth',4)

%plot_circle(v(1),v(2),R);

%plot control points found
plot3(Q_value(1,:),Q_value(2,:),Q_value(3,:),'ro')

%plot the planes found
% for i=1:size(d_value,2)
%     fimplicit(@(x,y) a_value(1,i)*x + a_value(2,i)*y + d_value(i)) 
% end

%plot the convex hull
% for i=0:N-3
%     P=Q_value(:,(i+1):(i+1)+3)';
%     [k,av] = convhull(P);
%     fill(P(k,1),P(k,2),'r')
% end


x_dyn=ppval(x_spline,min(tt):0.1:max(tt));
y_dyn=ppval(y_spline,min(tt):0.1:max(tt));
z_dyn=ppval(z_spline,min(tt):0.1:max(tt));
h=colormapline(x_dyn,y_dyn,z_dyn,jet(length(y_dyn)));
set(h,'linewidth',4,'linestyle','--')

alpha(.5)

saveampl(constraints,obj,'mya_model1.mod');

disp("**********************************************************************************")
disp("**********************************************************************************")

%% Function definitions

function [x_spline, y_spline, z_spline] = getDynTraj(tt, limits_x_spline, limits_y_spline,  limits_z_spline)

    amp=2;
    x = tt + amp*rand(1,length(tt));
    y = tt + amp*rand(1,length(tt));
    z=  tt + amp*rand(1,length(tt));
    
   
%     x=limits_x_spline(1) + (limits_x_spline(2)-limits_x_spline(1)).*rand(1,length(tt));
%     y=limits_y_spline(1) + (limits_y_spline(2)-limits_y_spline(1)).*rand(1,length(tt));
%     z=limits_z_spline(1) + (limits_z_spline(2)-limits_z_spline(1)).*rand(1,length(tt));
    
    x_spline=spline(tt,x);
    y_spline=spline(tt,y);
    z_spline=spline(tt,z);
end


function [r0, r1, r2, r3]= getCPOfDynTraj(interval, x_spline, y_spline, z_spline)

% y_spline=tmp.coefs;
%     t=0:0.01:10;
%     plot(ppval(x_spline,t), ppval(y_spline,t))
    
    %See https://arxiv.org/pdf/2001.04420.pdf, page 4
    
    j=interval+1;
    a=[x_spline.coefs(j,1) y_spline.coefs(j,1) z_spline.coefs(j,1)]';
    b=[x_spline.coefs(j,2) y_spline.coefs(j,2) z_spline.coefs(j,2)]';
    c=[x_spline.coefs(j,3) y_spline.coefs(j,3) z_spline.coefs(j,3)]';
    d=[x_spline.coefs(j,4) y_spline.coefs(j,4) z_spline.coefs(j,4)]';
    
    dt=x_spline.breaks(2)-x_spline.breaks(1);
    
    r0=d;
    r1=(c*dt + 3*d)/3;
    r2=(b*dt*dt + 2*c*dt +3*d)/3;
    r3=a*dt*dt*dt + b*dt*dt + c*dt +d;
    
    
%     alphaShape([r0(1) r1(1) r2(1) r3(1)],[r0(2) r1(2) r2(2) r3(2),[r0(3) r1(3) r2(3) r3(3)])
%     polyout2 = polybuffer(polyin,0.1,'JointType','miter','MiterLimit',2);
%     plot(polyin)
%     hold on
%     plot(polyout2)

    
end

function [x,y] = dyn_traj(t)
x = t;
y= t;
end

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
