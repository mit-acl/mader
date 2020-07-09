close all; clc; clear;
deg_polynomials=3;
num_polynomials=5;
v_initial=[0; 0];
R=1;
initial_point=[0;0; 0];
final_point=[10;10; 10];

p=deg_polynomials;
M=num_polynomials+2*p;
N=M-p-1;

tt=[0:num_polynomials];
limits_x_spline=[0 10];
limits_y_spline=[0 10];
limits_z_spline=[0 10];
[x_spline, y_spline, z_spline] = getDynTraj(tt, limits_x_spline, limits_y_spline,  limits_z_spline);

R=[];
for i=0:N-3
    [r0, r1, r2, r3]= getCPOfDynTraj(i, x_spline, y_spline, z_spline);
    R=[R r0 r1 r2 r3]
end

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
