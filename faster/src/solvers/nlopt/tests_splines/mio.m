close all; clc; clear;
%n = 4;
%knots = [0 0 0 0 1 2 3 4 5 5 5 5];  % knot vector
%P = [ 0          0.4    0.773333      2.10425       6.5357           10           10           10 ...
%      ;0            0            0  0.000488454   0.00515391            0            0            0];



p=3;
n = 4; %4 for degree 3
knots = [0     0     0     0 0.143 0.286 0.429 0.571 0.714 0.857     1     1     1     1];  % knot vector
% P = [ 1          0.4    0.773333      2.10425       6.5357           10           10           10 ...
%       ;1            0            0  0.000488454   0.00515391            0            0            0];

  

% P=[        0          0.4    -0.773333      2.10425       6.5357           10           10           10;
%            0            0            0  0.000488454   0.00515391            0            0            0;
%            0            0            0 -0.000481268   -0.0024835            0            0            0];
       
       
P=[  0  0.143  0.429  0.565  0.123 -0.404    7.42   9.07   9.07   9.07;
     0  0.143  0.429   0.46  0.505  0.316   -0.783  -1.07  -1.07  -1.07;
     1   1.14   1.43  0.471 -0.222  -0.18   0.837   -1.2   -1.2   -1.2];


z=1; %Matlab, why do you use one-indexing??

interv=0.0001;
       
X = bspline_deboor(n,knots,P, 0:interv:1);

vel_x=diff(X(1,:))/interv;
accel_x=diff(vel_x)/interv;

deltaT=knots(p+1+z)-knots(1+z);

my_velocity=(p/(deltaT))*(P(:,1+z)-P(:,z))

his_velocity=vel_x(1)


Mbspline=(1/6)*[ 1 4 1 0;
                 -3 0 3 0;
                 3 -6 3 0;
                -1 3 -3 1];
            
Mbezier=[1 0 0 0;
         -3 3 0 0;
         3 -6 3 0;
         -1 3 -3 1];
     
CPoints=[]

[1 u u*u u*u*u]*Mbspline*

inv(Mbezier)*Mbspline
     

     
% figure;
% hold all;
% plot(X(1,:), X(2,:), 'b');
% plot(P(1,:), P(2,:), 'ro');
% hold off;
% 
% 
% 
% P_augmented=[ P]; %P(:,1) P(:,1)  P(:,1) 
% 
% M=(1/6.0)*[-1 3 -3 1 ;
%             3 -6 3 0;
%            -3 0 3 0;
%             1 4 1 0];
% 
% 
%  hold on;
% 
% for t=2:0.01:3 %min(knots):0.01:max(knots)
%     segment=floor(t)+1
%     tmp=P_augmented(:,segment:segment+3)
%     tmp=tmp';
%     
%     tmp_x=tmp(:,1);
%     curve_x=[t*t*t t*t t 1]*M*tmp_x;
%     
%     tmp_y=tmp(:,2);
%     curve_y=[t*t*t t*t t 1]*M*tmp_y
%     
%     plot(curve_x, curve_y, 'go'); hold on;
% 
% end

