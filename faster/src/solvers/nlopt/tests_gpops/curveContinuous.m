function output = curveContinuous( input )
% y = input.phase.state(:,2);
u = input.phase.control;
% xdot = u(1);
% ydot = sin (u);



v1=input.phase.parameter(:,1:3)';
v2=input.phase.parameter(:,4:6)';
v3=input.phase.parameter(:,7:9)';
v4=input.phase.parameter(:,10:12)';

lambda=input.phase(1).state;

t=input.phase(1).time;



pol_x=[0.2 0.3 2 1]';%[a b c d]
pol_y=[-0.3 +3 -5 6]';%[a b c d]
pol_z=[1 -0.1 -1 -4]';%[a b c d]


path1_1=pol_x(1)*t.*t.*t + pol_x(2)*t.*t + pol_x(3)*t + pol_x(4) - (lambda(:,1).*v1(1) + lambda(:,2).*v2(1) + lambda(:,3).*v3(1) + lambda(:,4).*v4(1)); 
path1_2=pol_y(1)*t.*t.*t + pol_y(2)*t.*t + pol_y(3)*t + pol_y(4) - (lambda(:,1).*v1(2) + lambda(:,2).*v2(2) + lambda(:,3).*v3(2) + lambda(:,4).*v4(2)); 
path1_3=pol_z(1)*t.*t.*t + pol_z(2)*t.*t + pol_z(3)*t + pol_z(4) - (lambda(:,1).*v1(3) + lambda(:,2).*v2(3) + lambda(:,3).*v3(3) + lambda(:,4).*v4(3)); 

path1_4=ones(size(lambda(:,1)))-(lambda(:,1)+lambda(:,2)+lambda(:,3)+lambda(:,4));


path1=[path1_1,path1_2,path1_3, path1_4];
output.path=path1;


output.dynamics = [ u(:,1) , u(:,2), u(:,3), u(:,4)];
% output.integrand=(y.*cos(u))
