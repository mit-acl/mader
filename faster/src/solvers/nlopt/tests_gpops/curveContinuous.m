function output = curveContinuous( input )

global deg_pol dim coeff_pol

V=[]; %V is a matrix whose columns are the control points v1,v2,v3,...


for i=1:dim:size(input.phase.parameter,2)
    V=[V input.phase.parameter(1,i:i+(dim-1))']; %Note that each colums of input.phase.parameter has the same value 
end

lambda=input.phase(1).state;

t=input.phase(1).time;

% coeff_pol_x=rand(deg_pol+1,1);%[a b c d]
% coeff_pol_y=rand(deg_pol+1,1);%[a b c d]
% coeff_pol_z=rand(deg_pol+1,1);%[a b c d]


pol=zeros(size(t,1), dim);
sum_lambda_v=zeros(size(t,1), dim);
% pol_y=zeros(size(t));
% pol_z=zeros(size(t));

% sum_lambda_v_x=zeros(size(t));
% sum_lambda_v_y=zeros(size(t));
% sum_lambda_v_z=zeros(size(t));


for i=1:(deg_pol+1)
    
    for j=1:dim %loop over the dimensions x, y, z,...
%         disp('here1')
%         coeff_pol
%         disp('here1.2')
%         pol(:,j)
%         disp('here1.3')
%         coeff_pol(i,j)
%         disp('here1.4')
        pol(:,j)=pol(:,j) + coeff_pol(i,j)*(t.^(deg_pol+1-i));
%         disp('here2')
        sum_lambda_v(:,j) = sum_lambda_v(:,j) + lambda(:,i).*V(j,i);
%         disp('here3')
    end
% 
%     pol_x=pol_x+coeff_pol_x(i)*(t.^(deg_pol+1-i));
%     pol_y=pol_y+coeff_pol_y(i)*(t.^(deg_pol+1-i));
%     pol_z=pol_z+coeff_pol_z(i)*(t.^(deg_pol+1-i));
    
%     sum_lambda_v_x=sum_lambda_v_x + lambda(:,i).*V(1,i);
%     sum_lambda_v_y=sum_lambda_v_y + lambda(:,i).*V(2,i);
%     sum_lambda_v_z=sum_lambda_v_z + lambda(:,i).*V(3,i);
end


% pol_x_before=pol_x;
% sum_lambda_v_x_before=sum_lambda_v_x;

path1=pol-sum_lambda_v; 
path2=ones(size(lambda,1),1)-sum(lambda,2); %Vector of ones - sum of the rows of lambda

path1=[path1 path2];

output.path=path1;
output.dynamics = input.phase.control;

% path1_1=pol(:,1)-sum_lambda_v(:,1);
% path1_2=pol(:,2)-sum_lambda_v(:,2);
% path1_3=pol(:,3)-sum_lambda_v(:,3);



% 
% path1_before=[path1_1 path1_2 path1_3 path1_4];

%%%%%%%%%%%%%%%%%%%%%%%%%%%
% v1=input.phase.parameter(:,1:3)';
% v2=input.phase.parameter(:,4:6)';
% v3=input.phase.parameter(:,7:9)';
% v4=input.phase.parameter(:,10:12)';
% path1_1=coeff_pol_x(1)*t.*t.*t + coeff_pol_x(2)*t.*t + coeff_pol_x(3)*t + coeff_pol_x(4) - (lambda(:,1).*v1(1) + lambda(:,2).*v2(1) + lambda(:,3).*v3(1) + lambda(:,4).*v4(1)); 
% path1_2=coeff_pol_y(1)*t.*t.*t + coeff_pol_y(2)*t.*t + coeff_pol_y(3)*t + coeff_pol_y(4) - (lambda(:,1).*v1(2) + lambda(:,2).*v2(2) + lambda(:,3).*v3(2) + lambda(:,4).*v4(2)); 
% path1_3=coeff_pol_z(1)*t.*t.*t + coeff_pol_z(2)*t.*t + coeff_pol_z(3)*t + coeff_pol_z(4) - (lambda(:,1).*v1(3) + lambda(:,2).*v2(3) + lambda(:,3).*v3(3) + lambda(:,4).*v4(3)); 
% path1_4=ones(size(lambda(:,1)))-(lambda(:,1)+lambda(:,2)+lambda(:,3)+lambda(:,4));
% 
% pol_x_after=coeff_pol_x(1)*t.*t.*t + coeff_pol_x(2)*t.*t + coeff_pol_x(3)*t + coeff_pol_x(4);
% 
% sum_lambda_v_x_after=(lambda(:,1).*v1(1) + lambda(:,2).*v2(1) + lambda(:,3).*v3(1) + lambda(:,4).*v4(1));
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% 
% path1_after=[path1_1 path1_2 path1_3 path1_4];
% sum_lambda_v_x_after-sum_lambda_v_x_before;

% path1_after-path1_before



