function output = curveEndpoint (input)

global deg_pol coeff_pol_x coeff_pol_y coeff_pol_z dim coeff_pol

V=[]; %V is a matrix whose columns are the control points v1,v2,v3,...
for i=1:dim:size(input.parameter,2)
    V=[V input.parameter(1,i:i+(dim-1))'];
end

% v1=input.parameter(1:3)';
% v2=input.parameter(4:6)';
% v3=input.parameter(7:9)';
% v4=input.parameter(10:12)';
% 
% tmp=[v1 v2 v3 v4];
% tmp=[tmp;1 1 1 1];

tmp=[V;ones(1,size(V,2))]; %Add a row of ones at the bottom of V

output.objective = abs(det(tmp)); %I want to minimize the volume



