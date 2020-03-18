function output = curveEndpoint (input)


v1=input.parameter(1:3)';
v2=input.parameter(4:6)';
v3=input.parameter(7:9)';
v4=input.parameter(10:12)';

tmp=[v1 v2 v3 v4];
tmp=[tmp;1 1 1 1];



output.objective = abs(computeDet4(tmp)); %I want to minimize the volume



