function output = curveEndpoint (input)

[B R]=generateBR(input.parameter);
A=getA(B,R);

obj=abs(double(det(A)));

output.objective = -obj; %I want to maximize the volume of |A|



