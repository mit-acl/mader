function output = curveEndpoint (input)

[B R]=generateBR(input.phase.finalstate);
A=getA(B,R);

obj=abs(double(det(A)));

output.objective = -obj; %I want to maximize |A|



