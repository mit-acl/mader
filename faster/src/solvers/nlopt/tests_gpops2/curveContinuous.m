function output = curveContinuous( input )

A=[]; %A is a matrix whose rows are the coefficients of the polynomials 

[B, R]=generateBR(input.phase.parameter);
A=getA(B,R);

% output.path=B';
output.dynamics =input.phase.control; % zeros(size(input.phase(1).time));