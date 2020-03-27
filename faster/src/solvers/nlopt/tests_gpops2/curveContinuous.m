function output = curveContinuous( input )
%A is a matrix whose rows are the coefficients of the polynomials 

[B, R]=generateBR(input.phase.parameter);
A=getA(B,R);

tmp=sum(double(A))-[zeros(1,size(A,1)-1) 1];
output.path=repmat(tmp, size(input.phase.time,1),1);

output.dynamics =input.phase.control; % zeros(size(input.phase(1).time));