function output = curveContinuous( input )
%A is a matrix whose rows are the coefficients of the polynomials 

global num_of_states

[B, R]=generateBR(input.phase.state);
A=getA(B,R);

tmp=sum(double(A))-[zeros(1,size(A,1)-1) 1];

output.path=repmat(tmp, size(input.phase.time,1),1);

output.dynamics =zeros(size(input.phase(1).time,1),num_of_states);