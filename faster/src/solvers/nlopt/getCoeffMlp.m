% Author: Jesus Tordesillas, jtorde@mit.edu
%
% Obtain the coefficients of a polynomial in GloptiPoly, given a variable
% Return in decreasing order. Example: a*t^3+bt^2+ct+d --> [a b c d]  
% EXAMPLE:
% mpol('x',1,1);
% mpol('y',1,1);
% mpol('t',1,1);
% 
% my_polynomial=x*x +2*x*y*t*t*t +y + 5*x*y*t;
% 
% getCoeffMlp(my_polynomial,t)

function coefficients= getCoeffMlp(polynomial, variable)

scalar_coeffs=coef(polynomial);
variables=listvar(polynomial);
A=pow(polynomial);

coefficients=[];
for i=1:length(variables)
    if(indvar(variables(i))==indvar(variable))
        column_variable_in_A=i;
        break
    end
end

max_degree=max(A(:,column_variable_in_A));

coefficients=mpol(zeros(max_degree+1,1));

for i=1:size(A,1) %For each row (each monomial)
    tmp=scalar_coeffs(i)*mpol(1);
    
   for j=1:size(A,2) %For each column (each variable)
       if( j~=column_variable_in_A && A(i,j)~=0 )
        tmp=tmp*power(variables(j),A(i,j));
       end
   end 
   
   degree=A(i,column_variable_in_A);
   
   coefficients(degree+1) = coefficients(degree+1) + tmp;
end

coefficients=flip(coefficients)';


end
