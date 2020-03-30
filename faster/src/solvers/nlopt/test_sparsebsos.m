close all; clc; clear;
syms t

deg=3;

W=[];

B=sym('B',[((deg+1)/2),1],'real');
R=sym('R',[(deg+1)/2,(deg-1)/2],'real');

%Insert half of the polynomials
for i=1:((deg+1)/2)
    pol=-B(i)*(t-1);
    for j=1:(deg-1)/2
        pol=pol*((t-R(i,j))^2);
    end
    W=[W;pol];
end

%Insert the other half
W=[W;subs(W,t,-t)];

%Solve for the coefficients:
coeffic=coeffs(sum(W),t,'All'); %When using All, no flip!! (gives directly [a  b c d]
coeffic=[zeros(1,deg+1-length(coeffic)) coeffic];

%Create the A matrix:
A=[];
for i=1:length(W)
    tmp=coeffs(W(i),t,'All');
    A=[A; tmp];
end
%Compute the determinant
pop.F=polynomialToMatrix(-1.0*det(A),[B(:); R(:)]);
pop.n = length([B(:); R(:)]); % number of variables
pop.I = {1:pop.n}; % interacting variables in obj


constraints_eq=sum(A)-[zeros(1,deg) 1]; %This should be =0

constraint_ineq=[ B(:)'  constraints_eq  -constraints_eq];%Eeach element has to be >=0
constraint_ineq=constraint_ineq(constraint_ineq~=0)

for i=1:length(constraint_ineq)
    disp("*******")
    constraint_ineq(i)
    pop.G{i} =polynomialToMatrix(constraint_ineq(i), [B(:); R(:)])
    sum_colum=sum(pop.G{i},1)
    variables_in_constraint=find(sum_colum)
    variables_in_constraint=variables_in_constraint(1:end-1)
    pop.J{i} = [variables_in_constraint];
    disp("*******")
end
%pop.J = {1:pop.n}; % interacting variables in constraints



pop.k=7; %  parameter of SOS
pop.d=10; %  parameter of LP

sdp = gendata2(pop,'SBSOS');
sol = csol(sdp,'sdpt3');
psol = postproc(pop,sdp,sol);

%%

%Returns a matrix B where each row is a monomial, and each column is a
%variable. The element (i,j) is therefore the power of the variable j
%in the monomial i. In 5*x^2*y, the term "5" will appear in the last
%column of the returned matrix B
function B=polynomialToMatrix(polynomial, variables)

determ=expand(sym(polynomial));

monomials=children(determ); %terms will contain each monomial

variables=variables(:);

B=zeros(length(monomials), length(variables)+1);
for i=1:length(monomials)

        monomial=monomials(i);
        [coeff b]= coeffs(monomial); %coeff is the coefficient of the monomial
        monomial_without_coeff=monomial/coeff;
        sym_vars_in_monomial=children(monomial_without_coeff);
        
        B(i,end)=coeff;
        
        if(sym_vars_in_monomial~=1) %Don't do if the monomial is "4" for instance
            for j=1:length(sym_vars_in_monomial)   
                tmp=sym_vars_in_monomial(j);  %tmp is t  or    x^2 or  y^5
                variable=symvar(tmp); 
                j=find(variables==variable); %find index of that variable
                B(i,j)=log10(subs(tmp, variable,10)); %take the exponent

            end
        end
end

end

